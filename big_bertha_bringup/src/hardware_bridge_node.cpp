// Copyright 2026 Jjateen Gundesha
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// big_bertha_bringup — hardware bridge node.
//
// Subscribes to /position_controller/commands (12 joint targets from the gait
// policy), converts radians to 12-bit PWM values, and sends them as JSON over
// a TCP socket to the Python relay (which forwards via Bridge RPC to the MCU).
// A background thread polls IMU data via the same TCP socket and publishes
// sensor_msgs/Imu on /imu.

#include <arpa/inet.h>
#include <netinet/in.h>
#include <signal.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

class HardwareBridgeNode : public rclcpp::Node
{
public:
  HardwareBridgeNode() : Node("hardware_bridge")
  {
    // ── Parameters ────────────────────────────────────────────────────

    // Where to find the Python relay
    host_ = declare_parameter<std::string>("host", "127.0.0.1");
    port_ = declare_parameter<int>("port", 50007);

    // Servo pulse range (0 to 4095)
    pwm_min_ = declare_parameter<int>("pwm_min", 102);
    pwm_max_ = declare_parameter<int>("pwm_max", 512);

    // Safety: never let a servo go past this angle
    joint_limit_ = declare_parameter<double>("joint_limit", 3.14159);

    // Per-servo calibration values
    servo_lower_ = declare_parameter<std::vector<double>>(
      "servo_lower_limit", {140, 50, 0, 180, 50, 150, 30, 140, 180, 45, 135, 40});  // smallest angle (degrees)
    servo_upper_ = declare_parameter<std::vector<double>>(
      "servo_upper_limit", {0, 180, 150, 50, 180, 0, 150, 0, 40, 180, 0, 180});     // largest angle (degrees)
    servo_offset_ = declare_parameter<std::vector<double>>(
      "servo_offset", {0, 10, 5, 0, 10, 2, 0, 0, 8, 0, 0, 0});                     // mounting offset (degrees)
    {
      std::vector<int64_t> def_ch = {6, 5, 4, 2, 1, 0, 10, 9, 8, 14, 13, 12};
      auto ch = declare_parameter<std::vector<int64_t>>("servo_channel", def_ch);
      servo_channel_.assign(ch.begin(), ch.end());                                   // which plug on the board each servo uses
    }
    {
      std::vector<int64_t> def_dir = {-1, -1, -1, -1, -1, -1, 1, -1, -1, 1, -1, -1};
      auto dir = declare_parameter<std::vector<int64_t>>("servo_direction", def_dir);
      servo_direction_.assign(dir.begin(), dir.end());                               // +1 = keep policy sign, -1 = flip policy sign
    }

    // IMU noise levels (higher = noisier, lower = trust more)
    orient_cov_ = declare_parameter<std::vector<double>>(
      "orientation_covariance", {-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});  // -1 means "not available"
    accel_cov_ = declare_parameter<std::vector<double>>(
      "linear_acceleration_covariance", {0.001, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.001});
    gyro_cov_ = declare_parameter<std::vector<double>>(
      "angular_velocity_covariance", {0.00001, 0.0, 0.0, 0.0, 0.00001, 0.0, 0.0, 0.0, 0.00001});

    // Calibration: measure sensor drift at startup
    gyro_calibration_enabled_ = declare_parameter<bool>("gyro_calibration_enabled", true);
    gyro_calibration_samples_ = declare_parameter<int>("gyro_calibration_samples", 200);
    accel_calibration_enabled_ = declare_parameter<bool>("accel_calibration_enabled", true);
    accel_calibration_samples_ = declare_parameter<int>("accel_calibration_samples", 200);

    // Test mode: move only one servo, hold the rest still
    single_joint_mode_ = declare_parameter<bool>("single_joint_mode", false);
    single_joint_index_ = declare_parameter<int>("single_joint_index", 10);

    // Rate limiter: per-step change clamp (radians) — prevents the policy
    // from commanding speeds the MG995 servos cannot physically track.
    // MG995 max = 375 °/s = 6.54 rad/s. At 50 Hz (20ms step):
    //   6.54 × 0.02 = 0.131 rad/step. We use 0.12 for a 10% safety margin.
    rate_limit_rad_ = declare_parameter<double>("rate_limit_rad", 0.12);

    // ── TCP connect ───────────────────────────────────────────────────
    connect();

    // ── Publisher ─────────────────────────────────────────────────────
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("/imu", rclcpp::SensorDataQoS());

    // ── Subscriber ────────────────────────────────────────────────────
    cmd_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      "/position_controller/commands", rclcpp::QoS(1),
      std::bind(&HardwareBridgeNode::on_cmd, this, std::placeholders::_1));

    // ── Background reader thread ──────────────────────────────────────
    reader_thread_ = std::thread(&HardwareBridgeNode::reader_loop, this);
  }

  ~HardwareBridgeNode() override
  {
    running_ = false;                              // tell the reader thread to stop
    if (reader_thread_.joinable()) {
      reader_thread_.join();                        // wait for it to finish
    }
    if (sock_fd_ >= 0) {
      ::close(sock_fd_);                            // close the TCP connection
    }
  }

private:
  void connect()
  {
    // Set up the address we want to connect to
    struct sockaddr_in addr
    {
    };
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port_);
    inet_pton(AF_INET, host_.c_str(), &addr.sin_addr);

    // Open a TCP socket
    sock_fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
    if (sock_fd_ < 0) {
      RCLCPP_ERROR(get_logger(), "failed to create socket");
      return;
    }

    // Try to reach the Python relay
    if (::connect(sock_fd_, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
      RCLCPP_WARN(get_logger(), "failed to connect to %s:%d — will retry", host_.c_str(), port_);
      ::close(sock_fd_);
      sock_fd_ = -1;          // mark as disconnected, caller will retry
    } else {
      RCLCPP_INFO(get_logger(), "connected to Python relay at %s:%d", host_.c_str(), port_);
    }
  }

  // ── Outbound: joint targets → servo PWM via TCP + Bridge RPC ────────
  void on_cmd(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    // We expect exactly 12 joint positions
    if (msg->data.size() != 12) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "expected 12 joints, got %zu", msg->data.size());
      return;
    }

    // Rate-limit each joint target so servo tracking error never accumulates.
    // On the first command just copy targets (nothing to limit from yet).
    std::vector<double> targets(12);
    if (first_cmd_) {
      for (size_t i = 0; i < 12; ++i) targets[i] = msg->data[i];
      first_cmd_ = false;
    } else {
      for (size_t i = 0; i < 12; ++i) {
        targets[i] = std::clamp(
          msg->data[i],
          last_targets_[i] - rate_limit_rad_,
          last_targets_[i] + rate_limit_rad_);
      }
    }
    last_targets_ = targets;

    // Turn each joint angle (in radians) into a PWM value (0 to 4095)
    std::vector<int> pwms(12);
    for (size_t i = 0; i < 12; ++i) {
      double rad = std::clamp(targets[i], -joint_limit_, joint_limit_);    // keep within safe bounds
      double deg = rad * 180.0 / M_PI;                                      // change radians to degrees
      deg = deg * servo_direction_[i];                                       // flip the sign if the policy's convention is reversed
      deg = deg + servo_offset_[i];                                          // fix the servo's mounting angle
      deg = deg + 90.0;                                                      // servo middle is at 90 degrees
      double lo = std::min(servo_lower_[i], servo_upper_[i]);
      double hi = std::max(servo_lower_[i], servo_upper_[i]);
      deg = std::clamp(deg, lo, hi);                                         // keep within the servo's physical range
      double t = deg / 180.0;                                                // how far from 0 to 180
      double pwm = std::round(t * (pwm_max_ - pwm_min_) + pwm_min_);        // stretch to the PWM range
      pwms[i] = static_cast<int>(std::clamp(pwm, 0.0, 4095.0));             // keep within 12-bit range
    }

    // Test mode: only move one servo, hold the rest at neutral
    if (single_joint_mode_) {
      int active = pwms[single_joint_index_];
      int neutral = (pwm_min_ + pwm_max_) / 2;
      for (auto & p : pwms) {
        p = neutral;
      }
      pwms[single_joint_index_] = active;
    }

    // Sort by channel number so the firmware gets them in wiring order
    std::vector<std::pair<int, int>> ch_pwm(12);
    for (size_t i = 0; i < 12; ++i) {
      ch_pwm[i] = {servo_channel_[i], pwms[i]};
    }
    std::sort(ch_pwm.begin(), ch_pwm.end());

    // Build a JSON command for the Python relay
    std::string json = R"({"cmd":"servo","pwms":[)";
    for (size_t i = 0; i < 12; ++i) {
      if (i > 0) json += ',';
      json += std::to_string(ch_pwm[i].second);
    }
    json += "]}\n";

    {
      std::lock_guard<std::mutex> lk(sock_mutex_);    // only one thread writes to the socket at a time
      if (sock_fd_ < 0) return;
      ssize_t n = ::send(sock_fd_, json.data(), json.size(), MSG_NOSIGNAL | MSG_DONTWAIT);
      if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
        // Socket buffer full (Python relay saturated by Bridge RPC).
        // Skip this command rather than stalling the ROS spin loop.
      } else if (n <= 0) {
        ::close(sock_fd_);
        sock_fd_ = -1;                                // send failed — will reconnect on next try
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "socket write error — reconnecting");
      }
    }
  }

  // ── Inbound reader thread: poll IMU over TCP ────────────────────────
  void reader_loop()
  {
    // ── Startup: calibrate sensors once ──────────────────────────

    // Keep trying until both the TCP connection and calibration work
    while (running_) {
      if (sock_fd_ < 0) {
        std::lock_guard<std::mutex> lk(sock_mutex_);
        connect();
      }
      if (sock_fd_ >= 0 && calibrate_sensors()) {
        break;
      }
      std::this_thread::sleep_for(std::chrono::seconds(1));   // wait before retry
    }

    // ── Main IMU polling loop ────────────────────────────────────
    while (running_) {
      // If the socket died, wait and try to reconnect
      if (sock_fd_ < 0) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        {
          std::lock_guard<std::mutex> lk(sock_mutex_);
          connect();
        }
        continue;
      }

      // Ask the Python relay for the latest IMU reading
      if (!send_imu_request()) continue;

      {
        // Read the reply
        std::string line;
        if (!read_imu_line(line)) continue;

        // Parse the JSON and send it to ROS.
        // If the STM32 firmware didn't push IMU data (sensor absent or I2C
        // failure), the Python relay returns {"error":"no imu data yet"} —
        // parse_imu_json returns false and we skip the publish.
        // If a non-IMU line was consumed (e.g. {"ok":true} from an
        // interleaved servo response), try the next line from the buffer
        // without sending a new request.
        if (!line.empty()) {
          double ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0;
          bool got_imu = parse_imu_json(line, ax, ay, az, gx, gy, gz);
          if (!got_imu) {
            // Non-IMU line — check if another response is already buffered
            auto nl = read_buf_.find('\n');
            if (nl != std::string::npos) {
              std::string next = read_buf_.substr(0, nl);
              read_buf_.erase(0, nl + 1);
              got_imu = parse_imu_json(next, ax, ay, az, gx, gy, gz);
            }
          }
          if (got_imu) {
            publish_imu(ax, ay, az, gx, gy, gz);
          }
        }
      }

      // Drain any leftover data into the buffer
      drain_imu_buf();

      std::this_thread::sleep_for(std::chrono::milliseconds(10));  // ~100 Hz
    }
  }

  bool send_imu_request()
  {
    std::lock_guard<std::mutex> lk(sock_mutex_);
    // Send: {"cmd":"imu"}\n
    const char req[] = R"({"cmd":"imu"})"
                       "\n";
    ssize_t n = ::send(sock_fd_, req, sizeof(req) - 1, MSG_NOSIGNAL);
    if (n <= 0) {
      ::close(sock_fd_);
      sock_fd_ = -1;          // socket is dead, mark for reconnect
      return false;
    }
    return true;
  }

  // Read one line (up to '\n') from the socket
  bool read_imu_line(std::string & line)
  {
    line.clear();
    // Check what we already have in the buffer before reading more
    auto pos = read_buf_.find('\n');
    while (pos == std::string::npos) {
      if (!refill_read_buf()) return false;
      pos = read_buf_.find('\n');
    }
    // Split off everything up to the newline
    line = read_buf_.substr(0, pos);
    read_buf_.erase(0, pos + 1);
    return true;
  }

  // Read up to 4096 bytes from the socket into our buffer
  bool refill_read_buf()
  {
    char buf[4096];
    ssize_t n = ::read(sock_fd_, buf, sizeof(buf));
    if (n <= 0) {
      ::close(sock_fd_);
      sock_fd_ = -1;
      return false;
    }
    read_buf_.append(buf, n);
    return true;
  }

  // Drain any leftover data from the socket into the buffer.
  // This prevents the receive buffer from filling up when the Python relay
  // has queued extras (e.g. during brief thread interleaving).
  void drain_imu_buf()
  {
    if (sock_fd_ < 0) return;
    // Check if there is data waiting without blocking
    fd_set rfds;
    FD_ZERO(&rfds);
    FD_SET(sock_fd_, &rfds);
    struct timeval tv = {0, 0};                          // zero timeout = check and return immediately
    if (select(sock_fd_ + 1, &rfds, nullptr, nullptr, &tv) > 0) {
      // Data is waiting — read it into the buffer
      char buf[4096];
      ssize_t n;
      do {
        n = ::read(sock_fd_, buf, sizeof(buf));
        if (n > 0) {
          read_buf_.append(buf, n);
        }
      } while (n > 0);                                   // keep reading until no more data
      if (n < 0) {
        std::lock_guard<std::mutex> lk(sock_mutex_);
        ::close(sock_fd_);
        sock_fd_ = -1;
      }
    }
  }

  bool calibrate_sensors()
  {
    // Take as many samples as needed (whichever sensor needs more)
    int samples = std::max(gyro_calibration_samples_, accel_calibration_samples_);
    if (samples == 0) return true;

    RCLCPP_INFO(get_logger(), "calibrating sensors (%d samples)...", samples);

    auto cal_start = std::chrono::steady_clock::now();

    // Running totals so we can average them later
    double gx_sum = 0, gy_sum = 0, gz_sum = 0;
    double ax_sum = 0, ay_sum = 0, az_sum = 0;
    double ax_sq_sum = 0, ay_sq_sum = 0, az_sq_sum = 0;   // also track squares for variance
    int collected = 0;

    while (running_ && collected < samples) {
      if (!send_imu_request()) return false;

      std::string line;
      if (!read_imu_line(line)) return false;

      // Got a reading — add it to the totals
      if (!line.empty()) {
        double ax, ay, az, gx, gy, gz;
        if (!parse_imu_json(line, ax, ay, az, gx, gy, gz)) {
          continue;  // error response (sensor absent) — skip this sample
        }
        gx_sum += gx;
        gy_sum += gy;
        gz_sum += gz;
        ax_sum += ax;
        ay_sum += ay;
        az_sum += az;
        ax_sq_sum += ax * ax;
        ay_sq_sum += ay * ay;
        az_sq_sum += az * az;
        ++collected;
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // Detect a missing/not-responding IMU: if we got 0 samples (every response
    // was an error from Python because the firmware never pushed IMU data) or
    // all samples were zero, the sensor is absent or the I2C bus has an issue.
    // Disable calibration so we don't publish a nonsense bias.
    if (collected == 0 || (std::abs(gx_sum) < 1e-9 && std::abs(gy_sum) < 1e-9 && std::abs(gz_sum) < 1e-9 &&
        std::abs(ax_sum) < 1e-9 && std::abs(ay_sum) < 1e-9 && std::abs(az_sum) < 1e-9)) {
      RCLCPP_ERROR(
        get_logger(),
        "IMU appears to be missing — all %d samples were zero. Check wiring and I2C bus.",
        collected);
      gyro_calibration_enabled_ = false;
      accel_calibration_enabled_ = false;
      return true;
    }

    // Gyro bias: when the robot is still, gyro should read zero.
    // Any non-zero average is the drift we need to subtract.
    if (gyro_calibration_enabled_) {
      gyro_bias_x_ = gx_sum / collected;
      gyro_bias_y_ = gy_sum / collected;
      gyro_bias_z_ = gz_sum / collected;
      RCLCPP_INFO(
        get_logger(), "gyro bias: gx=%.6f gy=%.6f gz=%.6f rad/s", gyro_bias_x_, gyro_bias_y_,
        gyro_bias_z_);
    }

    // Accel bias: when the robot is level, X and Y should read zero,
    // and Z should read 9.81 (gravity). Anything different is bias.
    if (accel_calibration_enabled_) {
      double ax_mean = ax_sum / collected;
      double ay_mean = ay_sum / collected;
      double az_mean = az_sum / collected;
      accel_bias_x_ = ax_mean;
      accel_bias_y_ = ay_mean;
      accel_bias_z_ = az_mean - 9.81;           // remove gravity from vertical axis

      // Variance tells us how noisy each axis is
      double ax_var = ax_sq_sum / collected - ax_mean * ax_mean;
      double ay_var = ay_sq_sum / collected - ay_mean * ay_mean;
      double az_var = az_sq_sum / collected - az_mean * az_mean;
      accel_cov_ = {ax_var, 0.0, 0.0, 0.0, ay_var, 0.0, 0.0, 0.0, az_var};

      RCLCPP_INFO(
        get_logger(), "accel bias: ax=%.6f ay=%.6f az=%.6f m/s²", accel_bias_x_, accel_bias_y_,
        accel_bias_z_);
      RCLCPP_INFO(
        get_logger(), "accel var:  ax=%.6e ay=%.6e az=%.6e (m/s²)²", ax_var, ay_var, az_var);
    }

    auto cal_end = std::chrono::steady_clock::now();
    cal_duration_ms_ =
      std::chrono::duration_cast<std::chrono::milliseconds>(cal_end - cal_start).count();
    cal_finish_time_ = cal_end;

    RCLCPP_INFO(
      get_logger(), "calibration took %ld ms (%d samples, ~%.1f Hz)", cal_duration_ms_, collected,
      collected / (cal_duration_ms_ / 1000.0));

    // Estimate how much the heading will drift since calibration
    if (gyro_calibration_enabled_) {
      double drift_x = gyro_bias_x_ * (cal_duration_ms_ / 1000.0);
      double drift_y = gyro_bias_y_ * (cal_duration_ms_ / 1000.0);
      double drift_z = gyro_bias_z_ * (cal_duration_ms_ / 1000.0);
      RCLCPP_INFO(
        get_logger(), "gyro drift over cal period:  %.3e  %.3e  %.3e rad", drift_x, drift_y,
        drift_z);
    }

    return true;
  }

  // Parse a JSON IMU response. Returns true if all 6 fields (ax, ay, az, gx, gy, gz)
  // were found and parsed successfully. Returns false if any field is missing (e.g.
  // the Python relay returned an error response when the STM32 firmware did not
  // push IMU data because the sensor was absent).
  bool parse_imu_json(
    const std::string & line, double & ax, double & ay, double & az, double & gx, double & gy,
    double & gz)
  {
    // Find a key like "ax" in the JSON and return the number after it
    auto find_val = [&](const std::string & key) -> std::pair<bool, double> {
      auto pos = line.find("\"" + key + "\"");
      if (pos == std::string::npos) return {false, 0.0};
      auto colon = line.find(':', pos);
      if (colon == std::string::npos) return {false, 0.0};
      // Skip past the colon and any spaces
      auto start = colon + 1;
      while (start < line.size() && (line[start] == ' ' || line[start] == '\t')) ++start;
      // Read until we hit a comma, closing brace, or bracket
      auto end = start;
      while (end < line.size() && line[end] != ',' && line[end] != '}' && line[end] != ']') ++end;
      try {
        return {true, std::stod(line.substr(start, end - start))};
      } catch (...) {
        return {false, 0.0};
      }
    };

    // Grab all six values from the IMU JSON — fail if any are missing
    auto r = find_val("ax"); if (!r.first) return false; ax = r.second;
    r = find_val("ay"); if (!r.first) return false; ay = r.second;
    r = find_val("az"); if (!r.first) return false; az = r.second;
    r = find_val("gx"); if (!r.first) return false; gx = r.second;
    r = find_val("gy"); if (!r.first) return false; gy = r.second;
    r = find_val("gz"); if (!r.first) return false; gz = r.second;
    return true;
  }

  void publish_imu(double ax, double ay, double az, double gx, double gy, double gz)
  {
    auto msg = sensor_msgs::msg::Imu();
    msg.header.stamp = now();
    msg.header.frame_id = "imu_link";

    // Remove the bias we measured during calibration
    double ax_corr = ax - accel_bias_x_;
    double ay_corr = ay - accel_bias_y_;
    double az_corr = az - accel_bias_z_;

    double gx_corr = gx - gyro_bias_x_;
    double gy_corr = gy - gyro_bias_y_;
    double gz_corr = gz - gyro_bias_z_;

    // IMU is physically mounted 180° rotated on the carrier board (URDF
    // imu_joint rpy="0 0 pi"). Negate horizontal axes to express the
    // measurement in the base_link frame.
    msg.linear_acceleration.x = -ax_corr;
    msg.linear_acceleration.y = -ay_corr;
    msg.linear_acceleration.z = az_corr;

    msg.angular_velocity.x = -gx_corr;
    msg.angular_velocity.y = -gy_corr;
    msg.angular_velocity.z = gz_corr;

    // Fill in the noise levels (covariance matrices)
    std::copy(orient_cov_.begin(), orient_cov_.end(), msg.orientation_covariance.begin());
    std::copy(accel_cov_.begin(), accel_cov_.end(), msg.linear_acceleration_covariance.begin());
    std::copy(gyro_cov_.begin(), gyro_cov_.end(), msg.angular_velocity_covariance.begin());

    imu_pub_->publish(msg);

    // Track how much the heading has drifted since calibration
    auto now_steady = std::chrono::steady_clock::now();
    double elapsed = std::chrono::duration<double>(now_steady - cal_finish_time_).count();
    drift_angle_x_ = gyro_bias_x_ * elapsed;
    drift_angle_y_ = gyro_bias_y_ * elapsed;
    drift_angle_z_ = gyro_bias_z_ * elapsed;

    // Print drift every 10 seconds so we can see if calibration is still good
    static auto last_drift_log = std::chrono::steady_clock::now();
    double since_last_log = std::chrono::duration<double>(now_steady - last_drift_log).count();
    if (since_last_log > 10.0) {
      last_drift_log = now_steady;
      RCLCPP_INFO(
        get_logger(), "gyro accumulated drift:  %.3e  %.3e  %.3e rad  (elapsed %.0f s)",
        drift_angle_x_, drift_angle_y_, drift_angle_z_, elapsed);
    }
  }

  // ── TCP ─────────────────────────────────────────────────────────────
  int sock_fd_{-1};
  std::mutex sock_mutex_;
  std::string host_;
  int port_;
  std::string read_buf_;  // buffered reads — avoids byte-by-byte syscalls

  // ── Servo calibration ───────────────────────────────────────────────
  int pwm_min_;
  int pwm_max_;
  double joint_limit_;
  std::vector<double> servo_lower_;
  std::vector<double> servo_upper_;
  std::vector<double> servo_offset_;
  std::vector<int> servo_channel_;
  std::vector<int> servo_direction_;

  // ── ROS ─────────────────────────────────────────────────────────────
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_sub_;
  std::vector<double> orient_cov_;
  std::vector<double> accel_cov_;
  std::vector<double> gyro_cov_;

  // ── Calibration biases ──────────────────────────────
  double gyro_bias_x_{0.0}, gyro_bias_y_{0.0}, gyro_bias_z_{0.0};
  double accel_bias_x_{0.0}, accel_bias_y_{0.0}, accel_bias_z_{0.0};
  bool gyro_calibration_enabled_;
  int gyro_calibration_samples_;
  bool accel_calibration_enabled_;
  int accel_calibration_samples_;
  bool calibrated_{false};
  bool single_joint_mode_{false};
  int single_joint_index_{10};
  int64_t cal_duration_ms_{0};
  std::chrono::steady_clock::time_point cal_finish_time_;
  double drift_angle_x_{0.0}, drift_angle_y_{0.0}, drift_angle_z_{0.0};

  // ── Rate limiter ───────────────────────────────────────────────────
  double rate_limit_rad_;
  std::vector<double> last_targets_{12, 0.0};
  bool first_cmd_{true};

  // ── Threading ───────────────────────────────────────────────────────
  std::thread reader_thread_;
  std::atomic<bool> running_{true};
};

int main(int argc, char ** argv)
{
  signal(SIGPIPE, SIG_IGN);                      // don't crash if the Python relay disconnects
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HardwareBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
