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

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cmath>
#include <cstring>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

class HardwareBridgeNode : public rclcpp::Node
{
public:
  HardwareBridgeNode() : Node("hardware_bridge")
  {
    // ── Parameters ────────────────────────────────────────────────────
    host_ = declare_parameter<std::string>("host", "127.0.0.1");
    port_ = declare_parameter<int>("port", 50007);
    pwm_min_ = declare_parameter<int>("pwm_min", 102);
    pwm_max_ = declare_parameter<int>("pwm_max", 512);
    joint_limit_ = declare_parameter<double>("joint_limit", 3.14159);
    orient_cov_ = declare_parameter<std::vector<double>>("orientation_covariance",
      {-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    accel_cov_ = declare_parameter<std::vector<double>>("linear_acceleration_covariance",
      {0.001, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.001});
    gyro_cov_ = declare_parameter<std::vector<double>>("angular_velocity_covariance",
      {0.00001, 0.0, 0.0, 0.0, 0.00001, 0.0, 0.0, 0.0, 0.00001});
    gyro_calibration_enabled_ = declare_parameter<bool>("gyro_calibration_enabled", true);
    gyro_calibration_samples_ = declare_parameter<int>("gyro_calibration_samples", 200);
    accel_calibration_enabled_ = declare_parameter<bool>("accel_calibration_enabled", true);
    accel_calibration_samples_ = declare_parameter<int>("accel_calibration_samples", 200);

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
    running_ = false;
    if (reader_thread_.joinable()) {
      reader_thread_.join();
    }
    if (sock_fd_ >= 0) {
      ::close(sock_fd_);
    }
  }

private:
  void connect()
  {
    struct sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port_);
    inet_pton(AF_INET, host_.c_str(), &addr.sin_addr);

    sock_fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
    if (sock_fd_ < 0) {
      RCLCPP_ERROR(get_logger(), "failed to create socket");
      return;
    }

    if (::connect(sock_fd_, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
      RCLCPP_WARN(get_logger(), "failed to connect to %s:%d — will retry",
                  host_.c_str(), port_);
      ::close(sock_fd_);
      sock_fd_ = -1;
    } else {
      RCLCPP_INFO(get_logger(), "connected to Python relay at %s:%d",
                  host_.c_str(), port_);
    }
  }

  // ── Outbound: joint targets → servo PWM via TCP + Bridge RPC ────────
  void on_cmd(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    if (sock_fd_ < 0) return;
    if (msg->data.size() != 12) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "expected 12 joints, got %zu", msg->data.size());
      return;
    }

    // Build JSON: {"cmd":"servo","pwms":[12 ints]}
    std::string json = R"({"cmd":"servo","pwms":[)";
    for (size_t i = 0; i < 12; ++i) {
      double angle = std::clamp(msg->data[i], -joint_limit_, joint_limit_);
      double t = (angle + M_PI) / (2.0 * M_PI);
      double pwm = std::round(t * (pwm_max_ - pwm_min_) + pwm_min_);
      pwm = std::clamp(pwm, 0.0, 4095.0);
      if (i > 0) json += ',';
      json += std::to_string(static_cast<int>(pwm));
    }
    json += "]}\n";

    {
      std::lock_guard<std::mutex> lk(sock_mutex_);
      ssize_t n = ::write(sock_fd_, json.data(), json.size());
      if (n < 0) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "socket write error");
      }
    }
  }

  // ── Inbound reader thread: poll IMU over TCP ────────────────────────
  void reader_loop()
  {
    while (running_) {
      if (sock_fd_ < 0) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        {
          std::lock_guard<std::mutex> lk(sock_mutex_);
          connect();
        }
        continue;
      }

      if (!calibrated_) {
        calibrate_sensors();
        calibrated_ = true;
      }

      if (!send_imu_request()) continue;

      std::string line;
      if (!read_imu_line(line)) continue;

      if (!line.empty()) {
        double ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0;
        parse_imu_json(line, ax, ay, az, gx, gy, gz);
        publish_imu(ax, ay, az, gx, gy, gz);
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(10));  // ~100 Hz
    }
  }

  bool send_imu_request()
  {
    std::lock_guard<std::mutex> lk(sock_mutex_);
    const char req[] = R"({"cmd":"imu"})" "\n";
    ssize_t n = ::write(sock_fd_, req, sizeof(req) - 1);
    if (n <= 0) {
      ::close(sock_fd_);
      sock_fd_ = -1;
      return false;
    }
    return true;
  }

  bool read_imu_line(std::string & line)
  {
    line.clear();
    char c;
    while (running_) {
      ssize_t n = ::read(sock_fd_, &c, 1);
      if (n <= 0) {
        ::close(sock_fd_);
        sock_fd_ = -1;
        return false;
      }
      if (c == '\n') break;
      line += c;
    }
    return true;
  }

  void calibrate_sensors()
  {
    int samples = std::max(gyro_calibration_samples_, accel_calibration_samples_);
    if (samples == 0) return;

    RCLCPP_INFO(get_logger(), "calibrating sensors (%d samples)...", samples);

    auto cal_start = std::chrono::steady_clock::now();

    double gx_sum = 0, gy_sum = 0, gz_sum = 0;
    double ax_sum = 0, ay_sum = 0, az_sum = 0;
    double ax_sq_sum = 0, ay_sq_sum = 0, az_sq_sum = 0;
    int collected = 0;

    while (running_ && collected < samples) {
      if (!send_imu_request()) return;

      std::string line;
      if (!read_imu_line(line)) return;

      if (!line.empty()) {
        double ax, ay, az, gx, gy, gz;
        parse_imu_json(line, ax, ay, az, gx, gy, gz);
        gx_sum += gx; gy_sum += gy; gz_sum += gz;
        ax_sum += ax; ay_sum += ay; az_sum += az;
        ax_sq_sum += ax * ax; ay_sq_sum += ay * ay; az_sq_sum += az * az;
        ++collected;
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    if (collected == 0) return;

    if (gyro_calibration_enabled_) {
      gyro_bias_x_ = gx_sum / collected;
      gyro_bias_y_ = gy_sum / collected;
      gyro_bias_z_ = gz_sum / collected;
      RCLCPP_INFO(get_logger(), "gyro bias: gx=%.6f gy=%.6f gz=%.6f rad/s",
                  gyro_bias_x_, gyro_bias_y_, gyro_bias_z_);
    }

    if (accel_calibration_enabled_) {
      double ax_mean = ax_sum / collected;
      double ay_mean = ay_sum / collected;
      double az_mean = az_sum / collected;
      accel_bias_x_ = ax_mean;
      accel_bias_y_ = ay_mean;
      accel_bias_z_ = az_mean - 9.81;

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

    if (gyro_calibration_enabled_) {
      double drift_x = gyro_bias_x_ * (cal_duration_ms_ / 1000.0);
      double drift_y = gyro_bias_y_ * (cal_duration_ms_ / 1000.0);
      double drift_z = gyro_bias_z_ * (cal_duration_ms_ / 1000.0);
      RCLCPP_INFO(
        get_logger(), "gyro drift over cal period:  %.3e  %.3e  %.3e rad", drift_x, drift_y,
        drift_z);
    }
  }

  void parse_imu_json(
    const std::string & line, double & ax, double & ay, double & az, double & gx, double & gy,
    double & gz)
  {
    auto find_val = [&](const std::string & key) -> double {
      auto pos = line.find("\"" + key + "\"");
      if (pos == std::string::npos) return 0.0;
      auto colon = line.find(':', pos);
      if (colon == std::string::npos) return 0.0;
      // skip past colon and any whitespace
      auto start = colon + 1;
      while (start < line.size() && (line[start] == ' ' || line[start] == '\t')) ++start;
      // read until comma or '}'
      auto end = start;
      while (end < line.size() && line[end] != ',' && line[end] != '}' && line[end] != ']') ++end;
      return std::stod(line.substr(start, end - start));
    };

    ax = find_val("ax");
    ay = find_val("ay");
    az = find_val("az");
    gx = find_val("gx");
    gy = find_val("gy");
    gz = find_val("gz");
  }

  void publish_imu(double ax, double ay, double az, double gx, double gy, double gz)
  {
    auto msg = sensor_msgs::msg::Imu();
    msg.header.stamp = now();
    msg.header.frame_id = "imu_link";

    double ax_corr = ax - accel_bias_x_;
    double ay_corr = ay - accel_bias_y_;
    double az_corr = az - accel_bias_z_;

    double gx_corr = gx - gyro_bias_x_;
    double gy_corr = gy - gyro_bias_y_;
    double gz_corr = gz - gyro_bias_z_;

    msg.linear_acceleration.x = ax_corr;
    msg.linear_acceleration.y = ay_corr;
    msg.linear_acceleration.z = az_corr;

    msg.angular_velocity.x = gx_corr;
    msg.angular_velocity.y = gy_corr;
    msg.angular_velocity.z = gz_corr;

    std::copy(orient_cov_.begin(), orient_cov_.end(), msg.orientation_covariance.begin());
    std::copy(accel_cov_.begin(), accel_cov_.end(), msg.linear_acceleration_covariance.begin());
    std::copy(gyro_cov_.begin(), gyro_cov_.end(), msg.angular_velocity_covariance.begin());

    imu_pub_->publish(msg);

    if (!calibrated_) return;

    auto now_steady = std::chrono::steady_clock::now();
    double elapsed = std::chrono::duration<double>(now_steady - cal_finish_time_).count();
    drift_angle_x_ = gyro_bias_x_ * elapsed;
    drift_angle_y_ = gyro_bias_y_ * elapsed;
    drift_angle_z_ = gyro_bias_z_ * elapsed;

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

  // ── Servo calibration ───────────────────────────────────────────────
  int pwm_min_;
  int pwm_max_;
  double joint_limit_;

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
  int64_t cal_duration_ms_{0};
  std::chrono::steady_clock::time_point cal_finish_time_;
  double drift_angle_x_{0.0}, drift_angle_y_{0.0}, drift_angle_z_{0.0};

  // ── Threading ───────────────────────────────────────────────────────
  std::thread reader_thread_;
  std::atomic<bool> running_{true};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HardwareBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
