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

#include "big_bertha_bringup/hardware_bridge_node.hpp"

#include <signal.h>

#include <algorithm>
#include <cmath>
#include <string>
#include <utility>
#include <vector>

#include "sensor_msgs/msg/joint_state.hpp"

namespace big_bertha_bringup
{

HardwareBridgeNode::HardwareBridgeNode() : Node("hardware_bridge")
{
  // ── Parameters ────────────────────────────────────────────────────────
  servo_host_ = declare_parameter<std::string>("servo_host", "127.0.0.1");
  servo_port_ = declare_parameter<int>("servo_port", 50007);
  imu_host_ = declare_parameter<std::string>("imu_host", "127.0.0.1");
  imu_port_ = declare_parameter<int>("imu_port", 50008);

  int pwm_min = declare_parameter<int>("pwm_min", 102);
  int pwm_max = declare_parameter<int>("pwm_max", 512);
  double joint_limit = declare_parameter<double>("joint_limit", 3.14159);

  ServoConverter::Params scp;
  scp.pwm_min = pwm_min;
  scp.pwm_max = pwm_max;
  scp.joint_limit = joint_limit;

  const auto validate12 = [&](const std::string & name, auto & out, const auto & fallback) {
    using T = typename std::decay_t<decltype(fallback)>::value_type;
    auto v = declare_parameter<std::vector<T>>(name, fallback);
    if (v.size() != 12) {
      RCLCPP_WARN(
        get_logger(), "'%s' has %zu entries, expected 12; using defaults", name.c_str(), v.size());
      out.assign(fallback.begin(), fallback.end());
    } else {
      out.assign(v.begin(), v.end());
    }
  };

  validate12(
    "servo_lower_limit", scp.servo_lower_limit,
    std::vector<double>{45, 30, 180, 140, 135, 140, 50, 50, 40, 180, 150, 0});
  validate12(
    "servo_upper_limit", scp.servo_upper_limit,
    std::vector<double>{180, 150, 50, 0, 0, 0, 180, 180, 180, 40, 0, 150});
  validate12(
    "servo_offset", scp.servo_offset, std::vector<double>{0, 0, 0, 0, 0, 0, 10, 10, 0, 8, 2, 5});
  validate12(
    "servo_channel", scp.servo_channel, std::vector<int>{14, 10, 2, 6, 13, 9, 1, 5, 12, 8, 0, 4});
  validate12(
    "servo_direction", scp.servo_direction,
    std::vector<int>{1, 1, 1, 1, 1, 1, -1, -1, -1, -1, 1, 1});
  scp.policy_center = declare_parameter<std::vector<double>>(
    "policy_center", {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.57, 1.57, 1.57, 1.57});

  orient_cov_ = declare_parameter<std::vector<double>>(
    "orientation_covariance", {-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
  accel_cov_ = declare_parameter<std::vector<double>>(
    "linear_acceleration_covariance", {0.001, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.001});
  gyro_cov_ = declare_parameter<std::vector<double>>(
    "angular_velocity_covariance", {0.00001, 0.0, 0.0, 0.0, 0.00001, 0.0, 0.0, 0.0, 0.00001});
  {
    std::vector<double> def_sign = {-1.0, 1.0, -1.0};
    imu_axis_sign_ = declare_parameter<std::vector<double>>("imu_axis_sign", def_sign);
  }
  {
    std::vector<double> def_sign = {-1.0, 1.0, -1.0};
    mag_axis_sign_ = declare_parameter<std::vector<double>>("mag_axis_sign", def_sign);
  }
  gyro_calibration_enabled_ = declare_parameter<bool>("gyro_calibration_enabled", true);
  gyro_calibration_samples_ = declare_parameter<int>("gyro_calibration_samples", 200);
  accel_calibration_enabled_ = declare_parameter<bool>("accel_calibration_enabled", true);
  accel_calibration_samples_ = declare_parameter<int>("accel_calibration_samples", 200);
  scp.single_joint_mode = declare_parameter<bool>("single_joint_mode", false);
  scp.single_joint_index = declare_parameter<int>("single_joint_index", 10);
  double control_rate = declare_parameter<double>("control_rate", 50.0);
  double max_joint_rate = declare_parameter<double>("max_joint_rate_rad_s", 6.54);
  scp.rate_limit_rad = max_joint_rate / control_rate;
  scp.smoothing_alpha = declare_parameter<double>("smoothing_alpha", 1.0);

  servo_converter_ = ServoConverter(std::move(scp));

  // ── TCP connects ──────────────────────────────────────────────────────
  if (!servo_client_.connect(servo_host_, servo_port_)) {
    RCLCPP_WARN(
      get_logger(), "servo relay connect to %s:%d failed", servo_host_.c_str(), servo_port_);
  } else {
    RCLCPP_INFO(get_logger(), "connected servo relay at %s:%d", servo_host_.c_str(), servo_port_);
  }
  if (!imu_client_.connect(imu_host_, imu_port_)) {
    RCLCPP_WARN(get_logger(), "IMU relay connect to %s:%d failed", imu_host_.c_str(), imu_port_);
  } else {
    RCLCPP_INFO(get_logger(), "connected IMU relay at %s:%d", imu_host_.c_str(), imu_port_);
  }

  // ── ROS ───────────────────────────────────────────────────────────────
  joint_names_ = declare_parameter<std::vector<std::string>>(
    "joint_names", {"Revolute_110", "Revolute_113", "Revolute_116", "Revolute_119", "Revolute_111",
                    "Revolute_114", "Revolute_117", "Revolute_120", "Revolute_112", "Revolute_115",
                    "Revolute_118", "Revolute_121"});
  imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("/imu", rclcpp::SensorDataQoS());
  mag_pub_ = create_publisher<sensor_msgs::msg::MagneticField>("/imu/mag", rclcpp::SensorDataQoS());
  joint_state_pub_ =
    create_publisher<sensor_msgs::msg::JointState>("/joint_states", rclcpp::QoS(1));
  cmd_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
    "/position_controller/commands", rclcpp::QoS(1),
    std::bind(&HardwareBridgeNode::on_cmd, this, std::placeholders::_1));

  reader_thread_ = std::thread(&HardwareBridgeNode::reader_loop, this);
}

HardwareBridgeNode::~HardwareBridgeNode()
{
  running_ = false;
  if (reader_thread_.joinable()) {
    reader_thread_.join();
  }
}

// ── Outbound: joint targets → servo PWM via TCP + Bridge RPC ──────────
void HardwareBridgeNode::on_cmd(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
  if (msg->data.size() != 12) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "expected 12 joints, got %zu", msg->data.size());
    return;
  }

  auto pwms = servo_converter_.convert(msg->data);

  // Publish /joint_states from the post-smoothing, post-rate-limit values
  // (what was actually commanded to the servos), with finite-differenced
  // velocity. This is the policy's only feedback on hardware — without it
  // 46% of the observation is dead (joint_pos and joint_vel stay zero).
  auto js = sensor_msgs::msg::JointState();
  js.header.stamp = now();
  js.name = joint_names_;
  const auto & limited = servo_converter_.last_targets();
  double dt =
    last_joint_state_time_.nanoseconds() > 0 ? (now() - last_joint_state_time_).seconds() : 0.0;
  for (size_t i = 0; i < 12; ++i) {
    js.position.push_back(limited[i]);
    if (dt > 1e-6) {
      js.velocity.push_back((limited[i] - last_joint_state_positions_[i]) / dt);
    } else {
      js.velocity.push_back(0.0);
    }
  }
  last_joint_state_positions_ = limited;
  last_joint_state_time_ = now();
  joint_state_pub_->publish(js);

  // Build JSON command
  std::string json = R"({"cmd":"servo","pwms":[)";
  for (size_t i = 0; i < 12; ++i) {
    if (i > 0) json += ',';
    json += std::to_string(pwms[i]);
  }
  json += "]}\n";

  if (!servo_client_.send(json)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "servo socket write error — reconnecting");
    if (!servo_client_.connect(servo_host_, servo_port_)) {
      RCLCPP_WARN(get_logger(), "servo reconnect failed");
    }
  }
}

// ── Inbound reader thread: poll IMU over TCP ─────────────────────────
void HardwareBridgeNode::reader_loop()
{
  while (running_) {
    if (!imu_client_.is_connected()) {
      imu_client_.connect(imu_host_, imu_port_);
    }
    if (imu_client_.is_connected() && calibrate_sensors()) {
      break;
    }
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  while (running_) {
    if (!imu_client_.is_connected()) {
      std::this_thread::sleep_for(std::chrono::seconds(1));
      imu_client_.connect(imu_host_, imu_port_);
      continue;
    }

    if (!imu_client_.send(R"({"cmd":"imu"})"
                          "\n"))
      continue;

    {
      std::string line;
      if (!imu_client_.read_line(line)) continue;

      if (!line.empty()) {
        ImuData data;
        bool got = parse_imu_json(line, data);
        if (!got) {
          std::string next;
          if (imu_client_.read_line(next) && !next.empty()) {
            got = parse_imu_json(next, data);
          }
        }
        if (got) {
          publish_imu(data);
        }
      }
    }

    imu_client_.drain();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

bool HardwareBridgeNode::calibrate_sensors()
{
  int samples = std::max(gyro_calibration_samples_, accel_calibration_samples_);
  if (samples == 0) return true;

  RCLCPP_INFO(get_logger(), "calibrating sensors (%d samples)...", samples);

  auto cal_start = std::chrono::steady_clock::now();
  double gx_sum = 0, gy_sum = 0, gz_sum = 0;
  double ax_sum = 0, ay_sum = 0, az_sum = 0;
  double ax_sq_sum = 0, ay_sq_sum = 0, az_sq_sum = 0;
  int collected = 0;

  while (running_ && collected < samples) {
    if (!imu_client_.send(R"({"cmd":"imu"})"
                          "\n"))
      return false;

    std::string line;
    if (!imu_client_.read_line(line)) return false;

    if (!line.empty()) {
      ImuData data;
      if (!parse_imu_json(line, data)) continue;
      gx_sum += data.gx;
      gy_sum += data.gy;
      gz_sum += data.gz;
      ax_sum += data.ax;
      ay_sum += data.ay;
      az_sum += data.az;
      ax_sq_sum += data.ax * data.ax;
      ay_sq_sum += data.ay * data.ay;
      az_sq_sum += data.az * data.az;
      ++collected;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  if (
    collected == 0 ||
    (std::abs(gx_sum) < 1e-9 && std::abs(gy_sum) < 1e-9 && std::abs(gz_sum) < 1e-9 &&
     std::abs(ax_sum) < 1e-9 && std::abs(ay_sum) < 1e-9 && std::abs(az_sum) < 1e-9)) {
    RCLCPP_ERROR(
      get_logger(),
      "IMU appears to be missing — all %d samples were zero. Check wiring and I2C bus.", collected);
    gyro_calibration_enabled_ = false;
    accel_calibration_enabled_ = false;
    return true;
  }

  if (gyro_calibration_enabled_) {
    gyro_bias_x_ = gx_sum / collected;
    gyro_bias_y_ = gy_sum / collected;
    gyro_bias_z_ = gz_sum / collected;
    RCLCPP_INFO(
      get_logger(), "gyro bias: gx=%.6f gy=%.6f gz=%.6f rad/s", gyro_bias_x_, gyro_bias_y_,
      gyro_bias_z_);
  }

  if (accel_calibration_enabled_) {
    double ax_mean = ax_sum / collected;
    double ay_mean = ay_sum / collected;
    double az_mean = az_sum / collected;
    accel_bias_x_ = ax_mean;
    accel_bias_y_ = ay_mean;
    accel_bias_z_ = az_mean - imu_axis_sign_[2] * 9.81;

    double ax_var = ax_sq_sum / collected - ax_mean * ax_mean;
    double ay_var = ay_sq_sum / collected - ay_mean * ay_mean;
    double az_var = az_sq_sum / collected - az_mean * az_mean;

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
      get_logger(), "gyro drift over cal period:  %.3e  %.3e  %.3e rad", drift_x, drift_y, drift_z);
  }

  return true;
}

void HardwareBridgeNode::publish_imu(const ImuData & data)
{
  auto msg = sensor_msgs::msg::Imu();
  msg.header.stamp = now();
  msg.header.frame_id = "imu_link";

  double ax_corr = data.ax - accel_bias_x_;
  double ay_corr = data.ay - accel_bias_y_;
  double az_corr = data.az - accel_bias_z_;
  double gx_corr = data.gx - gyro_bias_x_;
  double gy_corr = data.gy - gyro_bias_y_;
  double gz_corr = data.gz - gyro_bias_z_;

  msg.linear_acceleration.x = ax_corr * imu_axis_sign_[0];
  msg.linear_acceleration.y = ay_corr * imu_axis_sign_[1];
  msg.linear_acceleration.z = az_corr * imu_axis_sign_[2];

  msg.angular_velocity.x = gx_corr * imu_axis_sign_[0];
  msg.angular_velocity.y = gy_corr * imu_axis_sign_[1];
  msg.angular_velocity.z = gz_corr * imu_axis_sign_[2];

  std::copy(orient_cov_.begin(), orient_cov_.end(), msg.orientation_covariance.begin());
  std::copy(accel_cov_.begin(), accel_cov_.end(), msg.linear_acceleration_covariance.begin());
  std::copy(gyro_cov_.begin(), gyro_cov_.end(), msg.angular_velocity_covariance.begin());

  imu_pub_->publish(msg);

  if (data.mag_ok && (std::abs(data.mx) + std::abs(data.my) + std::abs(data.mz) > 1e-9)) {
    auto m = sensor_msgs::msg::MagneticField();
    m.header.stamp = msg.header.stamp;
    m.header.frame_id = "imu_link";
    m.magnetic_field.x = data.mx * mag_axis_sign_[0];
    m.magnetic_field.y = data.my * mag_axis_sign_[1];
    m.magnetic_field.z = data.mz * mag_axis_sign_[2];
    mag_pub_->publish(m);
  }

  auto now_steady = std::chrono::steady_clock::now();
  double elapsed = std::chrono::duration<double>(now_steady - cal_finish_time_).count();
  double drift_angle_x = gyro_bias_x_ * elapsed;
  double drift_angle_y = gyro_bias_y_ * elapsed;
  double drift_angle_z = gyro_bias_z_ * elapsed;

  static auto last_drift_log = std::chrono::steady_clock::now();
  double since_last_log = std::chrono::duration<double>(now_steady - last_drift_log).count();
  if (since_last_log > 10.0) {
    last_drift_log = now_steady;
    RCLCPP_INFO(
      get_logger(), "gyro accumulated drift:  %.3e  %.3e  %.3e rad  (elapsed %.0f s)",
      drift_angle_x, drift_angle_y, drift_angle_z, elapsed);
  }
}

}  // namespace big_bertha_bringup

int main(int argc, char ** argv)
{
  signal(SIGPIPE, SIG_IGN);
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<big_bertha_bringup::HardwareBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
