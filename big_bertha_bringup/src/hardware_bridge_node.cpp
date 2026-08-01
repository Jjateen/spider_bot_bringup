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

namespace big_bertha_bringup
{

HardwareBridgeNode::HardwareBridgeNode() : Node("hardware_bridge")
{
  // ── Parameters ────────────────────────────────────────────────────────
  std::string router_socket = declare_parameter<std::string>("router_socket", "");
  if (router_socket.empty()) {
    router_socket = "/run/arduino-router/rpc.sock";
  }

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
  double command_rate = declare_parameter<double>("command_rate_hz", 200.0);
  double max_joint_rate = declare_parameter<double>("max_joint_rate_rad_s", 6.54);
  scp.rate_limit_rad = max_joint_rate / command_rate;
  scp.smoothing_alpha = declare_parameter<double>("smoothing_alpha", 1.0);

  servo_converter_ = ServoConverter(std::move(scp));

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

  status_srv_ = create_service<std_srvs::srv::Trigger>(
    "~/status",
    [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> & req,
           std::shared_ptr<std_srvs::srv::Trigger::Response> resp) {
      handle_status(req, resp);
    });
  scan_i2c_srv_ = create_service<std_srvs::srv::Trigger>(
    "~/scan_i2c",
    [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> & req,
           std::shared_ptr<std_srvs::srv::Trigger::Response> resp) {
      handle_scan_i2c(req, resp);
    });
  servo_diag_srv_ = create_service<std_srvs::srv::Trigger>(
    "~/servo_diag",
    [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> & req,
           std::shared_ptr<std_srvs::srv::Trigger::Response> resp) {
      handle_servo_diag(req, resp);
    });
  imu_diag_srv_ = create_service<std_srvs::srv::Trigger>(
    "~/imu_diag",
    [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> & req,
           std::shared_ptr<std_srvs::srv::Trigger::Response> resp) {
      handle_imu_diag(req, resp);
    });

  // ── Bridge RPC ───────────────────────────────────────────────────────
  // Candidate socket paths: the running arduino-router may expose either of
  // these (guide vs PoC differ). Probing means no board-side config needed.
  std::vector<std::string> candidates = {
    router_socket,
    "/run/arduino-router/rpc.sock",
    "/var/run/arduino-router.sock",
    "/tmp/arduino-router.sock",
  };
  bridge_ = std::make_unique<BridgeRPCClient>(std::move(candidates));

  bridge_->provide("imu", std::bind(&HardwareBridgeNode::on_imu, this, std::placeholders::_1));
  bridge_->provide(
    "hw_status", std::bind(&HardwareBridgeNode::on_hw_status, this, std::placeholders::_1));
  bridge_->provide(
    "i2c_scan", std::bind(&HardwareBridgeNode::on_i2c_scan, this, std::placeholders::_1));
  bridge_->provide("pong", std::bind(&HardwareBridgeNode::on_pong, this, std::placeholders::_1));
  bridge_->provide_str(
    "imu_diag", std::bind(&HardwareBridgeNode::on_imu_diag, this, std::placeholders::_1));
  bridge_->provide_str(
    "servo_diag_result",
    std::bind(&HardwareBridgeNode::on_servo_diag_result, this, std::placeholders::_1));
  bridge_->provide(
    "servo_timeout", std::bind(&HardwareBridgeNode::on_servo_timeout, this, std::placeholders::_1));

  if (!bridge_->start()) {
    RCLCPP_ERROR(get_logger(), "Failed to connect to arduino-router (candidates probed)");
  } else {
    RCLCPP_INFO(get_logger(), "Connected to arduino-router at %s",
                bridge_->connected_path().c_str());
  }
}

HardwareBridgeNode::~HardwareBridgeNode()
{
  if (bridge_) {
    bridge_->stop();
  }
}

// ── Outbound: joint targets → servo PWM via Bridge RPC ────────────────
void HardwareBridgeNode::on_cmd(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
  if (!bridge_->is_connected()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, 
      "Bridge disconnected, cannot send servo commands");
    return;
  }
  
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

  // The firmware's set_servo_pwms takes ONE comma-separated string (a
  // 12-argument Bridge RPC call would exceed the router's arg limit).
  std::string payload;
  for (size_t i = 0; i < 12; ++i) {
    if (i > 0) {
      payload += ',';
    }
    payload += std::to_string(pwms[i]);
  }
  bridge_->notify_str("set_servo_pwms", payload);
}

// ── Inbound Bridge RPC notifications (client reader thread) ──────────
void HardwareBridgeNode::on_imu(const std::vector<double> & params)
{
  // [ax, ay, az, gx, gy, gz, mx, my, mz, sample, ts]
  if (params.size() < 9) {
    return;
  }
  ImuData data;
  data.ax = params[0];
  data.ay = params[1];
  data.az = params[2];
  data.gx = params[3];
  data.gy = params[4];
  data.gz = params[5];
  data.mx = params[6];
  data.my = params[7];
  data.mz = params[8];
  data.mag_ok = (std::abs(data.mx) + std::abs(data.my) + std::abs(data.mz)) > 1e-9;

  accumulate_calibration(data);
  publish_imu(data);
}

void HardwareBridgeNode::on_hw_status(const std::vector<double> & params)
{
  // [scan, ai, servo_calls, ping_count, pwm_attempts, pwm_fails,
  //  pwm_last_fail_ch, pwm_last_fail_code, set_servo_last_len,
  //  set_servo_last_idx, pwm_readback_ch0]
  if (params.size() < 11) {
    return;
  }
  int scan = static_cast<int>(params[0]);
  std::string status = "scan=" + std::to_string(scan);
  status += ",ai_ok=" + std::string(params[1] > 0.5 ? "true" : "false");
  status += ",pca9685_ok=" + std::string((scan & 1) == 0 ? "true" : "false");
  status += ",mpu9250_ok=" + std::string((scan & 2) == 0 ? "true" : "false");
  status += ",servo_calls=" + std::to_string(static_cast<int>(params[2]));
  status += ",ping_count=" + std::to_string(static_cast<int>(params[3]));
  status += ",pwm_write_attempts=" + std::to_string(static_cast<int>(params[4]));
  status += ",pwm_write_fails=" + std::to_string(static_cast<int>(params[5]));
  status += ",pwm_last_fail_ch=" + std::to_string(static_cast<int>(params[6]));
  status += ",pwm_last_fail_code=" + std::to_string(static_cast<int>(params[7]));
  status += ",set_servo_last_len=" + std::to_string(static_cast<int>(params[8]));
  status += ",set_servo_last_idx=" + std::to_string(static_cast<int>(params[9]));
  status += ",pwm_readback_ch0=" + std::to_string(static_cast<int>(params[10]));
  {
    std::lock_guard<std::mutex> lk(diag_mutex_);
    hw_status_str_ = status;
  }
  diag_cv_.notify_all();
}

void HardwareBridgeNode::on_i2c_scan(const std::vector<double> & params)
{
  {
    std::lock_guard<std::mutex> lk(diag_mutex_);
    i2c_scan_addrs_ = params;
    ++i2c_gen_;
  }
  diag_cv_.notify_all();
}

void HardwareBridgeNode::on_pong(const std::vector<double> & /*params*/)
{
  // Connectivity is tracked via the router socket itself; nothing to do.
}

void HardwareBridgeNode::on_imu_diag(const std::string & text)
{
  {
    std::lock_guard<std::mutex> lk(diag_mutex_);
    imu_diag_str_ = text;
  }
  diag_cv_.notify_all();
}

void HardwareBridgeNode::on_servo_diag_result(const std::string & text)
{
  {
    std::lock_guard<std::mutex> lk(diag_mutex_);
    servo_diag_str_ = text;
    ++servo_diag_gen_;
  }
  diag_cv_.notify_all();
}

void HardwareBridgeNode::on_servo_timeout(const std::vector<double> & params)
{
  if (params.size() > 0) {
    RCLCPP_ERROR(
      get_logger(), 
      "MCU servo watchdog triggered - no commands for %.0f ms (timeout=250ms)", 
      params[0]);
  }
}

void HardwareBridgeNode::accumulate_calibration(const ImuData & data)
{
  std::lock_guard<std::mutex> lk(cal_mutex_);
  if (calibration_done_ || !(gyro_calibration_enabled_ || accel_calibration_enabled_)) {
    return;
  }

  int target = std::max(gyro_calibration_samples_, accel_calibration_samples_);
  if (target == 0) {
    calibration_done_ = true;
    return;
  }

  cal_gx_sum_ += data.gx;
  cal_gy_sum_ += data.gy;
  cal_gz_sum_ += data.gz;
  cal_ax_sum_ += data.ax;
  cal_ay_sum_ += data.ay;
  cal_az_sum_ += data.az;
  cal_ax_sq_sum_ += data.ax * data.ax;
  cal_ay_sq_sum_ += data.ay * data.ay;
  cal_az_sq_sum_ += data.az * data.az;
  ++cal_collected_;

  if (cal_collected_ >= target) {
    finish_calibration();
  }
}

void HardwareBridgeNode::finish_calibration()
{
  const int collected = cal_collected_;
  const double gx_sum = cal_gx_sum_, gy_sum = cal_gy_sum_, gz_sum = cal_gz_sum_;
  const double ax_sum = cal_ax_sum_, ay_sum = cal_ay_sum_, az_sum = cal_az_sum_;
  const double ax_sq_sum = cal_ax_sq_sum_, ay_sq_sum = cal_ay_sq_sum_, az_sq_sum = cal_az_sq_sum_;

  if (
    collected == 0 ||
    (std::abs(gx_sum) < 1e-9 && std::abs(gy_sum) < 1e-9 && std::abs(gz_sum) < 1e-9 &&
     std::abs(ax_sum) < 1e-9 && std::abs(ay_sum) < 1e-9 && std::abs(az_sum) < 1e-9)) {
    RCLCPP_ERROR(
      get_logger(),
      "IMU appears to be missing — all %d samples were zero. Check wiring and I2C bus.", collected);
    gyro_calibration_enabled_ = false;
    accel_calibration_enabled_ = false;
    calibration_done_ = true;
    return;
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

  calibration_done_ = true;
  cal_finish_time_ = std::chrono::steady_clock::now();
  RCLCPP_INFO(get_logger(), "calibration complete (%d samples)", collected);
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

  if (data.mag_ok) {
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

  // publish_imu runs only on the bridge reader thread, so a static is safe.
  static auto last_drift_log = std::chrono::steady_clock::now();
  double since_last_log = std::chrono::duration<double>(now_steady - last_drift_log).count();
  if (since_last_log > 10.0) {
    last_drift_log = now_steady;
    RCLCPP_INFO(
      get_logger(), "gyro accumulated drift:  %.3e  %.3e  %.3e rad  (elapsed %.0f s)",
      drift_angle_x, drift_angle_y, drift_angle_z, elapsed);
  }
}

// ── Diagnostic services ────────────────────────────────────────────────
void HardwareBridgeNode::handle_status(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
  std::shared_ptr<std_srvs::srv::Trigger::Response> resp)
{
  (void)req;
  std::lock_guard<std::mutex> lk(diag_mutex_);
  resp->success = !hw_status_str_.empty();
  resp->message = hw_status_str_.empty() ? "no status yet" : hw_status_str_;
}

void HardwareBridgeNode::handle_scan_i2c(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
  std::shared_ptr<std_srvs::srv::Trigger::Response> resp)
{
  (void)req;
  uint64_t before;
  {
    std::lock_guard<std::mutex> lk(diag_mutex_);
    before = i2c_gen_;
  }
  bridge_->notify("scan_i2c");

  std::unique_lock<std::mutex> lk(diag_mutex_);
  diag_cv_.wait_for(lk, std::chrono::seconds(2), [&]() { return i2c_gen_ > before; });
  std::string msg = "addrs=[";
  for (size_t i = 0; i < i2c_scan_addrs_.size(); ++i) {
    if (i > 0) msg += ", ";
    msg += std::to_string(static_cast<int>(i2c_scan_addrs_[i]));
  }
  msg += "]";
  resp->success = !i2c_scan_addrs_.empty();
  resp->message = msg;
}

void HardwareBridgeNode::handle_servo_diag(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
  std::shared_ptr<std_srvs::srv::Trigger::Response> resp)
{
  (void)req;
  uint64_t before;
  {
    std::lock_guard<std::mutex> lk(diag_mutex_);
    before = servo_diag_gen_;
    servo_diag_str_.clear();
  }
  bridge_->notify("servo_diag");

  std::unique_lock<std::mutex> lk(diag_mutex_);
  diag_cv_.wait_for(lk, std::chrono::seconds(5), [&]() { return servo_diag_gen_ > before; });
  resp->success = !servo_diag_str_.empty();
  resp->message = servo_diag_str_.empty() ? "no servo diag result after 5s" : servo_diag_str_;
}

void HardwareBridgeNode::handle_imu_diag(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
  std::shared_ptr<std_srvs::srv::Trigger::Response> resp)
{
  (void)req;
  std::lock_guard<std::mutex> lk(diag_mutex_);
  resp->success = !imu_diag_str_.empty();
  resp->message = imu_diag_str_.empty() ? "no imu_diag yet" : imu_diag_str_;
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
