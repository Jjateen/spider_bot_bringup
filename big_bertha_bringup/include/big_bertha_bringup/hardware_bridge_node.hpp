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

#ifndef BIG_BERTHA_BRINGUP__HARDWARE_BRIDGE_NODE_HPP_
#define BIG_BERTHA_BRINGUP__HARDWARE_BRIDGE_NODE_HPP_

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "big_bertha_bringup/imu_parser.hpp"
#include "big_bertha_bringup/servo_converter.hpp"
#include "bridge_rpc_client.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace big_bertha_bringup
{

class HardwareBridgeNode : public rclcpp::Node
{
public:
  explicit HardwareBridgeNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~HardwareBridgeNode() override;

private:
  void on_cmd(const std_msgs::msg::Float64MultiArray::SharedPtr msg);

  // Bridge RPC notification handlers (called from the client reader thread).
  void on_imu(const std::string & text);
  void on_hw_status(const std::string & text);
  void on_i2c_scan(const std::vector<double> & params);
  void on_pong(const std::vector<double> & params);
  void on_imu_diag(const std::string & text);
  void on_servo_diag_result(const std::string & text);
  void on_servo_timeout(const std::vector<double> & params);

  void publish_imu(const ImuData & data);
  void accumulate_calibration(const ImuData & data);
  void finish_calibration();

  // Diagnostic services.
  void handle_status(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
    std::shared_ptr<std_srvs::srv::Trigger::Response> resp);
  void handle_scan_i2c(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
    std::shared_ptr<std_srvs::srv::Trigger::Response> resp);
  void handle_servo_diag(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
    std::shared_ptr<std_srvs::srv::Trigger::Response> resp);
  void handle_imu_diag(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
    std::shared_ptr<std_srvs::srv::Trigger::Response> resp);
  void handle_recalibrate_imu(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
    std::shared_ptr<std_srvs::srv::Trigger::Response> resp);

  // Bridge RPC client.
  std::unique_ptr<BridgeRPCClient> bridge_;

  // Servo conversion
  ServoConverter servo_converter_;

  // Outbound servo throttle. Both derived from the command_rate_hz parameter
  // so the send rate always matches the converter's per-message slew budget.
  rclcpp::Duration servo_tx_period_{0, 0};
  rclcpp::Time last_servo_tx_{0, 0, RCL_ROS_TIME};

  // ROS
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_pub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr status_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr scan_i2c_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr servo_diag_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr imu_diag_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr recalibrate_imu_srv_;
  std::vector<double> orient_cov_;
  std::vector<double> accel_cov_;
  std::vector<double> gyro_cov_;
  std::vector<double> imu_axis_sign_{-1.0, 1.0, -1.0};
  std::vector<double> mag_axis_sign_{-1.0, 1.0, -1.0};

  // Calibration
  bool gyro_calibration_enabled_{true};
  bool accel_calibration_enabled_{true};
  int gyro_calibration_samples_{200};
  int accel_calibration_samples_{200};
  double gyro_bias_x_{0.0}, gyro_bias_y_{0.0}, gyro_bias_z_{0.0};
  double accel_bias_x_{0.0}, accel_bias_y_{0.0}, accel_bias_z_{0.0};
  int64_t cal_duration_ms_{0};
  std::chrono::steady_clock::time_point cal_finish_time_;

  // Calibration accumulator (fed by on_imu on the reader thread).
  std::mutex cal_mutex_;
  bool calibration_done_{false};
  int cal_collected_{0};
  double cal_gx_sum_{0}, cal_gy_sum_{0}, cal_gz_sum_{0};
  double cal_ax_sum_{0}, cal_ay_sum_{0}, cal_az_sum_{0};
  double cal_ax_sq_sum_{0}, cal_ay_sq_sum_{0}, cal_az_sq_sum_{0};

  // Diagnostics cache (also updated from the reader thread).
  std::mutex diag_mutex_;
  std::condition_variable diag_cv_;
  std::string hw_status_str_;
  std::string imu_diag_str_;
  std::string servo_diag_str_;
  std::vector<double> i2c_scan_addrs_;
  uint64_t i2c_gen_{0};
  uint64_t servo_diag_gen_{0};
};

}  // namespace big_bertha_bringup

#endif  // BIG_BERTHA_BRINGUP__HARDWARE_BRIDGE_NODE_HPP_
