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
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include "big_bertha_bringup/tcp_client.hpp"
#include "big_bertha_bringup/imu_parser.hpp"
#include "big_bertha_bringup/servo_converter.hpp"

namespace big_bertha_bringup
{

class HardwareBridgeNode : public rclcpp::Node
{
public:
  HardwareBridgeNode();
  ~HardwareBridgeNode() override;

private:
  void on_cmd(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
  void reader_loop();
  bool calibrate_sensors();
  void publish_imu(const ImuData & data);

  // TCP
  std::string servo_host_;
  int servo_port_;
  std::string imu_host_;
  int imu_port_;
  TcpClient servo_client_;
  TcpClient imu_client_;

  // Servo conversion
  ServoConverter servo_converter_;

  // ROS
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_pub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_sub_;
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

  // Threading
  std::thread reader_thread_;
  std::atomic<bool> running_{true};
};

}  // namespace big_bertha_bringup

#endif  // BIG_BERTHA_BRINGUP__HARDWARE_BRIDGE_NODE_HPP_
