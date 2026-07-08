// hw_bridge — ROS 2 node that communicates directly with the STM32U585
// co-processor via the arduino-router MessagePack RPC Unix socket.
//
// Architecture:
//   /position_controller/commands  →  notify("set_servo_pwms", 12 rad)
//   notify("imu", ...) from MCU   →  provide("imu", handler) → /imu
//
// No TCP, no Python, no Docker.  The router owns the UART; we talk
// MsgPack-RPC over /run/arduino-router/rpc.sock.

#include "bridge_rpc_client.hpp"

#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

class HwBridgeNode : public rclcpp::Node
{
public:
  HwBridgeNode() : Node("hardware_bridge")
  {
    // ── Parameters ──────────────────────────────────────────────────
    router_socket_ = declare_parameter<std::string>(
      "router_socket", "/run/arduino-router/rpc.sock");

    // IMU is mounted 180° rotated on the carrier board.
    // Negate horizontal axes to express in base_link frame.
    imu_orientation_ = declare_parameter<std::vector<double>>(
      "orientation_covariance", {-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    imu_accel_cov_ = declare_parameter<std::vector<double>>(
      "linear_acceleration_covariance", {0.001, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.001});
    imu_gyro_cov_ = declare_parameter<std::vector<double>>(
      "angular_velocity_covariance", {0.00001, 0.0, 0.0, 0.0, 0.00001, 0.0, 0.0, 0.0, 0.00001});

    // ── Publisher ───────────────────────────────────────────────────
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(
      "/imu", rclcpp::SensorDataQoS());

    // ── Subscriber ──────────────────────────────────────────────────
    cmd_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      "/position_controller/commands", rclcpp::QoS(1),
      std::bind(&HwBridgeNode::on_cmd, this, std::placeholders::_1));

    // ── Connect to router ───────────────────────────────────────────
    bridge_ = std::make_unique<BridgeRPCClient>(router_socket_);

    bridge_->provide("imu", [this](const std::vector<double> & params) {
      // params = [ax, ay, az, gx, gy, gz, sample_counter, mcu_timestamp_ms]
      if (params.size() < 6) return;

      auto msg = sensor_msgs::msg::Imu();
      msg.header.stamp = now();
      msg.header.frame_id = "imu_link";

      // Raw MCU values (MPU9250 in its own frame)
      double ax = params[0], ay = params[1], az = params[2];
      double gx = params[3], gy = params[4], gz = params[5];

      // IMU is physically mounted 180° rotated (URDF imu_joint rpy="0 0 pi").
      // Negate horizontal axes for base_link frame.
      msg.linear_acceleration.x = -ax;
      msg.linear_acceleration.y = -ay;
      msg.linear_acceleration.z = az;

      msg.angular_velocity.x = -gx;
      msg.angular_velocity.y = -gy;
      msg.angular_velocity.z = gz;

      // Covariance
      if (imu_orientation_.size() == 9)
        std::copy(imu_orientation_.begin(), imu_orientation_.end(),
                  msg.orientation_covariance.begin());
      if (imu_accel_cov_.size() == 9)
        std::copy(imu_accel_cov_.begin(), imu_accel_cov_.end(),
                  msg.linear_acceleration_covariance.begin());
      if (imu_gyro_cov_.size() == 9)
        std::copy(imu_gyro_cov_.begin(), imu_gyro_cov_.end(),
                  msg.angular_velocity_covariance.begin());

      imu_pub_->publish(msg);

      // Log MCU timestamp + sample counter for latency monitoring
      if (params.size() >= 8) {
        RCLCPP_DEBUG(get_logger(), "imu sample=%.0f mcu_ts=%.0f",
                     params[6], params[7]);
      }
    });

    if (!bridge_->start()) {
      RCLCPP_ERROR(get_logger(), "Failed to connect to router at %s",
                   router_socket_.c_str());
    } else {
      RCLCPP_INFO(get_logger(), "Connected to router at %s",
                  router_socket_.c_str());
    }
  }

private:
  void on_cmd(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    if (msg->data.size() != 12) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "expected 12 joints, got %zu", msg->data.size());
      return;
    }

    // Forward raw radians to firmware via Bridge RPC.
    // Firmware applies per-joint calibration (min, max, offset, direction).
    std::vector<double> params(msg->data.begin(), msg->data.end());
    bridge_->notify("set_servo_pwms", params);
  }

  // ── State ──────────────────────────────────────────────────────────

  std::string router_socket_;
  std::unique_ptr<BridgeRPCClient> bridge_;

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_sub_;

  std::vector<double> imu_orientation_;
  std::vector<double> imu_accel_cov_;
  std::vector<double> imu_gyro_cov_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HwBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
