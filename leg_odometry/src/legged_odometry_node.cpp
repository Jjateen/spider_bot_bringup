// Copyright 2026 Big Bertha Authors
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

#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2_ros/transform_broadcaster.h"

using namespace std::chrono_literals;
class LeggedOdometryNode : public rclcpp::Node
{
public:
  LeggedOdometryNode() : Node("legged_odometry")
  {
    joint_names_ = declare_parameter<std::vector<std::string>>("joint_names", kDefaultJointNames);
    default_joint_pos_ =
      declare_parameter<std::vector<double>>("default_joint_pos", kDefaultJointPos);
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    publish_tf_ = declare_parameter<bool>("publish_tf", false);
    velocity_source_ = declare_parameter<std::string>("velocity_source", "imu_dead_reckon");
    drift_damping_ = declare_parameter<double>("drift_damping", 0.98);
    stationary_joint_vel_threshold_ =
      declare_parameter<double>("stationary_joint_vel_threshold", 0.02);
    stationary_accel_threshold_ = declare_parameter<double>("stationary_accel_threshold", 0.5);
    stationary_samples_ = declare_parameter<int>("stationary_samples", 10);
    servo_tau_ = declare_parameter<double>("servo_tau", 0.06);
    publish_joint_states_ = declare_parameter<bool>("publish_joint_states", true);

    if (velocity_source_ == "leg_kinematics") {
      RCLCPP_WARN(
        get_logger(), "leg_kinematics mode is a stub — velocity estimates will be inaccurate");
    }
    RCLCPP_INFO(
      get_logger(), "velocity source: %s, servo_tau=%.3f, ZUPT samples=%d",
      velocity_source_.c_str(), servo_tau_, stationary_samples_);

    last_joint_positions_ = default_joint_pos_;
    filtered_positions_ = default_joint_pos_;
    node_start_time_ = now();

    cmd_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      "/position_controller/commands", rclcpp::QoS(1),
      std::bind(&LeggedOdometryNode::on_cmd, this, std::placeholders::_1));

    imu_topic_ = declare_parameter<std::string>("imu_topic", "/imu");
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, rclcpp::SensorDataQoS(),
      std::bind(&LeggedOdometryNode::on_imu, this, std::placeholders::_1));

    joint_state_pub_ =
      create_publisher<sensor_msgs::msg::JointState>("/joint_states", rclcpp::QoS(1));
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/odom", rclcpp::QoS(1));

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // Joint states are published on every cmd callback. A separate timer
    // is intentionally avoided to prevent two interleaved streams at
    // different rates for the same topic.
  }

private:
  void on_cmd(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    if (msg->data.size() != 12) return;

    std::lock_guard<std::mutex> lk(joint_mutex_);

    auto now = this->now();

    auto js = sensor_msgs::msg::JointState();
    js.header.stamp = now;
    js.name = joint_names_;

    double dt = 0.0;
    if (last_cmd_time_.nanoseconds() > 0) {
      dt = (now - last_cmd_time_).seconds();
    }

    // 1st-order servo dynamics: EWMA filter simulates MG995's physical lag.
    // alpha = 1 - exp(-dt / tau) where tau matches the servo's closed-loop
    // response time (~0.06s for MG995 at 6.54 rad/s max speed). This breaks the
    // positive-feedback loop caused by feeding commanded positions as "measured"
    // joint state (see ISSUES.md #15).
    const double alpha = (dt > 1e-6) ? (1.0 - std::exp(-dt / servo_tau_)) : 1.0;

    for (size_t i = 0; i < 12; ++i) {
      double cmd_pos = msg->data[i];

      // EWMA: blend commanded position toward filtered position
      double filt_pos = alpha * cmd_pos + (1.0 - alpha) * filtered_positions_[i];
      filtered_positions_[i] = filt_pos;
      js.position.push_back(filt_pos);

      if (dt > 1e-6) {
        double vel = (filt_pos - last_joint_positions_[i]) / dt;
        js.velocity.push_back(vel);
        last_joint_velocities_[i] = vel;
      } else {
        js.velocity.push_back(0.0);
        last_joint_velocities_[i] = 0.0;
      }
      last_joint_positions_[i] = filt_pos;
    }
    last_cmd_time_ = now;

    if (publish_joint_states_) {
      joint_state_pub_->publish(js);
    }
  }

  void on_imu(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    auto now = this->now();

    if (last_imu_time_.nanoseconds() == 0) {
      last_imu_time_ = now;
      last_orientation_.setValue(
        msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
      last_orientation_.normalize();
      return;
    }

    double dt = (now - last_imu_time_).seconds();
    if (dt <= 0.0 || dt > 0.1) {
      last_imu_time_ = now;
      return;
    }

    tf2::Quaternion orientation(
      msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
    orientation.normalize();

    tf2::Vector3 velocity;
    tf2::Vector3 position;

    if (velocity_source_ == "leg_kinematics") {
      compute_leg_kinematics_velocity(orientation, orientation, dt, velocity, position);
    } else {
      compute_imu_dead_reckon(msg, orientation, dt, velocity, position);
    }

    // ZUPT: zero velocity when robot is stationary (all joints still, no linear
    // acceleration). Without this the leaky integrator bleeds steady walking
    // velocity to zero and standing drift accumulates unbounded.
    if (is_robot_stationary(msg)) {
      velocity = tf2::Vector3(0.0, 0.0, 0.0);
      position.setZ(0.0);
      last_zupt_time_ = now;
    }

    auto odom = nav_msgs::msg::Odometry();
    odom.header.stamp = msg->header.stamp;
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id = base_frame_;

    odom.pose.pose.position.x = position.x();
    odom.pose.pose.position.y = position.y();
    odom.pose.pose.position.z = position.z();
    odom.pose.pose.orientation.x = orientation.x();
    odom.pose.pose.orientation.y = orientation.y();
    odom.pose.pose.orientation.z = orientation.z();
    odom.pose.pose.orientation.w = orientation.w();

    odom.twist.twist.linear.x = velocity.x();
    odom.twist.twist.linear.y = velocity.y();
    odom.twist.twist.linear.z = velocity.z();
    odom.twist.twist.angular.x = msg->angular_velocity.x;
    odom.twist.twist.angular.y = msg->angular_velocity.y;
    odom.twist.twist.angular.z = msg->angular_velocity.z;

    // Dead-reckoned uncertainty grows with time since last ZUPT reset.
    // Position error from accelerometer bias (this IMU reads 11.07 vs 9.81 at
    // rest, leaving ~0.4 m/s² residual horizontal bias when tilted during
    // walking) integrates quadratically. Roll/pitch benefit from gravity
    // reference. Yaw uses node uptime (not ZUPT) since ZUPT constrains
    // velocity, not heading — yaw covariance grows unbounded forever.
    double dt_zupt = last_zupt_time_.nanoseconds() > 0 ? (now - last_zupt_time_).seconds() : 0.0;
    double dt_yaw = (now - node_start_time_).seconds();
    double pos_xy_cov = 0.01 + 0.1 * dt_zupt + 0.5 * dt_zupt * dt_zupt;
    double pos_z_cov = 0.01 + 0.05 * dt_zupt;
    double rp_cov = 0.01 + 0.005 * dt_zupt;
    double yaw_cov = 0.001 + 0.02 * dt_yaw;
    double vel_xy_cov = 0.1 + 0.1 * dt_zupt;
    double vel_z_cov = 0.1 + 0.05 * dt_zupt;

    odom.pose.covariance[0] = pos_xy_cov;
    odom.pose.covariance[7] = pos_xy_cov;
    odom.pose.covariance[14] = pos_z_cov;
    odom.pose.covariance[21] = rp_cov;
    odom.pose.covariance[28] = rp_cov;
    odom.pose.covariance[35] = yaw_cov;
    odom.twist.covariance[0] = vel_xy_cov;
    odom.twist.covariance[7] = vel_xy_cov;
    odom.twist.covariance[14] = vel_z_cov;

    odom_pub_->publish(odom);

    if (publish_tf_) {
      geometry_msgs::msg::TransformStamped tf;
      tf.header.stamp = msg->header.stamp;
      tf.header.frame_id = odom_frame_;
      tf.child_frame_id = base_frame_;
      tf.transform.translation.x = position.x();
      tf.transform.translation.y = position.y();
      tf.transform.translation.z = position.z();
      tf.transform.rotation.x = orientation.x();
      tf.transform.rotation.y = orientation.y();
      tf.transform.rotation.z = orientation.z();
      tf.transform.rotation.w = orientation.w();
      tf_broadcaster_->sendTransform(tf);
    }

    last_imu_time_ = now;
    last_orientation_ = orientation;
    last_velocity_ = velocity;
    last_position_ = position;
  }

  void compute_imu_dead_reckon(
    const sensor_msgs::msg::Imu::SharedPtr & msg, const tf2::Quaternion & orientation, double dt,
    tf2::Vector3 & velocity, tf2::Vector3 & position)
  {
    tf2::Vector3 accel_body(
      msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);

    tf2::Matrix3x3 rot(orientation);
    tf2::Vector3 accel_world = rot * accel_body;
    accel_world.setZ(accel_world.z() - 9.81);

    velocity = last_velocity_ + accel_world * dt;
    position = last_position_ + velocity * dt;

    if (position.z() < 0.0) position.setZ(0.0);
  }

  void compute_leg_kinematics_velocity(
    const tf2::Quaternion & orient_prev, const tf2::Quaternion & orient_curr, double dt,
    tf2::Vector3 & velocity, tf2::Vector3 & position)
  {
    (void)orient_prev;
    (void)orient_curr;
    (void)dt;

    // TODO(Jjateen): implement full forward kinematics for each leg using URDF geometry
    // and body-to-foot Jacobian to compute body velocity from joint velocities.
    // Fall back to simple velocity estimate for now. Indices follow the Isaac
    // group order (leg_odometry.yaml): [hips(0-3), knees(4-7), ankles(8-11)].
    double vx = 0.0, vy = 0.0, vz = 0.0;
    int stance = 0;
    for (int leg = 0; leg < 4; ++leg) {
      double hip_vel = last_joint_velocities_[leg];
      double knee_vel = last_joint_velocities_[leg + 4];
      vx += 0.02 * hip_vel;
      vy += 0.02 * knee_vel;
      vz += 0.005 * (hip_vel + knee_vel);
      stance++;
    }
    if (stance > 0) {
      vx /= stance;
      vy /= stance;
      vz /= stance;
    }

    tf2::Matrix3x3 rot(last_orientation_);
    tf2::Vector3 vel_body(vx, vy, vz);
    velocity = rot * vel_body;
    position = last_position_ + velocity * dt;
  }

  bool is_robot_stationary(const sensor_msgs::msg::Imu::SharedPtr & msg)
  {
    bool joints_still = true;
    for (size_t i = 0; i < 12; ++i) {
      if (std::abs(last_joint_velocities_[i]) > stationary_joint_vel_threshold_) {
        joints_still = false;
        break;
      }
    }

    double ax = msg->linear_acceleration.x;
    double ay = msg->linear_acceleration.y;
    double az = msg->linear_acceleration.z;
    double accel_norm = std::sqrt(ax * ax + ay * ay + az * az);
    bool accel_still = std::abs(accel_norm - 9.81) < stationary_accel_threshold_;

    if (joints_still && accel_still) {
      stationary_counter_++;
    } else {
      stationary_counter_ = 0;
    }

    return stationary_counter_ >= stationary_samples_;
  }

  // Fallback defaults — the config yaml is always loaded via launch, so these
  // are only used if the parameter declaration fails. Must match Isaac group
  // order (all hips, all knees, all ankles) to agree with policy_controller_node.
  inline static const std::vector<std::string> kDefaultJointNames{
    "Revolute_110", "Revolute_113", "Revolute_116", "Revolute_119", "Revolute_111", "Revolute_114",
    "Revolute_117", "Revolute_120", "Revolute_112", "Revolute_115", "Revolute_118", "Revolute_121"};

  inline static const std::vector<double> kDefaultJointPos{0.0,   0.0,   0.0,  0.0,  -0.32, -0.32,
                                                           -0.32, -0.32, 2.00, 2.00, 2.00,  2.00};

  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  std::string imu_topic_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  std::vector<std::string> joint_names_;
  std::vector<double> default_joint_pos_;
  bool publish_tf_;
  bool publish_joint_states_;
  std::string odom_frame_;
  std::string base_frame_;
  std::string velocity_source_;
  double drift_damping_;

  double stationary_joint_vel_threshold_;
  double stationary_accel_threshold_;
  int stationary_samples_;
  int stationary_counter_{0};

  double servo_tau_;
  std::vector<double> filtered_positions_{std::vector<double>(12, 0.0)};
  std::vector<double> last_joint_positions_{std::vector<double>(12, 0.0)};
  std::vector<double> last_joint_velocities_{std::vector<double>(12, 0.0)};
  rclcpp::Time last_cmd_time_{0, 0, RCL_ROS_TIME};
  std::mutex joint_mutex_;

  rclcpp::Time last_imu_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_zupt_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time node_start_time_{0, 0, RCL_ROS_TIME};
  tf2::Quaternion last_orientation_;
  tf2::Vector3 last_velocity_{0.0, 0.0, 0.0};
  tf2::Vector3 last_position_{0.0, 0.0, 0.0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LeggedOdometryNode>());
  rclcpp::shutdown();
  return 0;
}
