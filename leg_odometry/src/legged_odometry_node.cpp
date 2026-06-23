#include <chrono>
#include <cmath>
#include <memory>
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
    joint_names_ = declare_parameter<std::vector<std::string>>(
      "joint_names", kDefaultJointNames);
    default_joint_pos_ = declare_parameter<std::vector<double>>(
      "default_joint_pos", kDefaultJointPos);
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    publish_tf_ = declare_parameter<bool>("publish_tf", true);
    velocity_source_ =
      declare_parameter<std::string>("velocity_source", "imu_dead_reckon");
    drift_damping_ = declare_parameter<double>("drift_damping", 0.98);

    RCLCPP_INFO(
      get_logger(), "velocity source: %s", velocity_source_.c_str());

    last_cmd_positions_.resize(12, 0.0);

    cmd_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      "/position_controller/commands", rclcpp::QoS(1),
      std::bind(
        &LeggedOdometryNode::on_cmd, this, std::placeholders::_1));

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "/imu", rclcpp::SensorDataQoS(),
      std::bind(
        &LeggedOdometryNode::on_imu, this, std::placeholders::_1));

    joint_state_pub_ =
      create_publisher<sensor_msgs::msg::JointState>("/joint_states", 1);
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/odom", 1);

    tf_broadcaster_ =
      std::make_shared<tf2_ros::TransformBroadcaster>(this);


    // Publish initial joint states after a short delay so subscriptions
    // (e.g. policy_controller) have time to connect via DDS discovery.
    init_timer_ = create_wall_timer(
      100ms, std::bind(&LeggedOdometryNode::publish_initial_joint_states, this));
  }

private:
  void on_cmd(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    if (msg->data.size() != 12) return;

    auto now = this->now();
    auto now_steady = std::chrono::steady_clock::now();

    auto js = sensor_msgs::msg::JointState();
    js.header.stamp = now;
    js.name = joint_names_;

    double dt = 0.0;
    if (last_cmd_time_.time_since_epoch().count() > 0) {
      dt = std::chrono::duration<double>(now_steady - last_cmd_time_).count();
    }

    for (size_t i = 0; i < 12; ++i) {
      double pos = default_joint_pos_[i] + msg->data[i];
      js.position.push_back(pos);

      if (dt > 1e-6) {
        double vel = (msg->data[i] - last_cmd_positions_[i]) / dt;
        js.velocity.push_back(vel);
        last_joint_velocities_[i] = vel;
      } else {
        js.velocity.push_back(0.0);
        last_joint_velocities_[i] = 0.0;
      }
      last_cmd_positions_[i] = msg->data[i];
      last_joint_positions_[i] = msg->data[i];
    }
    last_cmd_time_ = now_steady;

    joint_state_pub_->publish(js);
  }

  void on_imu(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    auto now_steady = std::chrono::steady_clock::now();

    if (last_imu_time_.time_since_epoch().count() == 0) {
      last_imu_time_ = now_steady;
      last_orientation_.setValue(
        msg->orientation.x, msg->orientation.y, msg->orientation.z,
        msg->orientation.w);
      last_orientation_.normalize();
      return;
    }

    double dt =
      std::chrono::duration<double>(now_steady - last_imu_time_).count();
    if (dt <= 0.0 || dt > 0.1) {
      last_imu_time_ = now_steady;
      return;
    }

    tf2::Quaternion orientation(
      msg->orientation.x, msg->orientation.y, msg->orientation.z,
      msg->orientation.w);
    orientation.normalize();

    tf2::Vector3 velocity;
    tf2::Vector3 position;

    if (velocity_source_ == "leg_kinematics") {
      compute_leg_kinematics_velocity(
        orientation, orientation, dt, velocity, position);
    } else {
      compute_imu_dead_reckon(
        msg, orientation, dt, velocity, position);
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

    odom.pose.covariance[0] = 0.01;
    odom.pose.covariance[7] = 0.01;
    odom.pose.covariance[14] = 0.01;
    odom.pose.covariance[21] = 0.01;
    odom.pose.covariance[28] = 0.01;
    odom.pose.covariance[35] = 0.001;
    odom.twist.covariance[0] = 0.1;
    odom.twist.covariance[7] = 0.1;
    odom.twist.covariance[14] = 0.1;

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

    last_imu_time_ = now_steady;
    last_orientation_ = orientation;
    last_velocity_ = velocity;
    last_position_ = position;
  }

  void publish_initial_joint_states()
  {
    init_timer_->cancel();

    auto js = sensor_msgs::msg::JointState();
    js.header.stamp = now();
    js.name = joint_names_;
    for (size_t i = 0; i < 12; ++i) {
      js.position.push_back(default_joint_pos_[i]);
      js.velocity.push_back(0.0);
    }
    joint_state_pub_->publish(js);
    RCLCPP_INFO(get_logger(), "published initial joint states (default pose)");
  }


  void compute_imu_dead_reckon(
    const sensor_msgs::msg::Imu::SharedPtr & msg,
    const tf2::Quaternion & orientation, double dt,
    tf2::Vector3 & velocity, tf2::Vector3 & position)
  {
    tf2::Vector3 accel_body(
      msg->linear_acceleration.x, msg->linear_acceleration.y,
      msg->linear_acceleration.z);

    tf2::Matrix3x3 rot(orientation);
    tf2::Vector3 accel_world = rot * accel_body;
    accel_world.setZ(accel_world.z() - 9.81);

    velocity = last_velocity_ + accel_world * dt;
    velocity *= drift_damping_;
    position = last_position_ + velocity * dt;

    if (position.z() < 0.0) position.setZ(0.0);
  }

  void compute_leg_kinematics_velocity(
    const tf2::Quaternion & orient_prev,
    const tf2::Quaternion & orient_curr,
    double dt,
    tf2::Vector3 & velocity,
    tf2::Vector3 & position)
  {
    (void)orient_prev;
    (void)orient_curr;
    (void)dt;

    // TODO: implement full forward kinematics for each leg using URDF geometry
    // and body-to-foot Jacobian to compute body velocity from joint velocities.
    // Fall back to simple velocity estimate for now.
    double vx = 0.0, vy = 0.0, vz = 0.0;
    int stance = 0;
    for (size_t i = 0; i < 12; i += 3) {
      double knee_vel = last_joint_velocities_[i + 2];
      double hip_vel = last_joint_velocities_[i + 1];
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
    velocity *= drift_damping_;
    position = last_position_ + velocity * dt;
  }

  inline static const std::vector<std::string> kDefaultJointNames{
    "Revolute_110", "Revolute_111", "Revolute_112",
    "Revolute_113", "Revolute_114", "Revolute_115",
    "Revolute_116", "Revolute_117", "Revolute_118",
    "Revolute_119", "Revolute_120", "Revolute_121"
  };

  inline static const std::vector<double> kDefaultJointPos{
    0.0, 0.5, 0.0,
    0.0, 0.5, 0.0,
    0.0, 0.5, 0.0,
    0.0, 0.5, 0.0
  };

  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr
    cmd_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr
    joint_state_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr init_timer_;

  std::vector<std::string> joint_names_;
  std::vector<double> default_joint_pos_;
  bool publish_tf_;
  std::string odom_frame_;
  std::string base_frame_;
  std::string velocity_source_;
  double drift_damping_;

  std::vector<double> last_cmd_positions_{std::vector<double>(12, 0.0)};
  std::vector<double> last_joint_positions_{std::vector<double>(12, 0.0)};
  std::vector<double> last_joint_velocities_{std::vector<double>(12, 0.0)};
  std::chrono::steady_clock::time_point last_cmd_time_;

  std::chrono::steady_clock::time_point last_imu_time_;
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
