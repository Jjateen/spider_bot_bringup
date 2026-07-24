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
// IMU-gated ground-return filter for the body-mounted 2D lidar.
//
// The lidar tilts with the gait; tilted-down rays strike the floor and the
// level-TF costmap (EKF two_d_mode) paints them as walls. Per ray: rotate by
// the live IMU attitude, drop (-> +inf) returns whose endpoint lands below the
// floor. Real obstacles are struck at ~lidar height and survive.
//
// FRAME NOTE: the scan angle is in lidar_link, the IMU in base_link, and the
// mount chain makes them differ by Rz(pi). Ignoring that negates dir_z and the
// filter culls the wrong half (it deleted real walls). The offset comes from
// TF so a URDF change cannot silently re-invert it.
//
// Physics limit: at ~0.157 m lidar height the mean 3.3 deg walking tilt puts
// the beam on the floor by ~2 m, so the downhill side is blind past that --
// this node stops the floor reading as walls, it cannot recover that range.
// Measured survival post-fix: 0-1.5 m 95%, mid bands 9-14%, 4-8 m 49%.
// Real levers: raise the lidar or reduce gait tilt (see HANDOFF_TRAINING.md).

#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace big_bertha_sim_bringup
{

class ScanGroundFilter : public rclcpp::Node
{
public:
  ScanGroundFilter() : rclcpp::Node("scan_ground_filter")
  {
    // Body height above ground (TF can't supply it: EKF two_d_mode pins z=0).
    // 0.095 measured from gz ground-truth odometry while walking.
    body_height_ = this->declare_parameter<double>("body_height", 0.095);
    // Cull endpoints below this height (m).
    // ponytail: also absorbs the untracked ~+/-0.03 m gait heave; revisit only
    // if measurement shows residual gait-phase ghosting.
    floor_margin_ = this->declare_parameter<double>("floor_margin", 0.04);
    // Max IMU age vs the scan (s); a stale attitude culls against the wrong tilt.
    imu_max_age_ = this->declare_parameter<double>("imu_max_age", 0.05);
    base_frame_ = this->declare_parameter<std::string>("base_frame", "base_link");
    lidar_frame_ = this->declare_parameter<std::string>("lidar_frame", "lidar_link");

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    const auto sensor_qos = rclcpp::SensorDataQoS();
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "imu", sensor_qos, std::bind(&ScanGroundFilter::onImu, this, std::placeholders::_1));
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", sensor_qos, std::bind(&ScanGroundFilter::onScan, this, std::placeholders::_1));
    // RELIABLE so it satisfies the costmap's reliable subscription (a
    // best-effort publisher would never reach one).
    pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan_filtered", 10);

    RCLCPP_INFO(
      this->get_logger(),
      "scan_ground_filter up (body_height=%.3f m, floor_margin=%.3f m); "
      "resolving %s->%s from TF",
      body_height_, floor_margin_, base_frame_.c_str(), lidar_frame_.c_str());
  }

private:
  /// One-shot: mount height AND lidar->base yaw offset from TF (fixed joint).
  /// Hardcoding either is what previously staled the height / inverted the yaw.
  bool resolveMount()
  {
    if (mount_ready_) {
      return true;
    }
    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer_->lookupTransform(base_frame_, lidar_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "waiting for %s->%s TF (%s); passing scan through unfiltered", base_frame_.c_str(),
        lidar_frame_.c_str(), e.what());
      return false;
    }
    mount_z_ = tf.transform.translation.z;
    // yaw straight from the quaternion; avoids pulling in tf2_geometry_msgs
    // for a single conversion.
    const auto & q = tf.transform.rotation;
    lidar_yaw_ = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    mount_ready_ = true;
    RCLCPP_INFO(
      this->get_logger(),
      "mount resolved: lidar %.4f m above %s, yaw offset %.1f deg -> "
      "lidar height above ground %.3f m",
      mount_z_, base_frame_.c_str(), lidar_yaw_ * 180.0 / M_PI, body_height_ + mount_z_);
    return true;
  }

  void onImu(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    // Third row of the rotation matrix maps a body-frame ray to its world-z
    // component. Yaw does not enter these terms, so only roll/pitch tilt counts.
    const auto & q = msg->orientation;
    rzx_ = 2.0 * (q.x * q.z - q.w * q.y);
    rzy_ = 2.0 * (q.y * q.z + q.w * q.x);
    imu_stamp_ = rclcpp::Time(msg->header.stamp);
    have_imu_ = true;
  }

  void onScan(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    // Fail OPEN: passing raw through beats culling against a wrong tilt.
    if (!resolveMount() || !have_imu_) {
      if (!have_imu_) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 5000,
          "no IMU yet; passing scan through unfiltered");
      }
      pub_->publish(*msg);
      return;
    }
    const double age = (rclcpp::Time(msg->header.stamp) - imu_stamp_).seconds();
    if (std::abs(age) > imu_max_age_) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "IMU %.3f s stale vs scan (max %.3f); passing through unfiltered", age, imu_max_age_);
      pub_->publish(*msg);
      return;
    }

    const double lidar_height = body_height_ + mount_z_;
    const double rmax = msg->range_max;
    for (size_t i = 0; i < msg->ranges.size(); ++i) {
      const float r = msg->ranges[i];
      if (!std::isfinite(r) || r >= rmax) {
        continue;
      }
      // Scan angle is in lidar_link; add the mount yaw to express the ray in
      // base_link, which is the frame rzx_/rzy_ came from.
      const double a = msg->angle_min + static_cast<double>(i) * msg->angle_increment + lidar_yaw_;
      // World-z component of this ray's unit direction (cos a, sin a, 0).
      const double dir_z = rzx_ * std::cos(a) + rzy_ * std::sin(a);
      if (dir_z < 0.0) {  // ray points downward -> may strike the floor
        const double endpoint_h = lidar_height + r * dir_z;
        if (endpoint_h < floor_margin_) {
          msg->ranges[i] = std::numeric_limits<float>::infinity();  // ground/ghost
        }
      }
    }
    pub_->publish(*msg);
  }

  double body_height_{0.09};
  double floor_margin_{0.04};
  double imu_max_age_{0.05};
  std::string base_frame_{"base_link"};
  std::string lidar_frame_{"lidar_link"};
  bool mount_ready_{false};
  double mount_z_{0.0};    // lidar_link height above base_link, from TF
  double lidar_yaw_{0.0};  // lidar_link -> base_link yaw offset, from TF
  bool have_imu_{false};
  rclcpp::Time imu_stamp_{0, 0, RCL_ROS_TIME};
  double rzx_{0.0};  // 2*(x*z - w*y)
  double rzy_{0.0};  // 2*(y*z + w*x)
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_;
};

}  // namespace big_bertha_sim_bringup

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<big_bertha_sim_bringup::ScanGroundFilter>());
  rclcpp::shutdown();
  return 0;
}
