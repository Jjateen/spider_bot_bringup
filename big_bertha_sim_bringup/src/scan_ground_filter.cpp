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
// Big Bertha's lidar is bolted to the body, so as the legged gait heaves/
// pitches/rolls the lidar tilts with it. A tilted-down beam strikes the floor
// and the 2D scan reports a near return; the costmap -- which (with EKF
// two_d_mode) believes the lidar is perfectly level -- paints that floor hit as
// a wall at body height. Every stride this flickers a fresh ring of 'ghost
// walls' that box the planner in.
//
// This node removes them at the perception layer, the realistic fix that also
// works on the real robot (the Uno Q lidar tilts too): for each ray it rotates
// the ray direction by the live IMU orientation, computes the 3D height of the
// return's endpoint above the ground, and drops (-> +inf) any return that lands
// at or below the floor. A real obstacle is struck at ~lidar height, so it
// survives; only floor hits -- the ghost-wall source -- are culled. Yaw cancels
// out of the height, so only the gait's roll/pitch matter.
//
// FRAME NOTE (this was a live bug): the scan angle is expressed in lidar_link,
// but the IMU orientation is base_link. Those frames are NOT aligned -- the
// mount chain (battery_joint rpy=(pi,0,0) then circuit_box_base_joint
// rpy=(0,pi,0)) composes to Rx(pi)*Ry(pi) = diag(-1,-1,1) = Rz(pi), a 180 deg
// yaw offset. Feeding a lidar-frame angle into base-frame roll/pitch terms
// negates dir_z, so the filter culled the UP-tilted half and passed every
// down-tilted ray -- i.e. it deleted real distant walls (measured: 33
// returns/scan at mean range 5.25 m) while removing zero floor hits. The offset
// is now taken from TF rather than hardcoded, so a future URDF change cannot
// silently re-invert it.
//
// WHAT THIS CAN AND CANNOT DO. Measured gait attitude while walking: total body
// tilt mean 3.3 deg, p95 6.6 deg, max 8.4 deg. With the lidar only ~0.157 m
// above ground (0.095 body + 0.0614 mount), a ray tilted down by the *mean*
// 3.3 deg is already at floor level by ~2.0 m. So while walking the sensor is
// physically blind past ~2 m on the downhill side -- there is no wall return to
// be had there, only floor. This node's job is to stop that floor being
// reported as a wall; it cannot recover range the tilt destroyed. Measured
// per-band survival after the fix: 0-1.5 m 95%, 1.5-2.5 m 14%, 2.5-4 m 9%,
// 4-8 m 49% (the surviving far returns are the uphill half). Near obstacles --
// the ones that matter for collision -- are preserved.
// The real levers for more usable range are raising the lidar or reducing gait
// tilt (training-side: flat_orientation_l2 is only -3.0 and the declared
// max_tilt_angle_deg is never read). See HANDOFF_TRAINING.md.

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
    // Body height above ground in nominal stance. TF cannot supply this: under
    // the full stack the EKF runs two_d_mode, so odom->base_link has z == 0 by
    // construction. 0.095 is MEASURED from gz ground-truth odometry while
    // walking (mean 0.0956, range 0.085-0.108); it also matches training's
    // base_height reward target of 0.09.
    body_height_ = this->declare_parameter<double>("body_height", 0.095);
    // Cull a return when its endpoint height is below this (m); slightly
    // positive so a near-floor grazing hit reads as ground, not wall.
    // ponytail: also absorbs the gait's ~+/-0.03 m heave, which we do not
    // track; revisit only if measurement shows residual gait-phase ghosting.
    floor_margin_ = this->declare_parameter<double>("floor_margin", 0.04);
    // Ignore an IMU sample older than this vs the scan (s). Scan is 10 Hz and
    // the gait is 0.67-1.4 Hz, so a stale attitude is a real fraction of a
    // stride and would cull against the wrong tilt.
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
  /// One-shot: take the lidar mount height AND the lidar->base yaw offset from
  /// TF (a fixed joint, so this never changes at runtime). Doing both from the
  /// same lookup is the point -- the height being hardcoded is what let it go
  /// stale when the URDF was re-ported, and the yaw being implicit is what
  /// inverted the filter.
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
    // Fail OPEN, never silently mis-filter: without a resolved mount or a fresh
    // attitude we cannot know which rays point at the floor, and passing the
    // raw scan through is strictly safer than culling against a wrong tilt.
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
