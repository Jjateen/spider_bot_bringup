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
// the IMU attitude AT THAT RAY'S OWN CAPTURE TIME, drop (-> +inf) returns
// whose endpoint lands below the floor. Real obstacles are struck at ~lidar
// height and survive.
//
// PER-RAY TIMING: a sweeping lidar (Isaac's RotatingLidarPhysX; ~100 ms/scan
// here) doesn't capture all 360 rays at once, so one IMU sample for the whole
// scan mis-corrects rays from the more-tilted part of the gait cycle -- a
// confirmed ghost-scan contributor (this gait's asymmetric leg-phase offsets
// produce a direction-biased lean the old single-sample correction couldn't
// track). Each ray's real capture time is derived from
// LaserScan.header.stamp/time_increment per the message's own documented
// contract (header.stamp = first ray's acquisition time, time_increment =
// spacing to the next) -- EXCEPT this bridge stamps at sweep-END, not
// sweep-start (confirmed against Isaac's own IsaacReadLidarBeams.beamTimeData
// buffer, which increases monotonically from angle_min to angle_max within
// one scan), so ray i's time is computed backward from the stamp:
// stamp - (n-1-i)*time_increment. Gazebo/hardware publish time_increment=0
// (instantaneous scan), which collapses every ray to the scan stamp --
// identical to the old single-sample behavior, unchanged.
//
// FRAME NOTE: the scan angle is in lidar_link, the IMU in base_link, and the
// mount chain makes them differ by Rz(pi). Ignoring that negates dir_z and the
// filter culls the wrong half (it deleted real walls). The offset comes from
// TF so a URDF change cannot silently re-invert it.
//
// Physics limit: at ~0.157 m lidar height the mean 3.3 deg walking tilt puts
// the beam on the floor by ~2 m, so the downhill side is blind past that --
// this node stops the floor reading as walls, it cannot recover that range.
//
// PER-RAY TIMING RESULT (measured, not assumed): A/B'd against the old
// single-sample-per-scan code at matched floor_margin=0.07, same walking
// gait, 2 fresh-respawn trials/side (measure_scan.py, 30 s each, confirmed
// walking throughout via joint_states). Old: near 95.3%, mid 59.0%/59.0%,
// far 72.4%. New trial 1: near 96.1%, mid 35.0%/66.1%, far 67.3%. New trial
// 2: near 86.3%, mid 60.0%/58.1%, far 73.7% -- landing almost exactly on the
// old baseline. The spread between the two new-code trials is as large as
// the old-vs-new gap, i.e. this sim's run-to-run path variance swamps
// whatever this fix contributes on the raw survival metric; it did NOT
// produce a clear, repeatable reduction in ghost-band culling here. The
// per-ray correction is still real and correct-by-construction (it's what
// the sensor's own timing data says is happening, verified against
// IsaacReadLidarBeams.beamTimeData, not inferred), and it's strictly more
// accurate than the one-sample-per-360-rays it replaced -- but per the
// rotation_rate=0 experiment above this was already known to be a secondary
// contributor, not the dominant one, and that holds up. Don't re-litigate
// this without either more trials or the geometric-reprojection tool
// mentioned in isaac_bringup.launch.py's scan_filter comment (not present in
// this repo -- raw survival % mixes real geometry with ghosts and isn't
// reliable across differently-walked paths).
// Real levers: raise the lidar or reduce gait tilt (see HANDOFF_TRAINING.md).

#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "big_bertha_sim_bringup/ground_ray.hpp"
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
  explicit ScanGroundFilter(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : rclcpp::Node("scan_ground_filter", options)
  {
    // Body height above ground (TF can't supply it: EKF two_d_mode pins z=0).
    // 0.095 measured from gz ground-truth odometry while walking.
    body_height_ = this->declare_parameter<double>("body_height", 0.095);
    // Cull endpoints below this height (m).
    // ponytail: also absorbs the untracked ~+/-0.03 m gait heave; revisit only
    // if measurement shows residual gait-phase ghosting.
    floor_margin_ = this->declare_parameter<double>("floor_margin", 0.04);
    // Max age of the newest buffered IMU sample vs the scan (s); if even the
    // freshest sample we have is this stale, the whole scan passes through
    // unfiltered rather than culling against attitude we don't trust.
    imu_max_age_ = this->declare_parameter<double>("imu_max_age", 0.05);
    // How much IMU history to retain for per-ray interpolation (s). Must
    // cover one scan's sweep duration (~0.1 s here) plus margin; irrelevant
    // for Gazebo/hardware (time_increment=0, every ray uses the newest
    // sample same as before).
    imu_history_window_ = this->declare_parameter<double>("imu_history_window", 0.3);
    // Orientation source. On hardware this is the Madgwick output (/filtered/imu,
    // has roll/pitch); sim's raw /imu already carries orientation. Default /imu
    // keeps the sim bringup unchanged (it passes no imu_topic).
    imu_topic_ = this->declare_parameter<std::string>("imu_topic", "/imu");
    base_frame_ = this->declare_parameter<std::string>("base_frame", "base_link");
    lidar_frame_ = this->declare_parameter<std::string>("lidar_frame", "lidar_link");

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    const auto sensor_qos = rclcpp::SensorDataQoS();
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, sensor_qos, std::bind(&ScanGroundFilter::onImu, this, std::placeholders::_1));
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
    const double t = rclcpp::Time(msg->header.stamp).seconds();
    imu_history_.push_back(
      {t, 2.0 * (q.x * q.z - q.w * q.y), 2.0 * (q.y * q.z + q.w * q.x)});
    const double cutoff = t - imu_history_window_;
    while (imu_history_.size() > 1 && imu_history_.front().t < cutoff) {
      imu_history_.erase(imu_history_.begin());
    }
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
    const double scan_stamp = rclcpp::Time(msg->header.stamp).seconds();
    const double newest_age = scan_stamp - imu_history_.back().t;
    if (std::abs(newest_age) > imu_max_age_) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000,
        "IMU %.3f s stale vs scan (max %.3f); passing through unfiltered", newest_age,
        imu_max_age_);
      pub_->publish(*msg);
      return;
    }

    const double lidar_height = body_height_ + mount_z_;
    const double rmax = msg->range_max;
    const double time_increment = static_cast<double>(msg->time_increment);
    const size_t n = msg->ranges.size();
    for (size_t i = 0; i < n; ++i) {
      const float r = msg->ranges[i];
      if (!std::isfinite(r) || r >= rmax) {
        continue;
      }
      // header.stamp is this bridge's sweep-END time; ray i (0 = angle_min)
      // was captured time_increment*(n-1-i) earlier. time_increment==0
      // (Gazebo/hardware) collapses this to scan_stamp for every ray.
      const double ray_time = scan_stamp - time_increment * static_cast<double>(n - 1 - i);
      double rzx, rzy;
      interpolate_rz(imu_history_, ray_time, rzx, rzy);
      const double a = msg->angle_min + static_cast<double>(i) * msg->angle_increment;
      if (is_ground_ray(a, lidar_yaw_, rzx, rzy, r, lidar_height, floor_margin_)) {
        msg->ranges[i] = std::numeric_limits<float>::infinity();
      }
    }
    pub_->publish(*msg);
  }

  double body_height_{0.09};
  double floor_margin_{0.04};
  double imu_max_age_{0.05};
  double imu_history_window_{0.3};
  std::string imu_topic_{"/imu"};
  std::string base_frame_{"base_link"};
  std::string lidar_frame_{"lidar_link"};
  bool mount_ready_{false};
  double mount_z_{0.0};    // lidar_link height above base_link, from TF
  double lidar_yaw_{0.0};  // lidar_link -> base_link yaw offset, from TF
  bool have_imu_{false};
  std::vector<RzSample> imu_history_;  // ascending by t; see onImu/onScan
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_;
};

}  // namespace big_bertha_sim_bringup

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(big_bertha_sim_bringup::ScanGroundFilter)
