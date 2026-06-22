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

#include <cmath>
#include <functional>
#include <limits>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace big_bertha_sim_bringup
{

class ScanGroundFilter : public rclcpp::Node
{
public:
  ScanGroundFilter()
  : rclcpp::Node("scan_ground_filter")
  {
    // Lidar height above ground in nominal stance (body ~0.12 m + the mount).
    // The gait's heave varies it by ~+/-0.03 m, absorbed by floor_margin.
    lidar_height_ = this->declare_parameter<double>("lidar_height", 0.22);
    // Cull a return when its endpoint height is below this (m); slightly
    // positive so a near-floor grazing hit reads as ground, not wall.
    floor_margin_ = this->declare_parameter<double>("floor_margin", 0.04);

    // Best-effort subs receive from the bridge's sensor publishers regardless
    // of their reliability. The filtered publisher is RELIABLE so it satisfies
    // the costmap's subscription either way (a best-effort publisher would
    // never reach a reliable subscriber).
    const auto sensor_qos = rclcpp::SensorDataQoS();
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "imu", sensor_qos,
      std::bind(&ScanGroundFilter::onImu, this, std::placeholders::_1));
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", sensor_qos,
      std::bind(&ScanGroundFilter::onScan, this, std::placeholders::_1));
    pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan_filtered", 10);

    RCLCPP_INFO(
      this->get_logger(),
      "scan_ground_filter up (lidar_height=%.3f m, floor_margin=%.3f m)",
      lidar_height_, floor_margin_);
  }

private:
  void onImu(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    // Third row of the rotation matrix maps a body-frame ray to its world-z
    // component. Yaw does not enter these terms, so only roll/pitch tilt counts.
    const auto & q = msg->orientation;
    rzx_ = 2.0 * (q.x * q.z - q.w * q.y);
    rzy_ = 2.0 * (q.y * q.z + q.w * q.x);
  }

  void onScan(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    const double rmax = msg->range_max;
    for (size_t i = 0; i < msg->ranges.size(); ++i) {
      const float r = msg->ranges[i];
      if (!std::isfinite(r) || r >= rmax) {
        continue;
      }
      const double a = msg->angle_min + static_cast<double>(i) * msg->angle_increment;
      // World-z component of this ray's unit direction (cos a, sin a, 0).
      const double dir_z = rzx_ * std::cos(a) + rzy_ * std::sin(a);
      if (dir_z < 0.0) {  // ray points downward -> may strike the floor
        const double endpoint_h = lidar_height_ + r * dir_z;
        if (endpoint_h < floor_margin_) {
          msg->ranges[i] = std::numeric_limits<float>::infinity();  // ground/ghost
        }
      }
    }
    pub_->publish(*msg);
  }

  double lidar_height_{0.22};
  double floor_margin_{0.04};
  double rzx_{0.0};  // 2*(x*z - w*y); identity until the first IMU msg
  double rzy_{0.0};  // 2*(y*z + w*x)
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
