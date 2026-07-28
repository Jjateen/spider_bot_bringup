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
// Periodic identity map->odom transform publisher (ground-truth mode).
//
// The stock tf2_ros static_transform_publisher publishes the transform exactly
// once at construction time, using the wall-clock timestamp.  When use_sim_time
// is active and the simulation clock starts later, the tf2_ros::Buffer detects
// a clock source change and clears its transform cache, permanently losing the
// one-shot static transform.  Nav2's global_costmap (which needs the map frame)
// then fails to initialise, which stalls the lifecycle manager and leaves
// bt_navigator inactive.
//
// This node side-steps the problem by re-publishing the identity map->odom
// transform on a 10-second timer, so it survives buffer clears and timestamp
// domain transitions.  It declares use_sim_time properly so the stamp honours
// the sim clock once it starts.

#include <memory>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/static_transform_broadcaster.h"

namespace big_bertha_sim_bringup
{

class MapToOdomPublisher : public rclcpp::Node
{
public:
  MapToOdomPublisher() : rclcpp::Node("map_to_odom_ground_truth")
  {
    broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    timer_ = this->create_wall_timer(
      std::chrono::seconds(10),
      std::bind(&MapToOdomPublisher::publish, this));
    publish();
  }

private:
  void publish()
  {
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = now();
    t.header.frame_id = "map";
    t.child_frame_id = "odom";
    t.transform.rotation.w = 1.0;
    broadcaster_->sendTransform(t);
  }

  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace big_bertha_sim_bringup

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(
    std::make_shared<big_bertha_sim_bringup::MapToOdomPublisher>());
  rclcpp::shutdown();
  return 0;
}
