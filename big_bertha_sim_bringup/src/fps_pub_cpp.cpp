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
// C++ throughput benchmark publisher: sensor_msgs::msg::Image, unthrottled
// (tight loop, no timer/executor overhead), to isolate the DDS transport's
// real throughput ceiling from rclpy's per-message binding overhead.
//
// Usage: fps_pub_cpp [width=640] [height=480] [qos_depth=10]
#include <chrono>
#include <cstdlib>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int width = argc > 1 ? std::atoi(argv[1]) : 640;
  int height = argc > 2 ? std::atoi(argv[2]) : 480;
  int qos_depth = argc > 3 ? std::atoi(argv[3]) : 10;

  auto node = rclcpp::Node::make_shared("fps_pub_cpp");
  rclcpp::QoS qos(qos_depth);
  qos.best_effort().durability_volatile();
  auto pub = node->create_publisher<sensor_msgs::msg::Image>("/fps_test_cpp", qos);

  sensor_msgs::msg::Image msg;
  msg.height = height;
  msg.width = width;
  msg.encoding = "rgb8";
  msg.step = width * 3;
  msg.data.resize(static_cast<size_t>(width) * height * 3, 0);

  while (rclcpp::ok()) {
    auto now = std::chrono::steady_clock::now().time_since_epoch();
    auto sec = std::chrono::duration_cast<std::chrono::seconds>(now);
    auto nsec = std::chrono::duration_cast<std::chrono::nanoseconds>(now - sec);
    msg.header.stamp.sec = static_cast<int32_t>(sec.count());
    msg.header.stamp.nanosec = static_cast<uint32_t>(nsec.count());
    pub->publish(msg);
  }

  rclcpp::shutdown();
  return 0;
}
