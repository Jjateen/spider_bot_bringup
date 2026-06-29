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
// C++ throughput benchmark subscriber: counts Image messages received over
// a fixed wall-clock duration, reports achieved rate, throughput (MB/s),
// and per-message latency -- the C++ counterpart to fps_sub.py, free of
// rclpy's per-message binding/serialization overhead.
//
// Usage: fps_sub_cpp [duration_s=5.0] [label] [qos_depth=10]
#include <algorithm>
#include <chrono>
#include <cinttypes>
#include <cstdlib>
#include <cstdio>
#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  double duration_s = argc > 1 ? std::atof(argv[1]) : 5.0;
  std::string label = argc > 2 ? argv[2] : "run";
  int qos_depth = argc > 3 ? std::atoi(argv[3]) : 10;

  auto node = rclcpp::Node::make_shared("fps_sub_cpp");
  rclcpp::QoS qos(qos_depth);
  qos.best_effort().durability_volatile();

  uint64_t count = 0;
  uint64_t bytes_total = 0;
  std::vector<double> latencies_us;
  latencies_us.reserve(2000000);
  std::chrono::steady_clock::time_point start_wall;
  bool started = false;

  auto sub = node->create_subscription<sensor_msgs::msg::Image>(
    "/fps_test_cpp", qos,
    [&](const sensor_msgs::msg::Image::SharedPtr msg) {
      auto now = std::chrono::steady_clock::now();
      if (!started) {
        start_wall = now;
        started = true;
      }
      auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        now.time_since_epoch()).count();
      int64_t stamp_ns = static_cast<int64_t>(msg->header.stamp.sec) * 1000000000LL +
        static_cast<int64_t>(msg->header.stamp.nanosec);
      double lat_us = static_cast<double>(now_ns - stamp_ns) / 1000.0;
      if (lat_us >= 0 && lat_us < 1000000.0) {
        latencies_us.push_back(lat_us);
      }
      count++;
      bytes_total += msg->data.size();
    });

  while (rclcpp::ok()) {
    rclcpp::spin_some(node);
    if (started) {
      auto elapsed = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - start_wall).count();
      if (elapsed >= duration_s) {
        break;
      }
    }
  }

  double elapsed = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - start_wall).count();
  double fps = count / elapsed;
  double mb_s = (bytes_total / 1000000.0) / elapsed;

  std::sort(latencies_us.begin(), latencies_us.end());
  double mean = 0.0;
  for (double v : latencies_us) {mean += v;}
  mean /= latencies_us.empty() ? 1 : latencies_us.size();
  double p99 = latencies_us.empty() ? 0.0 :
    latencies_us[static_cast<size_t>(latencies_us.size() * 0.99)];
  double max_v = latencies_us.empty() ? 0.0 : latencies_us.back();

  std::printf("=== %s (n=%" PRIu64 ", elapsed=%.3fs) ===\n", label.c_str(),
    count, elapsed);
  std::printf(
    "achieved_fps=%.1f throughput=%.1fMB/s latency_mean=%.1fus "
    "latency_p99=%.1fus latency_max=%.1fus\n",
    fps, mb_s, mean, p99, max_v);

  rclcpp::shutdown();
  return 0;
}
