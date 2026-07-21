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

#ifndef BIG_BERTHA_POLICY_CONTROLLER__OBSERVATION_BUILDER_HPP_
#define BIG_BERTHA_POLICY_CONTROLLER__OBSERVATION_BUILDER_HPP_

#include <array>
#include <cmath>
#include <string>
#include <vector>

namespace big_bertha_policy_controller
{

constexpr int kNumJoints = 12;
constexpr int kNumFeet = 4;
constexpr int kObsDim = 52;
constexpr int kActionDim = 12;

/// Latest cached sensor state used to assemble the 52-d observation.
///
/// Layout MUST match the trained policy contract (big_bertha_v1.0.0, PR #67).
/// Its Isaac env (big_bertha_env.py::_get_observations) builds:
///   [ 0: 3]  root_lin_vel_b      (from /odom twist, body frame)
///   [ 3: 6]  root_ang_vel_b      (from /imu angular velocity)
///   [ 6: 9]  projected_gravity_b (from /imu orientation)
///   [ 9:12]  commands vx,vy,yaw  (from /cmd_vel)
///   [12:24]  joint_pos - default (from /joint_states)
///   [24:36]  joint_vel           (from /joint_states)
///   [36:48]  prev_actions        (node's previous CLAMPED action output)
///   [48:52]  gait clock sin, per foot (legs 1/2/3/4 = arm_c_1_1..arm_c_4_1)
/// Confirmed against the exported onnx obs_normalizer (gravity_z ~ -1 at
/// index 8). The onnx bakes in the empirical normalizer, so feed RAW obs.
struct ObservationBuilder
{
  // Default per-joint-type pose (hip, knee/thigh, ankle/calf) x4 legs, ordered
  // [110,113,116,119, 111,114,117,120, 112,115,118,121] (Isaac tree-depth
  // order, matches policy.yaml's joint_names). Overwritten at node startup
  // from the default_joint_pos ROS param; this is just the compile-time
  // fallback -- see policy_controller_node.cpp.
  std::array<double, kNumJoints> default_joint_pos{0.0, 0.0, 0.0, 0.0, -0.32, -0.32,
                                                   -0.32, -0.32, 2.00, 2.00, 2.00, 2.00};

  // Cached inputs (updated by subscription callbacks).
  std::array<double, 3> root_lin_vel_b{0.0, 0.0, 0.0};
  std::array<double, 3> root_ang_vel_b{0.0, 0.0, 0.0};
  std::array<double, 3> projected_gravity_b{0.0, 0.0, -1.0};
  std::array<double, 3> commands{0.0, 0.0, 0.0};
  std::array<double, kNumJoints> joint_pos{};
  std::array<double, kNumJoints> joint_vel{};
  std::array<double, kActionDim> prev_actions{};

  // Wave-gait clock: reproduces big_bertha_env.py's _pre_physics_step phase
  // advance so obs[48:52] matches what the policy trained on. Tunables are
  // ROS params (policy.yaml); defaults here are the trained values.
  double gait_phase{0.0};
  double gait_frequency{0.667};
  double turn_clock_boost{0.8};
  double speed_clock_boost{1.1};
  std::array<double, kNumFeet> gait_offsets{0.0, 0.5, 0.25, 0.75};

  /// Advance the gait clock by one control step. Must be called once per
  /// control period (dt = 1/control_rate) with the CURRENT commanded vx/yaw,
  /// mirroring big_bertha_env.py: boost = 1 + turn_clock_boost*clamp(|yaw|/0.4,
  /// max=1) + speed_clock_boost*clamp(vx/0.3, max=1);
  /// phase = (phase + gait_frequency*boost*dt) % 1.
  void advance_clock(double vx_cmd, double yaw_cmd, double dt)
  {
    const double turn_term = turn_clock_boost * std::min(std::abs(yaw_cmd) / 0.4, 1.0);
    const double speed_term = speed_clock_boost * std::min(vx_cmd / 0.3, 1.0);
    const double boost = 1.0 + turn_term + speed_term;
    gait_phase = std::fmod(gait_phase + gait_frequency * boost * dt, 1.0);
    if (gait_phase < 0.0) {
      gait_phase += 1.0;
    }
  }

  /// Rotate the world-frame gravity vector (0,0,-1) into the body frame using
  /// the IMU orientation quaternion (x, y, z, w). Result is the gravity
  /// direction the policy was trained on (projected_gravity_b).
  void set_gravity_from_quaternion(double x, double y, double z, double w)
  {
    // g_world = (0, 0, -1). Apply R(q)^T (inverse rotation) to express it in
    // the body frame: g_body = R^T * g_world.
    const double gx = 0.0, gy = 0.0, gz = -1.0;
    // R^T columns from quaternion; project (0,0,-1).
    projected_gravity_b[0] =
      gx * (1 - 2 * (y * y + z * z)) + gy * (2 * (x * y + z * w)) + gz * (2 * (x * z - y * w));
    projected_gravity_b[1] =
      gx * (2 * (x * y - z * w)) + gy * (1 - 2 * (x * x + z * z)) + gz * (2 * (y * z + x * w));
    projected_gravity_b[2] =
      gx * (2 * (x * z + y * w)) + gy * (2 * (y * z - x * w)) + gz * (1 - 2 * (x * x + y * y));
  }

  /// Assemble the 52-d observation in the trained order:
  /// [lin_vel(3), ang_vel(3), gravity(3), commands(3),
  ///  joint_pos-default(12), joint_vel(12), prev_actions(12), gait_clock(4)]
  std::vector<float> build() const
  {
    std::vector<float> obs;
    obs.reserve(kObsDim);
    for (double v : root_lin_vel_b) {
      obs.push_back(static_cast<float>(v));
    }
    for (double v : root_ang_vel_b) {
      obs.push_back(static_cast<float>(v));
    }
    for (double v : projected_gravity_b) {
      obs.push_back(static_cast<float>(v));
    }
    for (double v : commands) {
      obs.push_back(static_cast<float>(v));
    }
    for (int i = 0; i < kNumJoints; ++i) {
      obs.push_back(static_cast<float>(joint_pos[i] - default_joint_pos[i]));
    }
    for (int i = 0; i < kNumJoints; ++i) {
      obs.push_back(static_cast<float>(joint_vel[i]));
    }
    for (double v : prev_actions) {
      obs.push_back(static_cast<float>(v));
    }
    for (double off : gait_offsets) {
      const double phase = std::fmod(gait_phase + off, 1.0);
      obs.push_back(static_cast<float>(std::sin(2.0 * M_PI * phase)));
    }
    return obs;
  }
};

}  // namespace big_bertha_policy_controller

#endif  // BIG_BERTHA_POLICY_CONTROLLER__OBSERVATION_BUILDER_HPP_
