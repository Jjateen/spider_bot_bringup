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
constexpr int kObsDim = 48;
constexpr int kActionDim = 12;

/// Latest cached sensor state used to assemble the 48-d observation.
///
/// Layout MUST match the trained policy contract. The deployed policy is the
/// rsl_rl run big_bertha/2026-06-11_10-00-20 (retrained with body-velocity
/// feedback for sim-to-sim transfer; reward track_lin_vel_xy_exp ~15, episode
/// length 999). Its Isaac env (big_bertha_env.py::_get_observations) builds:
///   [ 0: 3]  root_lin_vel_b      (from /odom twist, body frame)
///   [ 3: 6]  root_ang_vel_b      (from /imu angular velocity)
///   [ 6: 9]  projected_gravity_b (from /imu orientation)
///   [ 9:12]  commands vx,vy,yaw  (from /cmd_vel)
///   [12:24]  joint_pos - default (from /joint_states)
///   [24:36]  joint_vel           (from /joint_states)
///   [36:48]  prev_actions        (node's previous CLAMPED action output)
/// Confirmed against the exported onnx obs_normalizer (gravity_z ~ -1 at
/// index 8). The onnx bakes in the empirical normalizer, so feed RAW obs.
struct ObservationBuilder
{
  // Default joint pose in Isaac articulation order (all hips, knees, ankles).
  // The policy controller node overrides this from policy.yaml; this hardcoded
  // value is the fallback if the parameter is missing.
  std::array<double, kNumJoints> default_joint_pos{0.0, 0.0, 0.0, 0.0, -0.32, -0.32,
                                                    -0.32, -0.32, 1.82, 1.82, 1.82, 1.82};

  // Cached inputs (updated by subscription callbacks).
  std::array<double, 3> root_lin_vel_b{0.0, 0.0, 0.0};
  std::array<double, 3> root_ang_vel_b{0.0, 0.0, 0.0};
  std::array<double, 3> projected_gravity_b{0.0, 0.0, -1.0};
  std::array<double, 3> commands{0.0, 0.0, 0.0};
  std::array<double, kNumJoints> joint_pos{};
  std::array<double, kNumJoints> joint_vel{};
  std::array<double, kActionDim> prev_actions{};

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

  /// Assemble the 48-d observation in the trained order:
  /// [lin_vel(3), ang_vel(3), gravity(3), commands(3),
  ///  joint_pos-default(12), joint_vel(12), prev_actions(12)]
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
    return obs;
  }
};

}  // namespace big_bertha_policy_controller

#endif  // BIG_BERTHA_POLICY_CONTROLLER__OBSERVATION_BUILDER_HPP_
