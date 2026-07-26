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

#ifndef BIG_BERTHA_SIM_BRINGUP__GROUND_RAY_HPP_
#define BIG_BERTHA_SIM_BRINGUP__GROUND_RAY_HPP_

#include <cmath>

namespace big_bertha_sim_bringup
{

/// True if a lidar return is a floor strike to cull. Pure math, extracted so
/// the sign conventions are testable: a previous inversion (missing lidar_yaw)
/// culled the UP-tilted half and deleted real walls.
///
/// angle      ray angle in the LIDAR frame (angle_min + i*increment)
/// lidar_yaw  lidar_link -> base_link yaw offset (from TF; Rz(pi) mount)
/// rzx, rzy   third-row IMU rotation terms: 2(xz-wy), 2(yz+wx)
/// range      finite return distance (m)
/// lidar_h    lidar height above ground (body + mount)
/// margin     endpoint height below this (m) reads as ground
inline bool is_ground_ray(
  double angle, double lidar_yaw, double rzx, double rzy, double range, double lidar_h,
  double margin)
{
  const double a = angle + lidar_yaw;
  // World-z component of the ray's unit direction (cos a, sin a, 0).
  const double dir_z = rzx * std::cos(a) + rzy * std::sin(a);
  return dir_z < 0.0 && lidar_h + range * dir_z < margin;
}

}  // namespace big_bertha_sim_bringup

#endif  // BIG_BERTHA_SIM_BRINGUP__GROUND_RAY_HPP_
