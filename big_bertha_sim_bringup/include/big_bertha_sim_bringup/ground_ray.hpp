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
#include <cstddef>
#include <vector>

namespace big_bertha_sim_bringup
{

/// One IMU sample's third-row rotation terms (see is_ground_ray), timestamped
/// in the same clock as LaserScan.header.stamp.
struct RzSample
{
  double t;
  double rzx;
  double rzy;
};

/// Interpolate rzx/rzy at time t from samples (must be ascending by t; empty
/// is valid). Clamps to the nearest endpoint when t falls outside the
/// buffered range instead of extrapolating -- that degrades to "use the
/// closest sample we actually have", the same fail-safe spirit as the rest of
/// this filter, rather than guessing past data we don't have. Linear
/// interpolation (not quaternion SLERP) is deliberate: rzx/rzy are already
/// just the two third-row rotation terms is_ground_ray needs, changing
/// slowly (a walking gait's tilt swings a few degrees over ~100 ms), so
/// interpolating them directly is accurate enough and avoids reconstructing
/// a full orientation only to re-derive the same two numbers.
inline void interpolate_rz(
  const std::vector<RzSample> & samples, double t, double & rzx, double & rzy)
{
  if (samples.empty()) {
    rzx = 0.0;
    rzy = 0.0;
    return;
  }
  if (t <= samples.front().t) {
    rzx = samples.front().rzx;
    rzy = samples.front().rzy;
    return;
  }
  if (t >= samples.back().t) {
    rzx = samples.back().rzx;
    rzy = samples.back().rzy;
    return;
  }
  for (std::size_t i = 1; i < samples.size(); ++i) {
    if (t <= samples[i].t) {
      const auto & a = samples[i - 1];
      const auto & b = samples[i];
      const double span = b.t - a.t;
      const double f = span > 0.0 ? (t - a.t) / span : 0.0;
      rzx = a.rzx + f * (b.rzx - a.rzx);
      rzy = a.rzy + f * (b.rzy - a.rzy);
      return;
    }
  }
  // Unreachable: t < samples.back().t was already checked above, so the loop
  // always returns via the branch above before falling off the end.
  rzx = samples.back().rzx;
  rzy = samples.back().rzy;
}

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
