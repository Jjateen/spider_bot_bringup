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
// Guards the ground-cull sign conventions: the historical bug (missing
// lidar_yaw) culled the up-tilted half of the scan and deleted real walls.

#include <gtest/gtest.h>

#include <cmath>

#include "big_bertha_sim_bringup/ground_ray.hpp"

namespace bbs = big_bertha_sim_bringup;

constexpr double kYawPi = M_PI;  // the real Rz(pi) mount offset
constexpr double kH = 0.216;     // lidar height (post mount fix)
constexpr double kMargin = 0.04;

// 5 deg nose-down pitch: quaternion (0, sin, 0, cos) -> rzx = -2*w*y.
inline double rzx_pitch(double pitch) { return -std::sin(pitch); }

TEST(GroundRay, LevelScanCullsNothing)
{
  for (int i = 0; i < 360; ++i) {
    const double a = -M_PI + i * (2.0 * M_PI / 360.0);
    EXPECT_FALSE(bbs::is_ground_ray(a, kYawPi, 0.0, 0.0, 6.0, kH, kMargin));
  }
}

TEST(GroundRay, NoseDownCullsForwardKeepsRear)
{
  // 5 deg nose-down: the forward (base +x) ray dives, floor hit at
  // r = h / sin(5 deg) ~ 2.5 m; the rear ray tilts up.
  const double rzx = rzx_pitch(0.087);
  // Lidar-frame angle pi == base-frame 0 (forward) under the Rz(pi) mount.
  EXPECT_TRUE(bbs::is_ground_ray(M_PI, kYawPi, rzx, 0.0, 3.0, kH, kMargin));
  EXPECT_FALSE(bbs::is_ground_ray(0.0, kYawPi, rzx, 0.0, 3.0, kH, kMargin));
}

TEST(GroundRay, MissingMountYawInvertsTheCull)
{
  // The regression that shipped: lidar_yaw omitted (0 instead of pi) flips
  // which half of the scan is culled.
  const double rzx = rzx_pitch(0.087);
  EXPECT_FALSE(bbs::is_ground_ray(M_PI, 0.0, rzx, 0.0, 3.0, kH, kMargin));
  EXPECT_TRUE(bbs::is_ground_ray(0.0, 0.0, rzx, 0.0, 3.0, kH, kMargin));
}

TEST(GroundRay, NearWallSurvivesWalkingTilt)
{
  // A wall return at 1 m under the mean 3.3 deg walking tilt stays well
  // above the floor margin -> never culled.
  const double rzx = rzx_pitch(0.058);
  EXPECT_FALSE(bbs::is_ground_ray(M_PI, kYawPi, rzx, 0.0, 1.0, kH, kMargin));
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
