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
// Guards the outer-loop command shaping: envelope clamps, safe-stop,
// wrap_pi, heading anti-windup, steering integral clamp, position-hold.

#include <gtest/gtest.h>

#include <cmath>

#include "big_bertha_policy_controller/command_shaper.hpp"

namespace bbpc = big_bertha_policy_controller;

bbpc::CommandShaper make_shaper()
{
  bbpc::CommandShaper s;
  s.max_lin_vel_x = 0.3;
  s.max_lin_vel_y = 0.06;
  s.max_yaw_rate = 0.5;
  return s;
}

TEST(CommandShaper, EnvelopeClampsAndReverseSurvives)
{
  auto s = make_shaper();
  s.heading_hold = false;  // isolate the clamps
  const auto c = s.shape(1.0, -1.0, 2.0, false, false, 0, 0, 0, 0.02);
  EXPECT_DOUBLE_EQ(c.vx, 0.3);
  EXPECT_DOUBLE_EQ(c.vy, -0.06);
  EXPECT_DOUBLE_EQ(c.yaw, 0.5);
  // v2.0.0 trains reverse, so a BackUp command must reach the policy instead
  // of being clamped to zero. Nav2 backs up at -0.15, the trained magnitude.
  EXPECT_DOUBLE_EQ(s.shape(-0.15, 0, 0, false, false, 0, 0, 0, 0.02).vx, -0.15);
  // Beyond the trained reverse envelope it saturates, it does not pass through.
  EXPECT_DOUBLE_EQ(s.shape(-0.9, 0, 0, false, false, 0, 0, 0, 0.02).vx, -0.15);
}

TEST(CommandShaper, StaleCommandSafeStops)
{
  auto s = make_shaper();
  s.heading_hold = false;
  s.position_hold = false;
  const auto c = s.shape(0.3, 0.05, 0.4, true, false, 0, 0, 0, 0.02);
  EXPECT_DOUBLE_EQ(c.vx, 0.0);
  EXPECT_DOUBLE_EQ(c.vy, 0.0);
  EXPECT_DOUBLE_EQ(c.yaw, 0.0);
  EXPECT_DOUBLE_EQ(s.steer_cmd, 0.0);
}

TEST(CommandShaper, WrapPiDiscontinuity)
{
  EXPECT_NEAR(bbpc::CommandShaper::wrap_pi(M_PI + 0.1), -M_PI + 0.1, 1e-9);
  EXPECT_NEAR(bbpc::CommandShaper::wrap_pi(-M_PI - 0.1), M_PI - 0.1, 1e-9);
  EXPECT_NEAR(bbpc::CommandShaper::wrap_pi(3.0 * M_PI), M_PI, 1e-9);
}

TEST(CommandShaper, HeadingAntiWindup)
{
  // Command a turn the plant never executes: the setpoint must stay within
  // heading_err_clamp of the measured heading instead of running away.
  auto s = make_shaper();
  for (int i = 0; i < 500; ++i) {
    s.shape(0.1, 0.0, 0.5, false, true, 0.0, 0.0, 0.0, 0.02);
  }
  EXPECT_LE(std::abs(s.desired_yaw), s.heading_err_clamp + 1e-9);
}

TEST(CommandShaper, SteerIntegralClamped)
{
  // Hold a large heading error: |steer_cmd| saturates at steer_max and the
  // integral stops at its anti-windup limit.
  auto s = make_shaper();
  s.lateral_hold = false;
  for (int i = 0; i < 1000; ++i) {
    s.shape(0.1, 0.0, 0.0, false, true, 0.0, 0.0, 0.3, 0.02);
  }
  EXPECT_LE(std::abs(s.steer_cmd), s.steer_max + 1e-9);
  EXPECT_LE(std::abs(s.steer_i), s.steer_max / s.steer_ki + 1e-9);
}

TEST(CommandShaper, PositionHoldTurnsBeforeWalking)
{
  // Idle 1 m from the latched stop point, facing away: vx must be zero
  // (face the point first -- the gait has no reverse).
  auto s = make_shaper();
  s.heading_hold = false;
  s.shape(0.1, 0.0, 0.0, false, true, 0.0, 0.0, 0.0, 0.02);  // moving: latches prev_stale
  s.shape(0.0, 0.0, 0.0, true, true, 0.0, 0.0, 0.0, 0.02);   // stop here: hold point (0,0)
  const auto c = s.shape(0.0, 0.0, 0.0, true, true, 1.0, 0.0, 0.0, 0.02);  // drifted +1 m
  EXPECT_DOUBLE_EQ(c.vx, 0.0);
  EXPECT_GT(std::abs(c.yaw), 0.0);  // turning toward the hold point
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
