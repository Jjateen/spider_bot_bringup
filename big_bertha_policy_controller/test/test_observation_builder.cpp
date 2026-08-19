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
// Guards the trained-policy contract that has broken twice in deployment:
// the 52-d observation layout and the gait-clock cadence clamp.

#include <gtest/gtest.h>

#include <cmath>

#include "big_bertha_policy_controller/observation_builder.hpp"

namespace bbpc = big_bertha_policy_controller;

TEST(ObservationBuilder, LayoutOrderAndSize)
{
  bbpc::ObservationBuilder ob;
  ob.root_lin_vel_b = {1.0, 2.0, 3.0};
  ob.root_ang_vel_b = {4.0, 5.0, 6.0};
  ob.projected_gravity_b = {7.0, 8.0, 9.0};
  ob.commands = {10.0, 11.0, 12.0};
  for (int i = 0; i < bbpc::kNumJoints; ++i) {
    ob.joint_pos[i] = ob.default_joint_pos[i] + 0.5;  // -> obs = +0.5
    ob.joint_vel[i] = 20.0 + i;
    ob.prev_actions[i] = 40.0 + i;
  }
  ob.gait_phase = 0.0;  // offsets [0, .5, .25, .75] -> sin = 0, 0, 1, -1

  const auto obs = ob.build();
  ASSERT_EQ(static_cast<int>(obs.size()), bbpc::kObsDim);
  EXPECT_FLOAT_EQ(obs[0], 1.0f);      // lin_vel x
  EXPECT_FLOAT_EQ(obs[3], 4.0f);      // ang_vel x
  EXPECT_FLOAT_EQ(obs[6], 7.0f);      // gravity x
  EXPECT_FLOAT_EQ(obs[9], 10.0f);     // command vx
  EXPECT_FLOAT_EQ(obs[12], 0.5f);     // joint_pos - default
  EXPECT_FLOAT_EQ(obs[24], 20.0f);    // joint_vel[0]
  EXPECT_FLOAT_EQ(obs[36], 40.0f);    // prev_actions[0]
  EXPECT_NEAR(obs[48], 0.0f, 1e-6);   // clock leg1: sin(0)
  EXPECT_NEAR(obs[49], 0.0f, 1e-6);   // leg2: sin(pi)
  EXPECT_NEAR(obs[50], 1.0f, 1e-6);   // leg3: sin(pi/2)
  EXPECT_NEAR(obs[51], -1.0f, 1e-6);  // leg4: sin(3pi/2)
}

TEST(ObservationBuilder, ClockBoostClampsAtMax)
{
  // vx=0.3 + |yaw|=0.4 saturate both terms: raw boost 1+0.8+1.1 = 2.9,
  // which training clamps to 2.1. Omitting the clamp was a live bug
  // (clock ran 38% fast on every Nav2 arc).
  bbpc::ObservationBuilder ob;
  const double dt = 0.02;
  ob.gait_phase = 0.0;
  ob.advance_clock(0.3, 0.4, dt);
  EXPECT_NEAR(ob.gait_phase, ob.gait_frequency * 2.1 * dt, 1e-9);

  // Below both knees: boost = 1 + 0.8*(0.2/0.4) + 1.1*(0.15/0.3) = 1.95.
  ob.gait_phase = 0.0;
  ob.advance_clock(0.15, 0.2, dt);
  EXPECT_NEAR(ob.gait_phase, ob.gait_frequency * 1.95 * dt, 1e-9);
}

TEST(ObservationBuilder, ClockBoostUsesSpeedMagnitude)
{
  // Reverse must advance the clock at the same rate as the equivalent forward
  // speed: _clock_boost takes |vx|. With a signed vx the boost went BELOW 1
  // and, past -0.3, negative, which stops the robot stepping while Nav2 is
  // trying to back it out of an obstacle.
  bbpc::ObservationBuilder fwd;
  bbpc::ObservationBuilder rev;
  const double dt = 0.02;
  fwd.gait_phase = 0.0;
  rev.gait_phase = 0.0;
  fwd.advance_clock(0.15, 0.0, dt);
  rev.advance_clock(-0.15, 0.0, dt);
  EXPECT_NEAR(fwd.gait_phase, rev.gait_phase, 1e-12);
  EXPECT_GT(rev.gait_phase, 0.0);
}

TEST(ObservationBuilder, GravityProjection)
{
  bbpc::ObservationBuilder ob;
  ob.set_gravity_from_quaternion(0.0, 0.0, 0.0, 1.0);  // identity
  EXPECT_NEAR(ob.projected_gravity_b[2], -1.0, 1e-9);
  ob.set_gravity_from_quaternion(1.0, 0.0, 0.0, 0.0);  // 180 deg roll
  EXPECT_NEAR(ob.projected_gravity_b[2], 1.0, 1e-9);
  // 90 deg pitch about +y: body x-axis points world-down, so gravity reads
  // +1 along body x and 0 along body z.
  const double s = std::sqrt(0.5);
  ob.set_gravity_from_quaternion(0.0, s, 0.0, s);
  EXPECT_NEAR(ob.projected_gravity_b[0], 1.0, 1e-9);
  EXPECT_NEAR(ob.projected_gravity_b[2], 0.0, 1e-9);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
