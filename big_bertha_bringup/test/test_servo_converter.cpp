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

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "big_bertha_bringup/servo_converter.hpp"

namespace bbb = big_bertha_bringup;

namespace
{

bbb::ServoConverter::Params make_params()
{
  bbb::ServoConverter::Params p;
  p.servo_lower_limit.assign(12, 0.0);
  p.servo_upper_limit.assign(12, 180.0);
  p.servo_offset.assign(12, 0.0);
  p.policy_center.assign(12, 0.0);
  p.servo_direction.assign(12, 1);
  p.servo_channel.resize(12);
  for (int i = 0; i < 12; ++i) {
    p.servo_channel[i] = i;
  }
  p.max_joint_rate_rad_s = 3.0;
  p.smoothing_tau_s = 0.0;  // isolate the slew limit
  return p;
}

/// Drive a step for `seconds` at `rate` Hz and report how far joint 0 travelled.
double travel_at_rate(double rate, double seconds)
{
  bbb::ServoConverter c(make_params());
  const double dt = 1.0 / rate;
  const std::vector<double> zero(12, 0.0);
  const std::vector<double> step(12, 1.0);  // 1 rad away, far past any slew
  c.convert(zero, dt);                      // first call latches the origin
  const int n = static_cast<int>(std::lround(seconds * rate));
  for (int k = 0; k < n; ++k) {
    c.convert(step, dt);
  }
  return c.last_targets()[0];
}

}  // namespace

// The bug this guards: the slew limit used to be a per-MESSAGE allowance, so
// halving the send rate halved the joint's speed silently. The throttle in
// hardware_bridge_node only bounds the rate from above, and a congested link
// or a starved callback drops it, which cost the most travel on the joints
// that need the most - the thighs.
TEST(ServoConverter, SlewIsPerSecondNotPerMessage)
{
  const double at_50 = travel_at_rate(50.0, 0.1);
  const double at_20 = travel_at_rate(20.0, 0.1);
  const double at_10 = travel_at_rate(10.0, 0.1);

  // 3.0 rad/s for 0.1 s is 0.3 rad, whatever the call rate.
  EXPECT_NEAR(at_50, 0.3, 1e-6);
  EXPECT_NEAR(at_20, 0.3, 1e-6);
  EXPECT_NEAR(at_10, 0.3, 1e-6);
}

// A stalled link can hand back a large dt; one late message must not let a
// joint jump an unbounded distance.
TEST(ServoConverter, LateMessageCannotAuthoriseAnUnboundedJump)
{
  bbb::ServoConverter c(make_params());
  const std::vector<double> zero(12, 0.0);
  const std::vector<double> step(12, 10.0);
  c.convert(zero, 0.02);
  c.convert(step, 5.0);  // five seconds of silence
  EXPECT_LE(c.last_targets()[0], bbb::ServoConverter::kMaxDt * 3.0 + 1e-9);
}

// A non-positive dt must not stall the joint or run the filter backwards.
TEST(ServoConverter, BadDtFallsBackToNominal)
{
  bbb::ServoConverter c(make_params());
  const std::vector<double> zero(12, 0.0);
  const std::vector<double> step(12, 1.0);
  c.convert(zero, 0.02);
  c.convert(step, 0.0);
  EXPECT_NEAR(c.last_targets()[0], bbb::ServoConverter::kNominalDt * 3.0, 1e-9);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
