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

#ifndef BIG_BERTHA_POLICY_CONTROLLER__COMMAND_SHAPER_HPP_
#define BIG_BERTHA_POLICY_CONTROLLER__COMMAND_SHAPER_HPP_

#include <algorithm>
#include <cmath>

namespace big_bertha_policy_controller
{

/// Shapes the raw Nav2 /cmd_vel into the policy command: trained-envelope
/// clamps, safe-stop, position-hold, heading-hold + lateral-hold corrections,
/// and the PI hip-bias steering signal. Header-only and ROS-free so the whole
/// outer-loop math is unit-testable (see test_command_shaper.cpp).
struct CommandShaper
{
  // Envelope + loop gains; runtime values come from policy.yaml.
  double max_lin_vel_x{0.3};
  double max_lin_vel_y{0.05};
  double max_yaw_rate{0.15};
  bool heading_hold{true};
  double heading_kp{2.0};
  bool heading_lock{false};  // demo_straight: hold a fixed world heading
  double heading_lock_yaw{0.0};
  double heading_max{0.5};  // trained |yaw| <= 0.5
  double heading_err_clamp{0.4};
  double fwd_slow_err{0.5};
  double fwd_min_scale{0.15};
  double steer_kp{0.6};
  double steer_ki{0.5};
  double steer_max{0.25};
  bool station_keep{true};
  bool position_hold{true};
  double pos_hold_deadband{0.30};
  double pos_hold_speed{0.08};
  double stand_vx_thresh{0.02};
  bool lateral_hold{true};
  double lateral_kp{0.25};
  double lateral_turn_thresh{0.05};
  double lateral_yaw_kp{0.6};
  double lateral_yaw_max{0.4};

  // Loop state.
  double desired_yaw{0.0};
  bool heading_init{false};
  bool prev_idle{false};
  bool prev_stale{false};
  double hold_x{0.0};
  double hold_y{0.0};
  double ref_x{0.0};
  double ref_y{0.0};
  double steer_i{0.0};
  double steer_cmd{0.0};       // hip-bias steering output, read by the caller
  double last_steer_cmd{0.0};  // for rate limiting

  struct Cmd
  {
    double vx;
    double vy;
    double yaw;
  };

  static double wrap_pi(double a) { return std::atan2(std::sin(a), std::cos(a)); }

  /// One control-period update. stale = /cmd_vel timed out (safe stop).
  Cmd shape(
    double cmd_vx, double cmd_vy, double cmd_wz, bool stale, bool have_odom, double cur_x,
    double cur_y, double cur_yaw, double dt)
  {
    double vx = stale ? 0.0 : cmd_vx;
    double vy = stale ? 0.0 : cmd_vy;
    double wz = stale ? 0.0 : cmd_wz;
    vx = std::clamp(vx, 0.0, max_lin_vel_x);
    vy = std::clamp(vy, -max_lin_vel_y, max_lin_vel_y);
    wz = std::clamp(wz, -max_yaw_rate, max_yaw_rate);

    // Position-hold: latch the stop point on the moving->idle transition;
    // beyond the deadband, face it then walk back (the gait has no reverse).
    if (position_hold && have_odom) {
      if (stale && !prev_stale) {
        hold_x = cur_x;
        hold_y = cur_y;
      }
      if (stale) {
        const double dx = hold_x - cur_x;
        const double dy = hold_y - cur_y;
        const double dist = std::hypot(dx, dy);
        if (dist > pos_hold_deadband) {
          const double yaw_to_ref = wrap_pi(std::atan2(dy, dx) - cur_yaw);
          if (std::abs(yaw_to_ref) > 0.6) {
            vx = 0.0;  // turn in place to face the stop point first
            wz = std::clamp(2.0 * yaw_to_ref, -max_yaw_rate, max_yaw_rate);
          } else {
            vx = std::clamp(0.6 * dist, stand_vx_thresh + 0.02, pos_hold_speed);
            wz = std::clamp(1.5 * yaw_to_ref, -max_yaw_rate, max_yaw_rate);
          }
        }
      }
    }
    prev_stale = stale;

    if (!heading_hold || !have_odom) {
      steer_cmd = 0.0;  // calibration mode: only /debug_hip_bias steers
      return {vx, vy, wz};
    }

    // Latch the heading setpoint + lateral line origin at startup and on the
    // moving->idle transition. With station_keep the latched heading is HELD
    // through idle so the steering corrects contact creep, not tracks it.
    const bool idle = std::abs(vx) < 0.01 && std::abs(wz) < 0.01;
    const bool just_stopped = idle && !prev_idle;
    if (!heading_init || just_stopped || (idle && !station_keep)) {
      desired_yaw = heading_lock ? heading_lock_yaw : cur_yaw;
      steer_i = 0.0;  // never hold a stale heading in the integrator
      ref_x = cur_x;
      ref_y = cur_y;
      heading_init = true;
    }
    prev_idle = idle;

    // Heading setpoint: integrate the commanded yaw rate, anti-windup clamped
    // to heading_err_clamp of the measured heading; heading_lock holds a
    // fixed world heading instead.
    double err;
    if (heading_lock) {
      desired_yaw = heading_lock_yaw;
      err = std::clamp(wrap_pi(desired_yaw - cur_yaw), -heading_err_clamp, heading_err_clamp);
    } else {
      desired_yaw = wrap_pi(desired_yaw + wz * dt);
      err = wrap_pi(desired_yaw - cur_yaw);
      if (std::abs(err) > heading_err_clamp) {
        err = std::clamp(err, -heading_err_clamp, heading_err_clamp);
        desired_yaw = wrap_pi(cur_yaw + err);
      }
    }

    // Lateral hold: perpendicular offset from the latched line -> vy toward
    // the line, plus a cross-track bias folded into the heading error so the
    // strong hip-bias steering walks the robot back onto it.
    double vy_out = vy;
    if (lateral_hold) {
      if (std::abs(wz) > lateral_turn_thresh) {
        ref_x = cur_x;
        ref_y = cur_y;
      } else {
        const double dx = cur_x - ref_x;
        const double dy = cur_y - ref_y;
        const double lat_err = -dx * std::sin(desired_yaw) + dy * std::cos(desired_yaw);
        vy_out = std::clamp(-lateral_kp * lat_err, -max_lin_vel_y, max_lin_vel_y);
        const double cross_bias =
          std::clamp(-lateral_yaw_kp * lat_err, -lateral_yaw_max, lateral_yaw_max);
        err = std::clamp(
          err + cross_bias, -heading_err_clamp - lateral_yaw_max,
          heading_err_clamp + lateral_yaw_max);
      }
    }

    const double yaw_cmd = std::clamp(wz + heading_kp * err, -heading_max, heading_max);
    // PI hip-bias steering does the real turning work; I removes the
    // steady-state offset against the constant drift.
    if (steer_ki > 1e-6) {
      steer_i += err * dt;
      const double i_lim = steer_max / steer_ki;  // anti-windup
      steer_i = std::clamp(steer_i, -i_lim, i_lim);
    }

    // Compute unlimited steering command
    double steer_unlimited = steer_kp * err + steer_ki * steer_i;

    // Apply rate limit: 0.01 rad/tick at 50 Hz = 0.5 rad/s slew rate
    // Prevents sudden hip jerks on large heading errors
    double max_delta = 0.01;  // From policy.yaml steer_rate_limit
    double steer_delta = steer_unlimited - last_steer_cmd;
    steer_delta = std::clamp(steer_delta, -max_delta, max_delta);
    steer_cmd = last_steer_cmd + steer_delta;

    // Clamp to absolute max
    steer_cmd = std::clamp(steer_cmd, -steer_max, steer_max);
    last_steer_cmd = steer_cmd;

    // Slow forward while a large heading error is being corrected.
    double fwd_scale = 1.0;
    if (fwd_slow_err > 1e-3) {
      fwd_scale = std::clamp(1.0 - std::abs(err) / fwd_slow_err, fwd_min_scale, 1.0);
    }
    return {vx * fwd_scale, vy_out, yaw_cmd};
  }
};

}  // namespace big_bertha_policy_controller

#endif  // BIG_BERTHA_POLICY_CONTROLLER__COMMAND_SHAPER_HPP_
