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

#ifndef BIG_BERTHA_BRINGUP__SERVO_CONVERTER_HPP_
#define BIG_BERTHA_BRINGUP__SERVO_CONVERTER_HPP_

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <utility>
#include <vector>

namespace big_bertha_bringup
{

class ServoConverter
{
public:
  struct Params
  {
    int pwm_min{102};  // Wide-range clone: 0.5ms
    int pwm_max{512};  // Wide-range clone: 2.5ms
    double joint_limit{3.14159};
    std::vector<double> servo_lower_limit;
    std::vector<double> servo_upper_limit;
    std::vector<double> servo_offset;
    std::vector<double> policy_center;
    std::vector<int> servo_channel;
    std::vector<int> servo_direction;
    // Both shaping stages are per SECOND, not per message. They used to be a
    // per-message allowance sized as max_joint_rate_rad_s / command_rate_hz,
    // which is only correct while the real send rate equals command_rate_hz.
    // Nothing enforced that: the throttle in hardware_bridge_node only sets an
    // upper bound, so a congested link or a starved callback silently capped
    // every joint's speed by the ratio between the two rates, and the joints
    // with the most travel (the thighs) lost the most. Driving both from the
    // measured dt makes the shaping independent of the send rate.
    double max_joint_rate_rad_s{3.0};
    double smoothing_tau_s{0.0896};
    bool single_joint_mode{false};
    int single_joint_index{10};
  };

  /// Fallback dt for the first call and for a bad stamp: the design point.
  static constexpr double kNominalDt = 0.02;
  /// One late message must not authorise an unbounded jump.
  static constexpr double kMaxDt = 0.10;

  explicit ServoConverter(Params params) : params_(std::move(params)) {}

  ServoConverter() = default;

  /// dt is the time since the previous call, in seconds. Both the smoothing
  /// and the slew limit scale with it, so the delivered waveform is the same
  /// whether this runs at 50 Hz or at 20 Hz.
  std::vector<int> convert(const std::vector<double> & targets, double dt)
  {
    if (targets.size() != 12) {
      return {};
    }

    // A stalled link can hand back a huge dt. Cap it so one late message
    // cannot authorise an unbounded jump, and reject non-positive stamps.
    if (!(dt > 0.0)) {
      dt = kNominalDt;
    }
    dt = std::min(dt, kMaxDt);

    // EWMA smoothing, time-based: alpha = 1 - exp(-dt/tau) is the same filter
    // at any call rate, where a fixed alpha is not.
    const double alpha = (params_.smoothing_tau_s > 1e-9)
      ? 1.0 - std::exp(-dt / params_.smoothing_tau_s)
      : 1.0;
    std::vector<double> smoothed(12);
    if (first_cmd_) {
      smoothed = targets;
    } else {
      for (size_t i = 0; i < 12; ++i) {
        smoothed[i] = alpha * targets[i] + (1.0 - alpha) * smoothed_targets_[i];
      }
    }
    smoothed_targets_ = smoothed;

    // Slew limit, time-based: rad/s * dt, so a slower send rate buys a
    // proportionally larger step instead of throttling the joint.
    const double step = params_.max_joint_rate_rad_s * dt;
    std::vector<double> limited(12);
    if (first_cmd_) {
      limited = smoothed;
      first_cmd_ = false;
    } else {
      for (size_t i = 0; i < 12; ++i) {
        limited[i] = std::clamp(smoothed[i], last_targets_[i] - step, last_targets_[i] + step);
      }
    }
    last_targets_ = limited;

    // Radians -> PWM
    std::vector<int> pwms(12);
    for (size_t i = 0; i < 12; ++i) {
      double rad = std::clamp(limited[i], -params_.joint_limit, params_.joint_limit);
      double deg = rad * 180.0 / M_PI;
      double center_deg = params_.policy_center[i] * 180.0 / M_PI;
      deg = (deg - center_deg) * params_.servo_direction[i];
      deg = deg + params_.servo_offset[i];
      deg = deg + 90.0;
      double lo = std::min(params_.servo_lower_limit[i], params_.servo_upper_limit[i]);
      double hi = std::max(params_.servo_lower_limit[i], params_.servo_upper_limit[i]);
      deg = std::clamp(deg, lo, hi);
      double t = deg / 180.0;
      double pwm = std::round(t * (params_.pwm_max - params_.pwm_min) + params_.pwm_min);
      pwms[i] = static_cast<int>(std::clamp(pwm, 0.0, 4095.0));
    }

    // Test mode
    if (params_.single_joint_mode) {
      int idx = params_.single_joint_index;
      if (idx < 0 || idx >= 12) {
        idx = 0;
      }
      int active = pwms[idx];
      int neutral = (params_.pwm_min + params_.pwm_max) / 2;
      for (auto & p : pwms) {
        p = neutral;
      }
      pwms[idx] = active;
    }

    // Sort by channel
    std::vector<std::pair<int, int>> ch_pwm(12);
    for (size_t i = 0; i < 12; ++i) {
      ch_pwm[i] = {params_.servo_channel[i], pwms[i]};
    }
    std::sort(ch_pwm.begin(), ch_pwm.end());

    std::vector<int> sorted(12);
    for (size_t i = 0; i < 12; ++i) {
      sorted[i] = ch_pwm[i].second;
    }
    return sorted;
  }

  void reset()
  {
    last_targets_.assign(12, 0.0);
    smoothed_targets_.assign(12, 0.0);
    first_cmd_ = true;
  }

  const std::vector<double> & last_targets() const { return last_targets_; }

private:
  Params params_;
  // Parens, not braces: vector<double>{12, 0.0} selects the initializer_list
  // constructor and yields the 2-element vector {12.0, 0.0}, not 12 zeros.
  // first_cmd_ happens to overwrite both on the first convert() call, so the
  // mistake is currently invisible, but it would index out of bounds the
  // moment either buffer is read before that.
  std::vector<double> last_targets_ = std::vector<double>(12, 0.0);
  std::vector<double> smoothed_targets_ = std::vector<double>(12, 0.0);
  bool first_cmd_{true};
};

}  // namespace big_bertha_bringup

#endif  // BIG_BERTHA_BRINGUP__SERVO_CONVERTER_HPP_
