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
#include <vector>

namespace big_bertha_bringup
{

class ServoConverter
{
public:
  struct Params
  {
    int pwm_min{102};
    int pwm_max{512};
    double joint_limit{3.14159};
    std::vector<double> servo_lower_limit;
    std::vector<double> servo_upper_limit;
    std::vector<double> servo_offset;
    std::vector<double> policy_center;
    std::vector<int> servo_channel;
    std::vector<int> servo_direction;
    double rate_limit_rad{0.03};
    double smoothing_alpha{0.3};
    bool single_joint_mode{false};
    int single_joint_index{10};
  };

  explicit ServoConverter(Params params) : params_(std::move(params)) {}

  ServoConverter() = default;

  std::vector<int> convert(const std::vector<double> & targets)
  {
    if (targets.size() != 12) return {};

    // EWMA smoothing
    std::vector<double> smoothed(12);
    if (first_cmd_) {
      smoothed = targets;
    } else {
      for (size_t i = 0; i < 12; ++i) {
        smoothed[i] = params_.smoothing_alpha * targets[i] +
                      (1.0 - params_.smoothing_alpha) * smoothed_targets_[i];
      }
    }
    smoothed_targets_ = smoothed;

    // Rate limiting
    std::vector<double> limited(12);
    if (first_cmd_) {
      limited = smoothed;
      first_cmd_ = false;
    } else {
      for (size_t i = 0; i < 12; ++i) {
        limited[i] = std::clamp(
          smoothed[i],
          last_targets_[i] - params_.rate_limit_rad,
          last_targets_[i] + params_.rate_limit_rad);
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
      if (idx < 0 || idx >= 12) idx = 0;
      int active = pwms[idx];
      int neutral = (params_.pwm_min + params_.pwm_max) / 2;
      for (auto & p : pwms) p = neutral;
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

private:
  Params params_;
  std::vector<double> last_targets_{12, 0.0};
  std::vector<double> smoothed_targets_{12, 0.0};
  bool first_cmd_{true};
};

}  // namespace big_bertha_bringup

#endif  // BIG_BERTHA_BRINGUP__SERVO_CONVERTER_HPP_
