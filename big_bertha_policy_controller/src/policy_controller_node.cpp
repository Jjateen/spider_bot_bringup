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
// big_bertha_policy_controller - C++ ONNX Runtime gait controller.
//
// Subscribes to /odom, /imu, /joint_states, /cmd_vel; assembles the 52-d
// observation (big_bertha_v1.0.0, PR #67); runs the exported PPO policy via ONNX
// Runtime; and publishes 12 position targets on
// /position_controller/commands as std_msgs/Float64MultiArray
//   joint_target = action_scale * action + default_joint_pos.
// Also publishes spider_msgs/PolicyStatus and offers the SetPolicyEnabled +
// LoadPolicy services to arm/disarm and hot-swap the model.

#include <onnxruntime_cxx_api.h>

#include <algorithm>
#include <chrono>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "big_bertha_policy_controller/observation_builder.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "spider_msgs/msg/policy_status.hpp"
#include "spider_msgs/srv/load_policy.hpp"
#include "spider_msgs/srv/set_policy_enabled.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

using namespace std::chrono_literals;
namespace bbpc = big_bertha_policy_controller;

class PolicyControllerNode : public rclcpp::Node
{
public:
  PolicyControllerNode() : Node("policy_controller")
  {
    // ----------------------------- Parameters ----------------------------
    model_path_ = declare_parameter<std::string>("model_path", "");
    action_scale_ = declare_parameter<double>("action_scale", 0.25);
    control_rate_ = declare_parameter<double>("control_rate", 50.0);
    enabled_ = declare_parameter<bool>("start_enabled", true);
    cmd_timeout_ = declare_parameter<double>("cmd_vel_timeout", 0.5);
    joint_limit_ = declare_parameter<double>("joint_limit", 3.14159);
    action_clip_ = declare_parameter<double>("action_clip", 1.0);
    // /cmd_vel clamp to the policy's trained command envelope.
    max_lin_vel_x_ = declare_parameter<double>("max_lin_vel_x", 0.3);
    max_lin_vel_y_ = declare_parameter<double>("max_lin_vel_y", 0.05);
    max_yaw_rate_ = declare_parameter<double>("max_yaw_rate", 0.15);
    // Heading-hold outer loop: rejects the systematic DART contact yaw drift
    // without retraining. yaw = wz + Kp*(desired_yaw - odom_yaw), clamped.
    heading_hold_ = declare_parameter<bool>("heading_hold", true);
    heading_kp_ = declare_parameter<double>("heading_kp", 2.0);
    // heading_lock: hold a FIXED world heading (demo_straight); false for nav.
    heading_lock_ = declare_parameter<bool>("heading_lock", false);
    heading_lock_yaw_ = declare_parameter<double>("heading_lock_yaw", 0.0);
    // Trained yaw envelope is [-0.5, 0.5] (big_bertha_env.py::_reset_idx).
    heading_max_ = declare_parameter<double>("heading_max", 0.5);
    // Anti-windup: max setpoint lead/lag vs the measured heading.
    heading_err_clamp_ = declare_parameter<double>("heading_err_clamp", 0.4);
    // Forward scaled to fwd_min_scale at |err|=fwd_slow_err, 1.0 at zero error.
    fwd_slow_err_ = declare_parameter<double>("fwd_slow_err", 0.5);
    fwd_min_scale_ = declare_parameter<double>("fwd_min_scale", 0.15);
    // Differential-stride steering: the policy yaw command barely turns the
    // gait in DART, so the heading loop also injects steer_cmd*hip_steer_sign
    // into the hip targets (rotates the stance feet -> yaws the body).
    steer_kp_ = declare_parameter<double>("steer_kp", 0.6);
    steer_ki_ = declare_parameter<double>("steer_ki", 0.5);
    steer_max_ = declare_parameter<double>("steer_max", 0.25);
    steer_rate_limit_ = declare_parameter<double>("steer_rate_limit", 0.01);
    hip_steer_sign_ =
      declare_parameter<std::vector<double>>("hip_steer_sign", {1.0, 1.0, 1.0, 1.0});
    hip_steer_sign_.resize(4, 0.0);
    // Gait gates off below these thresholds (the policy cannot stand still on
    // its own). stand_yaw must stay under the trained turn-in-place floor
    // (|yaw| in [0.15, 0.4]) or pure-yaw commands would never reach the policy.
    stand_vx_thresh_ = declare_parameter<double>("stand_vx_thresh", 0.02);
    stand_yaw_thresh_ = declare_parameter<double>("stand_yaw_thresh", 0.05);
    // Station-keeping: latch the heading on stop and keep the hip-bias steering
    // active while gated, so DART contact creep cannot pivot the idle stance.
    station_keep_ = declare_parameter<bool>("station_keep", true);
    // Position-hold: when idle, walk back to the latched stop point if contact
    // creep drifts the robot beyond the deadband (m); speed caps the crawl.
    position_hold_ = declare_parameter<bool>("position_hold", true);
    pos_hold_deadband_ = declare_parameter<double>("pos_hold_deadband", 0.30);
    pos_hold_speed_ = declare_parameter<double>("pos_hold_speed", 0.08);
    // Lateral-hold outer loop: closed-loop on the perpendicular offset from
    // the latched line, vy = clamp(-Kp*offset). Re-latches while turning.
    // A/B (post armature fix): forward 1:1, crab cut ~4x. Default ON.
    lateral_hold_ = declare_parameter<bool>("lateral_hold", true);
    lateral_kp_ = declare_parameter<double>("lateral_kp", 0.25);
    // |wz| above which the motion is a turn -> re-latch the reference line.
    lateral_turn_thresh_ = declare_parameter<double>("lateral_turn_thresh", 0.05);
    // Cross-track steering: lateral offset -> heading-error bias, so the strong
    // hip-bias steering (not the weak vy) pulls the robot back to the line.
    lateral_yaw_kp_ = declare_parameter<double>("lateral_yaw_kp", 0.6);
    lateral_yaw_max_ = declare_parameter<double>("lateral_yaw_max", 0.4);
    // In-node effort-PD (tau = kp*(q_des - q) - kd*qd) is UNUSED in the
    // shipped path: JointEffortPdController does the PD and reads this topic
    // as POSITION targets, so use_effort=true would feed it torques as
    // positions. Default false to match the wiring; kp/kd/effort_limit below
    // only matter if use_effort is deliberately re-enabled.
    use_effort_ = declare_parameter<bool>("use_effort", false);
    kp_ = declare_parameter<double>("kp", 20.0);
    kd_ = declare_parameter<double>("kd", 2.0);
    effort_limit_ = declare_parameter<double>("effort_limit", 1.0);
    // PD runs at pd_rate_ (200 Hz), policy decimated to control_rate_ (50 Hz,
    // matching training). PD at 50 Hz under-damps -> in-place jitter.
    pd_rate_ = declare_parameter<double>("pd_rate", 200.0);
    warmup_sec_ = declare_parameter<double>("warmup_sec", 3.0);
    policy_decimation_ = std::max(1, static_cast<int>(std::round(pd_rate_ / control_rate_)));
    joint_names_ = declare_parameter<std::vector<std::string>>(
      "joint_names", {"Revolute_110", "Revolute_111", "Revolute_112", "Revolute_113",
                      "Revolute_114", "Revolute_115", "Revolute_116", "Revolute_117",
                      "Revolute_118", "Revolute_119", "Revolute_120", "Revolute_121"});
    auto default_pose = declare_parameter<std::vector<double>>(
      "default_joint_pos",
      {0.0, 0.0, 0.0, 0.0, -0.32, -0.32, -0.32, -0.32, 2.00, 2.00, 2.00, 2.00});

    for (int i = 0; i < bbpc::kNumJoints && i < static_cast<int>(default_pose.size()); ++i) {
      obs_.default_joint_pos[i] = default_pose[i];
      obs_.joint_pos[i] = default_pose[i];
    }
    for (size_t i = 0; i < joint_names_.size(); ++i) {
      joint_index_[joint_names_[i]] = static_cast<int>(i);
    }

    obs_.gait_frequency = declare_parameter<double>("gait_frequency", 0.667);
    obs_.turn_clock_boost = declare_parameter<double>("turn_clock_boost", 0.8);
    obs_.speed_clock_boost = declare_parameter<double>("speed_clock_boost", 1.1);
    obs_.gait_boost_max = declare_parameter<double>("gait_boost_max", 2.1);
    auto gait_offsets =
      declare_parameter<std::vector<double>>("gait_offsets", {0.0, 0.5, 0.25, 0.75});
    for (int i = 0; i < bbpc::kNumFeet && i < static_cast<int>(gait_offsets.size()); ++i) {
      obs_.gait_offsets[i] = gait_offsets[i];
    }

    // ----------------------------- ONNX model -----------------------------
    if (!load_model(model_path_)) {
      RCLCPP_ERROR(
        get_logger(), "failed to load policy model '%s'; node will idle", model_path_.c_str());
    }

    // --------------------------- Pub / Sub / Srv --------------------------
    cmd_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "/position_controller/commands", rclcpp::QoS(1));
    status_pub_ = create_publisher<spider_msgs::msg::PolicyStatus>("policy_status", rclcpp::QoS(1));

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom", rclcpp::SensorDataQoS(),
      std::bind(&PolicyControllerNode::on_odom, this, std::placeholders::_1));
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "/imu", rclcpp::SensorDataQoS(),
      std::bind(&PolicyControllerNode::on_imu, this, std::placeholders::_1));
    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", rclcpp::SensorDataQoS(),
      std::bind(&PolicyControllerNode::on_joints, this, std::placeholders::_1));
    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", rclcpp::QoS(1),
      std::bind(&PolicyControllerNode::on_cmd, this, std::placeholders::_1));
    // Debug-only: constant per-hip bias (rad) for calibrating the steering.
    hip_bias_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      "/debug_hip_bias", rclcpp::QoS(1),
      [this](const std_msgs::msg::Float64MultiArray::SharedPtr m) {
        std::lock_guard<std::mutex> lk(state_mutex_);
        for (size_t i = 0; i < 4 && i < m->data.size(); ++i) {
          debug_hip_bias_[i] = m->data[i];
        }
      });

    set_enabled_srv_ = create_service<spider_msgs::srv::SetPolicyEnabled>(
      "set_policy_enabled",
      std::bind(
        &PolicyControllerNode::on_set_enabled, this, std::placeholders::_1, std::placeholders::_2));
    load_policy_srv_ = create_service<spider_msgs::srv::LoadPolicy>(
      "load_policy",
      std::bind(
        &PolicyControllerNode::on_load_policy, this, std::placeholders::_1, std::placeholders::_2));

    const auto period = std::chrono::duration<double>(1.0 / control_rate_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&PolicyControllerNode::control_loop, this));

    RCLCPP_INFO(
      get_logger(), "policy_controller up: rate=%.1f Hz, scale=%.2f, enabled=%s", control_rate_,
      action_scale_, enabled_ ? "true" : "false");
  }

private:
  bool load_model(const std::string & path)
  {
    if (path.empty()) {
      return false;
    }
    try {
      Ort::SessionOptions opts;
      opts.SetIntraOpNumThreads(1);
      opts.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
      auto session = std::make_unique<Ort::Session>(env_, path.c_str(), opts);
      std::lock_guard<std::mutex> lk(model_mutex_);
      session_ = std::move(session);
      model_path_ = path;
      RCLCPP_INFO(get_logger(), "loaded policy: %s", path.c_str());
      return true;
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "ONNX load error: %s", e.what());
      return false;
    }
  }

  void on_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(state_mutex_);
    // Odometry twist is reported in the child (body) frame by the gz
    // OdometryPublisher, matching root_lin_vel_b.
    obs_.root_lin_vel_b = {
      msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z};
  }

  void on_imu(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(state_mutex_);
    obs_.root_ang_vel_b = {
      msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z};
    obs_.set_gravity_from_quaternion(
      msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
  }

  void on_joints(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(state_mutex_);
    for (size_t i = 0; i < msg->name.size(); ++i) {
      auto it = joint_index_.find(msg->name[i]);
      if (it == joint_index_.end()) {
        continue;
      }
      const int idx = it->second;
      if (i < msg->position.size()) {
        obs_.joint_pos[idx] = msg->position[i];
      }
      if (i < msg->velocity.size()) {
        obs_.joint_vel[idx] = msg->velocity[i];
      }
    }
    have_joints_ = true;
  }

  void on_cmd(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(state_mutex_);
    // Raw command; clamping + heading hold happen in update_commands().
    cmd_vx_ = msg->linear.x;
    cmd_vy_ = msg->linear.y;
    cmd_wz_ = msg->angular.z;
    last_cmd_time_ = now();
  }

  static double wrap_pi(double a) { return std::atan2(std::sin(a), std::cos(a)); }

  // Build obs_.commands: safe-stop timeout, trained-envelope clamp, and (when
  // heading_hold_) the heading/lateral corrections. Caller holds state_mutex_.
  void update_commands()
  {
    const double dt = 1.0 / control_rate_;
    const bool stale = cmd_timeout_ > 0.0 && (now() - last_cmd_time_).seconds() > cmd_timeout_;
    double vx = stale ? 0.0 : cmd_vx_;
    double vy = stale ? 0.0 : cmd_vy_;
    double wz = stale ? 0.0 : cmd_wz_;
    vx = std::clamp(vx, 0.0, max_lin_vel_x_);
    vy = std::clamp(vy, -max_lin_vel_y_, max_lin_vel_y_);
    // Clamp yaw on the live path (the !heading_hold_ branch below is the only
    // other clamp and the shipped default never takes it).
    wz = std::clamp(wz, -max_yaw_rate_, max_yaw_rate_);

    // Position-hold: latch the stop point on the moving->idle transition;
    // beyond the deadband, face it then walk back (the gait has no reverse).
    if (position_hold_ && have_odom_yaw_) {
      if (stale && !prev_stale_) {
        hold_x_ = current_x_;
        hold_y_ = current_y_;
      }
      if (stale) {
        const double dx = hold_x_ - current_x_;
        const double dy = hold_y_ - current_y_;
        const double dist = std::hypot(dx, dy);
        if (dist > pos_hold_deadband_) {
          const double yaw_to_ref = wrap_pi(std::atan2(dy, dx) - current_yaw_);
          if (std::abs(yaw_to_ref) > 0.6) {
            vx = 0.0;  // turn in place to face the stop point first
            wz = std::clamp(2.0 * yaw_to_ref, -max_yaw_rate_, max_yaw_rate_);
          } else {
            vx = std::clamp(0.6 * dist, stand_vx_thresh_ + 0.02, pos_hold_speed_);
            wz = std::clamp(1.5 * yaw_to_ref, -max_yaw_rate_, max_yaw_rate_);
          }
        }
      }
    }
    prev_stale_ = stale;

    if (!heading_hold_ || !have_odom_yaw_) {
      obs_.commands = {vx, vy, std::clamp(wz, -max_yaw_rate_, max_yaw_rate_)};
      steer_cmd_ = 0.0;  // calibration mode: only /debug_hip_bias steers
      return;
    }

    // Latch the heading setpoint + lateral line origin at startup and on the
    // moving->idle transition. With station_keep the latched heading is HELD
    // through idle so the steering corrects contact creep instead of tracking it.
    const bool idle = std::abs(vx) < 0.01 && std::abs(wz) < 0.01;
    const bool just_stopped = idle && !prev_idle_;
    if (!heading_init_ || just_stopped || (idle && !station_keep_)) {
      desired_yaw_ = heading_lock_ ? heading_lock_yaw_ : current_yaw_;
      steer_i_ = 0.0;       // reset integrator so it never holds a stale heading
      ref_x_ = current_x_;  // latch the lateral-hold reference line origin
      ref_y_ = current_y_;
      heading_init_ = true;
    }
    prev_idle_ = idle;
    // Heading setpoint: integrate the commanded yaw rate, anti-windup clamped
    // to heading_err_clamp_ of the measured heading. heading_lock instead holds
    // a fixed world heading (demo_straight; the warmup drop settles ~14 deg off
    // and latching that made "straight" walk south-East).
    double err;
    if (heading_lock_) {
      desired_yaw_ = heading_lock_yaw_;
      err =
        std::clamp(wrap_pi(desired_yaw_ - current_yaw_), -heading_err_clamp_, heading_err_clamp_);
    } else {
      desired_yaw_ = wrap_pi(desired_yaw_ + wz * dt);
      err = wrap_pi(desired_yaw_ - current_yaw_);
      if (std::abs(err) > heading_err_clamp_) {
        err = std::clamp(err, -heading_err_clamp_, heading_err_clamp_);
        desired_yaw_ = wrap_pi(current_yaw_ + err);
      }
    }
    // Lateral hold: perpendicular offset from the latched line -> vy toward the
    // line, plus a cross-track bias folded into the heading error so the strong
    // hip-bias steering (not the weak vy) walks the robot back onto it.
    double vy_out = vy;
    if (lateral_hold_) {
      if (std::abs(wz) > lateral_turn_thresh_) {
        ref_x_ = current_x_;
        ref_y_ = current_y_;
      } else {
        const double dx = current_x_ - ref_x_;
        const double dy = current_y_ - ref_y_;
        const double lat_err = -dx * std::sin(desired_yaw_) + dy * std::cos(desired_yaw_);
        vy_out = std::clamp(-lateral_kp_ * lat_err, -max_lin_vel_y_, max_lin_vel_y_);
        const double cross_bias =
          std::clamp(-lateral_yaw_kp_ * lat_err, -lateral_yaw_max_, lateral_yaw_max_);
        err = std::clamp(
          err + cross_bias, -heading_err_clamp_ - lateral_yaw_max_,
          heading_err_clamp_ + lateral_yaw_max_);
      }
    }
    const double yaw_cmd = std::clamp(wz + heading_kp_ * err, -heading_max_, heading_max_);
    // PI hip-bias steering does the real turning work; I removes the
    // steady-state offset against the constant drift.
    if (steer_ki_ > 1e-6) {
      steer_i_ += err * dt;
      // Anti-windup: clamp the integral so its contribution stays within range.
      const double i_lim = steer_max_ / steer_ki_;
      steer_i_ = std::clamp(steer_i_, -i_lim, i_lim);
    }
    steer_cmd_ = std::clamp(steer_kp_ * err + steer_ki_ * steer_i_, -steer_max_, steer_max_);
    // Slow forward while a large heading error is being corrected.
    double fwd_scale = 1.0;
    if (fwd_slow_err_ > 1e-3) {
      fwd_scale = std::clamp(1.0 - std::abs(err) / fwd_slow_err_, fwd_min_scale_, 1.0);
    }
    obs_.commands = {vx * fwd_scale, vy_out, yaw_cmd};
  }

  void on_set_enabled(
    const std::shared_ptr<spider_msgs::srv::SetPolicyEnabled::Request> req,
    std::shared_ptr<spider_msgs::srv::SetPolicyEnabled::Response> res)
  {
    enabled_ = req->enabled;
    res->success = true;
    res->message = enabled_ ? "gait armed" : "gait disarmed";
    RCLCPP_INFO(get_logger(), "%s", res->message.c_str());
  }

  void on_load_policy(
    const std::shared_ptr<spider_msgs::srv::LoadPolicy::Request> req,
    std::shared_ptr<spider_msgs::srv::LoadPolicy::Response> res)
  {
    if (load_model(req->model_path)) {
      res->success = true;
      res->message = "loaded " + req->model_path;
    } else {
      res->success = false;
      res->message = "failed to load " + req->model_path;
    }
  }

  void control_loop()
  {
    auto status = spider_msgs::msg::PolicyStatus();
    status.header.stamp = now();
    status.header.frame_id = "base_link";
    status.rate_hz = control_rate_;
    status.enabled = enabled_;

    bool ready;
    {
      std::lock_guard<std::mutex> lk(model_mutex_);
      ready = session_ != nullptr;
    }
    if (!enabled_ || !ready || !have_joints_) {
      status.action_norm = 0.0;
      status.inference_ms = 0.0;
      status_pub_->publish(status);
      return;
    }

    // Startup warmup: hold the default pose so the PD recovers the stance from
    // the spawn drop before the policy takes over.
    if (warmup_start_.nanoseconds() == 0) {
      warmup_start_ = now();
    }
    const bool warming = warmup_sec_ > 0.0 && (now() - warmup_start_).seconds() < warmup_sec_;

    // Policy step, decimated to control_rate_ (50 Hz, matching training);
    // the PD below damps at the full pd_rate_.
    if (warming) {
      std::lock_guard<std::mutex> lk(state_mutex_);
      if (cmd_timeout_ > 0.0 && (now() - last_cmd_time_).seconds() > cmd_timeout_) {
        obs_.commands = {0.0, 0.0, 0.0};
      }
      decim_count_ = 0;
      last_action_norm_ = 0.0;
    } else if (++decim_count_ >= policy_decimation_) {
      decim_count_ = 0;
      std::vector<float> input;
      bool moving;
      {
        std::lock_guard<std::mutex> lk(state_mutex_);
        update_commands();
        // Clock advances every policy step, moving or idle, mirroring
        // big_bertha_env.py's _pre_physics_step.
        obs_.advance_clock(obs_.commands[0], obs_.commands[2], 1.0 / control_rate_);
        // Gate the gait when neither forward nor turn is commanded (the policy
        // cannot stand still). The |wz| half matters: gating on vx alone would
        // swallow the trained turn-in-place commands.
        moving =
          obs_.commands[0] > stand_vx_thresh_ || std::abs(obs_.commands[2]) > stand_yaw_thresh_;
        if (!moving) {
          // Hold the default stance; with station_keep the hip-bias steering
          // stays active to hold the latched heading against contact creep.
          if (!station_keep_) {
            steer_cmd_ = 0.0;
          }
          for (int i = 0; i < bbpc::kNumJoints; ++i) {
            double t = obs_.default_joint_pos[i];
            if (station_keep_ && i < 4) {  // hips: hold heading via steering bias
              t += debug_hip_bias_[i] + steer_cmd_ * hip_steer_sign_[i];
            }
            target_pos_[i] = std::clamp(t, -joint_limit_, joint_limit_);
            obs_.prev_actions[i] = 0.0f;
          }
          last_action_norm_ = 0.0;
        } else {
          input = obs_.build();
        }
      }
      if (!moving) {
        status.inference_ms = last_inf_ms_;
        status.action_norm = last_action_norm_;
        // fall through to the PD step so it holds the stance targets
      } else {
        for (float & v : input) {
          if (!std::isfinite(v)) {
            v = 0.0f;
          }
        }
        std::array<float, bbpc::kActionDim> action{};
        {
          std::lock_guard<std::mutex> lk(model_mutex_);
          try {
            const std::array<int64_t, 2> shape{1, bbpc::kObsDim};
            Ort::MemoryInfo mem = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
            Ort::Value in_tensor = Ort::Value::CreateTensor<float>(
              mem, input.data(), input.size(), shape.data(), shape.size());
            const char * in_names[] = {"obs"};
            const char * out_names[] = {"actions"};
            auto t0 = std::chrono::steady_clock::now();
            auto outputs =
              session_->Run(Ort::RunOptions{nullptr}, in_names, &in_tensor, 1, out_names, 1);
            auto t1 = std::chrono::steady_clock::now();
            last_inf_ms_ = std::chrono::duration<double, std::milli>(t1 - t0).count();
            const float * out = outputs.front().GetTensorData<float>();
            for (int i = 0; i < bbpc::kActionDim; ++i) {
              action[i] = out[i];
            }
          } catch (const std::exception & e) {
            RCLCPP_ERROR_THROTTLE(
              get_logger(), *get_clock(), 2000, "inference error: %s", e.what());
            status_pub_->publish(status);
            return;
          }
        }
        double norm_sq = 0.0;
        {
          std::lock_guard<std::mutex> lk(state_mutex_);
          for (int i = 0; i < bbpc::kNumJoints; ++i) {
            double a_raw = std::isfinite(action[i]) ? action[i] : 0.0;
            // Env clamps the raw action to [-1, 1] before scaling.
            double a = std::clamp(a_raw, -action_clip_, action_clip_);
            double t = action_scale_ * a + obs_.default_joint_pos[i];
            if (i < 4) {  // hips: inject differential-stride steering + debug bias
              t += debug_hip_bias_[i] + steer_cmd_ * hip_steer_sign_[i];
            }
            target_pos_[i] = std::clamp(t, -joint_limit_, joint_limit_);
            norm_sq += a * a;
            obs_.prev_actions[i] = static_cast<float>(a);
          }
        }
        last_action_norm_ = std::sqrt(norm_sq);
      }  // end if (moving)
    }

    // PD step every tick: tau = Kp*(q_des - q) - Kd*qd, torque-limited.
    std_msgs::msg::Float64MultiArray cmd;
    cmd.data.resize(bbpc::kNumJoints);
    double norm_sq = 0.0;
    {
      std::lock_guard<std::mutex> lk(state_mutex_);
      for (int i = 0; i < bbpc::kNumJoints; ++i) {
        double a_raw = std::isfinite(action[i]) ? action[i] : 0.0;
        // The training env clamps the raw policy action to [-1, 1] BEFORE
        // scaling (big_bertha_env.py _pre_physics_step:
        //   self._actions = torch.clamp(actions, -1.0, 1.0)).
        // The policy is trained to rely on this saturation, so we must
        // replicate it here; otherwise large raw outputs slam every joint
        // to the limit and the gait collapses.
        double a = std::clamp(a_raw, -action_clip_, action_clip_);
        double target = action_scale_ * a + obs_.default_joint_pos[i];
        // Final safety clamp to the joint range.
        target = std::clamp(target, -joint_limit_, joint_limit_);
        cmd.data[i] = target;
        norm_sq += a * a;
        // Feed back the CLAMPED action, matching the env's
        // self._previous_actions = self._actions.clone().
        obs_.prev_actions[i] = static_cast<float>(a);
      }
    }
    cmd_pub_->publish(cmd);

    status.inference_ms = inf_ms;
    status.action_norm = std::sqrt(norm_sq);
    status_pub_->publish(status);
  }

  // ONNX
  Ort::Env env_{ORT_LOGGING_LEVEL_WARNING, "policy_controller"};
  std::unique_ptr<Ort::Session> session_;
  std::mutex model_mutex_;

  // State
  bbpc::ObservationBuilder obs_;
  std::mutex state_mutex_;
  std::map<std::string, int> joint_index_;
  std::vector<std::string> joint_names_;
  bool have_joints_{false};
  rclcpp::Time last_cmd_time_{0, 0, RCL_ROS_TIME};

  // Params
  std::string model_path_;
  double action_scale_{0.25};
  double control_rate_{50.0};
  double cmd_timeout_{0.5};
  double joint_limit_{3.14159};
  double action_clip_{1.0};
  // Trained envelope (big_bertha_env.py::_reset_idx): vx [0, 0.40],
  // vy +/-0.05, yaw +/-0.5. Fallback defaults; policy.yaml sets runtime values.
  double max_lin_vel_x_{0.3};
  double max_lin_vel_y_{0.05};
  double max_yaw_rate_{0.15};
  bool enabled_{true};

  // Heading-hold outer loop (drift rejection without retraining).
  bool heading_hold_{true};
  double heading_kp_{2.0};
  bool heading_lock_{false};      // demo_straight: hold a fixed world heading
  double heading_lock_yaw_{0.0};  // the locked heading (rad, 0 = East)
  double heading_max_{0.5};
  double heading_err_clamp_{0.4};
  double fwd_slow_err_{0.5};
  double fwd_min_scale_{0.15};
  double cmd_vx_{0.0};
  double cmd_vy_{0.0};
  double cmd_wz_{0.0};
  double current_yaw_{0.0};
  bool have_odom_yaw_{false};
  double desired_yaw_{0.0};
  bool heading_init_{false};
  bool station_keep_{true};   // hold heading while gated/idle (anti pre-goal creep)
  bool prev_idle_{false};     // prev-step idle state, for the stop-transition latch
  bool prev_moving_{false};   // prev-step moving state, for steer_i_ reset on gait start
  bool position_hold_{true};  // return to the stop point when idle (anti post-goal wander)
  bool prev_stale_{false};    // prev-step stale state, for the hold-point latch
  double hold_x_{0.0};        // latched stop point for position-hold
  double hold_y_{0.0};
  double pos_hold_deadband_{0.30};
  double pos_hold_speed_{0.08};
  // Lateral-hold outer loop (closed-loop line tracking on the odom offset).
  bool lateral_hold_{true};
  double lateral_kp_{0.25};
  double lateral_turn_thresh_{0.05};
  double lateral_yaw_kp_{0.6};   // cross-track: offset -> heading-error bias
  double lateral_yaw_max_{0.4};  // max |cross-track heading bias| (rad)
  double current_x_{0.0};
  double current_y_{0.0};
  double ref_x_{0.0};
  double ref_y_{0.0};
  // Differential-stride steering (PI on heading error).
  double steer_kp_{0.6};
  double steer_ki_{0.5};
  double steer_i_{0.0};
  double steer_max_{0.25};
  double steer_rate_limit_{0.01};
  double last_steer_cmd_{0.0};
  std::vector<double> hip_steer_sign_{1.0, 1.0, 1.0, 1.0};
  double steer_cmd_{0.0};
  std::array<double, 4> debug_hip_bias_{};
  double stand_vx_thresh_{0.02};
  double stand_yaw_thresh_{0.05};

  // ROS
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_pub_;
  rclcpp::Publisher<spider_msgs::msg::PolicyStatus>::SharedPtr status_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Service<spider_msgs::srv::SetPolicyEnabled>::SharedPtr set_enabled_srv_;
  rclcpp::Service<spider_msgs::srv::LoadPolicy>::SharedPtr load_policy_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PolicyControllerNode>());
  rclcpp::shutdown();
  return 0;
}
