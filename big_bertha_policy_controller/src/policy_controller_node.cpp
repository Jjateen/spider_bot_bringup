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
#include <cmath>
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
    // /cmd_vel clamp to the policy's trained command envelope (yaw-for-nav
    // fix).
    max_lin_vel_x_ = declare_parameter<double>("max_lin_vel_x", 0.3);
    max_lin_vel_y_ = declare_parameter<double>("max_lin_vel_y", 0.05);
    max_yaw_rate_ = declare_parameter<double>("max_yaw_rate", 0.15);
    // --------------------- Heading-hold outer loop ----------------------
    // The PhysX->DART contact asymmetry gives a systematic yaw drift (the
    // robot curves right even when commanded straight). It is a steady
    // disturbance, so a closed-loop heading controller in the deployment node
    // rejects it WITHOUT retraining or touching the sim: integrate Nav2's
    // commanded yaw rate into a heading setpoint, then feed the policy a
    // CORRECTIVE yaw command  yaw = wz_ff + Kp*(desired_yaw - odom_yaw),
    // clamped to the trained turn envelope. When the heading error is large we
    // also throttle forward speed so the (weak) in-place turn authority can
    // realign before pushing on. heading_hold:=false restores raw passthrough.
    heading_hold_ = declare_parameter<bool>("heading_hold", true);
    heading_kp_ = declare_parameter<double>("heading_kp", 2.0);
    // Absolute heading lock for the straight-line demo: hold heading_lock_yaw_
    // (world rad, 0 = +x East) instead of latching the heading the robot settles
    // into during the DART warmup drop, so the gait walks a true straight line.
    // Leave false for nav (Nav2 owns the heading via wz).
    heading_lock_ = declare_parameter<bool>("heading_lock", false);
    heading_lock_yaw_ = declare_parameter<double>("heading_lock_yaw", 0.0);
    // Final yaw command clamp -- the policy was trained on yaw in [-0.6, 0.6],
    // so 0.5 gives the correction real authority (the old 0.15 clamp throttled
    // the turn so hard the drift always won).
    heading_max_ = declare_parameter<double>("heading_max", 0.5);
    // Anti-windup: never let the setpoint lead/lag the measured heading by more
    // than this, so a turn the policy cannot keep up with does not wind up and
    // overshoot when the command relaxes.
    heading_err_clamp_ = declare_parameter<double>("heading_err_clamp", 0.4);
    // Forward speed is scaled to fwd_min_scale at |err| = fwd_slow_err (turn in
    // place) and to 1.0 at zero error (full forward while tracking straight).
    fwd_slow_err_ = declare_parameter<double>("fwd_slow_err", 0.5);
    fwd_min_scale_ = declare_parameter<double>("fwd_min_scale", 0.15);
    // --------------- Differential-stride steering (the real knob) ----------
    // model_49999 barely responds to the yaw COMMAND in DART, so the heading
    // loop also drives a hip-bias steering signal: the hips rotate about the
    // body Z axis, so adding steer_cmd*hip_steer_sign[i] to the hip targets
    // rotates all stance feet the same way and yaws the body. This is an
    // output-level correction (no retrain, no sim edit). hip_steer_sign sets
    // the per-hip polarity (calibrated empirically via /debug_hip_bias).
    steer_kp_ = declare_parameter<double>("steer_kp", 0.6);
    steer_ki_ = declare_parameter<double>("steer_ki", 0.5);
    steer_max_ = declare_parameter<double>("steer_max", 0.25);
    hip_steer_sign_ =
      declare_parameter<std::vector<double>>("hip_steer_sign", {1.0, 1.0, 1.0, 1.0});
    hip_steer_sign_.resize(4, 0.0);
    // Below this forward command the gait is gated off and the robot holds the
    // default stance (the policy cannot stand still on its own).
    stand_vx_thresh_ = declare_parameter<double>("stand_vx_thresh", 0.02);
    // Station-keeping: while gated/idle (e.g. before the Nav2 goal arrives) the
    // gated default stance slowly PIVOTS on DART's asymmetric contact (~15 deg
    // before a goal is sent), so nav starts pointed at a wall. With this on, the
    // heading setpoint is latched the moment the robot stops (not re-latched
    // every idle frame, which would just track the creep) and the hip-bias
    // steering is applied even while gated, so the robot actively holds its
    // heading without walking. Off restores the fully-passive gated stance.
    station_keep_ = declare_parameter<bool>("station_keep", true);
    // Position-hold: when idle, actively return to the latched stop point so the
    // gated stance's DART contact creep cannot wander the robot into a wall after
    // the goal. deadband: only correct drifts beyond this (m); speed: cap the
    // return crawl (m/s). Off restores the plain gated stance.
    position_hold_ = declare_parameter<bool>("position_hold", true);
    pos_hold_deadband_ = declare_parameter<double>("pos_hold_deadband", 0.30);
    pos_hold_speed_ = declare_parameter<double>("pos_hold_speed", 0.08);
    // ------------------- Lateral-hold outer loop ------------------------
    // The heading loop holds yaw, but the gait still slides off its LINE in
    // DART (the systematic sideways crab + the residual yaw the weak turn
    // authority cannot fully undo). Training an Isaac velocity penalty did not
    // close this -- Isaac never reproduces DART's sustained sideways push -- so
    // reject it the same way as the yaw: a closed-loop controller on the
    // PERPENDICULAR offset from the line through the latched start pose along
    // the heading setpoint, commanding the policy's lateral velocity
    //   vy = clamp(-Kp * lateral_error, +/- max_lin_vel_y).
    // Feedback makes it robust to the drift's source/magnitude. The reference
    // line re-latches while turning so it only holds during straight segments.
    lateral_hold_ = declare_parameter<bool>("lateral_hold", true);
    lateral_kp_ = declare_parameter<double>("lateral_kp", 0.25);
    // |wz| above which we treat the motion as a turn and re-latch the line.
    lateral_turn_thresh_ = declare_parameter<double>("lateral_turn_thresh", 0.05);
    // Cross-track STEERING gain: lateral offset -> heading-error bias, so the
    // strong hip-bias steering (not the weak vy) drives the robot back to the
    // line. rad of heading bias per metre of offset, clamped to lateral_yaw_max.
    lateral_yaw_kp_ = declare_parameter<double>("lateral_yaw_kp", 0.6);
    lateral_yaw_max_ = declare_parameter<double>("lateral_yaw_max", 0.4);
    // Effort-PD actuator emulation of Isaac's ImplicitActuatorCfg
    // (stiffness=20, damping=2): output joint torque
    //   tau = kp*(q_des - q) + kd*(0 - qd), torque-limited,
    // so the legs swing with momentum like in training instead of snapping to
    // a pose. Kp is softened to 10 (from the Isaac 20) and the effort limit
    // raised to 12 so the torque is not pinned bang-bang at the rail: at
    // kp=20/limit=8 the PD railed +/-8 on most joints, while kp=10/limit=12
    // keeps it inside the band (measured |tau| <= ~5.6) for clean damping.
    use_effort_ = declare_parameter<bool>("use_effort", true);
    kp_ = declare_parameter<double>("kp", 20.0);
    kd_ = declare_parameter<double>("kd", 2.0);
    // Match Isaac's effort_limit_sim = 1 Nm exactly: the implicit PD there
    // clamps tau to [-1, 1], so the policy is trained against torque bounded
    // at 1 Nm. Giving the legs more authority (e.g. 12) makes them overshoot.
    effort_limit_ = declare_parameter<double>("effort_limit", 1.0);
    // The PD torque is evaluated at pd_rate_ (200 Hz) while the policy only
    // runs every policy_decimation_ ticks, so its effective rate matches
    // training (control_rate_ = 50 Hz, decimation 4). Evaluating the PD at the
    // policy rate (50 Hz) under-damps and oscillates -> the in-place jitter.
    pd_rate_ = declare_parameter<double>("pd_rate", 200.0);
    warmup_sec_ = declare_parameter<double>("warmup_sec", 3.0);
    policy_decimation_ = std::max(1, static_cast<int>(std::round(pd_rate_ / control_rate_)));
    joint_names_ = declare_parameter<std::vector<std::string>>(
      "joint_names", {"Revolute_110", "Revolute_111", "Revolute_112", "Revolute_113",
                      "Revolute_114", "Revolute_115", "Revolute_116", "Revolute_117",
                      "Revolute_118", "Revolute_119", "Revolute_120", "Revolute_121"});
    auto default_pose = declare_parameter<std::vector<double>>(
      "default_joint_pos", {0.0, 0.0, 0.0, 0.0, -0.32, -0.32, -0.32, -0.32, 2.00, 2.00, 2.00, 2.00});

    for (int i = 0; i < bbpc::kNumJoints && i < static_cast<int>(default_pose.size()); ++i) {
      obs_.default_joint_pos[i] = default_pose[i];
      target_pos_[i] = default_pose[i];
    }
    for (size_t i = 0; i < joint_names_.size(); ++i) {
      joint_index_[joint_names_[i]] = static_cast<int>(i);
    }

    obs_.gait_frequency = declare_parameter<double>("gait_frequency", 0.667);
    obs_.turn_clock_boost = declare_parameter<double>("turn_clock_boost", 0.8);
    obs_.speed_clock_boost = declare_parameter<double>("speed_clock_boost", 1.1);
    auto gait_offsets = declare_parameter<std::vector<double>>("gait_offsets", {0.0, 0.5, 0.25, 0.75});
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
    // Debug-only: inject a constant per-hip bias (rad) added to the policy's
    // hip targets, to calibrate the differential-stride steering knob. The
    // closed-loop demo uses the heading-driven steering, not this topic.
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

    const auto period = std::chrono::duration<double>(1.0 / pd_rate_);
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
    // Absolute heading (map/odom yaw) for the heading-hold outer loop.
    const auto & q = msg->pose.pose.orientation;
    current_yaw_ = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    // Absolute planar position for the lateral-hold outer loop.
    current_x_ = msg->pose.pose.position.x;
    current_y_ = msg->pose.pose.position.y;
    have_odom_yaw_ = true;
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
    // Store the raw Nav2 command; the trained-envelope clamp and the
    // heading-hold correction are applied in update_commands() at the policy
    // rate (so the heading integrator advances on a regular dt).
    cmd_vx_ = msg->linear.x;
    cmd_vy_ = msg->linear.y;
    cmd_wz_ = msg->angular.z;
    last_cmd_time_ = now();
  }

  static double wrap_pi(double a) { return std::atan2(std::sin(a), std::cos(a)); }

  // Build obs_.commands from the latest Nav2 command. Caller must hold
  // state_mutex_. Applies the safe-stop timeout, clamps to the trained command
  // envelope, and (when heading_hold_) replaces the yaw command with a
  // heading-setpoint tracking correction that rejects the systematic contact
  // drift. Returns nothing; writes obs_.commands.
  void update_commands()
  {
    const double dt = 1.0 / control_rate_;
    const bool stale = cmd_timeout_ > 0.0 && (now() - last_cmd_time_).seconds() > cmd_timeout_;
    double vx = stale ? 0.0 : cmd_vx_;
    double vy = stale ? 0.0 : cmd_vy_;
    double wz = stale ? 0.0 : cmd_wz_;
    vx = std::clamp(vx, 0.0, max_lin_vel_x_);
    vy = std::clamp(vy, -max_lin_vel_y_, max_lin_vel_y_);

    // Position-hold: when the robot is idle (Nav2 command gone stale, e.g. after
    // reaching the goal) it must HOLD where it stopped, not let the gated stance's
    // DART contact creep wander it across the arena into a wall. Latch the stop
    // point on the moving->idle transition; beyond a deadband, steer back to it
    // (face it, then walk -- the gait has no reverse). Within the deadband, fall
    // through to vx=0 so the gait gates and holds the stance.
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

    // Latch the heading setpoint + lateral-hold line origin. Without
    // station_keep this re-latches every idle frame (never corrects toward a
    // stale heading). With station_keep it latches only at startup and on the
    // moving->idle transition, then HOLDS the latched heading through the idle
    // window so the steering corrects the gated stance's contact creep instead
    // of tracking it (which left the robot pivoting ~15 deg before the goal).
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
    // Heading setpoint. Normally integrate the commanded yaw rate, then clamp to
    // stay within heading_err_clamp_ of the measured heading (anti-windup).
    // heading_lock (demo_straight) instead holds a FIXED world heading
    // (heading_lock_yaw_, e.g. 0 = due East): the robot settles ~14 deg south of
    // East during the warmup drop in DART, and latching that drifted heading made
    // "straight" walk south-East. Locking the setpoint lets the hip-bias steering
    // + cross-track null the settle and crab so the gait holds a true straight
    // line. (Disabled for nav, where Nav2 owns the heading via wz.)
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
    // Lateral-hold + cross-track steering. Compute the perpendicular offset from
    // the latched line (through ref_{x,y} along desired_yaw_). Command vy toward
    // the line AND -- since the policy's vy authority is too weak in DART to null
    // the systematic lateral crab via sideways motion -- fold a cross-track bias
    // into the heading error so the (strong) hip-bias steering + policy yaw aim
    // the gait's forward walk back onto the line. Catching the drift early this
    // way keeps the robot off the wall it otherwise crabs into. Re-latch the line
    // while turning (|wz| above threshold) and pass /cmd_vel vy through otherwise.
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
    // The hip-bias steering does the real turning work (the policy yaw command
    // barely moves the gait in DART). PI control: the P term reacts fast, the I
    // term removes the steady-state heading offset. With the cross-track bias
    // folded into err above, this same steering now also nulls the lateral crab.
    if (steer_ki_ > 1e-6) {
      steer_i_ += err * dt;
      // Anti-windup: clamp the integral so its contribution stays within range.
      const double i_lim = steer_max_ / steer_ki_;
      steer_i_ = std::clamp(steer_i_, -i_lim, i_lim);
    }
    steer_cmd_ = std::clamp(steer_kp_ * err + steer_ki_ * steer_i_, -steer_max_, steer_max_);
    // Slow forward while a large heading error is being corrected so the
    // steering has room to realign before pushing forward again.
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

    // ===== Startup warmup: for the first warmup_sec_ after joint state first
    // arrives (i.e. once the controller is active), just hold the default pose
    // (action = 0). The robot free-falls during the ~3 s gz_ros2_control load
    // gap because the gait must run frictionless (any stiction freezes it); the
    // stiff PD then pulls the splayed legs back to the default stance before
    // the policy takes over, so the policy starts from a clean upright pose
    // instead of crawling out of a collapse.
    if (warmup_start_.nanoseconds() == 0) {
      warmup_start_ = now();
    }
    const bool warming = warmup_sec_ > 0.0 && (now() - warmup_start_).seconds() < warmup_sec_;

    // ===== Policy step (decimated to ~control_rate_): update joint targets.
    // The timer fires at pd_rate_ (200 Hz); the policy only runs every
    // policy_decimation_ ticks so its effective rate matches training
    // (50 Hz, decimation 4). This keeps the gait timing right while letting
    // the PD below damp at the full physics rate -- without the split, a
    // 50 Hz torque held for 20 ms overshoots and oscillates (the jitter).
    if (warming) {
      std::lock_guard<std::mutex> lk(state_mutex_);
      for (int i = 0; i < bbpc::kNumJoints; ++i) {
        target_pos_[i] = obs_.default_joint_pos[i];
        obs_.prev_actions[i] = 0.0f;
      }
      decim_count_ = 0;
      last_action_norm_ = 0.0;
    } else if (++decim_count_ >= policy_decimation_) {
      decim_count_ = 0;
      std::vector<float> input;
      bool moving;
      {
        std::lock_guard<std::mutex> lk(state_mutex_);
        update_commands();  // safe-stop timeout + envelope clamp + heading hold
        // Gait clock advances every policy step regardless of moving/idle,
        // mirroring big_bertha_env.py's _pre_physics_step (dt = 1/control_rate_,
        // matching training's step_dt at 50 Hz).
        obs_.advance_clock(obs_.commands[0], obs_.commands[2], 1.0 / control_rate_);
        // The policy walks forward even when commanded vx=0 (vx=0 is out of its
        // training distribution), so a "stop" command would leave the robot
        // free-drifting. Gate it: when no forward motion is commanded (idle /
        // goal reached / cmd timeout), hold the default stance instead of
        // running the gait. Without this the robot wanders off A during the
        // pre-goal idle window and never navigates from the right start pose.
        moving = obs_.commands[0] > stand_vx_thresh_;
        if (!moving) {
          // Gated: hold the default stance (no forward gait). With station_keep,
          // still apply the hip-bias steering so the robot actively holds its
          // latched heading against the contact creep (no walking); otherwise go
          // fully passive (steer off).
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

    // ===== PD step every tick (pd_rate_): torque toward the held targets.
    // tau = Kp*(q_des - q) + Kd*(0 - qd), torque-limited -- Isaac's implicit
    // PD actuator, now evaluated at the full rate so it actually damps.
    std_msgs::msg::Float64MultiArray cmd;
    cmd.data.resize(bbpc::kNumJoints);
    {
      std::lock_guard<std::mutex> lk(state_mutex_);
      for (int i = 0; i < bbpc::kNumJoints; ++i) {
        if (use_effort_) {
          double effort =
            kp_ * (target_pos_[i] - obs_.joint_pos[i]) + kd_ * (0.0 - obs_.joint_vel[i]);
          cmd.data[i] = std::clamp(effort, -effort_limit_, effort_limit_);
        } else {
          cmd.data[i] = target_pos_[i];
        }
      }
    }
    cmd_pub_->publish(cmd);

    status.inference_ms = last_inf_ms_;
    status.action_norm = last_action_norm_;
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
  bool use_effort_{true};
  double kp_{20.0};
  double kd_{2.0};
  double effort_limit_{1.0};
  double pd_rate_{200.0};
  double warmup_sec_{3.0};
  rclcpp::Time warmup_start_{0, 0, RCL_ROS_TIME};
  int policy_decimation_{4};
  int decim_count_{0};
  std::array<double, bbpc::kNumJoints> target_pos_{};
  double last_inf_ms_{0.0};
  double last_action_norm_{0.0};
  double action_clip_{1.0};
  // Trained command ranges (big_bertha env _reset_idx sampling): vx in
  // [0.1,0.3] forward-only, vy in [+/-0.05], yaw in [+/-0.15]. Nav2 issues
  // commands well outside these (linear.x ~0.37, yaw ~0.44), which is
  // out-of-distribution for the policy. Clamp /cmd_vel to the trained envelope
  // so the policy stays in distribution (the yaw-for-nav blocker).
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
  std::vector<double> hip_steer_sign_{1.0, 1.0, 1.0, 1.0};
  double steer_cmd_{0.0};
  std::array<double, 4> debug_hip_bias_{};
  double stand_vx_thresh_{0.02};

  // ROS
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_pub_;
  rclcpp::Publisher<spider_msgs::msg::PolicyStatus>::SharedPtr status_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr hip_bias_sub_;
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
