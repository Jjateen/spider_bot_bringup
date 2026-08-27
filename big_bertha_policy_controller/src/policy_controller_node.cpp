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
#include <array>
#include <chrono>
#include <cmath>
#include <fstream>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "big_bertha_policy_controller/command_shaper.hpp"
#include "big_bertha_policy_controller/observation_builder.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ros_gz_interfaces/msg/contacts.hpp"
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
    // Per-joint-group action scale (idx 0-3 hip, 4-7 knee, 8-11 ankle). The
    // policy saturates action_clip on ~all 12 joints almost all the time
    // (measured via /policy_status action_norm sitting at sqrt(12) for a
    // large fraction of ticks) -- it carries phase, not amplitude, so
    // action_scale alone sets the gait's delivered amplitude and nothing
    // downstream (PD gains, landing assist) can add lift back once it's
    // capped low. A single uniform scale is also the wrong shape: analytic
    // FK at the default stance (see leverage_fk.py-derived numbers) gives
    // ~0 mm/rad foot lift for the hip (its axis is vertical, cannot lift),
    // ~-50 mm/rad for the knee with ~72 deg of joint range left at 0.25,
    // and ~+26 mm/rad for the ankle with only ~20 deg left -- the knee is
    // the joint with both the most leverage and the most headroom, the
    // ankle has almost none left. Default all three to action_scale_ so an
    // unset config is an exact-parity build.
    hip_action_scale_ = declare_parameter<double>("hip_action_scale", action_scale_);
    knee_action_scale_ = declare_parameter<double>("knee_action_scale", action_scale_);
    ankle_action_scale_ = declare_parameter<double>("ankle_action_scale", action_scale_);
    control_rate_ = declare_parameter<double>("control_rate", 50.0);
    enabled_ = declare_parameter<bool>("start_enabled", true);
    cmd_timeout_ = declare_parameter<double>("cmd_vel_timeout", 0.5);
    joint_limit_ = declare_parameter<double>("joint_limit", 3.14159);
    action_clip_ = declare_parameter<double>("action_clip", 1.0);
    // All command shaping (envelope clamps, heading/lateral hold, position
    // hold, PI hip-bias steering) lives in CommandShaper -- header-only and
    // unit-tested; this block only maps ROS params onto it.
    shaper_.max_lin_vel_x = declare_parameter<double>("max_lin_vel_x", 0.3);
    shaper_.max_reverse_vel_x = declare_parameter<double>("max_reverse_vel_x", 0.15);
    shaper_.max_lin_vel_y = declare_parameter<double>("max_lin_vel_y", 0.05);
    shaper_.max_yaw_rate = declare_parameter<double>("max_yaw_rate", 0.15);
    shaper_.heading_hold = declare_parameter<bool>("heading_hold", true);
    shaper_.heading_kp = declare_parameter<double>("heading_kp", 2.0);
    shaper_.heading_lock = declare_parameter<bool>("heading_lock", false);
    shaper_.heading_lock_yaw = declare_parameter<double>("heading_lock_yaw", 0.0);
    shaper_.heading_max = declare_parameter<double>("heading_max", 0.5);
    shaper_.heading_err_clamp = declare_parameter<double>("heading_err_clamp", 0.4);
    shaper_.fwd_slow_err = declare_parameter<double>("fwd_slow_err", 0.5);
    shaper_.fwd_min_scale = declare_parameter<double>("fwd_min_scale", 0.15);
    shaper_.steer_kp = declare_parameter<double>("steer_kp", 0.6);
    shaper_.steer_ki = declare_parameter<double>("steer_ki", 0.5);
    shaper_.steer_max = declare_parameter<double>("steer_max", 0.25);
    hip_steer_sign_ =
      declare_parameter<std::vector<double>>("hip_steer_sign", {1.0, 1.0, 1.0, 1.0});
    hip_steer_sign_.resize(4, 0.0);
    // Gait gates off below these thresholds (the policy cannot stand still on
    // its own). stand_yaw must stay under the trained turn-in-place floor
    // (|yaw| in [0.15, 0.4]) or pure-yaw commands would never reach the policy.
    shaper_.stand_vx_thresh = declare_parameter<double>("stand_vx_thresh", 0.02);
    stand_yaw_thresh_ = declare_parameter<double>("stand_yaw_thresh", 0.05);
    shaper_.station_keep = declare_parameter<bool>("station_keep", true);
    shaper_.position_hold = declare_parameter<bool>("position_hold", true);
    shaper_.pos_hold_deadband = declare_parameter<double>("pos_hold_deadband", 0.30);
    shaper_.pos_hold_speed = declare_parameter<double>("pos_hold_speed", 0.08);
    shaper_.lateral_hold = declare_parameter<bool>("lateral_hold", true);
    shaper_.lateral_kp = declare_parameter<double>("lateral_kp", 0.25);
    shaper_.lateral_turn_thresh = declare_parameter<double>("lateral_turn_thresh", 0.05);
    shaper_.lateral_yaw_kp = declare_parameter<double>("lateral_yaw_kp", 0.6);
    shaper_.lateral_yaw_max = declare_parameter<double>("lateral_yaw_max", 0.4);
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
      target_pos_[i] = default_pose[i];
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

    // ------------------------- Landing assist (gait) ----------------------
    // Not yet fixing anything conclusively -- see plan doc. Disabled by
    // default; flip on only after verify_locomotion.sh + per-leg duty-cycle
    // checks pass with it enabled.
    //
    // v1 (phase-gated, off gait_phase/gait_offsets): dropped -- 3 calibration
    // trials found no clean swing-vs-settle phase plateau to fit a touchdown
    // window from.
    // v2 (phase-free, velocity-proxy "settled"): tested end-to-end (5 trials);
    // net displacement was statistically unchanged from baseline. Its
    // velocity-only "settled" test also had a known blind spot: a leg that's
    // frozen-but-ungrounded has LOW velocity, so it read as "settled" and
    // never got assist -- exactly the failure mode most needing correction.
    // v3 (this version, real contact): "settled" is now ground-truth contact
    // from foot_contact_[] (big_bertha.gazebo.xacro's per-foot contact
    // sensors), fixing that blind spot -- a leg is "settled" only if its
    // primary foot collision is actually touching something. Mechanism is
    // otherwise unchanged: track how long a foot has stayed OUT of contact
    // continuously; once that exceeds landing_assist_stuck_time_thresh_, pull
    // the target toward landing_assist_land_pos_ (default 2.0 =
    // default_joint_pos's own trained ankle rest value, not a fit-from-data
    // guess). Resets the instant contact is regained.
    landing_assist_enabled_ = declare_parameter<bool>("landing_assist_enabled", false);
    landing_assist_land_pos_ = declare_parameter<double>("landing_assist_land_pos", 2.0);
    landing_assist_stuck_time_thresh_ =
      declare_parameter<double>("landing_assist_stuck_time_thresh", 0.5);
    landing_assist_rate_ = declare_parameter<double>("landing_assist_rate", 1.0);
    landing_assist_max_bias_ = declare_parameter<double>("landing_assist_max_bias", 0.2);
    if (landing_assist_land_pos_ + landing_assist_max_bias_ > joint_limit_ - 0.05) {
      landing_assist_max_bias_ = std::max(0.0, joint_limit_ - 0.05 - landing_assist_land_pos_);
      RCLCPP_WARN(
        get_logger(), "landing_assist_max_bias clamped to %.3f rad to stay clear of joint_limit_",
        landing_assist_max_bias_);
    }
    debug_log_gait_ = declare_parameter<bool>("debug_log_gait", false);
    debug_log_path_ = declare_parameter<std::string>("debug_log_path", "/tmp/gait_debug.csv");

    // gait_offsets[foot] (foot=0..3 -> leg1..leg4, Isaac's _feet_ids order) is
    // a DIFFERENT index space than target_pos_/obs_.joint_pos/joint_vel
    // (joint_names_ order). Derive the permutation from joint_index_ by name
    // rather than hardcoding it -- self-correcting if joint_names_ ever
    // changes, and fails safe (assist stays off) if a name lookup misses.
    {
      static const std::array<std::string, bbpc::kNumFeet> kAnkleJointByFoot{
        "Revolute_115", "Revolute_118", "Revolute_121", "Revolute_112"};  // leg1..leg4 ankles
      landing_assist_ready_ = true;
      for (int foot = 0; foot < bbpc::kNumFeet; ++foot) {
        auto it = joint_index_.find(kAnkleJointByFoot[foot]);
        if (it == joint_index_.end()) {
          RCLCPP_ERROR(
            get_logger(), "landing_assist: ankle joint '%s' not in joint_names; assist disabled",
            kAnkleJointByFoot[foot].c_str());
          landing_assist_ready_ = false;
          continue;
        }
        ankle_idx_for_foot_[foot] = it->second;
      }
      RCLCPP_INFO(
        get_logger(), "landing_assist ankle map: leg1->%d leg2->%d leg3->%d leg4->%d (enabled=%s)",
        ankle_idx_for_foot_[0], ankle_idx_for_foot_[1], ankle_idx_for_foot_[2],
        ankle_idx_for_foot_[3],
        (landing_assist_enabled_ && landing_assist_ready_) ? "true" : "false");
    }
    if (debug_log_gait_) {
      debug_log_stream_.open(debug_log_path_, std::ios::out | std::ios::trunc);
      if (debug_log_stream_.is_open()) {
        debug_log_stream_ << "t,gait_phase,"
                              "local_phase_leg1,local_phase_leg2,local_phase_leg3,local_phase_leg4,"
                              "pos_leg1,pos_leg2,pos_leg3,pos_leg4,"
                              "vel_leg1,vel_leg2,vel_leg3,vel_leg4,"
                              "contact_leg1,contact_leg2,contact_leg3,contact_leg4,"
                              // ROS index order: idx0-3 hip, 4-7 knee, 8-11 ankle,
                              // each x [leg4,leg1,leg2,leg3] (see ros2_control.yaml).
                              "j0,j1,j2,j3,j4,j5,j6,j7,j8,j9,j10,j11,cmd_vx,cmd_wz\n";
      } else {
        RCLCPP_ERROR(
          get_logger(), "debug_log_gait: failed to open '%s'", debug_log_path_.c_str());
      }
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
    // Ground-truth per-foot contact for the landing assist (see
    // big_bertha.gazebo.xacro's foot contact sensors + ros_gz_bridge.yaml).
    // foot index here is the same leg1..leg4 (arm_c_N_1) space as
    // obs_.gait_offsets/ankle_idx_for_foot_, NOT the joint_names order.
    static const std::array<std::string, bbpc::kNumFeet> kContactTopicByFoot{
      "/foot_contact/leg1", "/foot_contact/leg2", "/foot_contact/leg3", "/foot_contact/leg4"};
    for (int foot = 0; foot < bbpc::kNumFeet; ++foot) {
      contact_sub_[foot] = create_subscription<ros_gz_interfaces::msg::Contacts>(
        kContactTopicByFoot[foot], rclcpp::SensorDataQoS(),
        [this, foot](const ros_gz_interfaces::msg::Contacts::SharedPtr m) {
          std::lock_guard<std::mutex> lk(state_mutex_);
          foot_contact_[foot] = !m->contacts.empty();
        });
    }
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

    const auto period = std::chrono::duration<double>(1.0 / pd_rate_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&PolicyControllerNode::control_loop, this));

    RCLCPP_INFO(
      get_logger(),
      "policy_controller up: rate=%.1f Hz, scale=%.2f (hip=%.2f knee=%.2f ankle=%.2f), enabled=%s",
      control_rate_, action_scale_, hip_action_scale_, knee_action_scale_, ankle_action_scale_,
      enabled_ ? "true" : "false");
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
    // Raw command; clamping + heading hold happen in update_commands().
    cmd_vx_ = msg->linear.x;
    cmd_vy_ = msg->linear.y;
    cmd_wz_ = msg->angular.z;
    last_cmd_time_ = now();
  }

  // Build obs_.commands via CommandShaper. Caller holds state_mutex_.
  void update_commands()
  {
    const bool stale = cmd_timeout_ > 0.0 && (now() - last_cmd_time_).seconds() > cmd_timeout_;
    const auto cmd = shaper_.shape(
      cmd_vx_, cmd_vy_, cmd_wz_, stale, have_odom_yaw_, current_x_, current_y_, current_yaw_,
      1.0 / control_rate_);
    obs_.commands = {cmd.vx, cmd.vy, cmd.yaw};
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
      // Measured elapsed time since the last decimated policy step, NOT a
      // hardcoded 1/control_rate_. Under use_sim_time, gz's /clock can
      // arrive in coarse jumps when physics falls behind real time, and
      // rclcpp's sim-time wall timer then fires this callback in rapid
      // bursts to catch up -- confirmed empirically: with decimation
      // requiring 4 raw pd_rate_ ticks (nominal 20 ms at 200 Hz), 84% of
      // decimated steps were logged under 15 ms apart and 66% under 10 ms,
      // which is only possible if the raw timer is firing far faster than
      // 200 Hz in bursts. Feeding advance_clock a fixed 20 ms on every one
      // of those burst calls ran the gait clock at ~2.3x the intended
      // cadence (measured commanded cadence 3.25 Hz vs the ~1.4 Hz
      // gait_boost_max cap), desynchronizing the trained gait phase from
      // real elapsed time -- a strong candidate for the phase-incoherent
      // foot motion and weak/misaligned per-leg propulsion windows seen in
      // the FK-based foot-tip trajectory analysis (see plan doc). Cap only
      // the upper end (startup / a long disable-then-reenable gap); a
      // small measured dt from a burst tick is the correct value and must
      // pass through uncapped -- that's the actual fix.
      double policy_step_dt = 1.0 / control_rate_;
      {
        const rclcpp::Time step_now = now();
        if (last_policy_step_time_.nanoseconds() > 0) {
          const double measured = (step_now - last_policy_step_time_).seconds();
          if (measured > 0.0) {
            policy_step_dt = std::min(measured, 4.0 / control_rate_);
          }
        }
        last_policy_step_time_ = step_now;
      }
      {
        std::lock_guard<std::mutex> lk(state_mutex_);
        update_commands();
        // Clock advances every policy step, moving or idle, mirroring
        // big_bertha_env.py's _pre_physics_step.
        obs_.advance_clock(obs_.commands[0], obs_.commands[2], policy_step_dt);
        // Gate the gait when neither forward nor turn is commanded (the policy
        // cannot stand still). The |wz| half matters: gating on vx alone would
        // swallow the trained turn-in-place commands.
        // Magnitude on vx: a reverse command is motion too. Testing vx > thresh
        // gated the gait off for every negative command, so even once the
        // clamp allowed reverse through, the robot stood still in stance.
        moving = std::abs(obs_.commands[0]) > shaper_.stand_vx_thresh ||
                 std::abs(obs_.commands[2]) > stand_yaw_thresh_;
        if (!moving) {
          // Landing assist is inert while idle/holding stance; reset so the
          // next moving transition doesn't inherit a stale bias or a stale
          // stuck-time count from before the robot stopped.
          landing_bias_.fill(0.0);
          unsettled_time_.fill(0.0);
          // Hold the default stance; with station_keep the hip-bias steering
          // stays active to hold the latched heading against contact creep.
          if (!shaper_.station_keep) {
            shaper_.steer_cmd = 0.0;
          }
          for (int i = 0; i < bbpc::kNumJoints; ++i) {
            double t = obs_.default_joint_pos[i];
            if (shaper_.station_keep && i < 4) {  // hips: hold heading via steering bias
              t += debug_hip_bias_[i] + shaper_.steer_cmd * hip_steer_sign_[i];
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
            // Per-group scale (idx 0-3 hip, 4-7 knee, 8-11 ankle) -- see the
            // param declarations above for why a uniform scale is wrong.
            const double group_scale =
              (i < 4) ? hip_action_scale_ : (i < 8) ? knee_action_scale_ : ankle_action_scale_;
            double t = group_scale * a + obs_.default_joint_pos[i];
            if (i < 4) {  // hips: inject differential-stride steering + debug bias
              t += debug_hip_bias_[i] + shaper_.steer_cmd * hip_steer_sign_[i];
            }
            target_pos_[i] = std::clamp(t, -joint_limit_, joint_limit_);
            norm_sq += a * a;
            obs_.prev_actions[i] = static_cast<float>(a);
          }

          // Per-foot local gait phase (fmod(gait_phase + offset, 1)) -- no
          // longer used by the assist itself (see the comment above the
          // landing_assist_* param block for why), kept only as diagnostic
          // context in the calibration logger below.
          std::array<double, bbpc::kNumFeet> local_phase{};
          if (landing_assist_ready_) {
            for (int foot = 0; foot < bbpc::kNumFeet; ++foot) {
              local_phase[foot] = std::fmod(obs_.gait_phase + obs_.gait_offsets[foot], 1.0);
            }
          }

          if (landing_assist_enabled_ && landing_assist_ready_) {
            const double dt = policy_step_dt;  // same measured dt as the gait clock, not a fixed nominal
            for (int foot = 0; foot < bbpc::kNumFeet; ++foot) {
              const int idx = ankle_idx_for_foot_[foot];
              const bool settled = foot_contact_[foot];

              if (settled) {
                // Ground-truth contact: leg found its own footing (or is
                // mid-swing and briefly grazed something) -- stop the clock
                // and release any assist immediately.
                unsettled_time_[foot] = 0.0;
                landing_bias_[foot] = 0.0;
              } else {
                unsettled_time_[foot] += dt;
                if (unsettled_time_[foot] > landing_assist_stuck_time_thresh_) {
                  // Error-driven, rate-limited pull toward land_pos_: cannot
                  // overshoot past it by construction. Hard-capped as a
                  // second line of defense (see the startup clamp on
                  // max_bias_ too).
                  const double error = landing_assist_land_pos_ - target_pos_[idx];
                  const double step =
                    std::clamp(error, -landing_assist_rate_ * dt, landing_assist_rate_ * dt);
                  landing_bias_[foot] = std::clamp(
                    landing_bias_[foot] + step, -landing_assist_max_bias_,
                    landing_assist_max_bias_);
                }
                // else: still within the normal-swing grace period, no bias yet.
              }
              target_pos_[idx] =
                std::clamp(target_pos_[idx] + landing_bias_[foot], -joint_limit_, joint_limit_);
            }
          }

          if (debug_log_gait_ && landing_assist_ready_ && debug_log_stream_.is_open()) {
            debug_log_stream_ << now().seconds() << ',' << obs_.gait_phase;
            for (int foot = 0; foot < bbpc::kNumFeet; ++foot) {
              debug_log_stream_ << ',' << local_phase[foot];
            }
            for (int foot = 0; foot < bbpc::kNumFeet; ++foot) {
              debug_log_stream_ << ',' << obs_.joint_pos[ankle_idx_for_foot_[foot]];
            }
            for (int foot = 0; foot < bbpc::kNumFeet; ++foot) {
              debug_log_stream_ << ',' << obs_.joint_vel[ankle_idx_for_foot_[foot]];
            }
            for (int foot = 0; foot < bbpc::kNumFeet; ++foot) {
              debug_log_stream_ << ',' << (foot_contact_[foot] ? 1 : 0);
            }
            // Full 12-joint raw positions (ROS index order: idx0-3 hip,
            // 4-7 knee, 8-11 ankle, each x[leg4,leg1,leg2,leg3]) so foot-tip
            // world/base-relative trajectories can be reconstructed offline
            // via pure joint-angle FK -- no live TF, no contact-sensor mesh
            // ambiguity.
            for (int i = 0; i < bbpc::kNumJoints; ++i) {
              debug_log_stream_ << ',' << obs_.joint_pos[i];
            }
            debug_log_stream_ << ',' << obs_.commands[0] << ',' << obs_.commands[2] << '\n';
          }
        }
        last_action_norm_ = std::sqrt(norm_sq);
      }  // end if (moving)
    }

    // PD step every tick: tau = Kp*(q_des - q) - Kd*qd, torque-limited.
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
  // Tracks real elapsed time between decimated policy steps, so the gait
  // clock advances by what actually elapsed instead of an assumed fixed
  // 1/control_rate_ -- see the comment at the decimated-step dt calc.
  rclcpp::Time last_policy_step_time_{0, 0, RCL_ROS_TIME};

  // Params
  std::string model_path_;
  double action_scale_{0.25};
  double hip_action_scale_{0.25};
  double knee_action_scale_{0.25};
  double ankle_action_scale_{0.25};
  double control_rate_{50.0};
  double cmd_timeout_{0.5};
  double joint_limit_{3.14159};
  bool use_effort_{false};
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
  bool enabled_{true};

  // Outer-loop command shaping (envelope clamps, heading/lateral/position
  // hold, PI hip-bias steering) -- see command_shaper.hpp.
  bbpc::CommandShaper shaper_;
  double cmd_vx_{0.0};
  double cmd_vy_{0.0};
  double cmd_wz_{0.0};
  double current_x_{0.0};
  double current_y_{0.0};
  double current_yaw_{0.0};
  bool have_odom_yaw_{false};
  std::vector<double> hip_steer_sign_{1.0, 1.0, 1.0, 1.0};
  std::array<double, 4> debug_hip_bias_{};
  double stand_yaw_thresh_{0.05};

  // Landing assist (v3, real contact): stuck-time-gated ankle-target
  // correction for a leg whose foot isn't in contact with the ground. See
  // plan doc for design history (v1 phase-gated, v2 velocity-proxy-gated --
  // both dropped). "Settled" is now ground-truth contact from
  // foot_contact_[], not a velocity/position proxy, which fixes v2's known
  // blind spot (a frozen-but-ungrounded leg has low velocity but is NOT in
  // contact, so it now correctly triggers the assist).
  bool landing_assist_enabled_{false};
  double landing_assist_land_pos_{2.0};
  double landing_assist_stuck_time_thresh_{0.5};
  double landing_assist_rate_{1.0};
  double landing_assist_max_bias_{0.2};
  std::array<double, bbpc::kNumFeet> landing_bias_{};
  std::array<double, bbpc::kNumFeet> unsettled_time_{};
  std::array<int, bbpc::kNumFeet> ankle_idx_for_foot_{};
  bool landing_assist_ready_{false};
  std::array<bool, bbpc::kNumFeet> foot_contact_{};

  // Temporary calibration logger (debug_log_gait param) -- not part of the
  // shipped behavior, only used to fit the landing_assist_* thresholds above.
  bool debug_log_gait_{false};
  std::string debug_log_path_;
  std::ofstream debug_log_stream_;

  // ROS
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_pub_;
  rclcpp::Publisher<spider_msgs::msg::PolicyStatus>::SharedPtr status_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr hip_bias_sub_;
  std::array<rclcpp::Subscription<ros_gz_interfaces::msg::Contacts>::SharedPtr, bbpc::kNumFeet>
    contact_sub_;
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
