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

#include "big_bertha_policy_controller/command_shaper.hpp"
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

namespace big_bertha_policy_controller
{

class PolicyControllerNode : public rclcpp::Node
{
public:
  explicit PolicyControllerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("policy_controller", options)
  {
    // ----------------------------- Parameters ----------------------------
    model_path_ = declare_parameter<std::string>("model_path", "");
    action_scale_ = declare_parameter<double>("action_scale", 0.25);
    // Knees and ankles get their own amplitude. Measured on the URDF at the
    // default stance, the hips produce 0.0 mm of foot lift per rad (their axis
    // is vertical there, so they only sweep), while the knees give -50 mm/rad
    // and the ankles +26 mm/rad. Raising all twelve equally therefore spends
    // the servo range on the joints that cannot lift, and the hips are the
    // joints with the least range left because they also carry steer_cmd.
    // Defaults to action_scale, so leaving it unset changes nothing.
    lift_action_scale_ = declare_parameter<double>("lift_action_scale", action_scale_);
    control_rate_ = declare_parameter<double>("control_rate", 50.0);
    enabled_ = declare_parameter<bool>("start_enabled", true);
    cmd_timeout_ = declare_parameter<double>("cmd_vel_timeout", 0.5);
    joint_limit_ = declare_parameter<double>("joint_limit", 3.14159);
    action_clip_ = declare_parameter<double>("action_clip", 1.0);
    // All command shaping (envelope clamps, heading/lateral hold, position
    // hold, PI hip-bias steering) lives in CommandShaper -- header-only and
    // unit-tested; this block only maps ROS params onto it.
    shaper_.max_lin_vel_x = declare_parameter<double>("max_lin_vel_x", 0.3);
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
    steer_rate_limit_ = declare_parameter<double>("steer_rate_limit", 0.01);
    hip_steer_sign_ =
      declare_parameter<std::vector<double>>("hip_steer_sign", {1.0, 1.0, 1.0, 1.0});
    if (hip_steer_sign_.size() != 4) {
      RCLCPP_WARN(
        get_logger(), "'hip_steer_sign' has %zu entries, expected 4; using defaults",
        hip_steer_sign_.size());
      hip_steer_sign_ = {1.0, 1.0, 1.0, 1.0};
    }
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
    // In-node effort-PD is UNUSED in the shipped path: JointEffortPdController
    // does the PD and reads this topic as POSITION targets.
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
      "joint_names", {"Revolute_110", "Revolute_113", "Revolute_116", "Revolute_119",
                      "Revolute_111", "Revolute_114", "Revolute_117", "Revolute_120",
                      "Revolute_112", "Revolute_115", "Revolute_118", "Revolute_121"});
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
      "/odom", rclcpp::QoS(1),
      std::bind(&PolicyControllerNode::on_odom, this, std::placeholders::_1));
    imu_topic_ = declare_parameter<std::string>("imu_topic", "/imu");
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, rclcpp::SensorDataQoS(),
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
    // Raw command; clamping + heading hold happen in update_commands().
    cmd_vx_ = msg->linear.x;
    cmd_vy_ = msg->linear.y;
    cmd_wz_ = msg->angular.z;
    raw_cmd_wz_ = msg->angular.z;  // Save raw wz for moving gate check
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
      {
        std::lock_guard<std::mutex> lk(state_mutex_);
        update_commands();
        // Clock advances every policy step, moving or idle, mirroring
        // big_bertha_env.py's _pre_physics_step.
        obs_.advance_clock(obs_.commands[0], obs_.commands[2], 1.0 / control_rate_);
        // Gate the gait when neither forward nor turn is commanded (the policy
        // cannot stand still). The |wz| half matters: gating on vx alone would
        // swallow the trained turn-in-place commands.
        // Check raw_cmd_wz (before heading correction) not obs_.commands[2]
        // (after correction). With heading_kp=2.0 and thresh=0.05, any heading
        // error >0.025 rad would trigger walking instead of standing correction.
        moving =
          obs_.commands[0] > shaper_.stand_vx_thresh || std::abs(raw_cmd_wz_) > stand_yaw_thresh_;
        if (!moving) {
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
            // i < 4 are the hips; 4..11 the knees and ankles, which do all
            // the lifting. See lift_action_scale_ above.
            const double scale = (i < 4) ? action_scale_ : lift_action_scale_;
            double t = scale * a + obs_.default_joint_pos[i];
            if (i < 4) {  // hips: inject differential-stride steering + debug bias
              t += debug_hip_bias_[i] + shaper_.steer_cmd * hip_steer_sign_[i];
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
  double lift_action_scale_{0.25};
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
  double raw_cmd_wz_{0.0};  // Raw wz before heading correction, for moving gate
  double current_x_{0.0};
  double current_y_{0.0};
  double current_yaw_{0.0};
  bool have_odom_yaw_{false};
  double steer_rate_limit_{0.01};
  std::vector<double> hip_steer_sign_{1.0, 1.0, 1.0, 1.0};
  std::array<double, 4> debug_hip_bias_{};
  double stand_yaw_thresh_{0.05};

  // ROS
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_pub_;
  rclcpp::Publisher<spider_msgs::msg::PolicyStatus>::SharedPtr status_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  std::string imu_topic_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr hip_bias_sub_;
  rclcpp::Service<spider_msgs::srv::SetPolicyEnabled>::SharedPtr set_enabled_srv_;
  rclcpp::Service<spider_msgs::srv::LoadPolicy>::SharedPtr load_policy_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace big_bertha_policy_controller

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(big_bertha_policy_controller::PolicyControllerNode)
