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
// Subscribes to /odom, /imu, /joint_states, /cmd_vel; assembles the 48-d
// observation (PLAN.md section 2); runs the exported PPO policy via ONNX
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
    // /cmd_vel clamp to the policy's trained command envelope (yaw-for-nav
    // fix).
    max_lin_vel_x_ = declare_parameter<double>("max_lin_vel_x", 0.3);
    max_lin_vel_y_ = declare_parameter<double>("max_lin_vel_y", 0.05);
    max_yaw_rate_ = declare_parameter<double>("max_yaw_rate", 0.15);
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
      "default_joint_pos", {0.0, 0.5, 0.0, 0.0, 0.5, 0.0, 0.0, 0.5, 0.0, 0.0, 0.5, 0.0});

    for (int i = 0; i < bbpc::kNumJoints && i < static_cast<int>(default_pose.size()); ++i) {
      obs_.default_joint_pos[i] = default_pose[i];
      target_pos_[i] = default_pose[i];
    }
    for (size_t i = 0; i < joint_names_.size(); ++i) {
      joint_index_[joint_names_[i]] = static_cast<int>(i);
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
    // Clamp to the trained command envelope (forward-only vx, narrow vy/yaw) so
    // Nav2's out-of-distribution commands (linear.x ~0.37, yaw ~0.44) don't
    // make the policy flail. Turns become wider/slower but stay executable.
    obs_.commands = {
      std::clamp(msg->linear.x, 0.0, max_lin_vel_x_),
      std::clamp(msg->linear.y, -max_lin_vel_y_, max_lin_vel_y_),
      std::clamp(msg->angular.z, -max_yaw_rate_, max_yaw_rate_)};
    last_cmd_time_ = now();
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
      {
        std::lock_guard<std::mutex> lk(state_mutex_);
        if (cmd_timeout_ > 0.0 && (now() - last_cmd_time_).seconds() > cmd_timeout_) {
          obs_.commands = {0.0, 0.0, 0.0};
        }
        input = obs_.build();
      }
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
          RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "inference error: %s", e.what());
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
          target_pos_[i] = std::clamp(t, -joint_limit_, joint_limit_);
          norm_sq += a * a;
          obs_.prev_actions[i] = static_cast<float>(a);
        }
      }
      last_action_norm_ = std::sqrt(norm_sq);
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
