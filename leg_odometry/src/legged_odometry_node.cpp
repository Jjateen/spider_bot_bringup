// Copyright 2026 Big Bertha Authors
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

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

using namespace std::chrono_literals;

namespace leg_odometry
{

class LeggedOdometryNode : public rclcpp::Node
{
public:
  explicit LeggedOdometryNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("legged_odometry", options)
  {
    joint_names_ = declare_parameter<std::vector<std::string>>("joint_names", kDefaultJointNames);
    default_joint_pos_ =
      declare_parameter<std::vector<double>>("default_joint_pos", kDefaultJointPos);
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    publish_tf_ = declare_parameter<bool>("publish_tf", false);
    velocity_source_ = declare_parameter<std::string>("velocity_source", "imu_dead_reckon");
    // Replaces drift_damping, which was declared but never applied, so the
    // config advertised a drift control that did not exist. ZUPT owns drift;
    // this bounds the single bad sample ZUPT cannot catch (see the integrator).
    max_dead_reckon_speed_ = declare_parameter<double>("max_dead_reckon_speed", 0.45);
    stationary_joint_vel_threshold_ =
      declare_parameter<double>("stationary_joint_vel_threshold", 0.02);
    stationary_accel_threshold_ = declare_parameter<double>("stationary_accel_threshold", 0.5);
    stationary_hold_s_ = declare_parameter<double>("stationary_hold_s", 0.4);
    servo_tau_ = declare_parameter<double>("servo_tau", 0.06);
    // Matches hardware_bridge's max_joint_rate_rad_s -- see servo_tau's own
    // comment and legged_odometry.yaml for the full rationale.
    servo_max_rate_rad_s_ = declare_parameter<double>("servo_max_rate_rad_s", 3.0);
    // Matches hardware_bridge.yaml's command_rate_hz: the real chain
    // throttles the policy's raw ~200Hz stream down to this rate before its
    // EWMA+slew ever run (the PCA9685 only refreshes this fast). This node's
    // own EWMA+slew below must run on that same throttled cadence, not the
    // raw policy rate -- see on_cmd()'s throttle gate for what breaks without it.
    command_rate_hz_ = declare_parameter<double>("command_rate_hz", 50.0);
    cmd_throttle_period_ = rclcpp::Duration::from_seconds(1.0 / command_rate_hz_);
    // High-pass leak on the specific force. Estimates the slowly-varying DC
    // offset (a residual accel bias the bridge calibration missed, e.g. because
    // the robot was tilted) and subtracts it, so it cannot integrate into a
    // runaway /odom velocity. Gait acceleration oscillates about zero with a
    // ~1.5 s period, so its per-cycle integral nets out even under a short
    // tau; 3 s converges fast enough that a bad calibration stops pumping
    // velocity within a few seconds instead of holding it near the speed
    // clamp for the better part of a minute. 0 disables the leak.
    bias_leak_tau_ = declare_parameter<double>("bias_leak_tau", 3.0);
    // Stationary-gated leak on gyro-z, separate from bias_leak_tau above:
    // that one runs continuously because gait acceleration genuinely
    // oscillates about zero, so a short tau nets out real signal and rejects
    // bias. Yaw rate during a genuine sustained turn is NOT zero-mean, so the
    // same continuous-leak trick would reject real turning as if it were
    // bias. Instead, only refine gyro_bias_z_est_ while is_robot_stationary()
    // says the robot is confirmed still (true gyro-z reads exactly 0 then);
    // hold it during any motion, turns included. Without this, an upstream
    // residual gyro bias (no magnetometer on this board, so nothing else
    // corrects yaw) integrates straight into /odom's heading with nothing to
    // reject it.
    yaw_bias_leak_enabled_ = declare_parameter<bool>("yaw_bias_leak_enabled", true);
    yaw_bias_leak_tau_ = declare_parameter<double>("yaw_bias_leak_tau", 2.0);
    // Hard stationary gate on command staleness. The policy itself drops to
    // idle once /cmd_vel goes quiet (its timeout is 0.5 s), so a silent topic
    // means the robot is physically uncommanded and any dead-reckoned velocity
    // is phantom — a tilted calibration integrates straight into it and pins
    // at the clamp while ZUPT stays blocked by the biased raw IMU reading.
    // When stale, velocity is forced to zero and position anchored no matter
    // what the IMU says. Never-received counts as stale, which also covers
    // boots where teleop never connects. Must exceed the policy timeout so a
    // momentary gap between teleop packets never reads as "uncommanded".
    cmd_vel_stale_s_ = declare_parameter<double>("cmd_vel_stale_s", 1.0);
    publish_joint_states_ = declare_parameter<bool>("publish_joint_states", true);

    // IMU scaling: multiplier on world-frame accel before dead-reckon integration.
    // <1.0 reduces IMU contribution; >1.0 if IMU under-reports; 0 zeros accel.
    imu_accel_scale_ = declare_parameter<double>("imu_accel_scale", 1.0);
    imu_gyro_scale_ = declare_parameter<double>("imu_gyro_scale", 1.0);

    // Lidar-aided correction: monitor map→odom TF changes from slam_toolbox / AMCL
    // and pull the dead-reckoned pose toward the scan-matched estimate.
    // Gain = 0: pure IMU dead-reckon. Gain = 1: full lidar trust.
    lidar_correction_enabled_ = declare_parameter<bool>("lidar_correction_enabled", true);
    lidar_gain_xy_ = declare_parameter<double>("lidar_gain_xy", 0.15);
    lidar_gain_yaw_ = declare_parameter<double>("lidar_gain_yaw", 0.20);
    max_correction_step_m_ = declare_parameter<double>("max_correction_step_m", 0.05);
    max_correction_step_yaw_rad_ = declare_parameter<double>("max_correction_step_yaw_rad", 0.05);
    correction_timeout_s_ = declare_parameter<double>("correction_timeout_s", 1.0);

    if (velocity_source_ == "leg_kinematics") {
      RCLCPP_WARN(
        get_logger(), "leg_kinematics mode is a stub — velocity estimates will be inaccurate");
    }
    RCLCPP_INFO(
      get_logger(), "velocity source: %s, servo_tau=%.3f, ZUPT hold=%.2fs",
      velocity_source_.c_str(), servo_tau_, stationary_hold_s_);

    last_joint_positions_ = default_joint_pos_;
    filtered_positions_ = default_joint_pos_;
    node_start_time_ = now();

    cmd_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      "/position_controller/commands", rclcpp::QoS(1),
      std::bind(&LeggedOdometryNode::on_cmd, this, std::placeholders::_1));

    imu_topic_ = declare_parameter<std::string>("imu_topic", "/imu");
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_, rclcpp::SensorDataQoS(),
      std::bind(&LeggedOdometryNode::on_imu, this, std::placeholders::_1));

    // Only the arrival time of /cmd_vel matters here; the value stays with the
    // policy, which owns shaping it into joint targets.
    twist_cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", rclcpp::QoS(1),
      std::bind(&LeggedOdometryNode::on_twist_cmd, this, std::placeholders::_1));

    joint_state_pub_ =
      create_publisher<sensor_msgs::msg::JointState>("/joint_states", rclcpp::QoS(1));
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/odom", rclcpp::QoS(1));

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

    // Initial /joint_states publish breaks the policy<->leg_odometry deadlock:
    // policy_controller gates /position_controller/commands on having seen a
    // /joint_states message, while this node only publishes on commands.
    // Re-emit the default pose every 200 ms until the first command arrives.
    // The stop condition is have_cmd_ alone, never a timeout: policy_controller
    // builds its Ort::Session before creating the /joint_states subscription,
    // and on the UNO Q's A35 that load takes longer than any fixed discovery
    // window, so a bounded one is missed every boot and the deadlock is
    // permanent. have_cmd_ latches, so steady state stays command-driven with
    // no interleaved timer stream.
    init_timer_ =
      create_wall_timer(200ms, std::bind(&LeggedOdometryNode::publish_initial_joint_states, this));

    // Dense odom->base_link tf (50 Hz) so slam/Nav2 always have a fresh odom
    // transform regardless of the IMU delivery rate (see broadcast_odom_tf).
    odom_tf_timer_ = create_wall_timer(20ms, [this]() { broadcast_odom_tf(this->now()); });
  }

private:
  void on_cmd(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    if (msg->data.size() != 12) return;
    have_cmd_ = true;

    std::lock_guard<std::mutex> lk(joint_mutex_);

    auto now = this->now();

    // Throttle to command_rate_hz, mirroring hardware_bridge_node.cpp's
    // on_cmd(): the real chain forwards at this rate (the PCA9685 only
    // refreshes this fast) and drops the rest, so the EWMA+slew model below
    // should run on THAT throttled stream, not the raw ~200Hz policy rate.
    // Without this, a brief (<20ms) commanded transient that a throttled
    // real chain would only partially see (or drop entirely) gets processed
    // here at full resolution, letting the model chase and overshoot toward
    // a target the real servo may never fully receive.
    if (last_cmd_time_.nanoseconds() > 0 && (now - last_cmd_time_) < cmd_throttle_period_) {
      return;
    }

    auto js = sensor_msgs::msg::JointState();
    js.header.stamp = now;
    js.name = joint_names_;

    double dt = 0.0;
    if (last_cmd_time_.nanoseconds() > 0) {
      dt = (now - last_cmd_time_).seconds();
    }

    // Cap dt ONCE, before deriving anything from it -- mirrors
    // servo_converter.hpp, which caps dt upfront and computes both its EWMA
    // alpha and its slew step from that single capped value. This node used
    // to cap dt for the slew step only and leave alpha computed from the RAW
    // dt: a late/gapped command (a stalled publisher, a scheduling hiccup)
    // then made alpha -> 1 (no smoothing at all), writing the almost-raw
    // target straight into filtered_positions_[i] -- the EWMA's persistent
    // state -- even though that same call's PUBLISHED position was correctly
    // slew-clamped. The next few calls then kept pulling toward that
    // corrupted internal state, producing a multi-sample slew-limited spike
    // toward an erroneous value that the policy read as real servo feedback.
    const bool first_call = !(dt > 1e-6);
    const double dt_capped = first_call ? 0.0 : std::min(dt, kMaxDt_);

    // 1st-order servo dynamics: EWMA filter simulates MG995's physical lag.
    // alpha = 1 - exp(-dt / tau) where tau matches the servo's closed-loop
    // response time (~0.09s, matching hardware_bridge's servo_converter.hpp
    // effective tau -- see servo_tau's own comment for why). This breaks the
    // positive-feedback loop caused by feeding commanded positions as "measured"
    // joint state (see ISSUES.md #15).
    const double alpha = first_call ? 1.0 : (1.0 - std::exp(-dt_capped / servo_tau_));

    // Slew limit, mirroring servo_converter.hpp's convert(): the EWMA alone
    // is not what the real chain does to the target before it reaches the
    // servo -- hardware_bridge additionally caps the rate of change at
    // servo_max_rate_rad_s. Without this, this node's feedback converges to
    // any commanded step faster than the real servo can move, independent of
    // how far tau alone is matched (see leg_odometry/docs/ for the measured
    // before/after: EWMA-only left leg_odometry's synthesized feedback
    // leading the real delivery by tens of ms; adding this closed nearly all
    // of that remaining gap).
    const double step = servo_max_rate_rad_s_ * dt_capped;

    for (size_t i = 0; i < 12; ++i) {
      double cmd_pos = msg->data[i];

      // EWMA: blend commanded position toward filtered position
      double smoothed_pos = alpha * cmd_pos + (1.0 - alpha) * filtered_positions_[i];
      filtered_positions_[i] = smoothed_pos;

      double limited_pos;
      if (first_call) {
        limited_pos = smoothed_pos;  // no prior pose to slew-limit from
      } else {
        limited_pos = std::clamp(
          smoothed_pos, last_joint_positions_[i] - step, last_joint_positions_[i] + step);
      }
      js.position.push_back(limited_pos);

      if (dt > 1e-6) {
        double vel = (limited_pos - last_joint_positions_[i]) / dt;
        js.velocity.push_back(vel);
        last_joint_velocities_[i] = vel;
      } else {
        js.velocity.push_back(0.0);
        last_joint_velocities_[i] = 0.0;
      }
      last_joint_positions_[i] = limited_pos;
    }
    last_cmd_time_ = now;

    if (publish_joint_states_) {
      joint_state_pub_->publish(js);
    }
  }

  void publish_initial_joint_states()
  {
    if (have_cmd_ || !publish_joint_states_) {
      init_timer_->cancel();
      return;
    }
    ++init_publishes_;

    auto js = sensor_msgs::msg::JointState();
    js.header.stamp = now();
    js.name = joint_names_;
    for (size_t i = 0; i < 12; ++i) {
      js.position.push_back(default_joint_pos_[i]);
      js.velocity.push_back(0.0);
    }
    joint_state_pub_->publish(js);
    if (init_publishes_ == 1) {
      RCLCPP_INFO(get_logger(), "published initial joint states (default pose)");
    }
  }

  void on_twist_cmd(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    (void)msg;
    std::lock_guard<std::mutex> lk(pose_mutex_);
    last_twist_cmd_time_ = this->now();
  }

  void on_imu(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    auto now = this->now();

    if (last_imu_time_.nanoseconds() == 0) {
      last_imu_time_ = now;
      last_orientation_.setValue(
        msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
      last_orientation_.normalize();
      // Seed the self-integrated yaw from whatever upstream reports at boot,
      // rather than 0 -- see yaw_estimate_'s own comment for why this node
      // integrates yaw itself instead of trusting the incoming quaternion's
      // yaw component on every cycle.
      double seed_roll, seed_pitch, seed_yaw;
      tf2::Matrix3x3(last_orientation_).getRPY(seed_roll, seed_pitch, seed_yaw);
      yaw_estimate_ = seed_yaw;
      return;
    }

    double dt = (now - last_imu_time_).seconds();
    // dt <= 0 rejects reordered/stale stamps; dt > 1.0 rejects long gaps (startup
    // or a dropped stream). Was 0.1, which silently dropped everything on real
    // hardware: the router/bridge delivers IMU at ~3.5 Hz under load (dt ~0.29 s),
    // so no /odom was ever published and the odom frame never appeared.
    if (dt <= 0.0 || dt > 1.0) {
      last_imu_time_ = now;
      return;
    }

    tf2::Quaternion orientation(
      msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
    orientation.normalize();

    tf2::Vector3 velocity;
    tf2::Vector3 position;

    if (velocity_source_ == "leg_kinematics") {
      compute_leg_kinematics_velocity(orientation, orientation, dt, velocity, position);
    } else {
      compute_imu_dead_reckon(msg, orientation, dt, velocity, position);
    }

    // The robot is planar and every consumer of this odom (slam_toolbox, Nav2,
    // both costmaps) is 2D, so z carries no information and integrating it only
    // banks error. It banked a lot: compute_imu_dead_reckon subtracts a fixed
    // 9.81 from an accelerometer that reads 11.07 at rest on this board, which
    // leaves ~1.26 m/s^2 standing in world z. The old guard clamped position at
    // the floor but never above it, and never clamped vz at all, so base_link
    // climbed away from the map plane without bound. Pin both once here, for
    // whichever velocity source ran, instead of inside each of them.
    velocity.setZ(0.0);
    position.setZ(0.0);

    // Computed once here and reused below for the ZUPT gate, rather than
    // calling is_robot_stationary() twice -- it also gates the yaw-bias leak.
    bool stationary = is_robot_stationary(msg);

    // Stationary-gated gyro-yaw-bias leak (see yaw_bias_leak_tau's own
    // comment for the full rationale). Then integrate yaw itself from the
    // bias-corrected gyro-z, instead of trusting the incoming quaternion's
    // yaw component -- upstream (Madgwick or equivalent) has no magnetometer
    // on this board and no drift-bias correction of its own, so its yaw is
    // exactly as bias-prone as raw integration would be; doing the
    // integration here is what lets this leak reject that bias.
    if (yaw_bias_leak_enabled_ && stationary && dt > 0.0) {
      const double alpha = dt / (yaw_bias_leak_tau_ + dt);
      gyro_bias_z_est_ += (msg->angular_velocity.z - gyro_bias_z_est_) * alpha;
    }
    const double corrected_gz = msg->angular_velocity.z - gyro_bias_z_est_;
    yaw_estimate_ = std::atan2(
      std::sin(yaw_estimate_ + corrected_gz * dt), std::cos(yaw_estimate_ + corrected_gz * dt));
    {
      // Roll/pitch keep the upstream gravity reference (that part isn't
      // bias-prone the way free-running yaw is); only yaw is replaced.
      double roll, pitch, unused_yaw;
      tf2::Matrix3x3(orientation).getRPY(roll, pitch, unused_yaw);
      orientation.setRPY(roll, pitch, yaw_estimate_);
      orientation.normalize();
    }

    // Uncommanded gate: a silent /cmd_vel stream means nobody is driving (the
    // policy idles after 0.5 s of silence itself). Any velocity the integrator
    // still carries then is phantom — a tilted calibration pumps it straight
    // to the clamp and no IMU-based stationary check can be trusted, because
    // both its inputs (raw accel, joint motion from position_hold) can read
    // "moving" while the physical robot sits still. So staleness alone forces
    // the full ZUPT treatment: velocity zeroed, position anchored.
    bool uncommanded;
    {
      std::lock_guard<std::mutex> lk(pose_mutex_);
      uncommanded = last_twist_cmd_time_.nanoseconds() == 0 ||
                    (now - last_twist_cmd_time_).seconds() > cmd_vel_stale_s_;
    }

    // ZUPT: zero velocity when robot is stationary (all joints still, no linear
    // acceleration) or simply uncommanded. Without this the leaky integrator
    // bleeds steady walking velocity to zero and standing drift accumulates
    // unbounded.
    if (uncommanded || stationary) {
      velocity = tf2::Vector3(0.0, 0.0, 0.0);
      // Freeze the dead-reckoned position while stationary. Zeroing only velocity
      // is not enough: the world-frame accel still carries a small residual bias
      // (gravity projected through the IMU orientation), so position recomputes
      // as last_position + vel*dt every sample and creeps linearly even at rest.
      // Anchor at the pose where the robot stopped and hold it until it moves.
      if (!zupt_active_) {
        zupt_anchor_ = position;
        zupt_active_ = true;
      } else {
        position = zupt_anchor_;
      }
      last_zupt_time_ = now;
    } else {
      zupt_active_ = false;
    }

    // Lidar-aided correction: pull the dead-reckoned pose toward the
    // scan-matched estimate (from slam_toolbox/AMCL map→odom TF).
    // Runs at TF publish rate (50 Hz) but only applies nonzero deltas
    // when slam processes a new scan (≥0.2 s apart, ≥0.1 m travel),
    // so it's cheap and non-circular: the delta originates from lidar
    // scan-matching against the environment, not from our own TF.
    apply_lidar_correction(now);

    auto odom = nav_msgs::msg::Odometry();
    odom.header.stamp = msg->header.stamp;
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id = base_frame_;

    odom.pose.pose.position.x = position.x();
    odom.pose.pose.position.y = position.y();
    odom.pose.pose.position.z = position.z();
    odom.pose.pose.orientation.x = orientation.x();
    odom.pose.pose.orientation.y = orientation.y();
    odom.pose.pose.orientation.z = orientation.z();
    odom.pose.pose.orientation.w = orientation.w();

    odom.twist.twist.linear.x = velocity.x();
    odom.twist.twist.linear.y = velocity.y();
    odom.twist.twist.linear.z = velocity.z();
    odom.twist.twist.angular.x = msg->angular_velocity.x;
    odom.twist.twist.angular.y = msg->angular_velocity.y;
    odom.twist.twist.angular.z = msg->angular_velocity.z * imu_gyro_scale_;

    // Dead-reckoned uncertainty grows with time since last ZUPT reset.
    // Position error from accelerometer bias (this IMU reads 11.07 vs 9.81 at
    // rest, leaving ~0.4 m/s² residual horizontal bias when tilted during
    // walking) integrates quadratically. Roll/pitch benefit from gravity
    // reference. Yaw uses node uptime (not ZUPT) since ZUPT constrains
    // velocity, not heading — yaw covariance grows unbounded forever.
    double dt_zupt = last_zupt_time_.nanoseconds() > 0 ? (now - last_zupt_time_).seconds() : 0.0;
    double dt_yaw = (now - node_start_time_).seconds();
    double pos_xy_cov = 0.01 + 0.1 * dt_zupt + 0.5 * dt_zupt * dt_zupt;
    double pos_z_cov = 0.01 + 0.05 * dt_zupt;
    double rp_cov = 0.01 + 0.005 * dt_zupt;
    double yaw_cov = 0.001 + 0.02 * dt_yaw;
    double vel_xy_cov = 0.1 + 0.1 * dt_zupt;
    double vel_z_cov = 0.1 + 0.05 * dt_zupt;

    odom.pose.covariance[0] = pos_xy_cov;
    odom.pose.covariance[7] = pos_xy_cov;
    odom.pose.covariance[14] = pos_z_cov;
    odom.pose.covariance[21] = rp_cov;
    odom.pose.covariance[28] = rp_cov;
    odom.pose.covariance[35] = yaw_cov;
    odom.twist.covariance[0] = vel_xy_cov;
    odom.twist.covariance[7] = vel_xy_cov;
    odom.twist.covariance[14] = vel_z_cov;

    odom_pub_->publish(odom);

    {
      std::lock_guard<std::mutex> lk(pose_mutex_);
      last_imu_time_ = now;
      last_orientation_ = orientation;
      last_velocity_ = velocity;
      last_position_ = position;
    }
  }

  // Publish odom -> base_link at a fixed high rate, decoupled from the IMU
  // callback. On hardware the IMU is delivered at only ~3.5 Hz effective, so a
  // tf stamped per-IMU-sample leaves gaps that slam_toolbox's scan message
  // filter can't cross (scans at 10 Hz fall in the future of the newest tf and
  // get dropped — "queue is full"). Re-broadcasting the latest pose at 50 Hz
  // gives the tf a dense timeline so every scan transforms cleanly.
  void broadcast_odom_tf(const rclcpp::Time & stamp)
  {
    if (!publish_tf_) {
      return;
    }
    tf2::Vector3 position;
    tf2::Quaternion orientation;
    {
      // Copy under the lock, broadcast outside it: a torn pose here would mix
      // the position of one IMU sample with the orientation of the next.
      std::lock_guard<std::mutex> lk(pose_mutex_);
      if (last_imu_time_.nanoseconds() == 0) {
        return;
      }
      position = last_position_;
      orientation = last_orientation_;
    }
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = stamp;
    tf.header.frame_id = odom_frame_;
    tf.child_frame_id = base_frame_;
    tf.transform.translation.x = position.x();
    tf.transform.translation.y = position.y();
    tf.transform.translation.z = position.z();
    tf.transform.rotation.x = orientation.x();
    tf.transform.rotation.y = orientation.y();
    tf.transform.rotation.z = orientation.z();
    tf.transform.rotation.w = orientation.w();
    tf_broadcaster_->sendTransform(tf);
  }

  // Lidar-aided correction: track changes in map→odom TF published by
  // slam_toolbox (mapping mode) or AMCL (known-map mode). The increment of
  // map→odom isolates exactly the drift accumulated in our dead-reckoned
  // odom since the previous TF update — slam recomputes
  //   map→odom = T_map_base(scanmatched) ∘ T_odom_base⁻¹
  // so if odom drifts forward by e, the next map→odom shift contains −e.
  // Applying that with gain < 1 pulls odom toward truth without the
  // double-counting trap that killed the previous EKF setup (which fused
  // /odom with the IMU that /odom was derived from).
  //
  // Works in both mapping (slam_toolbox) and localization (AMCL) modes —
  // both publish map→odom TF continuously. Inactive when no nav is running
  // (TF absent) — graceful fallback to pure dead-reckon.
  void apply_lidar_correction(const rclcpp::Time & /*now*/)
  {
    if (!lidar_correction_enabled_) return;

    geometry_msgs::msg::TransformStamped map_to_odom;
    try {
      // lookupTransform(target_frame, source_frame, stamp) returns
      // T_target_source. We want map→odom, so target=map, source=odom.
      // Using tf2::TimePointZero asks for the latest available transform.
      map_to_odom =
        tf_buffer_->lookupTransform("map", odom_frame_, tf2::TimePointZero, tf2::Duration(0));
    } catch (const tf2::TransformException & ex) {
      // Nav container not running or TF not yet available — correction
      // silently inactive. Common during bench work (with_nav:=false) and
      // at startup before slam_toolbox publishes its first map→odom.
      (void)ex;
      return;
    }

    tf2::Transform T_map_odom;
    T_map_odom.setOrigin(tf2::Vector3(
      map_to_odom.transform.translation.x, map_to_odom.transform.translation.y,
      map_to_odom.transform.translation.z));
    T_map_odom.setRotation(tf2::Quaternion(
      map_to_odom.transform.rotation.x, map_to_odom.transform.rotation.y,
      map_to_odom.transform.rotation.z, map_to_odom.transform.rotation.w));

    if (!have_last_map_odom_) {
      // First TF received — store baseline, no correction yet.
      last_map_to_odom_ = T_map_odom;
      have_last_map_odom_ = true;
      return;
    }

    // Delta = how much slam adjusted map→odom since the last sample.
    // If odom were perfect, delta ≈ identity. If odom over-reported forward
    // motion by e, delta contains −e (slam absorbs the drift into map→odom
    // to keep scan-matched base pose consistent with the map).
    tf2::Transform delta = T_map_odom * last_map_to_odom_.inverse();
    last_map_to_odom_ = T_map_odom;

    // Extract position error (planar: only x/y matter).
    tf2::Vector3 err_pos = delta.getOrigin();
    double err_yaw = 0.0;
    {
      double roll, pitch;
      delta.getBasis().getRPY(roll, pitch, err_yaw);
    }

    // Ignore near-zero corrections (TF republishes unchanged values at 50 Hz;
    // only nonzero deltas when slam processes a new scan).
    if (err_pos.length() < 1e-6 && std::abs(err_yaw) < 1e-6) return;

    // Clamp per-cycle magnitude: rejects slam_toolbox loop-closure jumps
    // (>0.5 m in one update) and AMCL re-initializations. The jump belongs
    // in map→odom, not our odom frame — absorbing it would teleport base_link
    // and break Nav2 costmaps.
    if (err_pos.length() > max_correction_step_m_) {
      err_pos *= max_correction_step_m_ / err_pos.length();
    }
    if (std::abs(err_yaw) > max_correction_step_yaw_rad_) {
      err_yaw = std::copysign(max_correction_step_yaw_rad_, err_yaw);
    }

    // Apply correction under pose_mutex_ (shared with on_imu and
    // broadcast_odom_tf in the composed container's MultiThreadedExecutor).
    {
      std::lock_guard<std::mutex> lk(pose_mutex_);

      last_position_.setX(last_position_.x() + lidar_gain_xy_ * err_pos.x());
      last_position_.setY(last_position_.y() + lidar_gain_xy_ * err_pos.y());

      // Yaw correction via slerp between current and corrected orientation.
      if (std::abs(err_yaw) > 1e-6) {
        double current_yaw, current_roll, current_pitch;
        tf2::Matrix3x3(last_orientation_).getRPY(current_roll, current_pitch, current_yaw);
        double target_yaw = current_yaw + lidar_gain_yaw_ * err_yaw;
        tf2::Quaternion corrected;
        corrected.setRPY(0.0, 0.0, target_yaw);
        last_orientation_ = corrected;  // pure yaw correction; roll/pitch zeroed
        // (planar robot — roll/pitch pinned by gravity reference).
      }

      // Dampen velocity toward corrected direction to prevent overshoot.
      // Gain > 0 means "trust the scan match more" so velocity should shrink
      // proportionally; the dead-reckon integrator rebuilds it from the
      // next IMU sample onward.
      last_velocity_ *= (1.0 - lidar_gain_xy_);
    }

    RCLCPP_DEBUG_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "lidar correction: err_xy=(%.4f, %.4f) m err_yaw=%.4f rad "
      "applied xy=%.4f yaw=%.4f",
      err_pos.x(), err_pos.y(), err_yaw, lidar_gain_xy_ * err_pos.length(),
      lidar_gain_yaw_ * err_yaw);
  }

  void compute_imu_dead_reckon(
    const sensor_msgs::msg::Imu::SharedPtr & msg, const tf2::Quaternion & orientation, double dt,
    tf2::Vector3 & velocity, tf2::Vector3 & position)
  {
    tf2::Vector3 accel_body(
      msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);

    tf2::Matrix3x3 rot(orientation);
    tf2::Vector3 accel_world = rot * accel_body;
    // Bridge now publishes specific force (gravity removed at calibration), so no
    // further gravity subtraction here.

    // IMU accel scaling: multiplier on world-frame accel before integration.
    // Controls how much the IMU drives dead-reckoned velocity. <1.0 reduces
    // over-reporting from residual bias; 0 zeros accel entirely.
    accel_world *= imu_accel_scale_;

    // Bias-leak: track the slow DC component of accel_world and subtract it.
    // A constant phantom bias (from a tilted calibration) is rejected; genuine
    // gait-driven acceleration oscillates far above the leak time constant and
    // is preserved. Without this, one bad calibration drives /odom at the
    // max_dead_reckon_speed clamp indefinitely.
    if (bias_leak_tau_ > 0.0 && dt > 0.0) {
      const double alpha = dt / (bias_leak_tau_ + dt);
      accel_bias_est_ += (accel_world - accel_bias_est_) * alpha;
      accel_world -= accel_bias_est_;
      // Expose the post-leak reading to the ZUPT gate: it must judge motion on
      // the same bias-free signal the integrator uses, or a tilted calibration
      // keeps ZUPT blocked (raw reading above threshold) while velocity stops
      // growing — pinning /odom at the clamp with the robot standing still.
      last_filtered_accel_world_ = accel_world;
      have_filtered_accel_ = true;
    }

    velocity = last_velocity_ + accel_world * dt;

    // Bound the estimate to something the robot can physically do. The bridge
    // delivers IMU at ~3.5 Hz under load (dt ~0.29 s) and the bias calibration
    // leaves ~1.7 m/s^2 standing, so ONE accepted sample injects ~0.5 m/s of
    // pure error. ZUPT only clears that once the robot stops, so without a
    // bound the estimate runs away for the whole walk.
    const double speed = velocity.length();
    if (speed > max_dead_reckon_speed_ && speed > 1e-9) {
      velocity *= max_dead_reckon_speed_ / speed;
    }

    position = last_position_ + velocity * dt;

    if (position.z() < 0.0) position.setZ(0.0);
  }

  void compute_leg_kinematics_velocity(
    const tf2::Quaternion & orient_prev, const tf2::Quaternion & orient_curr, double dt,
    tf2::Vector3 & velocity, tf2::Vector3 & position)
  {
    (void)orient_prev;
    (void)orient_curr;
    (void)dt;

    // TODO(Jjateen): implement full forward kinematics for each leg using URDF geometry
    // and body-to-foot Jacobian to compute body velocity from joint velocities.
    // Fall back to simple velocity estimate for now. Indices follow the Isaac
    // group order (leg_odometry.yaml): [hips(0-3), knees(4-7), ankles(8-11)].
    double vx = 0.0, vy = 0.0, vz = 0.0;
    int stance = 0;
    for (int leg = 0; leg < 4; ++leg) {
      double hip_vel = last_joint_velocities_[leg];
      double knee_vel = last_joint_velocities_[leg + 4];
      vx += 0.02 * hip_vel;
      vy += 0.02 * knee_vel;
      vz += 0.005 * (hip_vel + knee_vel);
      stance++;
    }
    if (stance > 0) {
      vx /= stance;
      vy /= stance;
      vz /= stance;
    }

    tf2::Matrix3x3 rot(last_orientation_);
    tf2::Vector3 vel_body(vx, vy, vz);
    velocity = rot * vel_body;
    position = last_position_ + velocity * dt;
  }

  bool is_robot_stationary(const sensor_msgs::msg::Imu::SharedPtr & msg)
  {
    bool joints_still = true;
    double max_joint_vel = 0.0;
    for (size_t i = 0; i < 12; ++i) {
      max_joint_vel = std::max(max_joint_vel, std::abs(last_joint_velocities_[i]));
      if (std::abs(last_joint_velocities_[i]) > stationary_joint_vel_threshold_) {
        joints_still = false;
        break;
      }
    }

    // Prefer the leak-filtered world-frame specific force: it strips the DC
    // residual a tilted calibration leaves in the raw reading, so this gate
    // measures actual body motion instead of calibration error. Falls back to
    // the raw body-frame reading when the filter has not run (leak disabled or
    // leg_kinematics source). The norm is rotation-invariant, so comparing
    // magnitudes across frames is sound.
    double accel_dev;
    if (have_filtered_accel_) {
      accel_dev = last_filtered_accel_world_.length();
    } else {
      const tf2::Vector3 raw(
        msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
      accel_dev = raw.length();
    }
    bool accel_still = accel_dev < stationary_accel_threshold_;

    RCLCPP_DEBUG_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "ZUPT diag: joints=%s (max_vel=%.4f thresh=%.3f) accel=%s (dev=%.3f thresh=%.3f) "
      "still_for=%.2fs",
      joints_still ? "yes" : "no", max_joint_vel, stationary_joint_vel_threshold_,
      accel_still ? "yes" : "no", accel_dev, stationary_accel_threshold_,
      still_since_.nanoseconds() == 0 ? 0.0 : (this->now() - still_since_).seconds());

    // Hold-off measured in SECONDS, not samples. A sample count means the
    // hold-off scales with whatever rate the IMU happens to arrive at: the old
    // 10 samples was written for 125 Hz (80 ms) but the bridge delivers ~3.5 Hz
    // under load, which turned it into 2.9 s of standing perfectly still before
    // ZUPT would engage, longer than most stops last. joints_still is the
    // primary signal here; the accel gate only rejects the case where the body
    // is being carried or shoved with the legs locked.
    if (!(joints_still && accel_still)) {
      still_since_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
      return false;
    }
    if (still_since_.nanoseconds() == 0) {
      still_since_ = this->now();
    }

    return (this->now() - still_since_).seconds() >= stationary_hold_s_;
  }

  // Fallback defaults — the config yaml is always loaded via launch, so these
  // are only used if the parameter declaration fails. Must match Isaac group
  // order (all hips, all knees, all ankles) to agree with policy_controller_node.
  inline static const std::vector<std::string> kDefaultJointNames{
    "Revolute_110", "Revolute_113", "Revolute_116", "Revolute_119", "Revolute_111", "Revolute_114",
    "Revolute_117", "Revolute_120", "Revolute_112", "Revolute_115", "Revolute_118", "Revolute_121"};

  inline static const std::vector<double> kDefaultJointPos{0.0,   0.0,   0.0,  0.0,  -0.32, -0.32,
                                                           -0.32, -0.32, 2.00, 2.00, 2.00,  2.00};

  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_cmd_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  std::string imu_topic_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr init_timer_;
  rclcpp::TimerBase::SharedPtr odom_tf_timer_;
  bool have_cmd_{false};
  int init_publishes_{0};

  std::vector<std::string> joint_names_;
  std::vector<double> default_joint_pos_;
  bool publish_tf_;
  bool publish_joint_states_;
  std::string odom_frame_;
  std::string base_frame_;
  std::string velocity_source_;
  double max_dead_reckon_speed_;

  double stationary_joint_vel_threshold_;
  double stationary_accel_threshold_;
  double stationary_hold_s_;
  double cmd_vel_stale_s_;
  // Start of the current unbroken still stretch; 0 means "moving right now".
  rclcpp::Time still_since_{0, 0, RCL_ROS_TIME};

  double servo_tau_;
  double servo_max_rate_rad_s_;
  // Same cap as servo_converter.hpp's kMaxDt: one late/gapped command must
  // not authorise an unbounded slew step.
  static constexpr double kMaxDt_ = 0.10;
  double command_rate_hz_{50.0};
  rclcpp::Duration cmd_throttle_period_{0, 0};
  double bias_leak_tau_{3.0};
  tf2::Vector3 accel_bias_est_{0.0, 0.0, 0.0};
  // Stationary-gated gyro-z bias estimate and this node's own integrated
  // yaw (see yaw_bias_leak_tau's own comment). yaw_estimate_ is what actually
  // becomes /odom's published yaw -- see on_imu().
  bool yaw_bias_leak_enabled_{true};
  double yaw_bias_leak_tau_{2.0};
  double gyro_bias_z_est_{0.0};
  double yaw_estimate_{0.0};
  // Latest post-leak world-frame specific force and validity flag; consumed by
  // the ZUPT accel gate so it judges motion on the bias-free signal. Written in
  // on_imu (via compute_imu_dead_reckon) before is_robot_stationary runs.
  tf2::Vector3 last_filtered_accel_world_{0.0, 0.0, 0.0};
  bool have_filtered_accel_{false};
  // Last arrival time of /cmd_vel; 0 means "never commanded". Guarded by
  // pose_mutex_ because the composed container runs callbacks concurrently.
  rclcpp::Time last_twist_cmd_time_{0, 0, RCL_ROS_TIME};
  std::vector<double> filtered_positions_{std::vector<double>(12, 0.0)};
  std::vector<double> last_joint_positions_{std::vector<double>(12, 0.0)};
  std::vector<double> last_joint_velocities_{std::vector<double>(12, 0.0)};
  rclcpp::Time last_cmd_time_{0, 0, RCL_ROS_TIME};
  std::mutex joint_mutex_;

  rclcpp::Time last_imu_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_zupt_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time node_start_time_{0, 0, RCL_ROS_TIME};
  tf2::Quaternion last_orientation_;
  tf2::Vector3 last_velocity_{0.0, 0.0, 0.0};
  tf2::Vector3 last_position_{0.0, 0.0, 0.0};
  bool zupt_active_{false};
  tf2::Vector3 zupt_anchor_{0.0, 0.0, 0.0};
  // Guards the pose last_* fields shared between on_imu (writer) and the 50 Hz
  // broadcast_odom_tf timer (reader). Standalone this node runs on a
  // single-threaded executor and the two can never overlap, but inside the
  // composed container it shares a MultiThreadedExecutor with the bridge and
  // the policy, where they can. An unguarded read there yields a torn pose:
  // position from one IMU sample, orientation from the next.
  std::mutex pose_mutex_;

  // IMU scaling.
  double imu_accel_scale_{1.0};
  double imu_gyro_scale_{1.0};

  // Lidar-aided correction.
  bool lidar_correction_enabled_{true};
  double lidar_gain_xy_{0.15};
  double lidar_gain_yaw_{0.20};
  double max_correction_step_m_{0.05};
  double max_correction_step_yaw_rad_{0.05};
  double correction_timeout_s_{1.0};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  tf2::Transform last_map_to_odom_;
  bool have_last_map_odom_{false};
};

}  // namespace leg_odometry

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(leg_odometry::LeggedOdometryNode)
