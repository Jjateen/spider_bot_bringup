// Synchronous joint-space effort-PD controller for Big Bertha.
//
// Why this exists: the learned gait was trained in Isaac Lab against an
// ImplicitActuatorCfg (stiffness=20, damping=2) -- a PD that runs *inside* the
// physics solver at the full rate. Emulating that PD in the policy node and
// shipping the resulting torque over a ROS topic closes the loop across async
// DDS hops: the joint velocity feedback is several milliseconds stale by the
// time the torque is applied, so the damping term injects energy and the legs
// jitter. No gain tuning fixes a laggy feedback loop.
//
// This controller instead computes tau = Kp*(q_des - q) + Kd*(0 - qd) inside
// the controller_manager update() -- synchronous with the simulation, reading
// q and qd with zero latency and writing effort directly. The policy node only
// has to publish slowly-varying joint *position targets* (latency-tolerant),
// exactly mirroring how the policy drives the implicit actuator in training.
#ifndef BIG_BERTHA_CONTROLLERS__JOINT_EFFORT_PD_CONTROLLER_HPP_
#define BIG_BERTHA_CONTROLLERS__JOINT_EFFORT_PD_CONTROLLER_HPP_

#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp/subscription.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace big_bertha_controllers
{

class JointEffortPdController : public controller_interface::ControllerInterface
{
public:
  JointEffortPdController() = default;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  std::vector<std::string> joint_names_;
  std::vector<double> default_positions_;
  double kp_{20.0};
  double kd_{2.0};
  double effort_limit_{4.0};
  double velocity_limit_{0.0};     // no-load joint speed (rad/s); 0 disables
  double saturation_effort_{0.0};  // stall torque for the torque-speed curve; 0 disables

  // Position targets from the policy node; written non-RT, read in update().
  realtime_tools::RealtimeBuffer<std::vector<double>> target_buffer_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr target_sub_;
};

}  // namespace big_bertha_controllers

#endif  // BIG_BERTHA_CONTROLLERS__JOINT_EFFORT_PD_CONTROLLER_HPP_
