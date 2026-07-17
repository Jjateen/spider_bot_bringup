#include "big_bertha_controllers/joint_effort_pd_controller.hpp"

#include <algorithm>
#include <cstddef>
#include <exception>

#include "pluginlib/class_list_macros.hpp"

namespace big_bertha_controllers
{
using config_type = controller_interface::interface_configuration_type;

controller_interface::CallbackReturn JointEffortPdController::on_init()
{
  try {
    auto_declare<std::vector<std::string>>("joints", {});
    auto_declare<std::vector<double>>("default_positions", {});
    auto_declare<double>("kp", 20.0);
    auto_declare<double>("kd", 2.0);
    auto_declare<double>("effort_limit", 4.0);
    auto_declare<double>("velocity_limit", 0.0);
  } catch (const std::exception & e) {
    fprintf(stderr, "JointEffortPdController on_init failed: %s\n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn JointEffortPdController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  joint_names_ = get_node()->get_parameter("joints").as_string_array();
  if (joint_names_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "'joints' parameter is empty");
    return controller_interface::CallbackReturn::ERROR;
  }
  kp_ = get_node()->get_parameter("kp").as_double();
  kd_ = get_node()->get_parameter("kd").as_double();
  effort_limit_ = get_node()->get_parameter("effort_limit").as_double();
  velocity_limit_ = get_node()->get_parameter("velocity_limit").as_double();

  default_positions_ = get_node()->get_parameter("default_positions").as_double_array();
  if (default_positions_.size() != joint_names_.size()) {
    RCLCPP_WARN(
      get_node()->get_logger(), "'default_positions' size %zu != %zu joints; defaulting to zeros",
      default_positions_.size(), joint_names_.size());
    default_positions_.assign(joint_names_.size(), 0.0);
  }
  target_buffer_.writeFromNonRT(default_positions_);

  target_sub_ = get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
    "~/commands", rclcpp::SystemDefaultsQoS(),
    [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
      if (msg->data.size() == joint_names_.size()) {
        target_buffer_.writeFromNonRT(msg->data);
      } else {
        RCLCPP_WARN_THROTTLE(
          get_node()->get_logger(), *get_node()->get_clock(), 2000,
          "command size %zu != %zu joints; ignored", msg->data.size(), joint_names_.size());
      }
    });

  RCLCPP_INFO(
    get_node()->get_logger(),
    "joint_effort_pd configured: %zu joints, kp=%.2f kd=%.2f "
    "effort_limit=%.2f",
    joint_names_.size(), kp_, kd_, effort_limit_);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
JointEffortPdController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf;
  conf.type = config_type::INDIVIDUAL;
  conf.names.reserve(joint_names_.size());
  for (const auto & joint : joint_names_) {
    conf.names.push_back(joint + "/effort");
  }
  return conf;
}

controller_interface::InterfaceConfiguration
JointEffortPdController::state_interface_configuration() const
{
  // Positions first [0..n), then velocities [n..2n) -- update() indexes by
  // this.
  controller_interface::InterfaceConfiguration conf;
  conf.type = config_type::INDIVIDUAL;
  conf.names.reserve(joint_names_.size() * 2);
  for (const auto & joint : joint_names_) {
    conf.names.push_back(joint + "/position");
  }
  for (const auto & joint : joint_names_) {
    conf.names.push_back(joint + "/velocity");
  }
  return conf;
}

controller_interface::CallbackReturn JointEffortPdController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Hold the default pose until the first policy target arrives.
  target_buffer_.writeFromNonRT(default_positions_);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn JointEffortPdController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  for (auto & command_interface : command_interfaces_) {
    (void)command_interface.set_value(0.0);
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type JointEffortPdController::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  const std::size_t n = joint_names_.size();
  const std::vector<double> * targets = target_buffer_.readFromRT();
  const bool have_targets = targets != nullptr && targets->size() == n;

  for (std::size_t i = 0; i < n; ++i) {
    auto q_opt = state_interfaces_[i].get_optional();
    if (!q_opt.has_value()) {
      RCLCPP_WARN_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 5000,
        "position state interface missing for joint %zu", i);
    }
    auto qd_opt = state_interfaces_[n + i].get_optional();
    if (!qd_opt.has_value()) {
      RCLCPP_WARN_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 5000,
        "velocity state interface missing for joint %zu", i);
    }
    const double q = q_opt.value_or(0.0);
    const double qd = qd_opt.value_or(0.0);
    const double q_des = have_targets ? (*targets)[i] : default_positions_[i];

    double tau = kp_ * (q_des - q) + kd_ * (0.0 - qd);
    tau = std::clamp(tau, -effort_limit_, effort_limit_);
    // Emulate Isaac's velocity_limit_sim (a hard joint-speed cap PhysX enforces
    // but DART does not): refuse torque that would accelerate a joint already
    // past the cap. Without this the explicit PD lets legs spin up and flail.
    if (velocity_limit_ > 0.0) {
      if (qd > velocity_limit_ && tau > 0.0) {
        tau = 0.0;
      }
      if (qd < -velocity_limit_ && tau < 0.0) {
        tau = 0.0;
      }
    }
    (void)command_interfaces_[i].set_value(tau);
  }
  return controller_interface::return_type::OK;
}

}  // namespace big_bertha_controllers

PLUGINLIB_EXPORT_CLASS(
  big_bertha_controllers::JointEffortPdController, controller_interface::ControllerInterface)
