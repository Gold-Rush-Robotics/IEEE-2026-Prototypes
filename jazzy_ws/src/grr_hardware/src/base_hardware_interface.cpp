#include "grr_hardware/base_hardware_interface.hpp"

namespace grr_hardware
{

hardware_interface::CallbackReturn BaseDriveHardware::on_init(const hardware_interface::HardwareInfo & info)
{
  info_ = info;
  node_ = rclcpp::Node::make_shared("base_drive_hardware");

  hw_state_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_state_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_velocity_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_effort_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  for (const auto & joint : info_.joints)
  {
    joint_names_.push_back(joint.name);

    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(node_->get_logger(), "Joint '%s' must have exactly 1 command interface.", joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    const std::string & cmd = joint.command_interfaces[0].name;
    if (cmd != hardware_interface::HW_IF_VELOCITY &&
        cmd != hardware_interface::HW_IF_EFFORT &&
        cmd != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(node_->get_logger(), "Unsupported command interface '%s' for joint '%s'", cmd.c_str(), joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2 ||
        joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION ||
        joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(node_->get_logger(), "Joint '%s' must have POSITION and VELOCITY state interfaces", joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::CallbackReturn BaseDriveHardware::on_activate(const rclcpp_lifecycle::State &)
{
  for (size_t i = 0; i < joint_names_.size(); ++i)
  {
    if (std::isnan(hw_state_positions_[i])) {
      hw_state_positions_[i] = 0;
      hw_state_velocities_[i] = 0;
      hw_commands_velocity_[i] = 0;
      hw_commands_effort_[i] = 0;
      hw_commands_positions_[i] = 0;
    }
    joint_names_map_[joint_names_[i]] = i + 1;  // avoid 0 as null
  }

  RCLCPP_INFO(node_->get_logger(), "BaseDriveHardware activated.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn BaseDriveHardware::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(node_->get_logger(), "BaseDriveHardware deactivated.");
  return hardware_interface::CallbackReturn::SUCCESS;
}


std::vector<hardware_interface::StateInterface> BaseDriveHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (size_t i = 0; i < joint_names_.size(); ++i)
  {
    state_interfaces.emplace_back(joint_names_[i], hardware_interface::HW_IF_POSITION, &hw_state_positions_[i]);
    state_interfaces.emplace_back(joint_names_[i], hardware_interface::HW_IF_VELOCITY, &hw_state_velocities_[i]);
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> BaseDriveHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (size_t i = 0; i < joint_names_.size(); ++i)
  {
    const std::string & cmd_type = info_.joints[i].command_interfaces[0].name;

    if (cmd_type == hardware_interface::HW_IF_VELOCITY)
    {
      command_interfaces.emplace_back(joint_names_[i], cmd_type, &hw_commands_velocity_[i]);
    }
    else if (cmd_type == hardware_interface::HW_IF_EFFORT)
    {
      command_interfaces.emplace_back(joint_names_[i], cmd_type, &hw_commands_effort_[i]);
    }
    else if (cmd_type == hardware_interface::HW_IF_POSITION)
    {
      command_interfaces.emplace_back(joint_names_[i], cmd_type, &hw_commands_positions_[i]);
    }
  }

  return command_interfaces;
}

}  // namespace grr_hardware
