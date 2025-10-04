#pragma once

#include <vector>
#include <string>
#include <memory>
#include <map>
#include <cstring>

// ROS 2 headers
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace grr_hardware
{

class BaseDriveHardware : public hardware_interface::SystemInterface
{
public:
  virtual hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  virtual hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  virtual hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  virtual std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  virtual std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

protected:
  // Shared configuration
  hardware_interface::HardwareInfo info_;
  std::shared_ptr<rclcpp::Node> node_;

  std::vector<std::string> joint_names_;
  std::map<std::string, uint> joint_names_map_;

  // Shared state/command vectors
  std::vector<double> hw_state_positions_;
  std::vector<double> hw_state_velocities_;
  std::vector<double> hw_commands_velocity_;
  std::vector<double> hw_commands_effort_;
  std::vector<double> hw_commands_positions_;  // optional

  // Helper method for joint param parsing (optional)
  void populate_joint_mappings();

};

}  // namespace grr_hardware
