#include "grr_hardware/isaac_drive.hpp"

#include <pluginlib/class_list_macros.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>

namespace grr_hardware
{

hardware_interface::CallbackReturn IsaacDriveHardware::on_init(const hardware_interface::HardwareInfo & info)
{
  hardware_interface::CallbackReturn ret = BaseDriveHardware::on_init(info);
  if (ret != hardware_interface::CallbackReturn::SUCCESS)
  {
    return ret;
  }

  isaac_publisher_ = node_->create_publisher<sensor_msgs::msg::JointState>("/isaac_joint_cmd", 10);

  isaac_subscriber_ = node_->create_subscription<sensor_msgs::msg::JointState>(
    "/isaac_joint_state", rclcpp::SensorDataQoS(),
    [this](const sensor_msgs::msg::JointState::SharedPtr msg)
    {
      received_joint_msg_ptr_ = msg;
    });

  subscriber_is_active_ = true;
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type IsaacDriveHardware::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!received_joint_msg_ptr_)
  {
    return hardware_interface::return_type::OK;
  }

  for (size_t i = 0; i < received_joint_msg_ptr_->name.size(); ++i)
  {
    const std::string &name = received_joint_msg_ptr_->name[i];
    auto it = joint_names_map_.find(name);
    if (it != joint_names_map_.end())
    {
      size_t idx = it->second - 1;
      hw_state_positions_[idx] = received_joint_msg_ptr_->position[i];
      hw_state_velocities_[idx] = received_joint_msg_ptr_->velocity[i];
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type IsaacDriveHardware::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!subscriber_is_active_)
  {
    RCLCPP_WARN(node_->get_logger(), "Isaac subscriber not active yet.");
    return hardware_interface::return_type::ERROR;
  }

  auto msg = sensor_msgs::msg::JointState();
  msg.header.stamp = node_->get_clock()->now();

  for (size_t i = 0; i < joint_names_.size(); ++i)
  {
    msg.name.push_back(joint_names_[i]);
    msg.velocity.push_back(hw_commands_velocity_[i]);
  }

  isaac_publisher_->publish(msg);
  return hardware_interface::return_type::OK;
}

}  // namespace grr_hardware

PLUGINLIB_EXPORT_CLASS(grr_hardware::IsaacDriveHardware, hardware_interface::SystemInterface)
