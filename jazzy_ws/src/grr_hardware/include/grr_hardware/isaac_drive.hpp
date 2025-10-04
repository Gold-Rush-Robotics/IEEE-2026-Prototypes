#pragma once
#include "grr_hardware/base_hardware_interface.hpp"

namespace grr_hardware
{

class IsaacDriveHardware : public BaseDriveHardware
{
public:
  // Overrides
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) override;
  hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override;

private:
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::JointState>> isaac_publisher_;
  std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::JointState>> isaac_subscriber_;
  std::shared_ptr<sensor_msgs::msg::JointState> received_joint_msg_ptr_;
  bool subscriber_is_active_ = false;
};

}