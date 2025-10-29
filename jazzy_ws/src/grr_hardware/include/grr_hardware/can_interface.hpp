#pragma once
#include "grr_hardware/base_hardware_interface.hpp"
#include <rclcpp/rclcpp.hpp>
#include <hardware_interface/system_interface.hpp>
#include <sys/socket.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <map>

namespace grr_hardware
{
class CanInterface : public BaseDriveHardware
{
public:
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) override;
  hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override;

private:
  // CAN communication settings
  std::string can_interface_name_ = "can0"; // Default interface
  int socket_fd_ = -1;

  // Joint name to CAN joint name mapping
  std::map<std::string, std::string> ros_to_can_joint_names_;

  // CAN IDs for "FRONT" node
  static constexpr uint32_t COMMAND_ID = 0x100; // rxId in firmware
  static constexpr uint32_t FEEDBACK_ID = 0x101; // txId in firmware

  // Utility
  bool open_can_socket(const std::string & interface_name);
  void close_can_socket();
};
} // namespace grr_hardware