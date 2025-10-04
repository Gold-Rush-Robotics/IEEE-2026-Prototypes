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
  std::string can_interface_name_;
  int socket_fd_;

  // Utility
  bool open_can_socket(const std::string & interface_name);
  void close_can_socket();
  enum CAN_IDs
  {
    E_STOP = 0x000,
    HALT = 0x100,
    RESTART = 0x200,
    HEARTBEAT = 0x300,
    QUERY = 0x400,
    ASSIGN_ID = 0x500,
    FIRMWARE = 0x600,
    FATAL = 0x1000,
    ERROR = 0x2000,
    MOTOR_COMMAND = 0x3000,
    SERVO_CONTROL = 0x4000,
    DIO = 0x5000,
    SENSORS = 0x6000,
    WARNINGS = 0x7000,
    LOGS = 0x8000,
    MOANING = 0xFFFE,
    SGA_WARRANTY = 0xFFFF
  };
};

}  // namespace grr_hardware
