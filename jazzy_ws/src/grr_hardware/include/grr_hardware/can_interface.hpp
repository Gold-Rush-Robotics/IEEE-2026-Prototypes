#ifndef GRR_HARDWARE__CAN_INTERFACE_HPP_
#define GRR_HARDWARE__CAN_INTERFACE_HPP_

#include <memory>
#include <string>
#include <vector>
#include <map>
#include <chrono>
#include <cmath>
#include <limits>

#include <linux/can.h>
#include <linux/can/raw.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "grr_hardware/visibility_control.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "realtime_tools/realtime_box.h"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"

namespace grr_hardware
{

// Structure to hold per-joint configuration parameters.
struct JointParameters
{
  int DIR;
  int PWM;
  int SLP;
  int FLT;
  int ENC_OUTA;
  int ENC_OUTB;
  int CS;
};

class CanInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(CanInterface)

  GRR_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  GRR_HARDWARE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  GRR_HARDWARE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  GRR_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  GRR_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  GRR_HARDWARE_PUBLIC
  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  GRR_HARDWARE_PUBLIC
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Parameters for the DiffBot simulation
  double hw_start_sec_;
  double hw_stop_sec_;
  int can_socket_;

  // Store the command for the simulated robot
  rclcpp::Node::SharedPtr node_;

  std::vector<double> hw_commands_velocity_;
  std::vector<double> hw_commands_effort_;
  std::vector<double> hw_commands_positions_;
  std::vector<double> hw_state_positions_;
  std::vector<double> hw_state_velocities_;
  std::vector<std::string> joint_names_;
  std::map<std::string, uint> joint_names_map_;

  std::vector<uint16_t> recorded_serial_numbers_;
  std::vector<uint16_t> serial_numbers_configured_;

  // New Teensy hardware configuration parameters
  std::string teensy1_serial_number_;
  std::vector<std::string> teensy1_joint_names_;

  std::string teensy2_serial_number_;
  std::vector<std::string> teensy2_joint_names_;

  // Map of joint names to their configuration parameters
  std::map<std::string, JointParameters> joint_parameters_;
  std::vector<uint8_t> control_type_;

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

#endif  // GRR_HARDWARE__CAN_INTERFACE_HPP_