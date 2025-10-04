#include "grr_hardware/can_interface.hpp"

#include <pluginlib/class_list_macros.hpp>
#include <sys/socket.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <unistd.h>
#include <cstring>

namespace grr_hardware
{

hardware_interface::CallbackReturn CanInterface::on_init(const hardware_interface::HardwareInfo & info)
{
  hardware_interface::CallbackReturn ret = BaseDriveHardware::on_init(info);
  if (ret != hardware_interface::CallbackReturn::SUCCESS)
  {
    return ret;
  }

  can_interface_name_ = info_.hardware_parameters.at("can_interface");
  if (!open_can_socket(can_interface_name_))
  {
    RCLCPP_ERROR(node_->get_logger(), "Failed to open CAN socket.");
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type CanInterface::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  struct can_frame frame;
  int nbytes = ::read(socket_fd_, &frame, sizeof(struct can_frame));
  if (nbytes < 0)
  {
    RCLCPP_WARN(node_->get_logger(), "CAN read failed");
    return hardware_interface::return_type::ERROR;
  }

  // Dummy parse example (ID-based mapping or use frame.data[])
  size_t index = 0; // Map CAN ID or content to joint index
  hw_state_positions_[index] += 0.01;  // Simulate
  hw_state_velocities_[index] = 0.5;

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type CanInterface::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  for (size_t i = 0; i < joint_names_.size(); ++i)
  {
    struct can_frame frame{};
    frame.can_id = 0x100 + i;
    frame.can_dlc = 2;
    uint16_t velocity = static_cast<uint16_t>(hw_commands_velocity_[i] * 100);  // scale
    frame.data[0] = velocity >> 8;
    frame.data[1] = velocity & 0xFF;

  int nbytes = ::write(socket_fd_, &frame, sizeof(frame));
    if (nbytes < 0)
    {
      RCLCPP_WARN(node_->get_logger(), "CAN write failed for joint %s", joint_names_[i].c_str());
    }
  }

  return hardware_interface::return_type::OK;
}

bool CanInterface::open_can_socket(const std::string & interface_name)
{
  socket_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (socket_fd_ < 0)
  {
    perror("Socket");
    return false;
  }

  struct ifreq ifr;
  std::strncpy(ifr.ifr_name, interface_name.c_str(), IFNAMSIZ - 1);
  ioctl(socket_fd_, SIOCGIFINDEX, &ifr);

  struct sockaddr_can addr{};
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  if (bind(socket_fd_, (struct sockaddr *)&addr, sizeof(addr)) < 0)
  {
    perror("Bind");
    return false;
  }

  return true;
}

void CanInterface::close_can_socket()
{
  if (socket_fd_ >= 0)
  {
    close(socket_fd_);
    socket_fd_ = -1;
  }
}

}  // namespace grr_hardware

PLUGINLIB_EXPORT_CLASS(grr_hardware::CanInterface, hardware_interface::SystemInterface)
