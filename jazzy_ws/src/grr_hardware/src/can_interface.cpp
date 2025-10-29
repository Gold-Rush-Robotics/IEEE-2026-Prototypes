#include "grr_hardware/can_interface.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <sys/socket.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <unistd.h>
#include <fcntl.h>
#include <cstring>

namespace grr_hardware
{

// CAN IDs
static constexpr uint32_t FRONT_CMD_ID  = 0x100;  // rxId for FRONT
static constexpr uint32_t FRONT_FB_ID   = 0x101;  // txId for FRONT
static constexpr uint32_t REAR_CMD_ID   = 0x200;  // rxId for REAR
static constexpr uint32_t REAR_FB_ID    = 0x201;  // txId for REAR

hardware_interface::CallbackReturn CanInterface::on_init(const hardware_interface::HardwareInfo & info)
{
  hardware_interface::CallbackReturn ret = BaseDriveHardware::on_init(info);
  if (ret != hardware_interface::CallbackReturn::SUCCESS)
  {
    RCLCPP_ERROR(node_->get_logger(), "BaseDriveHardware initialization failed");
    return ret;
  }

  // Default CAN interface
  try
  {
    can_interface_name_ = info_.hardware_parameters.at("can_interface");
  }
  catch (const std::out_of_range &)
  {
    RCLCPP_WARN(node_->get_logger(), "Parameter 'can_interface' not found, using default: %s", can_interface_name_.c_str());
  }

  if (!open_can_socket(can_interface_name_))
  {
    RCLCPP_ERROR(node_->get_logger(), "Failed to open CAN socket for interface %s", can_interface_name_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Map ROS joint names → firmware joint names
  ros_to_can_joint_names_.clear();
  for (const auto & joint : info_.joints)
  {
    if (joint.name.find("front_left") != std::string::npos)
      ros_to_can_joint_names_[joint.name] = "FL";
    else if (joint.name.find("front_right") != std::string::npos)
      ros_to_can_joint_names_[joint.name] = "FR";
    else if (joint.name.find("rear_left") != std::string::npos)
      ros_to_can_joint_names_[joint.name] = "RL";
    else if (joint.name.find("rear_right") != std::string::npos)
      ros_to_can_joint_names_[joint.name] = "RR";
    else
    {
      RCLCPP_WARN(node_->get_logger(), "No CAN joint name mapping for joint %s", joint.name.c_str());
    }
  }

  if (ros_to_can_joint_names_.empty())
  {
    RCLCPP_ERROR(node_->get_logger(), "No valid joint name mappings defined");
    close_can_socket();
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(node_->get_logger(), "Initialized CAN interface %s with %zu joint mappings",
              can_interface_name_.c_str(), ros_to_can_joint_names_.size());
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type CanInterface::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  struct can_frame frame;
  int nbytes = ::recv(socket_fd_, &frame, sizeof(struct can_frame), MSG_DONTWAIT);
  if (nbytes < 0)
  {
    if (errno == EAGAIN || errno == EWOULDBLOCK) return hardware_interface::return_type::OK;
    RCLCPP_WARN(node_->get_logger(), "CAN read failed: %s", strerror(errno));
    return hardware_interface::return_type::ERROR;
  }
  if (nbytes < (int)sizeof(struct can_frame) || frame.can_dlc != 8) return hardware_interface::return_type::OK;

  // Determine if it's FRONT or REAR feedback
  uint32_t expected_id = (frame.can_id == FRONT_FB_ID) ? FRONT_FB_ID : (frame.can_id == REAR_FB_ID) ? REAR_FB_ID : 0;
  if (expected_id == 0) return hardware_interface::return_type::OK;

  float velocity;
  char joint_name[5] = {0};
  memcpy(&velocity, frame.data, sizeof(float));
  memcpy(joint_name, frame.data + 4, 4);

  for (size_t i = 0; i < joint_names_.size(); ++i)
  {
    auto it = ros_to_can_joint_names_.find(joint_names_[i]);
    if (it != ros_to_can_joint_names_.end() && it->second == std::string(joint_name))
    {
      hw_state_velocities_[i] = velocity;
      hw_state_positions_[i] += velocity * 0.01;  // Placeholder integration
      RCLCPP_DEBUG(node_->get_logger(), "Feedback [%s] → %s: vel=%.2f",
                   (frame.can_id == FRONT_FB_ID) ? "FRONT" : "REAR", joint_names_[i].c_str(), velocity);
      break;
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type CanInterface::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  for (size_t i = 0; i < joint_names_.size(); ++i)
  {
    auto it = ros_to_can_joint_names_.find(joint_names_[i]);
    if (it == ros_to_can_joint_names_.end())
    {
      RCLCPP_WARN(node_->get_logger(), "No CAN joint name mapping for %s", joint_names_[i].c_str());
      continue;
    }

    const std::string & can_name = it->second;
    uint32_t can_id = 0;
    if (can_name == "FL" || can_name == "FR") can_id = FRONT_CMD_ID;
    else if (can_name == "RL" || can_name == "RR") can_id = REAR_CMD_ID;
    else continue;

    // NEGATE RIGHT SIDE VELOCITIES
    float velocity = static_cast<float>(hw_commands_velocity_[i]);
    if (can_name == "FR" || can_name == "RR")
      velocity = -velocity;

    struct can_frame frame{};
    frame.can_id = can_id;
    frame.can_dlc = 8;
    memcpy(frame.data, &velocity, sizeof(float));

    char name[4] = {' ', ' ', ' ', ' '};
    strncpy(name, can_name.c_str(), 2);  // "FL", "FR", "RL", "RR"
    memcpy(frame.data + 4, name, 4);

    int nbytes = ::write(socket_fd_, &frame, sizeof(frame));
    if (nbytes < 0)
    {
      RCLCPP_WARN(node_->get_logger(), "CAN write failed for %s (ID 0x%03X): %s",
                  joint_names_[i].c_str(), can_id, strerror(errno));
    }
    else
    {
      RCLCPP_DEBUG(node_->get_logger(), "Sent [%s] → %s: vel=%.2f (raw=%.2f)",
                   (can_id == FRONT_CMD_ID) ? "FRONT" : "REAR", joint_names_[i].c_str(), -velocity, velocity);
    }
  }
  return hardware_interface::return_type::OK;
}

bool CanInterface::open_can_socket(const std::string & interface_name)
{
  socket_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (socket_fd_ < 0)
  {
    RCLCPP_ERROR(node_->get_logger(), "Failed to create CAN socket: %s", strerror(errno));
    return false;
  }

  struct ifreq ifr;
  std::strncpy(ifr.ifr_name, interface_name.c_str(), IFNAMSIZ - 1);
  ifr.ifr_name[IFNAMSIZ - 1] = '\0';
  if (ioctl(socket_fd_, SIOCGIFINDEX, &ifr) < 0)
  {
    RCLCPP_ERROR(node_->get_logger(), "ioctl SIOCGIFINDEX failed: %s", strerror(errno));
    close(socket_fd_);
    return false;
  }

  struct sockaddr_can addr{};
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  if (bind(socket_fd_, (struct sockaddr *)&addr, sizeof(addr)) < 0)
  {
    RCLCPP_ERROR(node_->get_logger(), "bind failed: %s", strerror(errno));
    close(socket_fd_);
    return false;
  }

  // Non-blocking read
  int flags = fcntl(socket_fd_, F_GETFL, 0);
  if (flags == -1 || fcntl(socket_fd_, F_SETFL, flags | O_NONBLOCK) == -1)
  {
    RCLCPP_ERROR(node_->get_logger(), "Failed to set O_NONBLOCK: %s", strerror(errno));
    close(socket_fd_);
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

} // namespace grr_hardware

PLUGINLIB_EXPORT_CLASS(grr_hardware::CanInterface, hardware_interface::SystemInterface)