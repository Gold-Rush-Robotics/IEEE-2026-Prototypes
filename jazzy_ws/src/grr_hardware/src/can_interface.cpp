// Copyright 2021 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "grr_hardware/can_interface.hpp"


using std::placeholders::_1;

namespace grr_hardware
{
  hardware_interface::CallbackReturn CanInterface::on_init(const hardware_interface::HardwareInfo & info)
  {
    // Save the hardware info for later use
    info_ = info;
  
    // Create a ROS node (if you don't have one already)
    node_ = rclcpp::Node::make_shared("can_interface");

    recorded_serial_numbers_.resize(10, 0);          // or use a different default value/size
    serial_numbers_configured_.resize(10, 0);
  
    // Initialize CAN socket (as before)
    can_socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (can_socket_ < 0)
    {
      RCLCPP_FATAL(rclcpp::get_logger("CanInterface"), "Failed to create CAN socket");
      return CallbackReturn::ERROR;
    }
    // ... CAN socket setup code ...
  
    // Now initialize your new parameters from the hardware_parameters map
    if(info_.hardware_parameters.find("teensy1_serial_number") != info_.hardware_parameters.end())
    {
      teensy1_serial_number_ = info_.hardware_parameters.at("teensy1_serial_number");
      RCLCPP_INFO(node_->get_logger(), "Teensy1 serial number: %s", teensy1_serial_number_.c_str());
    }
    else
    {
      RCLCPP_WARN(node_->get_logger(), "teensy1_serial_number parameter not found");
    }
  
    // For vector parameters (assuming a comma-separated string)
    if(info_.hardware_parameters.find("teensy1_joint_names") != info_.hardware_parameters.end())
    {
      std::string joint_list = info_.hardware_parameters.at("teensy1_joint_names");
      std::stringstream ss(joint_list);
      std::string joint;
      while(std::getline(ss, joint, ','))
      {
        // Trim whitespace if necessary
        teensy1_joint_names_.push_back(joint);
      }
      RCLCPP_INFO(node_->get_logger(), "Loaded %zu teensy1 joint names", teensy1_joint_names_.size());
    }
    else
    {
      RCLCPP_WARN(node_->get_logger(), "teensy1_joint_names parameter not found");
    }
  
    if(info_.hardware_parameters.find("teensy2_serial_number") != info_.hardware_parameters.end())
    {
      teensy2_serial_number_ = info_.hardware_parameters.at("teensy2_serial_number");
      RCLCPP_INFO(node_->get_logger(), "Teensy2 serial number: %s", teensy2_serial_number_.c_str());
    }
    else
    {
      RCLCPP_WARN(node_->get_logger(), "teensy2_serial_number parameter not found");
    }
  
    if(info_.hardware_parameters.find("teensy2_joint_names") != info_.hardware_parameters.end())
    {
      std::string joint_list = info_.hardware_parameters.at("teensy2_joint_names");
      std::stringstream ss(joint_list);
      std::string joint;
      while(std::getline(ss, joint, ','))
      {
        teensy2_joint_names_.push_back(joint);
      }
      RCLCPP_INFO(node_->get_logger(), "Loaded %zu teensy2 joint names", teensy2_joint_names_.size());
    }
    else
    {
      RCLCPP_WARN(node_->get_logger(), "teensy2_joint_names parameter not found");
    }
    for (const auto & joint_info : info_.joints)
  {
    // Use the joint name as a key; it's assumed that a parameter with this name exists
    if (info_.hardware_parameters.find(joint_info.name) != info_.hardware_parameters.end())
    {
      std::string joint_param_str = info_.hardware_parameters.at(joint_info.name);
      // Assume the string is of the form "DIR:1,PWM:2,SLP:7,FLT:8,ENC_OUTA:11,ENC_OUTB:12,CS:23"
      JointParameters params;
      std::stringstream ss(joint_param_str);
      std::string token;
      while(std::getline(ss, token, ','))
      {
        auto pos = token.find(':');
        if (pos != std::string::npos)
        {
          std::string key = token.substr(0, pos);
          std::string value = token.substr(pos + 1);
          if (key == "DIR")
            params.DIR = std::stoi(value);
          else if (key == "PWM")
            params.PWM = std::stoi(value);
          else if (key == "SLP")
            params.SLP = std::stoi(value);
          else if (key == "FLT")
            params.FLT = std::stoi(value);
          else if (key == "ENC_OUTA")
            params.ENC_OUTA = std::stoi(value);
          else if (key == "ENC_OUTB")
            params.ENC_OUTB = std::stoi(value);
          else if (key == "CS")
            params.CS = std::stoi(value);
        }
      }
      // Store the parsed parameters in the joint_parameters_ map.
      joint_parameters_[joint_info.name] = params;
      RCLCPP_INFO(node_->get_logger(), "Loaded parameters for joint '%s': DIR=%d, PWM=%d, SLP=%d, FLT=%d, ENC_OUTA=%d, ENC_OUTB=%d, CS=%d",
                  joint_info.name.c_str(), params.DIR, params.PWM, params.SLP,
                  params.FLT, params.ENC_OUTA, params.ENC_OUTB, params.CS);
    }
    else
    {
      RCLCPP_WARN(node_->get_logger(), "No nested parameter found for joint '%s'", joint_info.name.c_str());
    }
  }
  
    
  
    // Continue with the rest of on_init() (e.g., resizing state vectors etc.)
    hw_state_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_state_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_commands_velocity_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_commands_effort_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_commands_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());


  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    joint_names_.push_back(joint.name);
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("CanInterface"),
        "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY && joint.command_interfaces[0].name != hardware_interface::HW_IF_EFFORT)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("CanInterface"),
        "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("CanInterface"),
        "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("CanInterface"),
        "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("CanInterface"),
        "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return CallbackReturn::ERROR;
    }
  }

  return CallbackReturn::SUCCESS;
}



std::vector<hardware_interface::StateInterface> CanInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_state_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_state_velocities_[i]));
  }

  return state_interfaces;
}



std::vector<hardware_interface::CommandInterface> CanInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++)
  {
    if (info_.joints[i].command_interfaces[0].name == hardware_interface::HW_IF_VELOCITY )
    {
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
       info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_velocity_[i]));
    }
    else if (info_.joints[i].command_interfaces[0].name == hardware_interface::HW_IF_EFFORT )
    {
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
       info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_commands_effort_[i]));
    }
    else if (info_.joints[i].command_interfaces[0].name == hardware_interface::HW_IF_POSITION )
    {
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
       info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_positions_[i]));
    }
    else{
      RCLCPP_FATAL(
        rclcpp::get_logger("CanInterface"),
        "Joint '%s' have '%s' as command interface. '%s' or '%s' expected.", info_.joints[i].name.c_str(),
        info_.joints[i].command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY, hardware_interface::HW_IF_EFFORT);
      throw std::runtime_error("Invalid command interface");
    }
    
  }

  return command_interfaces;
}



hardware_interface::CallbackReturn CanInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("CanInterface"), "Activating ...please wait...");

  // set some default values
  for (auto i = 0u; i < hw_state_positions_.size(); i++)
  {
    if (std::isnan(hw_state_positions_[i]))
    {
      hw_state_positions_[i] = 0;
      hw_state_velocities_[i] = 0;
      hw_commands_velocity_[i] = 0;
      hw_commands_effort_[i] = 0;
      hw_commands_positions_[i] = 0;
    }
    joint_names_map_[joint_names_[i]] = i + 1; // ADD 1 to differentiate null key
  }


  RCLCPP_INFO(rclcpp::get_logger("CanInterface"), "Successfully activated!");

  return CallbackReturn::SUCCESS;
}



hardware_interface::CallbackReturn CanInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("CanInterface"), "Deactivating ...please wait...");
  RCLCPP_INFO(rclcpp::get_logger("CanInterface"), "Successfully deactivated!");

  return CallbackReturn::SUCCESS;
}


// ||                        ||
// \/ THE STUFF THAT MATTERS \/

hardware_interface::return_type grr_hardware::CanInterface::read(const rclcpp::Time & time, const rclcpp::Duration & period)
{
  std::vector<uint16_t> recorded_serial_numbers;  // Replace with actual initialization if needed
  struct can_frame frame;
  ssize_t nbytes = ::read(can_socket_, &frame, sizeof(struct can_frame));  // Call global read()

  if (nbytes < 0)
  {
    RCLCPP_WARN(rclcpp::get_logger("CanInterface"), "Failed to read from CAN socket");
    return hardware_interface::return_type::ERROR;
  }

  if (nbytes == sizeof(struct can_frame))
  {
    // Example: Update hardware state based on CAN frame data
    if (frame.can_id == HEARTBEAT)  // Replace with your CAN ID
    {
      std::string serial_number = "";
      for (int i = 0; i < 8; i++) {
        serial_number += char(frame.data[i]);
      }
      for (size_t i = 0; i < recorded_serial_numbers.size(); i++){
        for (int i = 0;i<recorded_serial_numbers_.size();i++){
          if (recorded_serial_numbers[i] == std::stoi(serial_number)){
            RCLCPP_INFO(rclcpp::get_logger("CanInterface"), "Serial number already recorded");
            return hardware_interface::return_type::OK;
          }
          else{
            recorded_serial_numbers.push_back(std::stoi(serial_number));
            RCLCPP_INFO(rclcpp::get_logger("CanInterface"), "Serial number recorded");
          }
        }
      }
    }
    if(frame.can_id == SENSORS){
      //Get name of joint from buffer 8-24
      std::string joint_name = "";
      for (int i = 8; i < 24; i++) {
        joint_name += char(frame.data[i]);
      }
      //Get joint value
      double joint_value = 0;
      for (int i = 24; i < 32; i++) {
        joint_value += frame.data[i];
      }
      //Update joint state
      if (joint_names_map_.find(joint_name) != joint_names_map_.end())
      {
        hw_state_positions_[joint_names_map_[joint_name]] = joint_value;
      }
    }

  return hardware_interface::return_type::OK;
  }
}

hardware_interface::return_type grr_hardware::CanInterface::write(const rclcpp::Time & time, const rclcpp::Duration & period)
{
  // Instead of using sizeof, use the vector's size:
  if(serial_numbers_configured_.size() < recorded_serial_numbers_.size()){
    struct can_frame frame;
    frame.can_id = ASSIGN_ID;
    frame.can_dlc = 39;
    
    // Copy serial numbers (assuming each serial number fits into one byte, adjust as needed)
    for (size_t i = 0; i < 8 && i < serial_numbers_configured_.size(); i++) {
      frame.data[i] = static_cast<uint8_t>(serial_numbers_configured_[i]);
    }
    
    // For joint names, copy one character per slot (or you can copy multiple bytes if desired):
    // Here we fill positions 8 through 23 with the first character of each joint name, or 0 if not available.
    for (size_t i = 8; i < 24; i++) {
      size_t joint_idx = i - 8;
      if(joint_idx < joint_names_.size() && !joint_names_[joint_idx].empty()){
        frame.data[i] = static_cast<uint8_t>(joint_names_[joint_idx][0]);
      }
      else{
        frame.data[i] = 0;
      }
    }
    
    // For control type, we use the control_type_ vector.
    // Make sure control_type_ is properly initialized (it should have at least (32-24)=8 elements)
    for (size_t i = 24; i < 32; i++) {
      size_t ctrl_idx = i - 24;
      // If control_type_ isn’t large enough, fill with 0 or your default.
      if(ctrl_idx < control_type_.size()){
        frame.data[i] = control_type_[ctrl_idx];
      } else {
        frame.data[i] = 0;
      }
    }
    
    ssize_t nbytes = ::write(can_socket_, &frame, sizeof(struct can_frame));  // Call global write()
    if (nbytes < 0)
    {
      RCLCPP_WARN(rclcpp::get_logger("CanInterface"), "Failed to write to CAN socket");
      return hardware_interface::return_type::ERROR;
    }
    return hardware_interface::return_type::OK;
  }
  else{
    RCLCPP_INFO(rclcpp::get_logger("CanInterface"), "All serial numbers configured");
    return hardware_interface::return_type::OK;
  }
}

}  // namespace grr_hardware
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  grr_hardware::CanInterface, hardware_interface::SystemInterface)
