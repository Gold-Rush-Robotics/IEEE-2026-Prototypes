// Copyright 2020 PAL Robotics S.L.
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

/*
 * Author: Bence Magyar, Enrique Fernández, Manuel Meraz
 */

#include <memory>
#include <queue>
#include <string>
#include <utility>
#include <vector>

#include "grr_cmake_controller/grr_cmake_controller.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/time.hpp"

namespace
{
constexpr auto DEFAULT_COMMAND_TOPIC = "~/cmd_vel";
constexpr auto DEFAULT_COMMAND_UNSTAMPED_TOPIC = "~/cmd_vel_unstamped";
constexpr auto DEFAULT_COMMAND_OUT_TOPIC = "~/cmd_vel_out";
}  // namespace

namespace grr_cmake_controller
{
using namespace std::chrono_literals;
using controller_interface::interface_configuration_type;
using controller_interface::InterfaceConfiguration;
using hardware_interface::HW_IF_POSITION;
using hardware_interface::HW_IF_VELOCITY;
using lifecycle_msgs::msg::State;

Wheel::Wheel(std::reference_wrapper<hardware_interface::LoanedCommandInterface> command_velocity, 
             std::reference_wrapper<const hardware_interface::LoanedStateInterface> state_velocity, 
             std::string name) 
    : command_velocity_(command_velocity), 
      state_velocity_(state_velocity), 
      name(std::move(name)) {}

void Wheel::set_velocity(double velocity)
{
  std::lock_guard<std::mutex> lock(*mutex_);
  command_velocity_.get().set_value(velocity);
}
double Wheel::get_velocity(){
  std::lock_guard<std::mutex> lock(*mutex_);
  return state_velocity_.get().get_value();
}

MecanumController::MecanumController() : controller_interface::ControllerInterface() {}

CallbackReturn MecanumController::on_init()
{
  try
  {
    // with the lifecycle node being initialized, we can declare parameters
    auto_declare<std::string>("front_left_joint", front_left_wheel_joint_name_);
    auto_declare<std::string>("front_right_joint", front_right_wheel_joint_name_);
    auto_declare<std::string>("rear_left_joint", rear_left_wheel_joint_name_);
    auto_declare<std::string>("rear_right_joint", rear_right_wheel_joint_name_);

    auto_declare<double>("chassis_center_to_axle", wheel_params_.x_offset);
    auto_declare<double>("axle_center_to_wheel", wheel_params_.y_offset);
    auto_declare<double>("wheel_radius", wheel_params_.radius);

    auto_declare<double>("cmd_vel_timeout", cmd_vel_timeout_.count() / 1000.0);
    auto_declare<bool>("use_stamped_vel", use_stamped_vel_);
    odom->init(rclcpp::Time());
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

InterfaceConfiguration MecanumController::command_interface_configuration() const
{
  std::vector<std::string> conf_names;
  conf_names.push_back(front_left_wheel_joint_name_ + "/" + HW_IF_VELOCITY);
  conf_names.push_back(front_right_wheel_joint_name_ + "/" + HW_IF_VELOCITY);
  conf_names.push_back(rear_left_wheel_joint_name_ + "/" + HW_IF_VELOCITY);
  conf_names.push_back(rear_right_wheel_joint_name_ + "/" + HW_IF_VELOCITY);
  return {interface_configuration_type::INDIVIDUAL, conf_names};
}

InterfaceConfiguration MecanumController::state_interface_configuration() const
  {
    std::vector<std::string> conf_names;
    conf_names.push_back(front_left_wheel_joint_name_ + "/" + HW_IF_VELOCITY);
    conf_names.push_back(front_right_wheel_joint_name_ + "/" + HW_IF_VELOCITY);
    conf_names.push_back(rear_left_wheel_joint_name_ + "/" + HW_IF_VELOCITY);
    conf_names.push_back(rear_right_wheel_joint_name_ + "/" + HW_IF_VELOCITY);
    return {interface_configuration_type::INDIVIDUAL, conf_names};
  }

controller_interface::return_type MecanumController::update(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  auto logger = get_node()->get_logger();
  if (get_lifecycle_state().id() == State::PRIMARY_STATE_INACTIVE)
  {
    if (!is_halted)
    {
      halt();
      is_halted = true;
    }
    return controller_interface::return_type::OK;
  }

  const auto current_time = time;

  std::shared_ptr<Twist> last_command_msg;
  received_velocity_msg_ptr_.get(last_command_msg);

  if (last_command_msg == nullptr)
  {
    RCLCPP_WARN(logger, "Velocity message received was a nullptr.");
    return controller_interface::return_type::ERROR;
  }

  const auto age_of_last_command = current_time - last_command_msg->header.stamp;
  // Brake if cmd_vel has timeout, override the stored command
  if (age_of_last_command > cmd_vel_timeout_)
  {
    last_command_msg->twist.linear.x = 0.0;
    last_command_msg->twist.angular.z = 0.0;
  }

  Twist command = *last_command_msg;
  double & linear_x_cmd = command.twist.linear.x;
  double & linear_y_cmd = command.twist.linear.y;
  if (this->fieldOrientationEnabled)
    {
      double robot_odom_orientation = odom->getHeading();
      // RCLCPP_INFO(rclcpp::get_logger("TeleopTwistJoy"), "robot_orientation: %f", robot_odom_orientation);
      double new_linear_x_cmd = linear_x_cmd * cos(robot_odom_orientation) + linear_y_cmd * sin(robot_odom_orientation);
      linear_y_cmd = -1 * linear_x_cmd * sin(robot_odom_orientation) + linear_y_cmd * cos(robot_odom_orientation);
      linear_x_cmd = new_linear_x_cmd;
    }
  double & angular_cmd = command.twist.angular.z;

  double x_offset = wheel_params_.x_offset;
  double y_offset = wheel_params_.y_offset;
  double radius = wheel_params_.radius;

  // Compute Wheel Velocities
  const double front_left_offset = (-1 * x_offset + -1 * y_offset);
  const double front_right_offset = (x_offset + y_offset);
  const double rear_left_offset = (-1 * x_offset + -1 * y_offset);
  const double rear_right_offset = (x_offset + y_offset);

  const double front_left_velocity = (front_left_offset * angular_cmd + linear_x_cmd - linear_y_cmd) / radius;
  const double front_right_velocity = (front_right_offset * angular_cmd + linear_x_cmd + linear_y_cmd) / radius;
  const double rear_left_velocity = (rear_left_offset * angular_cmd + linear_x_cmd + linear_y_cmd) / radius;
  const double rear_right_velocity = (rear_right_offset * angular_cmd + linear_x_cmd - linear_y_cmd) / radius;

  // Set Wheel Velocities
  front_left_wheel_interface_handle_->set_velocity(front_left_velocity);
  front_right_wheel_interface_handle_->set_velocity(front_right_velocity);
  rear_left_wheel_interface_handle_->set_velocity(rear_left_velocity);
  rear_right_wheel_interface_handle_->set_velocity(rear_right_velocity);

  //Update Odometry
  odom->update(front_left_wheel_interface_handle_->get_velocity(),front_right_wheel_interface_handle_->get_velocity(),rear_left_wheel_interface_handle_->get_velocity(),rear_right_wheel_interface_handle_->get_velocity(),current_time);
  // RCLCPP_INFO(logger,"wheel velocities: %f, %f, %f, %f",front_left_wheel_interface_handle_->get_velocity(),front_right_wheel_interface_handle_->get_velocity(),rear_left_wheel_interface_handle_->get_velocity(),rear_right_wheel_interface_handle_->get_velocity());
  // RCLCPP_INFO(logger,"update called: %d",update);
  // RCLCPP_INFO(logger,"time: %f",time.seconds());

  // Time update
  const auto update_dt = current_time - previous_update_timestamp_;
  previous_update_timestamp_ = current_time;

  return controller_interface::return_type::OK;
}

CallbackReturn MecanumController::on_configure(const rclcpp_lifecycle::State &)
{
  auto logger = get_node()->get_logger();

  // Get Parameters
  front_left_wheel_joint_name_ = get_node()->get_parameter("front_left_joint").as_string();
  front_right_wheel_joint_name_ = get_node()->get_parameter("front_right_joint").as_string();
  rear_left_wheel_joint_name_ = get_node()->get_parameter("rear_left_joint").as_string();
  rear_right_wheel_joint_name_ = get_node()->get_parameter("rear_right_joint").as_string();

  if (front_left_wheel_joint_name_.empty()) {
    RCLCPP_ERROR(logger, "front_left_joint_name is not set");
    return CallbackReturn::ERROR;
  }
  if (front_right_wheel_joint_name_.empty()) {
    RCLCPP_ERROR(logger, "front_right_joint_name is not set");
    return CallbackReturn::ERROR;
  }
  if (rear_left_wheel_joint_name_.empty()) {
    RCLCPP_ERROR(logger, "rear_left_joint_name is not set");
    return CallbackReturn::ERROR;
  }
  if (rear_right_wheel_joint_name_.empty()) {
    RCLCPP_ERROR(logger, "rear_right_joint_name is not set");
    return CallbackReturn::ERROR;
  }

  wheel_params_.x_offset = get_node()->get_parameter("chassis_center_to_axle").as_double();
  wheel_params_.y_offset = get_node()->get_parameter("axle_center_to_wheel").as_double();
  wheel_params_.radius = get_node()->get_parameter("wheel_radius").as_double();

  cmd_vel_timeout_ = std::chrono::milliseconds{
    static_cast<int>(get_node()->get_parameter("cmd_vel_timeout").as_double() * 1000.0)};
  use_stamped_vel_ = get_node()->get_parameter("use_stamped_vel").as_bool();
    
  odom->setWheelParams(wheel_params_.radius,wheel_params_.x_offset,wheel_params_.y_offset);


  // Run reset to make sure everything is initialized correctly
  if (!reset())
  {
    return CallbackReturn::ERROR;
  }

  const Twist empty_twist;
  received_velocity_msg_ptr_.set(std::make_shared<Twist>(empty_twist));

  // initialize command subscriber
  if (use_stamped_vel_)
  {
    velocity_command_subscriber_ = get_node()->create_subscription<Twist>(
      DEFAULT_COMMAND_TOPIC, rclcpp::SystemDefaultsQoS(),
      [this](const std::shared_ptr<Twist> msg) -> void {
        if (!subscriber_is_active_)
        {
          RCLCPP_WARN(get_node()->get_logger(), "Can't accept new commands. subscriber is inactive");
          return;
        }
        if ((msg->header.stamp.sec == 0) && (msg->header.stamp.nanosec == 0))
        {
          RCLCPP_WARN_ONCE(
            get_node()->get_logger(),
            "Received TwistStamped with zero timestamp, setting it to current "
            "time, this message will only be shown once");
          msg->header.stamp = get_node()->get_clock()->now();
        }
        received_velocity_msg_ptr_.set(std::move(msg));
      });
  }
  else
  {
    velocity_command_unstamped_subscriber_ = get_node()->create_subscription<geometry_msgs::msg::Twist>(
      DEFAULT_COMMAND_UNSTAMPED_TOPIC, rclcpp::SystemDefaultsQoS(),
      [this](const std::shared_ptr<geometry_msgs::msg::Twist> msg) -> void {
        if (!subscriber_is_active_)
        {
          RCLCPP_WARN(get_node()->get_logger(), "Can't accept new commands. subscriber is inactive");
          return;
        }

        // Write fake header in the stored stamped command
        std::shared_ptr<Twist> twist_stamped;
        received_velocity_msg_ptr_.get(twist_stamped);
        twist_stamped->twist = *msg;
        twist_stamped->header.stamp = get_node()->get_clock()->now();
      });
  }
  pose_command_subscriber_ = get_node()->create_subscription<geometry_msgs::msg::Pose>(
    "pose", rclcpp::SystemDefaultsQoS(),
    [this](const std::shared_ptr<geometry_msgs::msg::Pose> msg) -> void {
      if (!subscriber_is_active_)
      {
        RCLCPP_WARN(get_node()->get_logger(), "Can't accept new commands. subscriber is inactive");
        return;
      }
      RCLCPP_INFO(get_node()->get_logger(), "Received pose reset command: x: %f, y: %f, z: %f", msg->position.x, msg->position.y, msg->position.z);
      odom->setOdometry(msg->position.x, msg->position.y, msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
    });

  previous_update_timestamp_ = get_node()->get_clock()->now();
  return CallbackReturn::SUCCESS;
}

CallbackReturn MecanumController::on_activate(const rclcpp_lifecycle::State &)
{
  front_left_wheel_interface_handle_ = get_wheel(front_left_wheel_joint_name_);
  front_right_wheel_interface_handle_ = get_wheel(front_right_wheel_joint_name_);
  rear_left_wheel_interface_handle_ = get_wheel(rear_left_wheel_joint_name_);
  rear_right_wheel_interface_handle_ = get_wheel(rear_right_wheel_joint_name_);

  if (!front_left_wheel_interface_handle_ || !front_right_wheel_interface_handle_ || !rear_left_wheel_interface_handle_ || !rear_right_wheel_interface_handle_)
  {
    return CallbackReturn::ERROR;
  }

  is_halted = false;
  subscriber_is_active_ = true;

  RCLCPP_DEBUG(get_node()->get_logger(), "Subscriber and publisher are now active.");
  return CallbackReturn::SUCCESS;
}

CallbackReturn MecanumController::on_deactivate(const rclcpp_lifecycle::State &)
{
  subscriber_is_active_ = false;
  return CallbackReturn::SUCCESS;
}

CallbackReturn MecanumController::on_cleanup(const rclcpp_lifecycle::State &)
{
  if (!reset())
  {
    return CallbackReturn::ERROR;
  }

  received_velocity_msg_ptr_.set(std::make_shared<Twist>());
  return CallbackReturn::SUCCESS;
}

CallbackReturn MecanumController::on_error(const rclcpp_lifecycle::State &)
{
  if (!reset())
  {
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

bool MecanumController::reset()
{
  subscriber_is_active_ = false;
  velocity_command_subscriber_.reset();
  velocity_command_unstamped_subscriber_.reset();

  received_velocity_msg_ptr_.set(nullptr);
  is_halted = false;
  return true;
}

CallbackReturn MecanumController::on_shutdown(const rclcpp_lifecycle::State &)
{
  return CallbackReturn::SUCCESS;
}

void MecanumController::halt()
{
  front_left_wheel_interface_handle_->set_velocity(0.0);
  front_right_wheel_interface_handle_->set_velocity(0.0);
  rear_left_wheel_interface_handle_->set_velocity(0.0);
  rear_right_wheel_interface_handle_->set_velocity(0.0);
  auto logger = get_node()->get_logger();
  RCLCPP_WARN(logger, "-----HALT CALLED : STOPPING ALL MOTORS-----");
}

std::shared_ptr<Wheel> MecanumController::get_wheel( const std::string & wheel_name )
{
  auto logger = get_node()->get_logger();
    if (wheel_name.empty())
    {
      RCLCPP_ERROR(logger, "Wheel joint name not given. Make sure all joints are specified.");
      return nullptr;
    }

    // Get Command Handle for joint
    const auto command_handle = std::find_if(
        command_interfaces_.begin(), command_interfaces_.end(),
        [&wheel_name](const auto &interface)
        {
          return interface.get_name() == wheel_name + "/velocity" &&
                 interface.get_interface_name() == HW_IF_VELOCITY;
        });
    const auto state_handle = std::find_if(
        state_interfaces_.cbegin(), state_interfaces_.cend(),
        [&wheel_name](const auto &interface)
        {
          return interface.get_name() == wheel_name + "/velocity" &&
                 interface.get_interface_name() == HW_IF_VELOCITY;
        });

    if (command_handle == command_interfaces_.end())
    {
      RCLCPP_ERROR(logger, "Unable to obtain joint command handle for %s", wheel_name.c_str());
      return nullptr;
    }
    if (state_handle == state_interfaces_.cend())
    {
      RCLCPP_ERROR(logger, "Unable to obtain joint state handle for %s", wheel_name.c_str());
      return nullptr;
    }
    auto cmd_interface_name = command_handle->get_name();
    RCLCPP_INFO(logger, "FOUND! wheel cmd interface %s", cmd_interface_name.c_str());
    return std::make_shared<Wheel>(std::ref(*command_handle),std::ref(*state_handle), wheel_name);
}
}  // namespace grr_cmake_controller

#include "class_loader/register_macro.hpp"

CLASS_LOADER_REGISTER_CLASS(
  grr_cmake_controller::MecanumController, controller_interface::ControllerInterface)
