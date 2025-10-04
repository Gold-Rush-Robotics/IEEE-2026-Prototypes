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
 * Author: Enrique Fernández
 */

#include "grr_cmake_controller/odometry.hpp"


namespace grr_cmake_controller
{
Odometry::Odometry(size_t velocity_rolling_window_size)
: timestamp_(0.0),
  x_(0.0),
  y_(0.0),
  heading_(0.0),
  linear_x_(0.0),
  linear_y_(0.0),
  angular_z_(0.0),
  chassis_center_to_axle_(0.0),
  axle_center_to_wheel_(0.0),
  wheel_radius_(0.0),
  velocity_rolling_window_size_(velocity_rolling_window_size),
  linear_accumulator_x_(velocity_rolling_window_size),
  linear_accumulator_y_(velocity_rolling_window_size),
  angular_accumulator_(velocity_rolling_window_size)
{
}

void Odometry::init(const rclcpp::Time & time)
{
  // Reset accumulators and timestamp:
  timestamp_ = time;
  node_ = rclcpp::Node::make_shared("odometry");

   // PUBLISHER SETUP
  odom_publisher_ = node_->create_publisher<nav_msgs::msg::Odometry>("odom", rclcpp::SystemDefaultsQoS());
  realtime_odom_publisher_ = std::make_shared<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>>(
      odom_publisher_);
}

bool Odometry::update(double front_left_wheel_velocity,double front_right_wheel_velocity,double rear_left_wheel_velocity,double rear_right_wheel_velocity, const rclcpp::Time & time)
{
  // We cannot estimate the speed with very small time intervals:
  double dt = time.seconds() - timestamp_.seconds();
  timestamp_ = time;
  if (dt < 0.0001)
  {
    RCLCPP_WARN(node_->get_logger(), "Time interval between odometry updates too small, skipping update.");
    return false;
  }

  linear_x_ = (front_left_wheel_velocity + front_right_wheel_velocity + rear_left_wheel_velocity + rear_right_wheel_velocity)*wheel_radius_/4.0;
  linear_y_ = (front_right_wheel_velocity+rear_left_wheel_velocity-front_left_wheel_velocity-rear_right_wheel_velocity)*wheel_radius_/4;
  angular_z_ = (rear_right_wheel_velocity+front_right_wheel_velocity-front_left_wheel_velocity-rear_left_wheel_velocity)*wheel_radius_/(4*(chassis_center_to_axle_+axle_center_to_wheel_));


  // Integrate odometry:
  x_ += (linear_x_ * cos(heading_) - linear_y_ * sin(heading_))*dt;
  y_ += (linear_x_ * sin(heading_) + linear_y_ * cos(heading_))*dt;
  heading_ += angular_z_*dt;
  RCLCPP_INFO_ONCE(node_->get_logger(), "x: %f, y: %f, heading: %f", x_, y_, heading_);
  RCLCPP_DEBUG_SKIPFIRST(node_->get_logger(), "x: %f, y: %f, heading: %f", x_, y_, heading_);
  publish();
  return true;
}

void Odometry::resetOdometry()
{
  x_ = 0.0;
  y_ = 0.0;
  heading_ = 0.0;
}

void Odometry::setWheelParams(double wheel_radius, double chassis_center_to_axle, double axle_center_to_wheel)
{
  wheel_radius_ = wheel_radius;
  chassis_center_to_axle_ = chassis_center_to_axle;
  axle_center_to_wheel_ = axle_center_to_wheel;
}
void Odometry::setOdometry(double x, double y, double quat_x, double quat_y, double quat_z, double quat_w)
{
  x_ = x;
  y_ = y;
  heading_ = QuaternionToYaw(quat_x, quat_y, quat_z, quat_w);
}

double Odometry::QuaternionToYaw(double x, double y, double z, double w)
{
  double siny_cosp = 2 * (w * z + x * y);
  double cosy_cosp = 1 - 2 * (y * y + z * z);
  return std::atan2(siny_cosp, cosy_cosp);
}
void Odometry::publish()
{

  if (realtime_odom_publisher_->trylock()) {
    realtime_odom_publisher_->msg_.pose.pose.position.x = x_;
    realtime_odom_publisher_->msg_.pose.pose.position.y = y_;
    realtime_odom_publisher_->msg_.pose.pose.position.z = 0;
    realtime_odom_publisher_->msg_.pose.pose.orientation.w = cos(heading_/2);
    realtime_odom_publisher_->msg_.pose.pose.orientation.x = 0;
    realtime_odom_publisher_->msg_.pose.pose.orientation.y = 0;
    realtime_odom_publisher_->msg_.pose.pose.orientation.z = sin(heading_/2);
    realtime_odom_publisher_->msg_.twist.twist.linear.x = linear_x_;
    realtime_odom_publisher_->msg_.twist.twist.linear.y = linear_y_;
    realtime_odom_publisher_->msg_.twist.twist.angular.z= angular_z_;
    realtime_odom_publisher_->unlockAndPublish();
  }
  


  
}
// float[] eulerToQuat()
//   // qx = np.sin() * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
//   qx=0;
//   // qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
//   qy=0;
//   // qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
//   qz= sin(yaw/2);
//   // qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
//   qw= cos(yaw/2);

}  // namespace grr_cmake_controller
