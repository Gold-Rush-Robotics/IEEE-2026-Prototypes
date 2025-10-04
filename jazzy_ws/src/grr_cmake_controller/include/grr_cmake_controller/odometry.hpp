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
 * Author: Luca Marchionni
 * Author: Bence Magyar
 * Author: Enrique Fernández
 * Author: Paul Mathieu
 */

#ifndef GRR_CMAKE_CONTROLLER__ODOMETRY_HPP_
#define GRR_CMAKE_CONTROLLER__ODOMETRY_HPP_

#include <cmath>

#include "grr_cmake_controller/rolling_mean_accumulator.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_with_covariance.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist_with_covariance.hpp"
#include "rclcpp/time.hpp"
#include "realtime_tools/realtime_box.h"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"

namespace grr_cmake_controller
{
class Odometry
{
public:
  explicit Odometry(size_t velocity_rolling_window_size = 10);

  void init(const rclcpp::Time & time);
  bool update(double front_left_wheel_velocity,double front_right_wheel_velocity,double rear_left_wheel_velocity,double rear_right_wheel_velocity, const rclcpp::Time & time);
  void resetOdometry();
  void publish();

  double getX() const { return x_; }
  double getY() const { return y_; }
  double getHeading() const { return heading_; }
  double getLinearX() const { return linear_x_; }
  double getLinearY() const { return linear_y_; }
  double getAngular() const { return angular_z_; }

  void setOdometry(double x, double y, double quat_x, double quat_y, double quat_z, double quat_w);
  double QuaternionToYaw(double x, double y, double z, double w);

  void setWheelParams(double wheel_radius, double chassis_center_to_axle, double axle_center_to_wheel);
  void setVelocityRollingWindowSize(size_t velocity_rolling_window_size);

private:
  using RollingMeanAccumulator = grr_cmake_controller::RollingMeanAccumulator<double>;

  // Current timestamp:
  rclcpp::Time timestamp_;

  // Current pose:
  double x_;        //   [m]
  double y_;        //   [m]
  double heading_;
  double linear_x_;
  double linear_y_;
  double angular_z_;
  double chassis_center_to_axle_;
  double axle_center_to_wheel_;
  double wheel_radius_;
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>> odom_publisher_ = nullptr;
  std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>> realtime_odom_publisher_ = nullptr;

  // Rolling mean accumulators for the linear and angular velocities:
  size_t velocity_rolling_window_size_;
  RollingMeanAccumulator linear_accumulator_x_;
  RollingMeanAccumulator linear_accumulator_y_;
  RollingMeanAccumulator angular_accumulator_;
};

}  // namespace grr_cmake_controller

#endif  // GRR_CMAKE_CONTROLLER__ODOMETRY_HPP_
