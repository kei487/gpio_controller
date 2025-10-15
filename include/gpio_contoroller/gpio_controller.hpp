// SPDX-FileCopyrightText: 2025 Keitaro Nakamura
// SPDX-License-Identifier:MIT license 

#ifndef GPIO_CONTROLLER__GPIO_CONTROLLER_HPP_
#define GPIO_CONTROLLER__GPIO_CONTROLLER_HPP_


#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "joint_msgs/msg/joint_angle.hpp"
#include <gpiod.hpp>
#include <iostream>
#include <unistd.h> // usleep関数のため

using std::placeholders::_1;

namespace gpio_controller
{
class GPIOController: public rclcpp::Node
{
public:
  explicit GPIOController(const rclcpp::NodeOptions & options);

protected:
  void getParam();
  void initGPIOPIN();

private:
  void topic_callback(const geometry_msgs::msg::Twist & message)
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

  void motor_move();
  double calc_pwm(double speed);

};

}  // namespace gpio_controller

#endif  // SIMPLE_WAYPOINT_FOLLOWER__SIMPLE_WAYPOINT_FOLLOWER_HPP_
