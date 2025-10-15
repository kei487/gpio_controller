// SPDX-FileCopyrightText: 2025 Keitaro Nakamura
// SPDX-License-Identifier: MIT-License
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "joint_msgs/msg/joint_angle.hpp"
#include <gpiod.hpp>
#include <iostream>
#include <unistd.h> // usleep関数のため

using std::placeholders::_1;
// BCMピン番号の定義
const int M1_PIN = 20;
const int M2_PIN = 21;
const int PWMA_PIN = 26;
const int M3_PIN = 6;
const int M4_PIN = 13;
const int PWMB_PIN = 12;

// GPIOチップ名 (Raspberry Pi 5では "gpiochip4")
const std::string CHIP = "gpiochip4";

// グローバルにラインオブジェクトを保持
gpiod::line m1_line, m2_line, pwma_line, m3_line, m4_line, pwmb_line;


namespace gpio_controller
{
GPIOController::GPIOController()
: Node("GPIOController")
{
  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
  "/cmd_vel", 10, std::bind(&GPIOController::topic_callback, this, _1));

  getParam();
  initGPIOPIN()  
}

void initGPIOPIN(){
  try {
      // GPIOチップを開く
      gpiod::chip chip(CHIP);

      // 各ピンを取得
      m1_line = chip.get_line(M1_PIN);
      m2_line = chip.get_line(M2_PIN);
      pwma_line = chip.get_line(PWMA_PIN);
      m3_line = chip.get_line(M3_PIN);
      m4_line = chip.get_line(M4_PIN);
      pwmb_line = chip.get_line(PWMB_PIN);

      // 全てのピンを出力としてリクエスト
      m1_line.request({"motor_cpp", gpiod::line_request::DIRECTION_OUTPUT, 0});
      m2_line.request({"motor_cpp", gpiod::line_request::DIRECTION_OUTPUT, 0});
      pwma_line.request({"motor_cpp", gpiod::line_request::DIRECTION_OUTPUT, 0});
      m3_line.request({"motor_cpp", gpiod::line_request::DIRECTION_OUTPUT, 0});
      m4_line.request({"motor_cpp", gpiod::line_request::DIRECTION_OUTPUT, 0});
      pwmb_line.request({"motor_cpp", gpiod::line_request::DIRECTION_OUTPUT, 0});
  } catch (const std::exception& e) {
      std::cerr << "エラー: " << e.what() << std::endl;
      return 1;
  }
}

void topic_callback(const geometry_msgs::msg::Twist & message) const
{

      std::cout << "前進します (2秒)" << std::endl;
      forward();
      sleep(2);

      std::cout << "後退します (2秒)" << std::endl;
      backward();
      sleep(2);
      
      std::cout << "停止します" << std::endl;
      motor_stop();
}

  // モーター回転
void motor_move() {
    m1_line.set_value(0);
    m2_line.set_value(0);
    pwma_line.set_value(0);
    m3_line.set_value(0);
    m4_line.set_value(0);
    pwmb_line.set_value(0);
}

};
} //namespace gpio_controller

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Listener>());
  rclcpp::shutdown();
  return 0;
}
