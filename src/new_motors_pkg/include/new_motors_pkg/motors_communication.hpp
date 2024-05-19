#ifndef MOTORS_COMMUNICATION_HPP_
#define MOTORS_COMMUNICATION_HPP_

#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "dynamixel_sdk/dynamixel_sdk.h"

using namespace std::chrono_literals;

class MotorsCommunication : public rclcpp::Node
{
public:
  MotorsCommunication();
  virtual ~MotorsCommunication();

  void timer_callback();

private:
  rclcpp::TimerBase::SharedPtr timer_;
  
};

#endif  // READ_WRITE_NODE_HPP_