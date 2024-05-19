#ifndef MOTORS_COMMUNICATION_HPP_
#define MOTORS_COMMUNICATION_HPP_

#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "dynamixel_sdk/dynamixel_sdk.h"

using namespace dynamixel;
using namespace std::chrono_literals;

int dxl_comm_result = COMM_TX_FAIL;

class MotorsCommunication : public rclcpp::Node
{
public:
  MotorsCommunication();
  virtual ~MotorsCommunication();

  PacketHandler * packetHandler;

  void timer_callback();

private:
  rclcpp::TimerBase::SharedPtr timer_;
  
};

#endif  // MOTORS_COMMUNICATION_HPP_