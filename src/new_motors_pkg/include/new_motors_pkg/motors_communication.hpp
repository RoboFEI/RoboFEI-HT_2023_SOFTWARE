#ifndef MOTORS_COMMUNICATION_HPP_
#define MOTORS_COMMUNICATION_HPP_

#include <chrono>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "dynamixel_sdk/dynamixel_sdk.h"

#include "custom_interfaces/msg/joint_state.hpp"


using namespace dynamixel;
using namespace std::chrono_literals;
using std::placeholders::_1;

int dxl_comm_result = COMM_TX_FAIL;

typedef struct Joints
{
  std::vector<std::uint32_t> position = std::vector<std::uint32_t>(21, 2048) ;
  std::vector<std::uint32_t> velocity = std::vector<std::uint32_t>(21, 0);
  std::vector<std::uint8_t> torque = std::vector<std::uint8_t>(21, 1);
}Joints;


class MotorsCommunication : public rclcpp::Node
{
public:
  using JointStateMsg = custom_interfaces::msg::JointState;

  PacketHandler * packetHandler;
  GroupSyncWrite *groupSyncWritePos;

  Joints joints;
  
  void joint_state_callback(const JointStateMsg::SharedPtr joint_state_info);
  void timer_callback();

  void setJoints(JointStateMsg jointInfo);

  MotorsCommunication();
  virtual ~MotorsCommunication();

private:
  rclcpp::Subscription<JointStateMsg>::SharedPtr joint_state_subscription_;

  rclcpp::TimerBase::SharedPtr timer_;
  
};

#endif  // MOTORS_COMMUNICATION_HPP_