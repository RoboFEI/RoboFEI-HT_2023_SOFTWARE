#ifndef MOTORS_COMMUNICATION_HPP_
#define MOTORS_COMMUNICATION_HPP_

#include <chrono>
#include <memory>
#include <vector>
#include <numeric>
#include <unistd.h>

#include <motors_pkg/motorsAttributes.hpp>

#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"
#include "dynamixel_sdk/dynamixel_sdk.h"

#include "custom_interfaces/msg/joint_state.hpp"


using namespace dynamixel;
using namespace std::chrono_literals;
using std::placeholders::_1;

typedef struct Joints
{
  std::vector<std::uint32_t> position = std::vector<std::uint32_t>(21, 2048);
  std::vector<std::uint32_t> velocity = std::vector<std::uint32_t>(21, 0);
  std::vector<std::uint8_t> torque = std::vector<std::uint8_t>(21, 1);
}Joints;



class MotorsCommunication : public rclcpp::Node
{
public:
  using JointStateMsg = custom_interfaces::msg::JointState;

  PacketHandler  *packetHandler;
  GroupSyncWrite *groupSyncWritePos;
  GroupSyncRead  *groupSyncReadPos;
  
  Joints joints;
  std::vector<int> allIds = std::vector<int>(20);
  int robotNumber;
  motorsAttributes *mtrs_att;

  int dxl_comm_result = COMM_TX_FAIL;

  void joint_state_callback(const JointStateMsg::SharedPtr joint_state_info);
  void timer_callback();

  void initialMotorsSetup(int id);
  void setJoints(JointStateMsg jointInfo);
  void setJointVel(int id, int goalVel);
  void setJointTorque(int id, int goalTorque);
  void setAllJointPos();
  void getNoTorquePos();


  uint8_t* convertInfo(int jointInfo);

  MotorsCommunication();
  virtual ~MotorsCommunication();

private:
  rclcpp::Subscription<JointStateMsg>::SharedPtr joint_state_subscription_;
  rclcpp::Publisher<JointStateMsg>::SharedPtr all_joints_position_publisher;

  rclcpp::TimerBase::SharedPtr timer_;  
};

#endif  // MOTORS_COMMUNICATION_HPP_