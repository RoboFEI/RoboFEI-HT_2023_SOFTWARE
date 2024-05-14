// Copyright 2021 ROBOTIS CO., LTD.
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

#ifndef MOTORS_COMMUNICATION_HPP_
#define MOTORS_COMMUNICATION_HPP_

#include <cstdio>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rcutils/cmdline_parser.h"
#include "dynamixel_sdk/dynamixel_sdk.h"
#include "custom_interfaces/msg/set_position.hpp"
#include "custom_interfaces/msg/set_position_original.hpp"
#include "custom_interfaces/srv/get_position.hpp"
#include "custom_interfaces/msg/neck_position.hpp"



class ReadWriteNode : public rclcpp::Node
{
public:
  using SetPosition = custom_interfaces::msg::SetPosition;
  using SetPositionOriginal = custom_interfaces::msg::SetPositionOriginal;
  using GetPosition = custom_interfaces::srv::GetPosition;
  using NeckPosition = custom_interfaces::msg::NeckPosition;

  void timer_callback();
  void save_motors_position(const SetPosition::SharedPtr msg);
  int get_position(const int &id);
  bool set_position(const int &id, const int &position);

  uint8_t motores[21][4];
  uint32_t motores2[21];

  // std::vector<bool> torque_status(21, 1); 

  ReadWriteNode();
  virtual ~ReadWriteNode();

private:
  rclcpp::Subscription<SetPosition>::SharedPtr set_position_subscriber_;
  rclcpp::Subscription<SetPositionOriginal>::SharedPtr set_position_subscriber_single;
  rclcpp::Subscription<SetPosition>::SharedPtr set_neck_position_subscriber_;
  rclcpp::Service<GetPosition>::SharedPtr get_position_server_;
  rclcpp::Publisher<NeckPosition>::SharedPtr neck_position_publisher;
  rclcpp::Publisher<SetPosition>::SharedPtr all_joint_positions_publisher;

  rclcpp::TimerBase::SharedPtr timer_; // declaration of timer to publish the neck position

  int max_limit_position;
  int min_limit_position;
  // std::vector<int> motors;
  int motors [21];
  std::vector<u_int8_t> id { 1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20 };
  
};

#endif  // READ_WRITE_NODE_HPP_