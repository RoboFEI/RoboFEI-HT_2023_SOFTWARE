#ifndef NECK_CONTROL_HPP_
#define NECK_CONTROL_HPP_

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "custom_interfaces/msg/vision.hpp"
#include "custom_interfaces/msg/neck_position.hpp"

using namespace std::chrono_literals;

struct BallInformation
{
  bool detected     = false;
  bool left         = false;
  bool center_left  = false;
  bool right        = false;
  bool center_right = false;
  bool far          = false;
  bool med          = false;
  bool close        = false;
};

struct Neck
{
  int pan   = 2048;
  int tilt  = 2048;
};

enum class Side 
{
  left,
  right,
  down,
  up 
};

enum class State
{
  follow_ball,
  search_ball
};

class NeckNode : public rclcpp::Node
{
  public:
    using VisionInfo = custom_interfaces::msg::Vision;
    using NeckPosition = custom_interfaces::msg::NeckPosition;

    BallInformation ball;
    Neck neck;
    State robot_state = State::search_ball;
    int cont_lost_ball = 0;
    uint64_t atual_time = this->Millis();
    int search_ball_pos[9][2] = {{2270,1300}, {2048, 1300}, {1826, 1300}, {1528, 1550}, {2048, 1550}, {2568, 1550}, {2866, 1800}, {2048, 1800},{1230, 1800}};
    int search_ball_state = 0;

    void listener_callback_vision(const VisionInfo::SharedPtr msg);
    void listener_callback_neck(const NeckPosition::SharedPtr msg); 
    void move_head(const enum Side &side, Neck &neck_position);
    void follow_ball();
    void search_ball();
    uint64_t Millis();
    void main_callback();
       

    NeckNode();
    virtual ~NeckNode();

  private:
    rclcpp::CallbackGroup::SharedPtr callback_group_subscriber_;
    rclcpp::CallbackGroup::SharedPtr main_thread_;

    rclcpp::Subscription<VisionInfo>::SharedPtr vision_subscriber_;
    rclcpp::Subscription<NeckPosition>::SharedPtr neck_position_subscriber_;
    rclcpp::Publisher<NeckPosition>::SharedPtr set_neck_position_publisher_;
    rclcpp::TimerBase::SharedPtr main_timer_;
    
};

#endif 

