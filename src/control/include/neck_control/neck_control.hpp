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


class NeckNode : public rclcpp::Node
{
  public:
    using VisionInfo = custom_interfaces::msg::Vision;
    using NeckPosition = custom_interfaces::msg::NeckPosition;
    
    BallInformation ball;
    Neck neck;

    void listener_callback_vision(const VisionInfo::SharedPtr msg);
    void listener_callback_neck(const NeckPosition::SharedPtr msg); 


    NeckNode();
    virtual ~NeckNode();

  private:
    rclcpp::Subscription<VisionInfo>::SharedPtr vision_subscriber_;
    rclcpp::Subscription<NeckPosition>::SharedPtr neck_position_subscriber_;
    // rclcpp::Publisher<NeckPosition>::SharedPtr set_neck_position_publisher_;
    
};

#endif 

