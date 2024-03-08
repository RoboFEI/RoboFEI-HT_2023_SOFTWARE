#ifndef NECK_CONTROL_HPP_
#define NECK_CONTROL_HPP_


#include <chrono>
#include <functional>
#include <memory>
#include <string>


#include "rclcpp/rclcpp.hpp"
#include "custom_interfaces/msg/vision.hpp"
#include "custom_interfaces/msg/neck_position.hpp"

struct ball_information
{
  bool detected   = false;
  bool left       = false;
  bool left       = false;
  bool right      = false;
  bool right      = false;
  bool far        = false;
  bool med        = false;
  bool close      = false;
};


using namespace std::chrono_literals;


class NeckNode : public rclcpp::Node
{
  public:
    using VisionInfo = custom_interfaces::msg::Vision;
    using NeckPosition = custom_interfaces::msg::NeckPosition;
    
    ball_information ball;


    NeckNode();
    virtual ~NeckNode();

  private:
    rclcpp::Subscription<BallPosition>::SharedPtr vision_subscriber_;
    rclcpp::Subscription<NeckPosition>::SharedPtr neck_position_subscriber_;
    rclcpp::Publisher<NeckPosition>::SharedPtr set_neck_position_publisher_;
    
};

#endif 

