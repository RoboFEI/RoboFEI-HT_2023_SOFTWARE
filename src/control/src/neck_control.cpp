#include "neck_control/neck_control.hpp"

NeckNode::NeckNode()
: Node("neck_node")
{
    RCLCPP_INFO(this->get_logger(), "Run neck node");

    vision_subscriber_ = this->create_subscription<VisionInfo>(
      "/ball_position", 10, std::bind(&NeckNode::listener_callback_vision, this, std::placeholders::_1));
    

    neck_position_subscriber_ = this->create_subscription<NeckPosition>(
      "/neck_position", 10, std::bind(&NeckNode::listener_callback_neck, this, std::placeholders::_1));
    

    set_neck_position_publisher_ = this->create_publisher<NeckPosition>("/set_neck_position", 10)

    
}