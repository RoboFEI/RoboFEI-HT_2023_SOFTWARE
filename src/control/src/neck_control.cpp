#include "neck_control/neck_control.hpp"


NeckNode::NeckNode()
: Node("neck_node")
{
  RCLCPP_INFO(this->get_logger(), "Run neck node");

  vision_subscriber_ = this->create_subscription<VisionInfo>(
    "/ball_position", 10, std::bind(&NeckNode::listener_callback_vision, this, std::placeholders::_1));
    
  // neck_position_subscriber_ = this->create_subscription<NeckPosition>(
  //   "/neck_position", 10, std::bind(&NeckNode::listener_callback_neck, this, std::placeholders::_1));
    
  // set_neck_position_publisher_ = this->create_publisher<NeckPosition>("/set_neck_position", 10);

  // main_timer_ = this->create_wall_timer(
  //   8ms, std::bind(&NeckNode::main_callback, this));
}

NeckNode::~NeckNode()
{
}

void NeckNode::listener_callback_vision(const VisionInfo::SharedPtr msg)
{
  ball.detected     =   msg->detected;
  RCLCPP_INFO(this->get_logger(), "BALL '%s'", ball.detected ? "true" : "false");
  ball.left         =   msg->left;
  ball.center_left  =   msg->center_left;
  ball.right        =   msg->right;
  ball.center_right =   msg->center_right;
  
  ball.far          =   msg->far;
  ball.med          =   msg->med;
  ball.close        =   msg->close;
}


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NeckNode>());
  rclcpp::shutdown();
  return 0;
}
