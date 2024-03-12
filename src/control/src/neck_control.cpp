#include "neck_control/neck_control.hpp"


NeckNode::NeckNode()
: Node("neck_node")
{
  RCLCPP_INFO(this->get_logger(), "Run neck node");


  callback_group_subscriber_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  auto sub_opt = rclcpp::SubscriptionOptions();

  vision_subscriber_ = this->create_subscription<VisionInfo>(
    "/ball_position", 10, std::bind(&NeckNode::listener_callback_vision, this, std::placeholders::_1), sub_opt);
    
  neck_position_subscriber_ = this->create_subscription<NeckPosition>(
    "/neck_position", 10, std::bind(&NeckNode::listener_callback_neck, this, std::placeholders::_1), sub_opt);
    
  set_neck_position_publisher_ = this->create_publisher<NeckPosition>("/set_neck_position", 10);


  main_thread_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto main_opt = rclcpp::SubscriptionOptions();
  main_opt.callback_group = main_thread_;

  main_timer_ = this->create_wall_timer(
    8ms, std::bind(&NeckNode::main_callback, this), main_thread_);
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

void NeckNode::listener_callback_neck(const NeckPosition::SharedPtr msg)
{
  neck.pan  = msg->position19;
  neck.tilt = msg->position20;
  RCLCPP_INFO(this->get_logger(), "id 19 '%d' / id 20: '%d'", neck.pan, neck.tilt);
}

void NeckNode::move_head(const enum Side &side, Neck &neck_position)
{
  switch (side)
  {
  case Side::left:
    neck_position.pan += 10;
    break;
  
  case Side::right:
    neck_position.pan -= 10;
    break;

  case Side::down:
    neck_position.tilt -= 10;
    break;
  
  case Side::up:
    neck_position.tilt += 10;
    break;
  }
}

void NeckNode::follow_ball()
{
  auto new_neck_position = NeckPosition();

  new_neck_position.position19 = neck.pan;
  new_neck_position.position20 = neck.tilt;

  if(ball.detected)
  {
    if(ball.left && neck.pan < 2650)
    {
      this->move_head(Side::left, this->neck);
      RCLCPP_INFO(this->get_logger(), "Move head to left");

    }
    else if(ball.right && neck.pan > 1350)
    {
      this->move_head(Side::right, this->neck);
      RCLCPP_INFO(this->get_logger(), "Move head to right");
    }
    else if(ball.far && neck.tilt < 2048)
    {
      this->move_head(Side::up, this->neck);
      RCLCPP_INFO(this->get_logger(), "Ball up");
    }
    else if(ball.close && neck.tilt > 1340)
    {
      this->move_head(Side::down, this->neck);
      RCLCPP_INFO(this->get_logger(), "Ball close");
    }
  }
  
  new_neck_position.position19 = neck.pan;
  new_neck_position.position20 = neck.tilt;

  set_neck_position_publisher_->publish(new_neck_position);


}

void NeckNode::main_callback()
{
  this->follow_ball();
}


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto neck_control_node = std::make_shared<NeckNode>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(neck_control_node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
