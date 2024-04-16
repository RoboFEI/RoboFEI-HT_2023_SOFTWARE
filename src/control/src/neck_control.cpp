#include "neck_control/neck_control.hpp"


NeckNode::NeckNode()
: Node("neck_node")
{
  RCLCPP_INFO(this->get_logger(), "Run neck node");


  callback_group_subscriber_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  auto sub_opt = rclcpp::SubscriptionOptions();

  vision_subscriber_ = this->create_subscription<VisionInfo>(
    "/ball_position", 10, std::bind(&NeckNode::listener_callback_vision, this, std::placeholders::_1), sub_opt);
  
  vision_px_subscriber_ = this->create_subscription<Point2d>(
    "/ball_px_position", 10, std::bind(&NeckNode::listener_callback_vision_px, this, std::placeholders::_1), sub_opt);
    
  neck_position_subscriber_ = this->create_subscription<NeckPosition>(
    "/neck_position", 10, std::bind(&NeckNode::listener_callback_neck, this, std::placeholders::_1), sub_opt);
    
  set_neck_position_publisher_ = this->create_publisher<SetPosition>("/set_neck_position", 10);


  main_thread_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto main_opt = rclcpp::SubscriptionOptions();
  main_opt.callback_group = main_thread_;

  main_timer_ = this->create_wall_timer(
    8ms, std::bind(&NeckNode::main_callback, this), main_thread_);
}

NeckNode::~NeckNode()
{
}

void NeckNode::listener_callback_vision_px(const Point2d::SharedPtr msg)
{
  ball_pos_px.x = msg->x - 640/2;
  ball_pos_px.y = msg->y - 480/2;

  auto new_neck_position = SetPosition();

  new_neck_position.id.push_back(19);
  new_neck_position.id.push_back(20);

  new_neck_position.position.push_back(neck.pan - ball_pos_px.x * 0.0);
  new_neck_position.position.push_back(neck.tilt - ball_pos_px.y * 0.15);

  if(new_neck_position.position[0] > 2650) new_neck_position.position[0] = 2650;
  else if(new_neck_position.position[0] < 1350) new_neck_position.position[0] = 1350;
  if(new_neck_position.position[1] > 2048) new_neck_position.position[1] = 2048;
  else if(new_neck_position.position[1] < 1340) new_neck_position.position[1] = 1340;

  RCLCPP_INFO(this->get_logger(), "search ball id 19: %d  |  id 20: %d", new_neck_position.position[0], new_neck_position.position[1]);

  set_neck_position_publisher_->publish(new_neck_position);

}


































void NeckNode::listener_callback_vision(const VisionInfo::SharedPtr msg)
{
  ball.detected     =   msg->detected;
  //RCLCPP_INFO(this->get_logger(), "BALL '%s'", ball.detected ? "true" : "false");
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
  //RCLCPP_INFO(this->get_logger(), "id 19 '%d' / id 20: '%d'", neck.pan, neck.tilt);
}

void NeckNode::move_head(const enum Side &side, Neck &neck_position)
{
  // switch (side)
  // {
  // case Side::left:
  //   neck_position.pan += 10;
  //   break;
  
  // case Side::right:
  //   neck_position.pan -= 10;
  //   break;

  // case Side::down:
  //   neck_position.tilt -= 10;
  //   break;
  
  // case Side::up:
  //   neck_position.tilt += 10;
  //   break;
  // }
}

void NeckNode::follow_ball()
{
  // auto new_neck_position = SetPosition();

  // new_neck_position.id.push_back(19);
  // new_neck_position.id.push_back(20);
  
  // new_neck_position.position.push_back(neck.pan);
  // new_neck_position.position.push_back(neck.tilt);

  // if(ball.detected)
  // {
  //   if(ball.left && neck.pan < 2650)
  //   {
  //     this->move_head(Side::left, this->neck);
  //     RCLCPP_INFO(this->get_logger(), "Move head to left");

  //   }
  //   else if(ball.right && neck.pan > 1350)
  //   {
  //     this->move_head(Side::right, this->neck);
  //     RCLCPP_INFO(this->get_logger(), "Move head to right");
  //   }
    
  //   if(ball.far && neck.tilt < 2048)
  //   {
  //     this->move_head(Side::up, this->neck);
  //     RCLCPP_INFO(this->get_logger(), "Ball up");
  //   }
  //   else if(ball.close && neck.tilt > 1340)
  //   {
  //     this->move_head(Side::down, this->neck);
  //     RCLCPP_INFO(this->get_logger(), "Ball close");
  //   }
  //   else RCLCPP_INFO(this->get_logger(), "Centralized ball");
  // }
  

  // new_neck_position.position[0] = neck.pan;
  // new_neck_position.position[1] = neck.tilt;

  // set_neck_position_publisher_->publish(new_neck_position);
}

uint64_t NeckNode::Millis() {
  using namespace std::chrono;
  return duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
}

void NeckNode::search_ball()
{
  
  // if(this->Millis()-this->atual_time > 2000)
  // {
  //   auto new_neck_position = SetPosition();

  //   new_neck_position.id.push_back(19);
  //   new_neck_position.id.push_back(20);
  
  //   new_neck_position.position.push_back(this->search_ball_pos[search_ball_state][0]);
  //   new_neck_position.position.push_back(this->search_ball_pos[search_ball_state][1]);
    
  //   RCLCPP_INFO(this->get_logger(), "search ball id 19: %d  |  id 20: %d", new_neck_position.position[0], new_neck_position.position[1]);

  //   set_neck_position_publisher_->publish(new_neck_position);
  //   this->atual_time = this->Millis();
  //   this->search_ball_state += 1;

  //   if(this->search_ball_state >= 8)
  //   {
  //     this->search_ball_state = 0;
  //   }
  // }
}


void NeckNode::main_callback()
{
  // switch (this->robot_state)
  // {
  // case State::follow_ball:
  //   //RCLCPP_INFO(this->get_logger(), "following the ball");
  //   this->follow_ball();
  //   break;

  // case State::search_ball:
  //   //RCLCPP_INFO(this->get_logger(), "Searching the ball");
  //   this->search_ball();
  //   break;
  // }

  // if(this->ball.detected && this->robot_state != State::follow_ball)
  // {
  //   this->robot_state = State::follow_ball;
  //   this->cont_lost_ball = 0;
  //   this->search_ball_state = 0;
  // }
  // else if(!this->ball.detected && this->robot_state != State::search_ball)
  // {
  //   this->cont_lost_ball += 1;
  // }
  // if(this->cont_lost_ball > 200)
  // {
  //   this->robot_state = State::search_ball;
  // }
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
