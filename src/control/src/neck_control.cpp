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

   lost_ball_timer.reset(); 
   search_ball_timer.reset();
}

NeckNode::~NeckNode()
{
}

void NeckNode::listener_callback_vision_px(const Point2d::SharedPtr msg)
{
  ball_pos_px.x = msg->x - 640/2;
  ball_pos_px.y = msg->y - 480/2;

  lost_ball_timer.reset(); 

  if( this->robot_state == State::follow_ball)
  {
    auto new_neck_position = SetPosition();

    new_neck_position.id.push_back(19);
    new_neck_position.id.push_back(20);

    new_neck_position.position.push_back(neck.pan - ball_pos_px.x * 0.2);
    new_neck_position.position.push_back(neck.tilt - ball_pos_px.y * 0.15);

    if(new_neck_position.position[0] > 2650) new_neck_position.position[0] = 2650;
    else if(new_neck_position.position[0] < 1350) new_neck_position.position[0] = 1350;
    if(new_neck_position.position[1] > 2048) new_neck_position.position[1] = 2048;
    else if(new_neck_position.position[1] < 1200) new_neck_position.position[1] = 1200;

    RCLCPP_INFO(this->get_logger(), "search ball id 19: %d  |  id 20: %d", new_neck_position.position[0], new_neck_position.position[1]);

    set_neck_position_publisher_->publish(new_neck_position);
  }
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

uint64_t NeckNode::Millis() {
  using namespace std::chrono;
  return duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
}

void NeckNode::search_ball()
{
  
  if(search_ball_timer.delay(2000))
  {
    auto new_neck_position = SetPosition();

    new_neck_position.id.push_back(19);
    new_neck_position.id.push_back(20);
  
    new_neck_position.position.push_back(this->search_ball_pos[search_ball_state][0]);
    new_neck_position.position.push_back(this->search_ball_pos[search_ball_state][1]);
    
    RCLCPP_INFO(this->get_logger(), "search ball id 19: %d  |  id 20: %d!", new_neck_position.position[0], new_neck_position.position[1]);

    set_neck_position_publisher_->publish(new_neck_position);
    this->search_ball_state += 1;

    if(this->search_ball_state >= 8)
    {
      this->search_ball_state = 0;
    }
  }
}


void NeckNode::main_callback()
{
  switch (this->robot_state)
  {
  case State::search_ball:
    //RCLCPP_INFO(this->get_logger(), "Searching the ball");
    this->search_ball();
    break;
  }

  if(lost_ball_timer.delayNR(2000) && this->robot_state == State::follow_ball)
  {
    this->robot_state = State::search_ball;
    search_ball_timer.reset();
    this->search_ball_state = 0;
  }
  else if (!lost_ball_timer.delayNR(2000))
  {
    this->robot_state = State::follow_ball;
  }
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
