// ros2 run control neck_control --ros-args -p only_body:=true

#include "neck_control/neck_control.hpp"


NeckNode::NeckNode()
: Node("neck_node")
{
  RCLCPP_INFO(this->get_logger(), "Run neck node");


  callback_group_subscriber_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  auto sub_opt = rclcpp::SubscriptionOptions();

  vision_subscriber_ = this->create_subscription<VisionInfo>(
    "ball_position", 10, std::bind(&NeckNode::listener_callback_vision, this, std::placeholders::_1), sub_opt);
  
  vision_px_subscriber_ = this->create_subscription<Point2d>(
    "ball_px_position", 10, std::bind(&NeckNode::listener_callback_vision_px, this, std::placeholders::_1), sub_opt);
    
  neck_position_subscriber_ = this->create_subscription<JointStateMsg>(
    "all_joints_position", 10, std::bind(&NeckNode::listener_callback_neck, this, std::placeholders::_1), sub_opt);
    
  set_neck_position_publisher_ = this->create_publisher<JointStateMsg>("set_joint_topic", 10);


  main_thread_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto main_opt = rclcpp::SubscriptionOptions();
  main_opt.callback_group = main_thread_;

  main_timer_ = this->create_wall_timer(
    8ms, std::bind(&NeckNode::main_callback, this), main_thread_);

   lost_ball_timer.reset(); 
   search_ball_timer.reset();

  this->declare_parameter("x_p_gain", 0.7);
  this->declare_parameter("y_p_gain", 0.35);

  this->get_parameter("x_p_gain", x_p_gain);
  this->get_parameter("y_p_gain", y_p_gain);

  neck_activate_ = this->declare_parameter("neck_activate", true);
  RCLCPP_INFO(this->get_logger(), "neck activate %d", neck_activate_);

  neck_up_limit = this->declare_parameter("neck_up_limit", 2048);
  neck_down_limit = this->declare_parameter("neck_down_limit", 1350);
  neck_left_limit = this->declare_parameter("neck_left_limit", 2650);
  neck_right_limit = this->declare_parameter("neck_right_limit", 1350);
  robotNumber = this->declare_parameter("robot_number", 2);
  if(robotNumber > 2) search_ball_pos = {{2270/4,1300/4}, {2048/4, 1300/4}, {1826/4, 1300/4}, {1528/4, 1550/4}, {2048/4, 1550/4}, {2568/4, 1550/4}, {2866/4, 1800/4}, {2048/4, 1800/4},{1230/4, 1800/4}};
}

NeckNode::~NeckNode()
{
}

void NeckNode::listener_callback_vision_px(const Point2d::SharedPtr msg)
{
  if(neck_activate_)
  {
    ball_pos_px.x = msg->x - 640/2;
    ball_pos_px.y = msg->y - 480/2;

    lost_ball_timer.reset(); 

    if( this->robot_state == State::follow_ball)
    {
      auto new_neck_position = JointStateMsg();

      new_neck_position.id.push_back(19);
      new_neck_position.id.push_back(20);

      new_neck_position.info.push_back(neck.pan - ball_pos_px.x * x_p_gain);
      new_neck_position.info.push_back(neck.tilt - ball_pos_px.y * y_p_gain);

      new_neck_position.type.push_back(JointStateMsg::POSITION);
      new_neck_position.type.push_back(JointStateMsg::POSITION);


      if(new_neck_position.info[0] > neck_left_limit) new_neck_position.info[0] = neck_left_limit;
      else if(new_neck_position.info[0] < neck_right_limit) new_neck_position.info[0] = neck_right_limit;
      if(new_neck_position.info[1] > neck_up_limit) new_neck_position.info[1] = neck_up_limit;
      else if(new_neck_position.info[1] < neck_down_limit) new_neck_position.info[1] = neck_down_limit;

      RCLCPP_INFO(this->get_logger(), "search ball id 19: %d  |  id 20: %d", new_neck_position.info[0], new_neck_position.info[1]);

      set_neck_position_publisher_->publish(new_neck_position);
    }
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

void NeckNode::listener_callback_neck(const JointStateMsg::SharedPtr msg)
{
  neck.pan  = msg->info[19];
  neck.tilt = msg->info[20];
  //RCLCPP_INFO(this->get_logger(), "id 19 '%d' / id 20: '%d'", neck.pan, neck.tilt);
}

void NeckNode::search_ball()
{
  
  if(search_ball_timer.delay(1000 + (325 * (search_ball_state/3))))
  {
    auto new_neck_position = JointStateMsg();

    new_neck_position.id.push_back(19);
    new_neck_position.id.push_back(20);
  
    new_neck_position.info.push_back(this->search_ball_pos[search_ball_state][0]);
    new_neck_position.info.push_back(this->search_ball_pos[search_ball_state][1]);

    new_neck_position.type.push_back(JointStateMsg::POSITION);
    new_neck_position.type.push_back(JointStateMsg::POSITION);
    
    RCLCPP_INFO(this->get_logger(), "search ball id 19: %d  |  id 20: %d!", new_neck_position.info[0], new_neck_position.info[1]);

    if(neck_activate_) set_neck_position_publisher_->publish(new_neck_position);

    this->search_ball_state += 1;

    if(this->search_ball_state >= 8) this->search_ball_state = 0;
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
