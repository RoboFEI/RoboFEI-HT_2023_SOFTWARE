// ros2 run control neck_control --ros-args -p only_body:=true

#include "neck_control/neck_control.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/bool.hpp"


NeckNode::NeckNode()
: Node("neck_node")
{
  RCLCPP_INFO(this->get_logger(), "Run neck node");


  callback_group_subscriber_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  auto sub_opt = rclcpp::SubscriptionOptions();

  vision_subscriber_ = this->create_subscription<VisionInfo>(
    "ball_position", 10, std::bind(&NeckNode::listener_callback_vision, this, std::placeholders::_1), sub_opt);

  //le se o gol foi detectado e mais algumas coisas 
  vision_goal_center_subscriber = this->create_subscription<VisionInfo>(    
    "goal_center_position", 10, std::bind(&NeckNode::listener_callback_goal_center_vision, this, std::placeholders::_1), sub_opt); 

  //Conta a quantidade de traves
  //vision_subscriber_goal_post_counter = this->create_subscription<Int32>(
    //"/goalpost_count", 10, std::bind(&NeckNode::listener_callback_goal_post_counter, this, std::placeholders::_1), sub_opt);
    
  //le a posição da trave 
  //vision_goal_post_subscriber = this->create_subscription<VisionInfo>(
    //"goalpost_position", 10, std::bind(&NeckNode::listener_callback_goal_post_pos, this, std::placeholders::_1), sub_opt);

  //le a posição do centro do gol   
  vision_subscriber_goal_center = this->create_subscription<Point2d>(    
    "goalpost_center_px", 10, std::bind(&NeckNode::listener_callback_goal_center, this, std::placeholders::_1), sub_opt);

  //le a troca de estado do pescoço feito pela decisao  
  neck_state_subscriber = this->create_subscription<Int32>(    
    "/neck_state", 10, std::bind(&NeckNode::change_neck_state, this, std::placeholders::_1), sub_opt);

  //le a posição da trave
  //vision_subscriber_goal_post = this->create_subscription<Point2d>(
    //"/goalpost_px_position", 10, std::bind(&NeckNode::listener_callback_goal_post, this, std::placeholders::_1), sub_opt);

  vision_px_subscriber_ = this->create_subscription<Point2d>(
    "ball_px_position", 10, std::bind(&NeckNode::listener_callback_vision_px, this, std::placeholders::_1), sub_opt);

  neck_position_subscriber_ = this->create_subscription<JointStateMsg>(
    "all_joints_position", 10, std::bind(&NeckNode::listener_callback_neck, this, std::placeholders::_1), sub_opt);
  
  neck_control_lock_sub_ = this->create_subscription<std_msgs::msg::Bool>(
      "neck_control_locked", 10,
      std::bind(&NeckNode::neck_lock_callback, this, std::placeholders::_1)
  );
  

  set_neck_position_publisher_ = this->create_publisher<JointStateMsg>("set_joint_topic", 10);

  //Publisher para verificar se completou um ciclo de searching ball
  neck_finished_search_publisher = this->create_publisher<std_msgs::msg::Bool>("neck_search_finished", 10);

  main_thread_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto main_opt = rclcpp::SubscriptionOptions();
  main_opt.callback_group = main_thread_;

  main_timer_ = this->create_wall_timer(
    8ms, std::bind(&NeckNode::main_callback, this), main_thread_);

   lost_ball_timer.reset(); 
   search_ball_timer.reset();

  this->declare_parameter("x_p_gain", 0.6);
  this->declare_parameter("y_p_gain", 0.7);

  this->declare_parameter("x_i_gain", 0.01);
  this->declare_parameter("y_i_gain", 0.01);

  this->declare_parameter("x_d_gain", 0.2);
  this->declare_parameter("y_d_gain", 0.2);

  this->get_parameter("x_i_gain", x_i_gain);
  this->get_parameter("y_i_gain", y_i_gain);

  this->get_parameter("x_d_gain", x_d_gain);
  this->get_parameter("y_d_gain", y_d_gain);


  this->get_parameter("x_p_gain", x_p_gain);
  this->get_parameter("y_p_gain", y_p_gain);

  neck_activate_ = this->declare_parameter("neck_activate", true);
  //RCLCPP_INFO(this->get_logger(), "neck activate %d", neck_activate_);

  neck_up_limit = this->declare_parameter("neck_up_limit", 2048);
  neck_down_limit = this->declare_parameter("neck_down_limit", 1050);
  neck_left_limit = this->declare_parameter("neck_left_limit", 3050);
  neck_right_limit = this->declare_parameter("neck_right_limit", 1350);
  robotNumber = this->declare_parameter("robot_number", 2);
  if(robotNumber > 2)
  {
    neck.pan  = 2048;
    neck.tilt = 2048;  
  }

  received_neck_lock_ = false;
}

NeckNode::~NeckNode()
{
}

void NeckNode::listener_callback_vision_px(const Point2d::SharedPtr msg)
{

    ball_pos_px.x = msg->x - 640/2;
    ball_pos_px.y = msg->y - 480/2;

    lost_ball_timer.reset(); 

    if(neck_activate_ && this->robot_state == State::search_ball)
    {
      auto new_neck_position = JointStateMsg();

      error_x = ball_pos_px.x;
      error_y = ball_pos_px.y;

      P_x = error_x * x_p_gain;
      P_y = error_y * y_p_gain;

      new_neck_position.id.push_back(19);
      new_neck_position.id.push_back(20);

      new_neck_position.info.push_back(neck.pan - P_x);
      new_neck_position.info.push_back(neck.tilt - P_y);

      new_neck_position.type.push_back(JointStateMsg::POSITION);
      new_neck_position.type.push_back(JointStateMsg::POSITION);


      if(new_neck_position.info[0] > neck_left_limit) new_neck_position.info[0] = neck_left_limit;
      else if(new_neck_position.info[0] < neck_right_limit) new_neck_position.info[0] = neck_right_limit;
      if(new_neck_position.info[1] > neck_up_limit) new_neck_position.info[1] = neck_up_limit;
      else if(new_neck_position.info[1] < neck_down_limit) new_neck_position.info[1] = neck_down_limit;

      RCLCPP_DEBUG(this->get_logger(), "search ball id 19: %d  |  id 20: %d", new_neck_position.info[0], new_neck_position.info[1]);

      set_neck_position_publisher_->publish(new_neck_position);

      last_error_x = error_x;
      last_error_y = error_y;
    }
    if(ball.detected == false){
      search_ball();
    }
  }

/*
void NeckNode::listener_callback_goal_post(const Point2d::SharedPtr msg)
{

    goal_post.x = msg->x - 640/2;    
    goal_post.y = msg->y - 480/2;
    lost_ball_timer.reset();   

    if(neck_activate_)    
    { 
      auto new_neck_position = JointStateMsg();
      new_neck_position.id.push_back(19);
      new_neck_position.id.push_back(20);

      new_neck_position.info.push_back(neck.pan - goal_center.x * x_p_gain);
      new_neck_position.info.push_back(neck.tilt - goal_center.y * y_p_gain);

      new_neck_position.type.push_back(JointStateMsg::POSITION);
      new_neck_position.type.push_back(JointStateMsg::POSITION);

      if(new_neck_position.info[0] > neck_left_limit) new_neck_position.info[0] = neck_left_limit;
      else if(new_neck_position.info[0] < neck_right_limit) new_neck_position.info[0] = neck_right_limit;
      if(new_neck_position.info[1] > neck_up_limit) new_neck_position.info[1] = neck_up_limit;      
      else if(new_neck_position.info[1] < neck_down_limit) new_neck_position.info[1] = neck_down_limit;

      RCLCPP_DEBUG(this->get_logger(), "search ball id 19: %d  |  id 20: %d", new_neck_position.info[0], new_neck_position.info[1]);

      set_neck_position_publisher_->publish(new_neck_position);
    }  
    if(goalPost.detected == false){
      search_goalcenter();
    }
}
*/
void NeckNode::listener_callback_goal_center(const Point2d::SharedPtr msg)
{

    goal_center.x = msg->x - 640/2;    
    goal_center.y = msg->y - 480/2;
    lost_ball_timer.reset();   

    if(neck_activate_ && this->robot_state == State::search_goalcenter) 
    { 
      auto new_neck_position = JointStateMsg();

      error_x = goal_center.x;
      error_y = goal_center.y;

      P_x = error_x * x_p_gain;
      P_y = error_y * y_p_gain;

      acc_error_x = acc_error_x + error_x;
      acc_error_y = acc_error_y + error_y;

      if(acc_error_x > 100) acc_error_x = 100;
      if(acc_error_y < -100) acc_error_y = -100;

      I_x = acc_error_x * x_i_gain;
      I_y = acc_error_y * y_i_gain;

      vel_error_x = (error_x - last_error_x);
      vel_error_y = (error_y - last_error_y);

      D_x = vel_error_x * x_d_gain;
      D_y = vel_error_y * y_d_gain;

      new_neck_position.id.push_back(19);
      new_neck_position.id.push_back(20);
      
      new_neck_position.info.push_back(neck.pan - (P_x + I_x + D_x));
      new_neck_position.info.push_back(neck.tilt - (P_y + I_y + D_y));

      new_neck_position.type.push_back(JointStateMsg::POSITION);
      new_neck_position.type.push_back(JointStateMsg::POSITION);

      if(new_neck_position.info[0] > neck_left_limit) new_neck_position.info[0] = neck_left_limit;
      else if(new_neck_position.info[0] < neck_right_limit) new_neck_position.info[0] = neck_right_limit;
      if(new_neck_position.info[1] > neck_up_limit) new_neck_position.info[1] = neck_up_limit;      
      else if(new_neck_position.info[1] < neck_down_limit) new_neck_position.info[1] = neck_down_limit;

      RCLCPP_DEBUG(this->get_logger(), "search ball id 19: %d  |  id 20: %d", new_neck_position.info[0], new_neck_position.info[1]);

      set_neck_position_publisher_->publish(new_neck_position);

      last_error_x = error_x;
      last_error_y = error_y;
    }  
    if(goalCenter.detected == false){
      search_goalcenter();
    }
}

void NeckNode::listener_callback_vision(const VisionInfo::SharedPtr msg)
{
  ball.detected     =   msg->detected;
  //RCLCPP_INFO(this->get_logger(), "BALL '%s'", ball.detected ? "true" : "false");
  ball.left         =   msg->left;
  ball.center       =   msg->center;
  ball.right        =   msg->right;

  ball.far          =   msg->far;
  ball.med          =   msg->med;
  ball.close        =   msg->close;
}

void NeckNode::listener_callback_goal_center_vision(const VisionInfo::SharedPtr msg)
{  
  goalCenter.detected     =   msg->detected;  
  goalCenter.left         =   msg->left;
  goalCenter.center       =   msg->center;
  //RCLCPP_INFO(this->get_logger(), "CENTER '%s'", goalCenter.center ? "true" : "false");
  goalCenter.right        =   msg->right;

  goalCenter.far          =   msg->far;
  goalCenter.med          =   msg->med;
  //RCLCPP_INFO(this->get_logger(), "MED '%s'", goalCenter.med ? "true" : "false");
  goalCenter.close        =   msg->close;
}
/*
void NeckNode::listener_callback_goal_post_pos(const VisionInfo::SharedPtr msg)
{
  goalPost.detected     =   msg->detected;  
  //RCLCPP_INFO(this->get_logger(), "GOAL '%s'", goalPost.detected ? "true" : "false");
  goalPost.left         =   msg->left;
  goalPost.center       =   msg->center;
  goalPost.right        =   msg->right;

  goalPost.far          =   msg->far;
  goalPost.med          =   msg->med;
  goalPost.close        =   msg->close;
}

void NeckNode::listener_callback_goal_post_counter(const Int32::SharedPtr msg)
{
  goal_post_counter = msg->data;
  RCLCPP_INFO(this->get_logger(), "Tem '%d' traves", goal_post_counter);
}
*/
void NeckNode::listener_callback_neck(const JointStateMsg::SharedPtr msg)
{
  neck.pan  = msg->info[19];
  neck.tilt = msg->info[20];
  //RCLCPP_INFO(this->get_logger(), "id 19 '%d' / id 20: '%d'", neck.pan, neck.tilt);
}

void NeckNode::neck_lock_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
    neck_locked_ = msg->data;
    received_neck_lock_ = true;
}

void NeckNode::search_ball()
{
  if (neck_locked_ || !received_neck_lock_)
  {
      if(search_ball_timer.delay(search_ball_delay))
      {
        auto new_neck_position = JointStateMsg();

        if(this->search_ball_state < search_ball_samples[0])
          {
            search_ball_pos = {search_ball_limits[0],1300};
            if(this->search_ball_state > 0) this->search_ball_pos[0] -= (((search_ball_limits[0]*2)-4096)/(search_ball_samples[0]))*search_ball_state;
          }
        if(this->search_ball_state >= search_ball_samples[0] && this->search_ball_state <= (search_ball_samples[0]+search_ball_samples[1]))
          {
            search_ball_pos = {(4096-search_ball_limits[1]),1550};
            if(this->search_ball_state > (search_ball_samples[0])) this->search_ball_pos[0] += (((search_ball_limits[1]*2)-4096)/(search_ball_samples[1]))*(search_ball_state-search_ball_samples[0]);
          }
        if(this->search_ball_state >= (search_ball_samples[0]+search_ball_samples[1]))
          {
            search_ball_pos = {search_ball_limits[2],1800};
            if(this->search_ball_state > (search_ball_samples[0]+search_ball_samples[1])) this->search_ball_pos[0] -= (((search_ball_limits[2]*2)-4096)/(search_ball_samples[2]))*(search_ball_state-(search_ball_samples[0]+search_ball_samples[1]));
          }
        if(robotNumber == 4) 
          {
            this->search_ball_pos[0] = (search_ball_pos[0])*(0.25);
            this->search_ball_pos[1] = (search_ball_pos[1])*(0.25);
          }
        new_neck_position.id.push_back(19);
        new_neck_position.id.push_back(20);
      
        new_neck_position.info.push_back(this->search_ball_pos[0]);
        new_neck_position.info.push_back(this->search_ball_pos[1]);

        new_neck_position.type.push_back(JointStateMsg::POSITION);
        new_neck_position.type.push_back(JointStateMsg::POSITION);

        //RCLCPP_DEBUG(this->get_logger(), "search ball id 19: %d  |  id 20: %d!", new_neck_position.info[0], new_neck_position.info[1]);

        if(neck_activate_) set_neck_position_publisher_->publish(new_neck_position);

        this->search_ball_state += 1;
            if(this->search_ball_state > (search_ball_samples[0]+search_ball_samples[1]+search_ball_samples[2])) this->search_ball_state = 0;
      }
  }
}
/*
void NeckNode::search_other_goal_post()
{
  if (neck_locked_ || !received_neck_lock_)
  {
      if(search_ball_timer.delay(search_ball_delay))
      {
        auto new_neck_position = JointStateMsg();

        if(this->search_ball_state < search_ball_samples[0])
          {
            search_ball_pos = {search_ball_limits[0], 2000};
            if(this->search_ball_state > 0) this->search_ball_pos[0] -= (((search_ball_limits[0]*2)-4096)/(search_ball_samples[0]))*search_ball_state;
          }
        if(robotNumber == 4) 
          {
            this->search_ball_pos[0] = (search_ball_pos[0])*(0.25);
            this->search_ball_pos[1] = (search_ball_pos[1])*(0.25);
          }
        new_neck_position.id.push_back(19);
        new_neck_position.id.push_back(20);
      
        new_neck_position.info.push_back(this->search_ball_pos[0]);
        new_neck_position.info.push_back(this->search_ball_pos[1]);

        new_neck_position.type.push_back(JointStateMsg::POSITION);
        new_neck_position.type.push_back(JointStateMsg::POSITION);

        RCLCPP_DEBUG(this->get_logger(), "search ball id 19: %d  |  id 20: %d!", new_neck_position.info[0], new_neck_position.info[1]);

        if(neck_activate_) set_neck_position_publisher_->publish(new_neck_position);

        this->search_ball_state += 1;
            if(this->search_ball_state > search_ball_samples[0]) this->search_ball_state = 0;
      }
  }
}
*/
void NeckNode::search_goalcenter()
{
  std_msgs::msg::Bool msg;
  if (neck_locked_ || !received_neck_lock_)  
  {   
    
      if(search_ball_timer.delay(search_ball_delay))    
      {       
        auto new_neck_position = JointStateMsg();
        if(this->search_ball_state < search_ball_samples[0])        
        { 
          msg.data = false;
              
          search_ball_pos = {search_ball_limits[0],1800};          
          if(this->search_ball_state > 0) this->search_ball_pos[0] -= (((search_ball_limits[0]*2)-4096)/(search_ball_samples[0]))*search_ball_state;        
        }      
        if(this->search_ball_state >= search_ball_samples[0] && this->search_ball_state <= (search_ball_samples[0]+search_ball_samples[1]))        
        { 
          msg.data = false;
             
          search_ball_pos = {(4096-search_ball_limits[1]),2000};          
          if(this->search_ball_state > (search_ball_samples[0])) this->search_ball_pos[0] += (((search_ball_limits[1]*2)-4096)/(search_ball_samples[1]))*(search_ball_state-search_ball_samples[0]);        
        }      
        if(this->search_ball_state >= (search_ball_samples[0]+search_ball_samples[1]))        
        { 
          msg.data = false;
             
          search_ball_pos = {search_ball_limits[2],2200};          
          if(this->search_ball_state > (search_ball_samples[0]+search_ball_samples[1])) this->search_ball_pos[0] -= (((search_ball_limits[2]*2)-4096)/(search_ball_samples[2]))*(search_ball_state-(search_ball_samples[0]+search_ball_samples[1]));        
        }      
        if(robotNumber == 4)         
        {          
          this->search_ball_pos[0] = (search_ball_pos[0])*(0.25);          
          this->search_ball_pos[1] = (search_ball_pos[1])*(0.25);        
        }      
        new_neck_position.id.push_back(19);      
        new_neck_position.id.push_back(20);      
        new_neck_position.info.push_back(this->search_ball_pos[0]);      
        new_neck_position.info.push_back(this->search_ball_pos[1]);
        new_neck_position.type.push_back(JointStateMsg::POSITION);      
        new_neck_position.type.push_back(JointStateMsg::POSITION);

        //RCLCPP_DEBUG(this->get_logger(), "search ball id 19: %d  |  id 20: %d!", new_neck_position.info[0], new_neck_position.info[1]);

        if(neck_activate_) set_neck_position_publisher_->publish(new_neck_position);
        this->search_ball_state += 0.5;          
            if(this->search_ball_state > (search_ball_samples[0]+search_ball_samples[1]+search_ball_samples[2]))
            {
              msg.data = true;
    
             this->search_ball_state = 0;
            }    

        neck_finished_search_publisher->publish(msg);
      }  

  }
}

void NeckNode::change_neck_state(const Int32::SharedPtr msg)
{    
  //RCLCPP_INFO(this->get_logger(), "Estado_recebido: %d", msg->data);  
  //RCLCPP_INFO(this->get_logger(), "robot_state atual: %d", static_cast<int>(this->robot_state));
  //muda o estado do pescoço do robo de acordo com o valor lido pelo subscriber  


  new_state = static_cast<State>(msg->data);

  if(new_state != this->robot_state){
    this->robot_state = new_state;
  }
}

void NeckNode::main_callback()
{
  switch (this->robot_state)
  {
  case State::search_ball:
    //RCLCPP_INFO(this->get_logger(), "Searching the ball");
    if(ball.detected == false){
    this->search_ball();
    }
    break;
  case State::search_goalcenter:    
    //RCLCPP_INFO(this->get_logger(), "Searching goal center");    
    if(goalCenter.detected == false){
    this->search_goalcenter();    
    }
    break;  
  }
}
  /*
  case State::search_goal_post:
    //RCLCPP_INFO(this->get_logger(), "Searching goal post"); 
    if(goalPost.detected == false){
      this->search_other_goal_post();
    }
    break;

  }
  if(goal_post_counter == 1) && robot{

    RCLCPP_INFO(this->get_logger(), "Find 1 goalPost");
    this->robot_state = State::search_goal_post;
  }
  else if(goal_post_counter >= 2 && goalCenter.detected == true)
  {

    RCLCPP_INFO(this->get_logger(), "Find goal center");
    this->robot_state = State::search_goalcenter;
  }
*/


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