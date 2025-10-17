#include "decision_pkg_cpp/decision_node.hpp"

using namespace std::chrono_literals;

// #define NECK_TILT_CENTER 512
// #define NECK_CENTER_TH 185
// #define NECK_LEFT_LIMIT 2650
// #define NECK_LEFT_TH (NECK_LEFT_LIMIT-(NECK_TILT_CENTER+NECK_CENTER_TH))
// #define NECK_RIGHT_LIMIT 1350
// #define NECK_RIGHT_TH (NECK_TILT_CENTER-NECK_CENTER_TH) - NECK_RIGHT_LIMIT

DecisionNode::DecisionNode() : Node("decision_node")
{
    RCLCPP_INFO(this->get_logger(), "Running Decision Node"); 
    
    // GameController Subscriber
    gc_subscriber_ = this->create_subscription<GameControllerMsg>(
        "gamestate",
        rclcpp::QoS(10),
        std::bind(&DecisionNode::listener_callback_GC, this, _1)
    );

    neck_position_subscriber_ = this->create_subscription<JointStateMsg>(
        "all_joints_position",
        rclcpp::QoS(10),
        std::bind(&DecisionNode::listener_callback_neck_pos, this, _1)
    );

    imu_gyro_subscriber_ = this->create_subscription<ImuGyroMsg>(
        "imu/rpy",
        rclcpp::QoS(10),
        std::bind(&DecisionNode::listener_callback_imu_gyro, this, _1)
    );

    imu_rpy_subscriber_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>(
      "/imu/rpy", rclcpp::QoS(10),
      std::bind(&DecisionNode::listener_callback_imu_rpy, this, std::placeholders::_1));  
  
    imu_accel_subscriber_ = this->create_subscription<ImuAccelMsg>(
        "imu/data",
        rclcpp::QoS(10),
        std::bind(&DecisionNode::listener_callback_imu_accel, this, _1)
    );

    imu_6050_subscriber = this->create_subscription<IMU6050>(
        "robot_state",
        rclcpp::QoS(10),
        std::bind(&DecisionNode::listener_callback_imu_6050, this, _1)
    );

    vision_subscriber_ = this->create_subscription<VisionMsg>(
      "/ball_position",
      rclcpp::QoS(10),
      std::bind(&DecisionNode::listener_callback_vision, this, std::placeholders::_1)
    );

    running_move_subscriber_ = this->create_subscription<intMsg>(
        "move_running",
        rclcpp::QoS(10),
        std::bind(&DecisionNode::listener_calback_running_move, this, _1)
    );

    // subscriber que vai ler se jogaremos com localização ou sem
    localization_subscriber = this->create_subscription<std_msgs::msg::Bool>(
      "localization_active", 
      rclcpp::QoS(10),
      std::bind(&DecisionNode::listener_callback_localization_status, this, _1)
    );

    goalpost_division_lines = this->create_subscription<VisionMsg>(
      "goalpost_position", 
      rclcpp::QoS(10),
      std::bind(&DecisionNode::listener_callback_goalpost_lines, this, _1)
    );

    goalpost_px_position = this->create_subscription<vision_msgs::msg::Point2D>(
      "goalpost_px_position", 
      rclcpp::QoS(10),
      std::bind(&DecisionNode::listener_callback_goalpost_px, this, _1)
    );

    goalpost_count = this->create_subscription<std_msgs::msg::Int32>(
      "goalpost_count", 
      rclcpp::QoS(10),
      std::bind(&DecisionNode::listener_callback_goalpost_count, this, _1)
    );
    
    neck_position_publisher_ = this->create_publisher<JointStateMsg>("set_joint_topic", 10);

    neck_control_lock_pub_ = this->create_publisher<std_msgs::msg::Bool>("neck_control_locked", 10);

    this->action_client_ = rclcpp_action::create_client<ControlActionMsg>(
      this,
      "control_action"
    );

    neck_track_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&DecisionNode::set_neck_position, this)
    );
    neck_track_timer_->cancel();  
    neck_timer_active_ = false;  

    send_goal_options.goal_response_callback =
      std::bind(&DecisionNode::goal_response_callback, this, _1);

    send_goal_options.feedback_callback =
      std::bind(&DecisionNode::feedback_callback, this, _1, _2);
    
    send_goal_options.result_callback =
      std::bind(&DecisionNode::result_callback, this, _1);
    
    goal_handle_ = nullptr;

    FALL_ACCEL_TH = this->declare_parameter("FALL_ACCEL_TH", 7.0);
    FALSES_FALLEN_TH = this->declare_parameter("FALSES_FALLEN_TH", 30);

    NECK_LEFT_LIMIT = this->declare_parameter("neck_left_limit", 2650);
    NECK_RIGHT_LIMIT = this->declare_parameter("neck_right_limit", 1350);
    NECK_CLOSE_LIMIT = this->declare_parameter("neck_down_limit", 1250);
    robot_number = this->declare_parameter("robot_number", 2);
}

DecisionNode::~DecisionNode()
{
}

void DecisionNode::listener_callback_GC(const GameControllerMsg::SharedPtr gc_info)
{
    if (this->gc_info.secondary_state == GameControllerMsg::STATE_PENALTYSHOOT && this->gc_info.game_state == GameControllerMsg::GAMESTATE_SET)
    {
      if(gc_info->secondary_state == GameControllerMsg::STATE_PENALTYSHOOT && gc_info->game_state == GameControllerMsg::GAMESTATE_PLAYING)
      {
        side_penalty = rand()%2;
      }
    }
    this->gc_info = *gc_info;
    // RCLCPP_INFO(this->get_logger(), "Recive GC Info");
    // RCLCPP_INFO(this->get_logger(), "Game State: %d", this->gc_info.game_state);
    // RCLCPP_INFO(this->get_logger(), "Secondary Game State: %d", this->gc_info.secondary_state);
}

void DecisionNode::listener_callback_imu_rpy(const geometry_msgs::msg::Vector3Stamped::SharedPtr rpy)
{
    robot.imu_yaw_rad = rpy->vector.z;
    //RCLCPP_INFO(this->get_logger(), "🧭 Yaw recebido: %.3f rad", robot.imu_yaw_rad);
}

void DecisionNode::listener_callback_neck_pos(const JointStateMsg::SharedPtr neck_pos)
{
    this->robot.neck_pos.position19 = neck_pos->info[19];
    this->robot.neck_pos.position20 = neck_pos->info[20];

    // RCLCPP_INFO(this->get_logger(), "Recive Neck Pos Info");
    // RCLCPP_INFO(this->get_logger(), "Id 19: %d  |  Id 20: %d", robot.neck_pos.position19, robot.neck_pos.position20);
}

void DecisionNode::listener_callback_imu_gyro(const ImuGyroMsg::SharedPtr imu_gyro)
{
    this->robot.imu_gyro = *imu_gyro;
    // RCLCPP_INFO(this->get_logger(), "Recive Imu Gyro Info");
    // RCLCPP_INFO(this->get_logger(), "Yaw: %f\n", this->imu_gyro.vector.z);
}

// imu mais antiga
void DecisionNode::listener_callback_imu_accel(const ImuAccelMsg::SharedPtr imu_accel)
{
  robot_detect_fallen(imu_accel->linear_acceleration.x,
                      imu_accel->linear_acceleration.y,
                      imu_accel->linear_acceleration.z); 
}

void DecisionNode::robot_detect_fallen(const float &robot_accel_x,
                                       const float &robot_accel_y,
                                       const float &robot_accel_z)
{
  // RCLCPP_INFO(this->get_logger(), "Recive Imu Accel Info");
  // RCLCPP_INFO(this->get_logger(), "\nAx: %f\nAy: %f\nAz: %f\n", robot_accel_x, robot_accel_y, robot_accel_z);

  if(abs(robot_accel_y) < FALL_ACCEL_TH)
  {
      falses_fallen_counter += 1;
  }
  else
  {
    falses_fallen_counter = 0;
    this->robot.fall = NotFallen;
  } 

  if(falses_fallen_counter > FALSES_FALLEN_TH)
  {
    if(robot_accel_z > FALL_ACCEL_TH)       this->robot.fall = FallenFront;
    else if(robot_accel_z < -FALL_ACCEL_TH) this->robot.fall = FallenBack;
    else if(robot_accel_x > FALL_ACCEL_TH)  this->robot.fall = FallenLeft;
    else this->robot.fall = FallenRight;  
  }
  // RCLCPP_INFO(this->get_logger(), "Robot Fall State: %d", robot.fall);
}


// imu de arduino 6050
void DecisionNode::listener_callback_imu_6050(const IMU6050::SharedPtr imu_6050)
{
  robot_detect_fallen_6050(imu_6050->fallen_forward, imu_6050->fallen_backwards);
}


void DecisionNode::robot_detect_fallen_6050(const bool fall_front, const bool fall_back)
{
  RCLCPP_INFO_STREAM(this->get_logger(),
                     "Caido de frente " << (fall_front ? "1" : "0")
                     << " \nCaido de costas " << (fall_back ? "1" : "0"));
  if (fall_front) {
    //RCLCPP_INFO(this->get_logger(), "CAIU DE FRENTE");
    this->robot.fall = FallenFront;
  }
  else if (fall_back){
    //RCLCPP_INFO(this->get_logger(), "CAIU DE COSTAS");
    this->robot.fall = FallenBack; 
  } 
  else this->robot.fall = NotFallen;
}




void DecisionNode::listener_callback_vision(const VisionMsg::SharedPtr vision_info)
{
  this->robot.camera_ball_position = *vision_info;
  if(robot.neck_pos.position20 < 1750)
  {
    NECK_CENTER_TH = get_center_th(robot.neck_pos.position20);
    // NECK_CENTER_TH = 100;
    NECK_LEFT_TH = NECK_TILT_CENTER + NECK_CENTER_TH;
    NECK_RIGHT_TH = NECK_TILT_CENTER - NECK_CENTER_TH;
  }
  else
  {
    NECK_CENTER_TH  = 185;
    NECK_LEFT_TH = NECK_TILT_CENTER + NECK_CENTER_TH;
    NECK_RIGHT_TH = NECK_TILT_CENTER - NECK_CENTER_TH;
  }

  // RCLCPP_INFO(this->get_logger(), "Recive Vision Info");
}

void DecisionNode::listener_calback_running_move(const intMsg::SharedPtr atualMove)
{
  this->robot.movement = static_cast<Move>(atualMove->data);
}    

void DecisionNode::send_goal(const Move &order)
{
  auto goal_msg = ControlActionMsg::Goal();

  if (!this->action_client_->wait_for_action_server()) {
    RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
    rclcpp::shutdown();
  }

  goal_msg.action_number = order;

  RCLCPP_DEBUG(this->get_logger(), "Sending goal %d", goal_msg.action_number);

  // ver se funciona e melhorar lógica depois
  if(order != this->robot.movement)
  {
    if(order == stand_up_back || order == stand_up_front || order == stand_up_side)
    {
      if(goal_handle_ != nullptr && !robot.finished_move)
      {
        if(robot.movement != stand_up_back && robot.movement != stand_up_front && robot.movement != stand_up_side)
        {
          auto goal_handle_future_ = this->action_client_->async_cancel_goal(goal_handle_);
          goal_handle_future_.wait_for(1500ms);
        }
      }
      if((robot.movement != stand_up_back && robot.movement != stand_up_front && robot.movement != stand_up_side) || robot.finished_move)
      {
        action_client_->async_send_goal(goal_msg, send_goal_options);
        robot.movement = order;
      }
    }
    else if(robot.finished_move)
    {
      action_client_->async_send_goal(goal_msg, send_goal_options);
      // robot.movement = order;
    }

    robot.finished_move = false;
  }
  else if(robot.finished_move)
  {
    action_client_->async_send_goal(goal_msg, send_goal_options); 
    // robot.movement = order;
    robot.finished_move = false;
  }

  //testar
  // if(order != this->robot.movement)
  // {
  //   if(order == stand_up_back || order == stand_up_front || order == stand_up_side)
  //   {
  //     auto goal_handle_future = this->action_client_->async_cancel_goal(goal_handle_);
  //     goal_handle_future_.wait_for(1500ms);
  //     action_client_->async_send_goal(goal_msg, send_goal_options);
  //     robot.movement = order;
  //     robot.finished_move = false;
  //   }
  // }
  // else if(robot.finished_move)
  // {
  //   action_client_->async_send_goal(goal_msg, send_goal_options); 
  //   robot.movement = order;
  //   robot.finished_move = false;
  // }
}

  void DecisionNode::listener_callback_goalpost_lines(const VisionMsg::SharedPtr goalpost_division_lines)
  {
    robot.goalpost_division_lines = *goalpost_division_lines;
  }

  void DecisionNode::listener_callback_localization_status(const std_msgs::msg::Bool localization_msg)
  {
    robot.localization_msg = localization_msg;
  }

  void DecisionNode::listener_callback_goalpost_px(const vision_msgs::msg::Point2D::SharedPtr goalpost_px_position)
  {
    robot.goalpost_px_position = *goalpost_px_position;
  }

  void DecisionNode::listener_callback_goalpost_count(const std_msgs::msg::Int32::SharedPtr goalpost_count)
  {
    robot.goalpost_count = *goalpost_count;
  }

  void DecisionNode::set_neck_position()
  {
    if (!unlock_msg.data)
    {
        float pan;
        float tilt;
        
        float x_offset = robot.goalpost_px_position.x;
        float y_offset = robot.goalpost_px_position.y;

        pan = 2000 - x_offset * 0.7;
        tilt = 2000 - y_offset * 0.35;

        // Limites
        if (pan > 3050) pan = 3050;
        else if (pan < 1350) pan = 1350;

        if (tilt > 2048) tilt = 2048;
        else if (tilt < 1050) tilt = 1050;
    

        auto new_neck_position = JointStateMsg();
        new_neck_position.id.push_back(19);
        new_neck_position.id.push_back(20);
        new_neck_position.info.push_back(pan);
        new_neck_position.info.push_back(tilt);
        new_neck_position.type.push_back(JointStateMsg::POSITION);
        new_neck_position.type.push_back(JointStateMsg::POSITION);

        neck_position_publisher_->publish(new_neck_position);
    }
  } 

  void DecisionNode::look_right()
  {
    if (!unlock_msg.data)
    {
      // Marca o início da publicação
      neck_publish_start_time_ = this->now();
  
      // Cria o timer que publica a cada 100ms
      neck_publish_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&DecisionNode::publish_neck_position, this));
  
      RCLCPP_INFO(this->get_logger(), "🚀 Iniciou publicação contínua da posição do pescoço por 2 segundos.");
    }
  }
  
  void DecisionNode::publish_neck_position()
  {
    rclcpp::Duration elapsed = this->now() - neck_publish_start_time_;
  
    if (elapsed.seconds() >= 5.0)
    {
      neck_publish_timer_->cancel();
      waiting_look_right = true;
      RCLCPP_INFO(this->get_logger(), "✅ Finalizou publicação da posição do pescoço após 2 segundos.");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Tempo: %f", elapsed.seconds());
    // Publica a posição do pescoço
    float pan = 2000;
    float tilt = 2000;
  
    auto new_neck_position = JointStateMsg();
    new_neck_position.id.push_back(19);
    new_neck_position.id.push_back(20);
    new_neck_position.info.push_back(pan);
    new_neck_position.info.push_back(tilt);
    new_neck_position.type.push_back(JointStateMsg::POSITION);
    new_neck_position.type.push_back(JointStateMsg::POSITION);
  
    neck_position_publisher_->publish(new_neck_position);
  }
  

  void DecisionNode::look_down_to_ball()
{
    auto msg = JointStateMsg();

    float pan = 2000;  // centralizado
    float tilt = 1350; // olhando para baixo (ajuste fino aqui)

    msg.id = {19, 20};
    msg.info = {pan, tilt};
    msg.type = {JointStateMsg::POSITION, JointStateMsg::POSITION};

    neck_position_publisher_->publish(msg);
}


  void DecisionNode::free_neck()
  {
      unlock_msg.data = true;  // libera
      neck_control_lock_pub_->publish(unlock_msg);
  }

  void DecisionNode::lock_neck()
  {
      unlock_msg.data = false;  // bloqueia
      neck_control_lock_pub_->publish(unlock_msg);
  }


void DecisionNode::goal_response_callback(const GoalHandleControl::SharedPtr &goal_handle)
{
  goal_handle_ = goal_handle;
  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
  } else {
    // RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
  }
}

void DecisionNode::feedback_callback(
  GoalHandleControl::SharedPtr,
  const std::shared_ptr<const ControlActionMsg::Feedback> feedback)
{
  // RCLCPP_INFO(this->get_logger(), "Movements Remain: %d", feedback->movements_remaining);
}

void DecisionNode::result_callback(const GoalHandleControl::WrappedResult & result)
{
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
      return;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
      return;
    default:
      RCLCPP_ERROR(this->get_logger(), "Unknown result code");
      return;
  }
  robot.finished_move = true;
  RCLCPP_DEBUG(this->get_logger(), "Goal finish");
}

// int main(int argc, char * argv[])
// {
//   rclcpp::init(argc, argv);
//   auto decision_node = std::make_shared<DecisionNode>();
//   rclcpp::executors::MultiThreadedExecutor executor;
//   executor.add_node(decision_node);
//   executor.spin();
//   rclcpp::shutdown();
//   return 0;
// }
