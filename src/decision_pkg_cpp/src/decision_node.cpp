#include "decision_pkg_cpp/decision_node.hpp"

using namespace std::chrono_literals;

// #define NECK_TILT_CENTER 2048
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

    imu_accel_subscriber_ = this->create_subscription<ImuAccelMsg>(
        "imu/data",
        rclcpp::QoS(10),
        std::bind(&DecisionNode::listener_callback_imu_accel, this, _1)
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

    this->action_client_ = rclcpp_action::create_client<ControlActionMsg>(
      this,
      "control_action"
    );

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
    NECK_CLOSE_LIMIT = this->declare_parameter("neck_down_limit", 1350);
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

void DecisionNode::listener_callback_neck_pos(const JointStateMsg::SharedPtr neck_pos)
{
    this->robot.neck_pos.position19 = neck_pos->info[19];
    this->robot.neck_pos.position20 = neck_pos->info[20];

    // RCLCPP_INFO(this->get_logger(), "Recive Neck Pos Info");
    // RCLCPP_INFO(this->get_logger(), "Id 19: %d  |  Id 20: %d", robot.neck_pos.position19, robot.neck_pos.position20);
}

void DecisionNode::listener_callback_imu_gyro(const ImuGyroMsg::SharedPtr imu_gyro)
{
    this->imu_gyro = *imu_gyro;
    // RCLCPP_INFO(this->get_logger(), "Recive Imu Gyro Info");
    // RCLCPP_INFO(this->get_logger(), "Yaw: %f\n", this->imu_gyro.vector.z);
    
}

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

void DecisionNode::listener_callback_vision(const VisionMsg::SharedPtr vision_info)
{
  this->robot.camera_ball_position = *vision_info;
  if(robot.neck_pos.position20 < 1750)
  {
    NECK_CENTER_TH = get_center_th(robot.neck_pos.position20);
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

  RCLCPP_INFO(this->get_logger(), "Sending goal %d", goal_msg.action_number);

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
  RCLCPP_INFO(this->get_logger(), "Goal finish");
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
