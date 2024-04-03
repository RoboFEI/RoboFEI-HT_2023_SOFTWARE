#include "decision_pkg_cpp/decision_node.hpp"

using namespace std::chrono_literals;

#define FALL_ACCEL_TH 7.0
#define FALSES_FALLEN_TH 30

DecisionNode::DecisionNode() : Node("decision_node")
{
    RCLCPP_INFO(this->get_logger(), "Running Decision Node"); 
    
    // GameController Subscriber
    gc_subscriber_ = this->create_subscription<GameControllerMsg>(
        "gamestate",
        rclcpp::QoS(10),
        std::bind(&DecisionNode::listener_callback_GC, this, _1)
    );

    neck_position_subscriber_ = this->create_subscription<NeckPosMsg>(
        "neck_position",
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

    vision_subscriber_ = this->create_subscription<VisionInfo>(
      "/ball_position",
      rclcpp::QoS(10),
      std::bind(&NeckNode::listener_callback_vision, this, std::placeholders::_1)
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

}

DecisionNode::~DecisionNode()
{
}

void DecisionNode::listener_callback_GC(const GameControllerMsg::SharedPtr gc_info)
{
    this->gc_info = *gc_info;
    // RCLCPP_INFO(this->get_logger(), "Recive GC Info");
    // RCLCPP_INFO(this->get_logger(), "Game State: %d", this->gc_info.game_state);
    // RCLCPP_INFO(this->get_logger(), "Secondary Game State: %d", this->gc_info.secondary_state);
}

void DecisionNode::listener_callback_neck_pos(const NeckPosMsg::SharedPtr neck_pos)
{
    this->robot.neck_pos = *neck_pos;
    RCLCPP_INFO(this->get_logger(), "Recive Neck Pos Info");
    RCLCPP_INFO(this->get_logger(), "Id 19: %d  |  Id 20: %d", robot.neck_pos.position19, robot.neck_pos.position20);
}

void DecisionNode::listener_callback_imu_gyro(const ImuGyroMsg::SharedPtr imu_gyro)
{
    this->imu_gyro = *imu_gyro;
    RCLCPP_INFO(this->get_logger(), "Recive Imu Gyro Info");
    RCLCPP_INFO(this->get_logger(), "Yaw: %f\n", this->imu_gyro.vector.z);
    
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
  RCLCPP_INFO(this->get_logger(), "Recive Imu Accel Info");
  RCLCPP_INFO(this->get_logger(), "\nAx: %f\nAy: %f\nAz: %f\n", robot_accel_x, robot_accel_y, robot_accel_z);

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
  RCLCPP_INFO(this->get_logger(), "Robot Fall State: %d", robot.fall);
}

void DecisionNode::listener_callback_vision(const VisionInfo::SharedPtr vision_info)
{
  this->robot.vision_info = *vision_info;
  RCLCPP_INFO(this->get_logger(), "Recive Vision Info");

}

void DecisionNode::send_goal(const Move &order)
{
  // this->main_timer_->cancel();

  auto goal_msg = ControlActionMsg::Goal();

  if (!this->action_client_->wait_for_action_server()) {
    RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
    rclcpp::shutdown();
  }

  goal_msg.action_number = order;

  RCLCPP_INFO(this->get_logger(), "Sending goal %d", goal_msg.action_number);

  if(order != this->robot.movement)
  {
    if(order == stand_up_back || order == stand_up_front || order == stand_up_side)
    {
      auto goal_handle_future_ = this->action_client_->async_cancel_goal(goal_handle_);
      goal_handle_future_.wait_for(1500ms);
      action_client_->async_send_goal(goal_msg, send_goal_options);
      robot.movement = order;
    }
    else if(robot.finished_move)
    {
      action_client_->async_send_goal(goal_msg, send_goal_options);
      robot.movement = order;
    }

    robot.finished_move = false;
  }
  else if(robot.finished_move)
  {
    action_client_->async_send_goal(goal_msg, send_goal_options); 
    robot.movement = order;
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
    RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
  }
}

void DecisionNode::feedback_callback(
  GoalHandleControl::SharedPtr,
  const std::shared_ptr<const ControlActionMsg::Feedback> feedback)
{
  RCLCPP_INFO(this->get_logger(), "Movements Remain: %d", feedback->movements_remaining);
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

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto decision_node = std::make_shared<DecisionNode>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(decision_node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}