#include "robot_joy_control/robot_joy_control.hpp"

#include "robot_joy_control/buttons.hpp"

using namespace std::chrono_literals;


JoyRobotNode::JoyRobotNode() : Node("joy_robot_node")
{
    RCLCPP_INFO(this->get_logger(), "Running Joy Robot Node"); 
    
    joy_subscriber_ = this->create_subscription<JoyMsg>(
        "/joy",
        rclcpp::QoS(10),
        std::bind(&JoyRobotNode::listener_callback_joy, this, std::placeholders::_1)
    );

    this->action_client_ = rclcpp_action::create_client<ControlActionMsg>(
      this,
      "control_action"
    );

    send_goal_options.goal_response_callback =
      std::bind(&JoyRobotNode::goal_response_callback, this, std::placeholders::_1);

    send_goal_options.feedback_callback =
      std::bind(&JoyRobotNode::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
    
    send_goal_options.result_callback =
      std::bind(&JoyRobotNode::result_callback, this, std::placeholders::_1);
    
    goal_handle_ = nullptr;
}

JoyRobotNode::~JoyRobotNode()
{
}

void JoyRobotNode::listener_callback_joy(const JoyMsg::SharedPtr joy_msg)
{
    if(this->joy_info.buttons.size() == 0) this->joy_info = *joy_msg;
    button_reactions(this->joy_info, *joy_msg);

    this->joy_info = *joy_msg;
}

void JoyRobotNode::button_reactions(const JoyMsg old_joy_info, const JoyMsg new_joy_info)
{
    if((new_joy_info.buttons[SELECT] != old_joy_info.buttons[SELECT]) && new_joy_info.buttons[SELECT])
    {
      gait_mode = !gait_mode;
      RCLCPP_INFO(this->get_logger(), "Gait Mode: %d", gait_mode);
    }   


    if(new_joy_info.buttons[A_BUTTON])
    {
      send_goal(stand_still); 
      gait_mode = false;
    } 
    else if(new_joy_info.buttons[X_BUTTON]) send_goal(left_kick); // Left kick
    else if(new_joy_info.buttons[B_BUTTON]) send_goal(right_kick); // Right kick
    else if(new_joy_info.buttons[Y_BUTTON]) send_goal(goodbye);
    else if(new_joy_info.axes[LS_VERTICAL] > 0.90)  send_goal(walk);
    else if(new_joy_info.axes[LS_VERTICAL] < -0.90) RCLCPP_INFO(this->get_logger(), "send walk back");
    else if(new_joy_info.axes[LS_HORIZONTAL] > 0.90) RCLCPP_INFO(this->get_logger(), "send walk left side");
    else if(new_joy_info.axes[LS_HORIZONTAL] < -0.90) RCLCPP_INFO(this->get_logger(), "send walk right side");
    else if(new_joy_info.axes[RS_HORIZONTAL] > 0.90) send_goal(turn_left);
    else if(new_joy_info.axes[RS_HORIZONTAL] < -0.90) send_goal(turn_right);
    else if(gait_mode)  send_goal(gait);
    else send_goal(stand_still);
}

void JoyRobotNode::send_goal(const Move &order)
{
  auto goal_msg = ControlActionMsg::Goal();

  if (!this->action_client_->wait_for_action_server()) {
    RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
    rclcpp::shutdown();
  }

  goal_msg.action_number = order;

  RCLCPP_INFO(this->get_logger(), "Sending goal %d", goal_msg.action_number);

  if(order != this->last_move)
  {
    if(order == stand_up_back || order == stand_up_front || order == stand_up_side)
    {
      if(goal_handle_ != nullptr && !finished_move)
      {
        auto goal_handle_future_ = this->action_client_->async_cancel_goal(goal_handle_);
        goal_handle_future_.wait_for(1500ms);
      }
      action_client_->async_send_goal(goal_msg, send_goal_options);
      last_move = order;
    }
    else if(finished_move)
    {
      action_client_->async_send_goal(goal_msg, send_goal_options);
      last_move = order;
    }

    finished_move = false;
  }
  else if(finished_move)
  {
    action_client_->async_send_goal(goal_msg, send_goal_options); 
    last_move = order;
    finished_move = false;
  }

}

void JoyRobotNode::goal_response_callback(const GoalHandleControl::SharedPtr &goal_handle)
{
  this->goal_handle_ = goal_handle;
  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
  } else {
    // RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
  }
}

void JoyRobotNode::feedback_callback(
  GoalHandleControl::SharedPtr,
  const std::shared_ptr<const ControlActionMsg::Feedback> feedback)
{
  // RCLCPP_INFO(this->get_logger(), "Movements Remain: %d", feedback->movements_remaining);
}

void JoyRobotNode::result_callback(const GoalHandleControl::WrappedResult & result)
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
  finished_move = true;
  RCLCPP_INFO(this->get_logger(), "Goal finish");
}



int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto joy_robot_node = std::make_shared<JoyRobotNode>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(joy_robot_node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}


