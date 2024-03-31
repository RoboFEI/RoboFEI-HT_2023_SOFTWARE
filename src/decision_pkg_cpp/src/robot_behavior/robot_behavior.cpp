#include "decision_pkg_cpp/robot_behavior.hpp"

RobotBehavior::RobotBehavior()
{
    robot_behavior_ = this->create_wall_timer(
        8ms,
        std::bind(&RobotBehavior::players_behavior, this));
}

RobotBehavior::~RobotBehavior()
{
}

void RobotBehavior::players_behavior()
{
    if(robot_fallen(robot)) get_up();
    else if (is_penalized()) 
    else
    {
        switch (gc_info.secondary_state)
        {
        case GameControllerMsg::STATE_NORMAL:
            play_normal_game();
            break;
        
        default:
            break;
        }
    }
}

bool RobotBehavior::is_penalized()
{
    if(gc_info.penalized)
    {
        RCLCPP_INFO(this->get_logger(), "Robot Penalized, remain %d s", gc_info.seconds_till_unpenalized);

        if(gc_info.seconds_till_unpenalized < 5)
        {
            RCLCPP_INFO(this->get_logger(), "Preparing to return, gait started";
            send_goal(gait);
        } 
        else return;
    }
}

void RobotBehavior::get_up()
{
    RCLCPP_INFO(this->get_logger(), "Robot Fallen");

    switch (robot.fall)
    {
    case FallenFront:
        RCLCPP_INFO(this->get_logger(), "Stand up front");
        send_goal(stand_up_front);
        break;
    
    case FallenBack:
        RCLCPP_INFO(this->get_logger(), "Stand up back");
        send_goal(stand_up_back);
        break;

    default:
        RCLCPP_INFO(this->get_logger(), "Stand up sides");
        send_goal(stand_up_side)
        break;
    }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto decision_node = std::make_shared<RobotBehavior>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(decision_node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}