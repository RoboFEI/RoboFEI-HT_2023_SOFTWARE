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
            normal_game();
            break;
        
        default:
            break;
        }
    }
}

void RobotBehavior::normal_game()
{
    switch (gc_info.game_state)
    {
    case GameControllerMsg::GAMESTATE_INITAL:
        send_goal(stand_still);
        break;
    
    case GameControllerMsg::GAMESTATE_READY:
        // normal_game_prepair();
        break;
    
    case GameControllerMsg::GAMESTATE_SET:
        send_goal(stand_still);
        break;
    
    case GameControllerMsg::GAMESTATE_PLAYING:
        if(is_goalkeeper(ROBOT_NUMBER)) player_normal_game();
        else goalkeeper_normal_game();
        break;
    
    case GameControllerMsg::GAMESTATE_FINISHED:

        break;

    default:
        break;
    }
}

void RobotBehavior::player_normal_game()
{
    switch (robot.state)
    {
    case searching_ball:
        if(true/*ball_found()*/) robot.state = aligning_with_the_ball;
        else send_goal(turn_left);
        break;
    
    default:
        break;
    }
}

bool RobotBehavior::ball_found()
{
    if(!robot.ball.detected)
    {
        return false;
    }
    else if(ball_centralized()) // fazer a função
    return true;
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