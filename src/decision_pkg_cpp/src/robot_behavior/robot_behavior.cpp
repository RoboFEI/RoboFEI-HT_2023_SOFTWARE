#include "decision_pkg_cpp/robot_behavior.hpp"

#define NECK_LEFT_LIMIT 2650
#define NECK_RIGHT_LIMIT 1350
#define NECK_CLOSE_LIMIT 1340
#define LIMIT_TH 40

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
        break;
    
    default:
        break;
    }
}

bool RobotBehavior::ball_found() // feito
{
    if(!robot.ball.detected) return false;
    else if(vision_stable()) return true;
    return false;
}

bool RobotBehavior::vision_stable()// feito
{
    if(ball_in_camera_center() || ball_in_robot_limits()) return true;
    return false;
}

bool RobotBehavior::ball_in_robot_limits() // feito
{
    if(robo.ball_pos.left  && ball_in_left_limit())     return true;
    if(robo.ball_pos.right && ball_in_right_limit())    return true;
    if(robo.ball_pos.right && ball_in_close_limit())    return true;
    return false
}

bool RobotBehavior::ball_in_close_limit() // feito
{
    return abs(robot.neck_pos.position20 - NECK_CLOSE_LIMIT) < LIMIT_TH;
}

bool RobotBehavior::ball_in_right_limit() // feito
{
    return abs(robot.neck_pos.position19 - NECK_RIGHT_LIMIT) < LIMIT_TH;
}

bool RobotBehavior::ball_in_left_limit() // feito
{
    return abs(robot.neck_pos.position19 - NECK_LEFT_LIMIT) < LIMIT_TH;
}

bool RobotBehavior::ball_in_camera_center() // feita
{
    return (robo.ball_pos.center_left || robo.ball_pos.center_right) && robo.ball_pos.med;
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