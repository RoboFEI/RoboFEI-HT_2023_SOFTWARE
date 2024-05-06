// ros2 run decision_pkg_cpp decision --ros-args -p body_activate_:=false

#include "decision_pkg_cpp/robot_behavior.hpp"
#include "decision_node.cpp"

// #define NECK_TILT_CENTER 2048
// #define NECK_CENTER_TH 185
// #define NECK_LEFT_LIMIT 2650
// // #define NECK_LEFT_TH (NECK_LEFT_LIMIT-(NECK_TILT_CENTER+NECK_CENTER_TH))
// #define NECK_RIGHT_LIMIT 1350
// // #define NECK_RIGHT_TH (NECK_TILT_CENTER-NECK_CENTER_TH) - NECK_RIGHT_LIMIT
// #define NECK_CLOSE_LIMIT 1340
// #define LIMIT_TH 40

#define MAX_LOST_BALL_TIME 10000 //10 seconds

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
    else if (is_penalized()) RCLCPP_INFO(this->get_logger(), "Penalizado");
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
    RCLCPP_INFO(this->get_logger(), "Normal Game: %d", gc_info.game_state);
    switch (gc_info.game_state)
    {
    case GameControllerMsg::GAMESTATE_INITAL: // conferido
        send_goal(stand_still);
        break;
    
    // case GameControllerMsg::GAMESTATE_READY: // fazer
    //     // normal_game_prepair();
    //     break;
    
    // case GameControllerMsg::GAMESTATE_SET: // feito
    //     send_goal(stand_still);
    //     break;
    
    case GameControllerMsg::GAMESTATE_PLAYING: // fazer
        player_normal_game();
        // if(is_goalkeeper(ROBOT_NUMBER)) goalkeeper_normal_game(); // fazer
        // // else player_normal_game();
        break;
    
    case GameControllerMsg::GAMESTATE_FINISHED: // feito
        send_goal(stand_still);
        break;
    }
}

void RobotBehavior::player_normal_game() // fazer
{
    RCLCPP_INFO(this->get_logger(), "robot state %d", robot.state);
    RCLCPP_INFO(this->get_logger(), "ball side %d", robot.ball_position);
    switch (robot.state)
    {
    case searching_ball:
        RCLCPP_INFO(this->get_logger(), "Searching ball");
        if(ball_is_locked())
        {
            robot.state = aligning_with_the_ball; // conferir
            send_goal(gait); //gait
        }
        else if(lost_ball_timer.delayNR(MAX_LOST_BALL_TIME)) turn_to_ball();
        else send_goal(gait); // gait
        break;
    
    case aligning_with_the_ball:
        RCLCPP_INFO(this->get_logger(), "Aligning with the_ball");
        ball_is_locked();
        if(robot_align_with_the_ball()) robot.state = ball_approach; // testar
        else if(ball_is_locked()) turn_to_ball();
        else if(!robot.camera_ball_position.detected) robot.state = searching_ball;
        break;

    case ball_approach:
        if(!robot.camera_ball_position.detected) robot.state = searching_ball;
        else if(!robot_align_with_the_ball()) robot.state = aligning_with_the_ball;
        else send_goal(walk);
        break;

    default:
        break;
    }
}

bool RobotBehavior::robot_align_with_the_ball() // fazer a parte de virar para a posição da bola
{
    if(ball_in_camera_center())
    {
        if(robot.state == ball_approach) return centered_neck();
        else return full_centered_neck();
    }
}

void RobotBehavior::turn_to_ball()
{
    if(robot.ball_position == left) send_goal(turn_left);
    if(robot.ball_position == right) send_goal(turn_right);
}

bool RobotBehavior::full_centered_neck()
{
    return abs(robot.neck_pos.position19 - NECK_TILT_CENTER) < 50;
}


bool RobotBehavior::centered_neck() // feito
{    
    return !neck_to_left() && !neck_to_right();
}

bool RobotBehavior::ball_is_locked() // feito 
{
    if(robot.camera_ball_position.detected)
    {
        lost_ball_timer.reset();
        if(vision_stable()) return true;
    }
    return false;
}

bool RobotBehavior::vision_stable()// feito   
{
    if(ball_in_camera_center() || ball_in_robot_limits())
    {
        detect_ball_position();
        return true;
    }
    return false;
}

void RobotBehavior::detect_ball_position() // Funciona
{
    // RCLCPP_INFO(this->get_logger(), "debug 1: centered_neck %d", centered_neck());
    // RCLCPP_INFO(this->get_logger(), "debug 1: neck_to_left %d", neck_to_left());
    // RCLCPP_INFO(this->get_logger(), "debug 1: neck_to_right %d", neck_to_right());
    
    if(centered_neck()) robot.ball_position = center;
    else if(neck_to_left()) robot.ball_position = left;
    else if(neck_to_right()) robot.ball_position = right;
    RCLCPP_INFO(this->get_logger(), "ball side %d", robot.ball_position);
}

bool RobotBehavior::neck_to_right() // testar
{
    return NECK_RIGHT_TH > robot.neck_pos.position19;
}

bool RobotBehavior::neck_to_left() // testar
{
    return NECK_LEFT_TH < robot.neck_pos.position19;
}

bool RobotBehavior::ball_in_robot_limits() // feito
{
    if(robot.camera_ball_position.left  && ball_in_left_limit())     return true;
    if(robot.camera_ball_position.right && ball_in_right_limit())    return true;
    if(robot.camera_ball_position.close && ball_in_close_limit())    return true;
    return false;
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

bool RobotBehavior::ball_in_camera_center() // feito
{
    return (robot.camera_ball_position.center_left || robot.camera_ball_position.center_right) && robot.camera_ball_position.med;
}

bool RobotBehavior::is_penalized() // feito
{
    if(gc_info.penalized)
    {
        RCLCPP_INFO(this->get_logger(), "Robot Penalized, remain %d s", gc_info.seconds_till_unpenalized);

        if(gc_info.seconds_till_unpenalized < 5)
        {
            RCLCPP_INFO(this->get_logger(), "Preparing to return, gait started");
            send_goal(gait);
        } 
        else
        {
            send_goal(stand_still);
        }
        return true;
    }
    return false;
}

void RobotBehavior::get_up() // feito
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
        send_goal(stand_up_side);
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