// ros2 run decision_pkg_cpp decision --ros-args -p body_activate_:=false

#include "decision_pkg_cpp/robot_behavior.hpp"
#include "decision_node.cpp"
#include<unistd.h> // apenas para delay, apagar

#define MAX_LOST_BALL_TIME 10000 //10 seconds


RobotBehavior::RobotBehavior()
{
    robot_behavior_ = this->create_wall_timer(
        8ms,
        std::bind(&RobotBehavior::players_behavior, this));    
}

RobotBehavior::~RobotBehavior()     //checkpoint
{
}

void RobotBehavior::players_behavior()
{
    if (is_penalized())  RCLCPP_DEBUG(this->get_logger(), "Penalizado");  //robo penalizado
    else
    {
        if(robot_fallen(robot)) get_up();
	    else
        {
            switch (gc_info.secondary_state)
            {
            case GameControllerMsg::STATE_NORMAL:
                normal_game();
                break;
            
            case GameControllerMsg::STATE_PENALTYSHOOT:
                penalty();
                break;

            default:
                break;
            }
        }
    }

}

void RobotBehavior::penalty()           //penalizado
{
    switch (gc_info.game_state)
    {
    case GameControllerMsg::GAMESTATE_INITAL: 
        send_goal(stand_still);
        break;
        
    case GameControllerMsg::GAMESTATE_SET: 
        send_goal(stand_still);
        break;
    
    case GameControllerMsg::GAMESTATE_PLAYING: 
        if(gc_info.has_kick_off)
        {
            player_penalty();
        }
        // else goalkeeper_penalty();
        break;
   
    case GameControllerMsg::GAMESTATE_FINISHED: 
        send_goal(stand_still);
        break;
    }
}


void RobotBehavior::normal_game()           //jogo normal
{
    int centro = 0;
    // RCLCPP_DEBUG(this->get_logger(), "Normal Game: %d", gc_info.game_state);
    switch (gc_info.game_state)
    {
    case GameControllerMsg::GAMESTATE_INITAL: // conferido
        send_goal(stand_still);
        break;
    
    case GameControllerMsg::GAMESTATE_READY: // fazer
        
        while(robot.neck_pos.position19 >= 1450 && robot.neck_pos.position19 <= 2380){
            send_goal(walk);
            //RCLCPP_INFO(this->get_logger(), "macaquito dançando : %d", robot.neck_pos.position19 );      
            break;
        }
        send_goal(stand_still);
        break;

    case GameControllerMsg::GAMESTATE_SET: // feito
        send_goal(stand_still);
        break;
    
    case GameControllerMsg::GAMESTATE_PLAYING:  // começo do jogo
        if(gc_info.has_kick_off || (!gc_info.has_kick_off && gc_info.secondary_seconds_remaining == 0))
        {
            if(is_goalkeeper(ROBOT_NUMBER)) goalkeeper_normal_game();
            else if (is_kicker(ROBOT_NUMBER)) kicker_normal_game();
            else if(is_bala(ROBOT_NUMBER)) bala_normal_game();
        }
        // if(is_goalkeeper(ROBOT_NUMBER)) goalkeeper_normal_game(); // fazer
        // // else player_normal_game();
        break;
   
    case GameControllerMsg::GAMESTATE_FINISHED: // feito
        send_goal(stand_still);
        break;
    }
}

void RobotBehavior::bala_normal_game()                //estado de jogo normal; jogo rolando 
{
    //RCLCPP_INFO(this->get_logger(), "robot state %d", robot.state);
    //RCLCPP_FATAL(this->get_logger(), "posição do 20: %d", robot.neck_pos.position20);
    //RCLCPP_WARN(this->get_logger(), "posição do 19: %d", robot.neck_pos.position19);
    
    switch (robot.state)
    {
    case searching_ball:
        //RCLCPP_ERROR(this->get_logger(), "Searching ball");
        //RCLCPP_ERROR(this->get_logger(), "lost ball timer  %d", lost_ball_timer.delayNR(MAX_LOST_BALL_TIME));

        if(ball_is_locked())
            {   //RCLCPP_ERROR(this->get_logger(), "ball locked");
                if(robot.ball_position == center) robot.state = ball_approach;      //anda ate a bola
                else robot.state = aligning_with_the_ball;
            }
        else if(lost_ball_timer.delayNR(MAX_LOST_BALL_TIME)) send_goal(turn_left);        //alinha o corpo com a bola
        //else send_goal(gait); // gait
        break;
    
    case aligning_with_the_ball:
        //RCLCPP_WARN(this->get_logger(), "Aligning with the_ball");
        if(robot_align_with_the_ball()) robot.state = ball_approach;
        //else if(ball_is_locked()) robot.state = ball_approach;
        else if(!robot.camera_ball_position.detected) robot.state = searching_ball;
        else if(robot.neck_pos.position19 > 2150) send_goal(turn_left);
        else if(robot.neck_pos.position19 < 1900) send_goal(turn_right);
        break;

    case ball_approach:
        //RCLCPP_ERROR(this->get_logger(), "neck limit %d, ball locked %d, ball close %d", ball_in_close_limit(), ball_is_locked(), robot.camera_ball_position.close);
        //RCLCPP_ERROR(this->get_logger(), "ball_approach");
        if((robot.neck_pos.position20 < 1270) && (ball_is_locked())) 
            {
            //RCLCPP_ERROR(this->get_logger(), "primeiro if");
            robot.state = ball_close;
            }         //perdeu a bola
        else if(!robot.camera_ball_position.detected) 
            {
            //RCLCPP_ERROR(this->get_logger(), "primeiro else if");
            robot.state = searching_ball;
            } // pode estar bugando
        else if(!robot_align_with_the_ball()) 
            {
            //RCLCPP_ERROR(this->get_logger(), "segundo else if");
            robot.state = aligning_with_the_ball;
            }
        else 
            {
            send_goal(walk);
            //RCLCPP_ERROR(this->get_logger(), "else final");
            }
        break;


    case ball_close:
        //RCLCPP_WARN(this->get_logger(), "ball right %d, ball left %d", robot_align_for_kick_right(), robot_align_for_kick_left());
        //RCLCPP_WARN(this->get_logger(), "ball close");
        if (robot.neck_pos.position19 > 2150)
        {
            send_goal(walk_left);
        }
        else if ((robot.neck_pos.position19 < 2150) && (robot.neck_pos.position19 > 1980))
        {
            //send_goal(walk);
            robot.state = kick_ball;
        }
        else if(!robot.camera_ball_position.detected || !robot.camera_ball_position.close) robot.state = searching_ball;
        else if (robot.neck_pos.position20 > 1450) robot.state = aligning_with_the_ball;
        else if (robot.neck_pos.position19 < 1980)
        {
            send_goal(walk_right);
        }
        //else if(robot_align_for_kick_left()) robot.state = kick_ball;
        break;

    case kick_ball:
        //RCLCPP_ERROR(this->get_logger(), "kick");
        if ((!robot.camera_ball_position.detected) || (robot.ball_position != center))
	    {
            send_goal(gait);
            robot.state = searching_ball;
            lost_ball_timer.reset();
	    }
        else send_goal(walk);

        //else if(lost_ball_timer.delayNR(2000)) robot.state = searching_ball; //para testar com o corpo desatiavdo
	    break;
    }
}

void RobotBehavior::kicker_normal_game()                //estado de jogo normal; jogo rolando 
{
    //RCLCPP_INFO(this->get_logger(), "robot state %d", robot.state);
    //RCLCPP_FATAL(this->get_logger(), "posição do 20: %d", robot.neck_pos.position20);
    //RCLCPP_WARN(this->get_logger(), "posição do 19: %d", robot.neck_pos.position19);
    
    switch (robot.state)
    {
    case searching_ball:
        //RCLCPP_ERROR(this->get_logger(), "Searching ball");
        //RCLCPP_ERROR(this->get_logger(), "lost ball timer  %d", lost_ball_timer.delayNR(MAX_LOST_BALL_TIME));

        if(ball_is_locked())
            {   //RCLCPP_ERROR(this->get_logger(), "ball locked");
                if(robot.ball_position == center) robot.state = ball_approach;      //anda ate a bola
                else robot.state = aligning_with_the_ball;
            }
        else if(lost_ball_timer.delayNR(MAX_LOST_BALL_TIME)) send_goal(turn_left);        //alinha o corpo com a bola
        //else send_goal(gait); // gait
        break;
    
    case aligning_with_the_ball:
        //RCLCPP_WARN(this->get_logger(), "Aligning with the_ball");
        if(robot_align_with_the_ball()) robot.state = ball_approach;
        //else if(ball_is_locked()) robot.state = ball_approach;
        else if(!robot.camera_ball_position.detected) robot.state = searching_ball;
        else if(robot.neck_pos.position19 > 2100) send_goal(turn_left);
        else if(robot.neck_pos.position19 < 1900) send_goal(turn_right);
        break;

    case ball_approach:
        //RCLCPP_ERROR(this->get_logger(), "neck limit %d, ball locked %d, ball close %d", ball_in_close_limit(), ball_is_locked(), robot.camera_ball_position.close);
        //RCLCPP_ERROR(this->get_logger(), "ball_approach");
        if((robot.neck_pos.position20 < 1270) && (ball_is_locked())) 
            {
            //RCLCPP_ERROR(this->get_logger(), "primeiro if");
            robot.state = ball_close;
            }         //perdeu a bola
        else if(!robot.camera_ball_position.detected) 
            {
            //RCLCPP_ERROR(this->get_logger(), "primeiro else if");
            robot.state = searching_ball;
            } // pode estar bugando
        else if(!robot_align_with_the_ball()) 
            {
            //RCLCPP_ERROR(this->get_logger(), "segundo else if");
            robot.state = aligning_with_the_ball;
            }
        else 
            {
            send_goal(walk);
            //RCLCPP_ERROR(this->get_logger(), "else final");
            }
        break;


    case ball_close:
        //RCLCPP_WARN(this->get_logger(), "ball right %d, ball left %d", robot_align_for_kick_right(), robot_align_for_kick_left());
        //RCLCPP_WARN(this->get_logger(), "ball close");
        if (robot.neck_pos.position19 > 1950)
        {
            send_goal(walk_left);
        }
        else if ((robot.neck_pos.position19 < 1950) && (robot.neck_pos.position19 > 1780))
        {
            robot.state = kick_ball;
        }
        else if(!robot.camera_ball_position.detected || !robot.camera_ball_position.close) robot.state = searching_ball;
        else if (robot.neck_pos.position20 > 1450) robot.state = aligning_with_the_ball;
        else if (robot.neck_pos.position19 < 1780)
        {
            send_goal(walk_right);
        }
        //else if(robot_align_for_kick_left()) robot.state = kick_ball;
        break;

    case kick_ball:
        //RCLCPP_ERROR(this->get_logger(), "kick");
        if(robot.movement != 3)  send_goal(right_kick);
        else if(robot.finished_move)
        {
            robot.state = searching_ball;
            lost_ball_timer.reset();
        }
        //else if(lost_ball_timer.delayNR(2000)) robot.state = searching_ball; //para testar com o corpo desatiavdo
        break;
    }
}

void RobotBehavior::goalkeeper_normal_game() // caso o jogador seja o goleiro
{
    RCLCPP_DEBUG(this->get_logger(), "robot state %d", robot.state);
    
    switch (robot.state)
    {
    case searching_ball:
        RCLCPP_DEBUG(this->get_logger(), "lost ball timer  %d", lost_ball_timer.delayNR(MAX_LOST_BALL_TIME));
        if(ball_is_locked()) robot.state = aligning_with_the_ball;  // caso a bola seja achada
        // else if(lost_ball_timer.delayNR(MAX_LOST_BALL_TIME)) turn_to_ball();   //alinha o corpo com a bola
        else send_goal(stand_still);
        break;
    
    case aligning_with_the_ball:
        RCLCPP_DEBUG(this->get_logger(), "Goalkeeper aligning with the_ball");
        goalkeeper_align_with_the_ball();
        if(goalkeeper_align_with_the_ball()) robot.state = ball_approach; // se o goleiro esta centralizado, troca de estado
        // else if(ball_is_locked()) robot.state = ball_approach;
        else if(!robot.camera_ball_position.detected) robot.state = searching_ball; // se perdeu a bola, deve achá-la
        else send_goal(gait);
        break;

    case ball_approach:
        RCLCPP_DEBUG(this->get_logger(), "neck limit %d, ball locked %d, ball close %d", ball_in_close_limit(), ball_is_locked(), robot.camera_ball_position.close);
        // if(ball_in_close_limit() && ball_is_locked() && robot.camera_ball_position.close) robot.state = ball_close;    //perdeu a bola
        if(!robot.camera_ball_position.detected) robot.state = searching_ball; // pode estar bugando
        else if(!goalkeeper_align_with_the_ball()) robot.state = aligning_with_the_ball;
        else send_goal(squat);       
        break;
    }
}

void RobotBehavior::player_penalty()
{
    RCLCPP_DEBUG(this->get_logger(), "robot state %d", robot.state);
    RCLCPP_DEBUG(this->get_logger(), "side Penalty: %d", side_penalty);

    switch (robot.state)
    {
    case searching_ball:
        //RCLCPP_DEBUG(this->get_logger(), "Searching ball");
        RCLCPP_DEBUG(this->get_logger(), "lost ball timer  %d", lost_ball_timer.delayNR(MAX_LOST_BALL_TIME));

	    if(ball_is_locked())
        {
            RCLCPP_DEBUG(this->get_logger(), "BOLA ALINHADA");
            if(robot.ball_position == center) robot.state = ball_approach;
            else robot.state = aligning_with_the_ball;
        }
        //else if(lost_ball_timer.delayNR(MAX_LOST_BALL_TIME)) turn_to_ball();
        else send_goal(gait); // gait
        break;
    
    case aligning_with_the_ball:
        RCLCPP_DEBUG(this->get_logger(), "Aligning with the_ball");
        if(robot_align_with_the_ball()) robot.state = ball_approach;
        //else if(ball_is_locked()) turn_to_ball();
        else if(!robot.camera_ball_position.detected) robot.state = searching_ball;
        else send_goal(gait);
        break;

    case ball_approach:
        RCLCPP_DEBUG(this->get_logger(), "neck limit %d, ball locked %d, ball close %d", ball_in_close_limit(), ball_is_locked(), robot.camera_ball_position.close);
        if(ball_in_close_limit() && ball_is_locked() && robot.camera_ball_position.close) robot.state = ball_close;
        else if(!robot.camera_ball_position.detected) robot.state = searching_ball; // pode estar bugando
        else if(!robot_align_with_the_ball()) robot.state = aligning_with_the_ball;
        else send_goal(walk);
        break;

    case ball_close:
        // 0 - Esqeurda | 1 - Direita 
        if(side_penalty == 0)
        {
            if(robot_align_for_kick_right()) robot.state = kick_ball;
            else if(!robot.camera_ball_position.detected) robot.state = searching_ball;
            //else if(!robot_align_with_the_ball()) robot.state = aligning_with_the_ball;
            else send_goal(gait);
        }
        else if(side_penalty == 1)
        {
            if(robot_align_for_kick_left()) robot.state = kick_ball;
            else if(!robot.camera_ball_position.detected) robot.state = searching_ball;
            //else if(!robot_align_with_the_ball()) robot.state = aligning_with_the_ball;
            else send_goal(gait);
        }


        
        break;

    case kick_ball:
        if(side_penalty == 0){
            if(robot.movement != right_kick_to_right) send_goal(right_kick_to_right);
            else if(robot.finished_move)
            {
                robot.state = searching_ball;
                lost_ball_timer.reset();
            }   
        }
        else if (side_penalty == 1)
        {
            if(robot.movement != left_kick_to_left) send_goal(left_kick_to_left);
            else if(robot.finished_move)
            {
                robot.state = ball_approach;
                lost_ball_timer.reset();
            }   
        }
	    break;
    }
}

bool RobotBehavior::robot_align_for_kick_left() //fazer
{
    if(ball_in_left_foot()) return true;
    else send_goal(walk_right); 
    return false;
}

bool RobotBehavior::robot_align_for_kick_right()
{
    if(ball_in_right_foot()) return true;
    else send_goal(walk_left); 
    return false;
}

bool RobotBehavior::ball_in_right_foot()
{
    if(ball_is_locked() && robot.neck_pos.position19 < 1850) return true;
    return false;
}

bool RobotBehavior::ball_in_left_foot()
{
    if(ball_is_locked() && robot.neck_pos.position19 > 2250) return true;
    return false;
}


bool RobotBehavior::is_goalkeeper(int robot_num)
{
    return robot_num == 1;
}

bool RobotBehavior::is_kicker(int robot_num)
{
    return robot_num == 2;
}

bool RobotBehavior::is_bala(int robot_num)
{
    return robot_num == 3;
}

bool RobotBehavior::goalkeeper_align_with_the_ball()
{
    if(vision_stable())
    {
        if(neck_to_left()) send_goal(walk_left);
        else if(neck_to_right()) send_goal(walk_right);
        else return true;
    }
    return false;
}

bool RobotBehavior::robot_align_with_the_ball()
{
    if(vision_stable())
    {
        if(robot.state == ball_approach) return centered_neck();
        else return full_centered_neck();
    }
    else return false;
}

void RobotBehavior::turn_to_ball()
{
    if(robot.ball_position == left) send_goal(turn_left);
    else if(robot.ball_position == right) send_goal(turn_right);     
    // Melhorar essa logica

    // if(robot.ball_position == right) send_goal(turn_right);
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

bool RobotBehavior::vision_stable() // feito   
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
    // RCLCPP_DEBUG(this->get_logger(), "debug 1: centered_neck %d", centered_neck());
    // RCLCPP_DEBUG(this->get_logger(), "debug 1: neck_to_left %d", neck_to_left());
    // RCLCPP_DEBUG(this->get_logger(), "debug 1: neck_to_right %d", neck_to_right());
    if(robot.state == aligning_with_the_ball)
    {
        if(robot.neck_pos.position19 > NECK_TILT_CENTER) robot.ball_position = left;
        else robot.ball_position = right;
    }
    else
    {
        if(centered_neck()) robot.ball_position = center;
        if(neck_to_left()) robot.ball_position = left;
        if(neck_to_right()) robot.ball_position = right;
        RCLCPP_DEBUG(this->get_logger(), "ball side %d", robot.ball_position);
    }

}

bool RobotBehavior::neck_to_right() // testar
{
    return NECK_RIGHT_TH > robot.neck_pos.position19;
}

bool RobotBehavior::neck_to_left() // testar
{
    RCLCPP_DEBUG(this->get_logger(), "Neck left th: %d\nrobot neck pos: %d", NECK_LEFT_TH, robot.neck_pos.position19); 
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
    return (robot.camera_ball_position.center) && robot.camera_ball_position.med;
}

bool RobotBehavior::is_penalized() // feito
{
    if(gc_info.penalized)
    {
        RCLCPP_DEBUG(this->get_logger(), "Robot Penalized, remain %d seconds", gc_info.seconds_till_unpenalized);

        if(gc_info.seconds_till_unpenalized < 5)
        {
            RCLCPP_DEBUG(this->get_logger(), "Preparing to return, gait started");
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
    RCLCPP_DEBUG(this->get_logger(), "Robot Fallen");

    switch (robot.fall)
    {
    case FallenFront:
        RCLCPP_DEBUG(this->get_logger(), "Stand up front");
        send_goal(stand_up_front);
        break;
    
    case FallenBack:
        RCLCPP_DEBUG(this->get_logger(), "Stand up back");
        send_goal(stand_up_back);
        break;

    default:
        RCLCPP_DEBUG(this->get_logger(), "Stand up sides");
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