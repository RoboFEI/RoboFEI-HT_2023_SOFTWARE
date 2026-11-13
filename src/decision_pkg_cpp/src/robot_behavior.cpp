// ros2 run decision_pkg_cpp decision --ros-args -p body_activate_:=false

#include "decision_pkg_cpp/robot_behavior.hpp"
#include "decision_node.cpp"
#include <unistd.h> // apenas para delay, apagar
#include "std_msgs/msg/int32.hpp"
#include "custom_interfaces/msg/vision.hpp"
#include "geometry_msgs/msg/point.hpp"
#include <cmath> 

#define PI 3.14
#define MAX_LOST_BALL_TIME 7000 //10 seconds
#define ALTURA_ROBO 0.45

// defining the opposite side --> 1 - right --> 0 - left
int opposite_side;
bool aligned_goalpost = false; // alinhado com a trave
int follow_goal_index = 3;
int follow_ball_index = 2;
bool is_align_with_goal = false;

float tamanhoCampoX = 6.7;
float tamanhoCampoY = 4.5;

float distanciaGol = 1.5;

RobotBehavior::RobotBehavior()
{
    robot_behavior_ = this->create_wall_timer(
        8ms,
    std::bind(&RobotBehavior::players_behavior, this));    

    publisher_neck_state = this->create_publisher<std_msgs::msg::Int32>("/neck_state", 10); //pulisher do estado do pescoço 
    
    //publisher_posicao_robo = this->create_publisher<geometry_msgs::msg::Point>("/posicao_robo", 10); //pulisher da posicao do robo

    vision_goal_center_subscriber = this->create_subscription<VisionInfo>(    
    "goal_center_position", 10, std::bind(&RobotBehavior::listener_callback_goal_center_vision, this, std::placeholders::_1));
    
    imu_yaw_subscriber = this->create_subscription<ImuAccelMsg>(
    "/imu/data", 10, std::bind(&RobotBehavior::listener_callback_imu_yaw, this, std::placeholders::_1));
}

RobotBehavior::~RobotBehavior()     //checkpoint
{
}

void RobotBehavior::players_behavior()
{
    
    if (is_penalized()) {

        // Feito

    }
    else {

        if(robot_fallen(robot)) get_up();
        else {
            switch(gc_info.game_phase){

                case GameControllerMsg::GAME_PHASE_NORMAL:
                    //RCLCPP_INFO(this->get_logger(), "GameController: GamePhase NORMAL");
                    normal_game();
                    break;

                case GameControllerMsg::GAME_PHASE_PENALTY_SHOOT_OUT:
                    //RCLCPP_INFO(this->get_logger(), "GameController: GamePhase PENALTY SHOOTOUT");
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
    //RCLCPP_INFO(this->get_logger(), "Entrou no PENALTY SHOOTOUT");
    switch (gc_info.state)
    {
    case GameControllerMsg::STATE_INITIAL:
        //RCLCPP_INFO(this->get_logger(), "GameController: State INITIAL");
        send_goal(stand_still);
        break;
        
    case GameControllerMsg::STATE_SET: 
        //RCLCPP_INFO(this->get_logger(), "GameController: State SET");
        send_goal(stand_still);
        break;
    
    case GameControllerMsg::STATE_PLAYING: 
        //RCLCPP_INFO(this->get_logger(), "GameController: State PLAYING");
        if(gc_info.has_kick_off)
        {
            //player_penalty();
            //RCLCPP_INFO(this->get_logger(), "CHUTANDO");
            send_goal(left_kick);
        }
        // else goalkeeper_penalty();
        break;
   
    case GameControllerMsg::STATE_FINISHED: 
        //RCLCPP_INFO(this->get_logger(), "GameController: State FINISHED");
        send_goal(stand_still);
        break;
    }
}


void RobotBehavior::normal_game()           //jogo normal
{

    //RCLCPP_DEBUG(this->get_logger(), "Normal Game, state: %d", gc_info.state);
    switch (gc_info.state)
    {

    case GameControllerMsg::STATE_INITIAL:
    {
        if(robot.neck_pos.position19 < 2048){ // definir lado do campo 1 - right 0 - left
            opposite_side = 1;
        } else {
            opposite_side = 0;
        }

        send_goal(stand_still);
        break;
    }
    
    case GameControllerMsg::STATE_READY:
    {
        //RCLCPP_INFO(this->get_logger(), "GameController: State READY");
        send_goal(stand_still);
        break;
    }
        
    case GameControllerMsg::STATE_SET: // feito
    {
        //RCLCPP_INFO(this->get_logger(), "GameController: State SET");
        send_goal(stand_still);
        yaw_reference_set_ = false;
        break;
    }
    
    case GameControllerMsg::STATE_PLAYING:
    {
        //RCLCPP_INFO(this->get_logger(), "GameController: State PLAYING");
        if (!(gc_info.has_kick_off) && !(gc_info.secondary_time <= 0) && (gc_info.seconds_remaining < 600)) 
        {
            //RCLCPP_INFO(this->get_logger(), "Kick-Off adversário, esperando");
            send_goal(stand_still);

        } else if ((gc_info.has_kick_off) && (gc_info.secondary_time <= 0) && (is_kicker(ROBOT_NUMBER)) && (gc_info.seconds_remaining < 600))
        {
            //RCLCPP_INFO(this->get_logger(), "Kick-Off nosso, começando a se mover");
            kicker_normal_game();
        }

        std_msgs::msg::Bool localization_active = robot.localization_msg; // topico que manda true/false para localização

        if (!localization_active.data) {
            if (is_goalkeeper(ROBOT_NUMBER)) goalkeeper_normal_game();
            else if (is_kicker(ROBOT_NUMBER)) kicker_normal_game();
            else if (is_bala(ROBOT_NUMBER)) bala_normal_game();
        } else {
            if (is_kicker(ROBOT_NUMBER)) kicker_localization_game();
            else if (is_bala(ROBOT_NUMBER)) bala_localization_game();
        }
        break;
    }
   
    case GameControllerMsg::STATE_FINISHED: // feito
        //RCLCPP_INFO(this->get_logger(), "GameController: State FINISHED");
        send_goal(stand_still);
        break;
    }

}


void RobotBehavior::bala_normal_game()                //estado de jogo normal; jogo rolando 
{

    //RCLCPP_INFO(this->get_logger(), "Recebido yaw_est: %f", yaw_est_value_);
    //RCLCPP_INFO(this->get_logger(), "robot state %d", robot.state);
    //RCLCPP_FATAL(this->get_logger(), "posição do 20: %d", robot.neck_pos.position20);
    //RCLCPP_WARN(this->get_logger(), "posição do 19: %d", robot.neck_pos.position19);
    //RCLCPP_INFO(this->get_logger(), "Bala Normal Game");

    switch (robot.state)
    {
    case searching_ball:
    {
        RCLCPP_ERROR(this->get_logger(), "Searching ball");

        //RCLCPP_ERROR(this->get_logger(), "lost ball timer  %d", lost_ball_timer.delayNR(MAX_LOST_BALL_TIME));

        std_msgs::msg::Int32 msg;
        msg.data = follow_ball_index;
        publisher_neck_state->publish(msg);

        if(ball_is_locked())
        {   //RCLCPP_ERROR(this->get_logger(), "ball locked");
            if(robot.neck_pos.position20 >= 1450)  robot.state = aligning_with_the_ball;
            else if(robot.neck_pos.position20 <= 1450)  robot.state = ball_close;
        }
        else if(lost_ball_timer.delayNR(MAX_LOST_BALL_TIME)) send_goal(turn_left);        //alinha o corpo com a bola
        //else send_goal(gait); // gait
        break;

    }
    case aligning_with_the_ball:
    {
        std_msgs::msg::Int32 msg;
        msg.data = follow_ball_index;
        publisher_neck_state->publish(msg);

        RCLCPP_WARN(this->get_logger(), "Aligning with the_ball");

        if(robot.neck_pos.position20 >= 1450 && robot.neck_pos.position19 < 2360 && robot.neck_pos.position19 > 1730) robot.state = ball_approach;
        //else if(ball_is_locked()) robot.state = ball_approach;
        else if(!robot.camera_ball_position.detected && lost_ball_timer.delayNR(MAX_LOST_BALL_TIME)) robot.state = searching_ball;
        else if(robot.neck_pos.position19 > 2360) send_goal(turn_left);
        else if(robot.neck_pos.position19 < 1730) send_goal(turn_right);
        break;
    }
    case ball_approach:
    {
        //RCLCPP_ERROR(this->get_logger(), "neck limit %d, ball locked %d, ball close %d", ball_in_close_limit(), ball_is_locked(), robot.camera_ball_position.close);
        RCLCPP_ERROR(this->get_logger(), "ball_approach");

        std_msgs::msg::Int32 msg;
        msg.data = follow_ball_index;
        publisher_neck_state->publish(msg);

        if((robot.neck_pos.position20 <= 1450) && (ball_is_locked())) 
        {
            robot.state = ball_close;
        }
        else if(!robot.camera_ball_position.detected && lost_ball_timer.delayNR(MAX_LOST_BALL_TIME)) robot.state = searching_ball; 
        else if(robot.neck_pos.position20 >= 1450 && (robot.neck_pos.position19 < 1730 || robot.neck_pos.position19 > 2360)) robot.state = aligning_with_the_ball;
        else send_goal(walk);
        break;
    }
    case ball_close:
    {
        //RCLCPP_WARN(this->get_logger(), "ball right %d, ball left %d", robot_align_for_kick_right(), robot_align_for_kick_left());
        RCLCPP_WARN(this->get_logger(), "ball close");

        std_msgs::msg::Int32 msg;
        msg.data = follow_ball_index;
        publisher_neck_state->publish(msg);
    
    
        if (robot.neck_pos.position19 > 2050)
        {
            send_goal(walk_left);
        }
        else if (robot.neck_pos.position19 < 1900)
        {
            send_goal(walk_right);
        }
        else if ((robot.neck_pos.position19 < 2050) && (robot.neck_pos.position19 > 1900))
        {
            robot.state = find_any_goal;
        }
        else if(!robot.camera_ball_position.detected && lost_ball_timer.delayNR(MAX_LOST_BALL_TIME)) robot.state = searching_ball;
        //else if(robot_align_for_kick_left()) robot.state = kick_ball;
        break;
    }
    case aligning_with_goal:
    {   
        std_msgs::msg::Int32 msg;
        msg.data = follow_ball_index;
        publisher_neck_state->publish(msg);

        RCLCPP_INFO(this->get_logger(), "aligning with goal");
        if(error19 < 180)
        {
            if(error19 > -350)
            {
                Turn_ball_left_little();
            }   
            else
            {
                Turn_ball_left_lot();
            }
        }
        else if(error19 > 180)
        {
            if(error19 < 350)
            {
                Turn_ball_right_little();
            }
            else
            {
                Turn_ball_right_lot();
            }
        }
        break;
    }
    case find_any_goal:
    {
        RCLCPP_INFO(this->get_logger(), "Find Goal");

        send_goal(stand_still);

        std_msgs::msg::Int32 msg;
        msg.data = follow_goal_index;
        publisher_neck_state->publish(msg);

        if (goalCenter.center == true && gol_vision_stable() && opositeSide())
        {

            RCLCPP_INFO(this->get_logger(), "Guardou a posição do 19");
            pos19 = robot.neck_pos.position19;
            pos20 = robot.neck_pos.position20;

            error19 = pos19 - 2048;
            error20 = 2048 - pos20;
            RCLCPP_INFO(this->get_logger(), "error %d", error19);
                
            Rotate_timer.reset();
            robot.state = aligning_with_goal;
        }
        else if(goalCenter.center == true && gol_vision_stable() && opositeSide() == false)
        {
            robot.state = look_correct_goal;
        }
        else if(neck_finish_search && In_kick_angle())
        {
            robot.state = searching_ball_after_goal;
        }
        else if(neck_finish_search && !In_kick_angle())
        {
            robot.state = look_correct_goal;
        }
        break;
    }
    case look_correct_goal:
    {
        RCLCPP_INFO(this->get_logger(), "look correct goal");

        std_msgs::msg::Int32 msg;
        msg.data = follow_goal_index;
        publisher_neck_state->publish(msg);
            
        send_goal(turn_ball_left);
        if(In_kick_angle())
        {
            timer_lost_ball.reset();
            robot.state = searching_ball_after_goal;
        }

        break;
    }    
    case searching_ball_after_goal:
    {
        RCLCPP_INFO(this->get_logger(), "Searching ball after goal");

        std_msgs::msg::Int32 msg;
        msg.data = follow_ball_index;
        publisher_neck_state->publish(msg);

        if(ball_is_locked())
        {
            if(robot.neck_pos.position20 < 1300 && robot.neck_pos.position19 >= 1730 && robot.neck_pos.position19 <= 2360 && vision_stable())
            {
                robot.state = kick_ball;
            }
            else if(robot.camera_ball_position.detected && robot.neck_pos.position20 > 1300)
            {
                robot.state = ball_approach_after_goal;
            }
            else
            {
                robot.state = ball_close;
            }
        }
        else if(neck_finish_search && timer_lost_ball.delay(10000))
        {
            robot.state = searching_ball;
        }
        else
        {
            send_goal(stand_still);
        }
        break;
    }   
    case ball_approach_after_goal:
    {
        std_msgs::msg::Int32 msg;
        msg.data = follow_ball_index;
        publisher_neck_state->publish(msg);

        RCLCPP_INFO(this->get_logger(), "ball approach after goal");
        if((robot.neck_pos.position20 <= 1300) && robot.camera_ball_position.detected) 
        {
            Rotate_timer.reset();
            robot.state = kick_ball;
        }        
        else if(robot.neck_pos.position19 > 2360) 
        {
            send_goal(turn_left);
        }
        else if(robot.neck_pos.position19 < 1730) 
        {
            send_goal(turn_right);
        }
        else if(neck_finish_search && lost_ball_timer.delayNR(MAX_LOST_BALL_TIME))
        {
            robot.state = searching_ball;
        }
        else 
        {
            send_goal(walk);
        }
        break;
    }   
    case kick_ball:
    {    
        RCLCPP_ERROR(this->get_logger(), "kick");
        if(lost_ball_timer.delayNR(MAX_LOST_BALL_TIME) && !robot.camera_ball_position.detected)
	    {
            send_goal(gait);
            robot.state = searching_ball;
            lost_ball_timer.reset();
	    } 
        if (robot.neck_pos.position20 >= 1300)
        {
            robot.state = ball_approach_after_goal;
        }
        else if(robot.neck_pos.position19 >= 2050){ //2300
            send_goal(walk_left);
        }
        else if(robot.neck_pos.position19 <= 1900){
            send_goal(walk_right);
        }
        else/* if((robot.neck_pos.position19 > 1996) && (robot.neck_pos.position19 < 2200))*/{
            send_goal(walk);
        }
        //else if(lost_ball_timer.delayNR(2000)) robot.state = searching_ball; //para testar com o corpo desatiavdo
	    break;
    }
}
}
void RobotBehavior::listener_callback_goal_center_vision(const VisionInfo::SharedPtr msg)
{  
    goalCenter.detected     =   msg->detected;  
    //RCLCPP_INFO(this->get_logger(), "GOAL '%s'", goalCenter.detected ? "true" : "false");
    goalCenter.left         =   msg->left;
    goalCenter.center       =   msg->center;
    goalCenter.right        =   msg->right;

    goalCenter.far          =   msg->far;
    goalCenter.med          =   msg->med;
    goalCenter.close        =   msg->close;
}
void RobotBehavior::listener_callback_imu_yaw(const ImuAccelMsg::SharedPtr msg)
{
    const double x = msg->orientation.x;
    const double y = msg->orientation.y;
    const double z = msg->orientation.z;
    const double w = msg->orientation.w;

    const double robot_yaw_rad = std::atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z)
    );

    yaw_deg = robot_yaw_rad * 180.0 / M_PI;

    if (first_yaw)
    {
        start_yaw_deg = yaw_deg;
        first_yaw = false;

        RCLCPP_INFO(this->get_logger(),"Yaw inicial %.2f",start_yaw_deg);
        RCLCPP_INFO(this->get_logger(), "Yaw do robo no momento %.2f", yaw_deg);
    }
}

void RobotBehavior::kicker_normal_game()                //estado de jogo normal; jogo rolando 
{
    //RCLCPP_INFO(this->get_logger(), "robot state %d", robot.state);
    RCLCPP_FATAL(this->get_logger(), "posição do 20: %d", robot.neck_pos.position20);
    RCLCPP_WARN(this->get_logger(), "posição do 19: %d", robot.neck_pos.position19); 
    //RCLCPP_INFO(this->get_logger(), "Recebeu yaw %.2f", yaw_deg);   
    //RCLCPP_INFO(this->get_logger(), "Lado oponente: %s",opositeSide() ? "true" : "false");
    
    //RCLCPP_INFO(this->get_logger(), "Kicker Normal Game");
    switch (robot.state)
    {
        case searching_ball:
        {
            std_msgs::msg::Int32 msg;
            msg.data = follow_ball_index;
            publisher_neck_state->publish(msg);

            RCLCPP_INFO(this->get_logger(), "searching ball");
            //RCLCPP_ERROR(this->get_logger(), "lost ball timer  %d", lost_ball_timer.delayNR(MAX_LOST_BALL_TIME));
            if(ball_is_locked()) 
            {
                //RCLCPP_INFO(this->get_logger(), "ball locked");
                if(robot.neck_pos.position20 >= 1450){
                    robot.state = aligning_with_the_ball;
                }
                else if(robot.neck_pos.position20 <= 1450)
                {
                    robot.state = ball_close;
                }
                // else if(robot.neck_pos.position20 <= 1500 && (robot.neck_pos.position19 < 1730 || robot.neck_pos.position19 > 2360)){
                    
                //     robot.state = ball_close;
                // }
            }
            else if(lost_ball_timer.delayNR(MAX_LOST_BALL_TIME)) send_goal(turn_left);        //alinha o corpo com a bola
            //else send_goal(gait); // gait
            break;
        }
        case aligning_with_the_ball:
        {

            RCLCPP_INFO(this->get_logger(), "aligning with the ball");
            
            if(robot.neck_pos.position20 >= 1450 && robot.neck_pos.position19 < 2360 && robot.neck_pos.position19 > 1730) robot.state = ball_approach;
            else if(robot.neck_pos.position20 <= 1450) robot.state = ball_close;
            //else if(ball_is_locked()) robot.state = ball_approach;

            else if(!robot.camera_ball_position.detected) robot.state = searching_ball;
            //else if(robot.neck_pos.position20 <= 1450 && robot.neck_pos.position19 < 1730 && robot.neck_pos.position19 > 2360) robot.state = ball_close;
            //else if(robot.neck_pos.position20 <= 1450 && robot.neck_pos.position19 >= 1730 && robot.neck_pos.position19 <= 2360) robot.state = aligning_with_goal;
            else if(robot.neck_pos.position19 > 2360) send_goal(turn_left);
            else if(robot.neck_pos.position19 < 1730) send_goal(turn_right);
            break;

        }
        case ball_approach:
        {
            //RCLCPP_ERROR(this->get_logger(), "neck limit %d, ball locked %d, ball close %d", ball_in_close_limit(), ball_is_locked(), robot.camera_ball_position.close);
            RCLCPP_INFO(this->get_logger(), "ball approach");
            
            if(robot.neck_pos.position20 >= 1450 && (robot.neck_pos.position19 < 1730 || robot.neck_pos.position19 > 2360)) 
            {
                robot.state = aligning_with_the_ball;
            }        
            else if(robot.neck_pos.position20 <= 1450 ) 
            {
                robot.state = ball_close;
            } 
            else if(lost_ball_timer.delayNR(MAX_LOST_BALL_TIME)) 
            {
                robot.state = searching_ball;
            }
            else 
            {
                send_goal(walk);
            }
            break;       
        }
        case find_any_goal:
        {     
            RCLCPP_INFO(this->get_logger(), "Find Goal");

            send_goal(stand_still);

            std_msgs::msg::Int32 msg;
            msg.data = follow_goal_index;
            publisher_neck_state->publish(msg);

            if (goalCenter.center == true && gol_vision_stable() && opositeSide())
            {

                RCLCPP_INFO(this->get_logger(), "Guardou a posição do 19");
                pos19 = robot.neck_pos.position19;
                pos20 = robot.neck_pos.position20;

                error19 = pos19 - 2048;
                error20 = 2048 - pos20;
                RCLCPP_INFO(this->get_logger(), "error %d", error19);
                
                Rotate_timer.reset();
                robot.state = aligning_with_goal;
            }
            else if(goalCenter.center == true && gol_vision_stable() && opositeSide() == false)
            {
                robot.state = look_correct_goal;
            }
            else if(neck_finish_search && !In_kick_angle())
            {
                //RCLCPP_INFO(this->get_logger(), "NAO ACHOU O GOL");
                
                robot.state = look_correct_goal;

            }
            else if(neck_finish_search && In_kick_angle())
            {
                robot.state = kick_ball;
            }
            break;
        }
        case find_oponent_goal:
        {
            RCLCPP_INFO(this->get_logger(), "Find oponent Goal");

            send_goal(stand_still);

            std_msgs::msg::Int32 msg;
            msg.data = follow_goal_index;
            publisher_neck_state->publish(msg);

            if(goalCenter.center == true && gol_vision_stable() && opositeSide())
            {

                RCLCPP_INFO(this->get_logger(), "Guardou a posição do 19");
                pos19 = robot.neck_pos.position19;

                error19 = pos19 - 2048;
                RCLCPP_INFO(this->get_logger(), "error %d", error19);
                
                Rotate_timer.reset();
                robot.state = aligning_with_goal;
            }
            else if(neck_finish_search && !In_kick_angle())
            {
                //RCLCPP_INFO(this->get_logger(), "NAO ACHOU O GOL");
                robot.state = look_correct_goal;

            }
            else if(neck_finish_search && In_kick_angle())
            {
                robot.state = kick_ball;
            }

            break;
        }
        case look_correct_goal:
        {
            RCLCPP_INFO(this->get_logger(), "look correct goal");

            std_msgs::msg::Int32 msg;
            msg.data = follow_goal_index;
            publisher_neck_state->publish(msg);
            
            send_goal(turn_ball_left);
            if(In_kick_angle())
            {
                timer_lost_ball.reset();
                robot.state = searching_ball_after_goal;
            }

            break;
        }
        case aligning_with_goal:
        {
            std_msgs::msg::Int32 msg;
            msg.data = follow_ball_index;
            publisher_neck_state->publish(msg);

            RCLCPP_INFO(this->get_logger(), "aligning with goal");
            if(error19 < 180)
            {
                if(error19 > -350)
                {
                    Turn_ball_left_little();
                }   
                else
                {
                    Turn_ball_left_lot();
                }
            }
            else if(error19 > 180)
            {
                if(error19 < 350)
                {
                    Turn_ball_right_little();
                }
                else
                {
                    Turn_ball_right_lot();
                }
            }
            break;
        }
        case searching_ball_after_goal:
        {
            RCLCPP_INFO(this->get_logger(), "Searching ball after goal");

            std_msgs::msg::Int32 msg;
            msg.data = follow_ball_index;
            publisher_neck_state->publish(msg);

            if(ball_is_locked())
            {
                if(robot.neck_pos.position20 < 1450 && robot.neck_pos.position19 >= 1730 && robot.neck_pos.position19 <= 2360 && vision_stable())
                {
                    robot.state = kick_ball;
                }
                else if(robot.camera_ball_position.detected && robot.neck_pos.position20 > 1450)
                {
                    robot.state = ball_approach_after_goal;
                }
                else
                {
                    robot.state = ball_close;
                }
            }
            else if(neck_finish_search && timer_lost_ball.delay(10000))
            {
                robot.state = searching_ball;
            }
            else
            {
                send_goal(stand_still);
            }

            break;
        }
        case ball_approach_after_goal:
        {
            std_msgs::msg::Int32 msg;
            msg.data = follow_ball_index;
            publisher_neck_state->publish(msg);

            RCLCPP_INFO(this->get_logger(), "ball approach after goal");
            if((robot.neck_pos.position20 <= 1450) && robot.camera_ball_position.detected) 
            {
                Rotate_timer.reset();
                robot.state = kick_ball;
            }        
            else if(robot.neck_pos.position19 > 2360) 
            {
                send_goal(turn_left);
            }
            else if(robot.neck_pos.position19 < 1730) 
            {
                send_goal(turn_right);
            }
            else if(neck_finish_search && lost_ball_timer.delayNR(MAX_LOST_BALL_TIME))
            {
                robot.state = searching_ball;
            }
            else 
            {
            send_goal(walk);
            }
            break;
        }
        case ball_close:
        {
             //RCLCPP_INFO(this->get_logger(), "ball right %d, ball left %d", robot_align_for_kick_right(), robot_align_for_kick_left());
             RCLCPP_INFO(this->get_logger(), "ball close");

            if(lost_ball_timer.delayNR(MAX_LOST_BALL_TIME)) robot.state = searching_ball;

            if(robot.neck_pos.position20 >= 1450)
            {
                robot.state = aligning_with_the_ball;
            } 
            else if (robot.neck_pos.position19 < 1900)
            {
                send_goal(walk_right);
                RCLCPP_INFO(this->get_logger(), "walking right");
            }
            else if (robot.neck_pos.position19 > 2050)
            {
                send_goal(walk_left);
                RCLCPP_INFO(this->get_logger(), "walking left");
            }
            else
            {
                Rotate_timer.reset();
                robot.state = find_any_goal;
            }
            break;
        }
        case kick_ball:
        {
            std_msgs::msg::Int32 msg;
            msg.data = follow_ball_index;
            publisher_neck_state->publish(msg);

            RCLCPP_INFO(this->get_logger(), "kick");
            if(robot.movement != 3 && robot.movement != 4) 
            {   
                if (robot.neck_pos.position19 >= 1960 && robot.neck_pos.position19 <= 2250) 
                {  
                    //RCLCPP_INFO(this->get_logger(), "kick_left");
                    send_goal(left_kick);
                    // RCLCPP_INFO(this->get_logger(), "posição do 20: %d", robot.neck_pos.position20);
                }
                else if (robot.neck_pos.position19 <= 1960 && robot.neck_pos.position19 >= 1780)
                {    
                    //RCLCPP_INFO(this->get_logger(), "kick_right");
                    send_goal(right_kick);
                    // RCLCPP_INFO(this->get_logger(), "posição do 20: %d", robot.neck_pos.position20);
                } 
                else if(robot.neck_pos.position19 < 1780)
                {
                    send_goal(walk_right);
                }
                else if(robot.neck_pos.position19 > 2250)
                {
                    send_goal(walk_left);
                }
            }
            if(robot.finished_move)
	        {
                std_msgs::msg::Int32 msg;
                msg.data = follow_ball_index;
                publisher_neck_state->publish(msg);
                robot.state = searching_ball;
                lost_ball_timer.reset();
            }
            if(neck_finish_search && lost_ball_timer.delay(2000) && !robot.camera_ball_position.detected) robot.state = searching_ball; //para testar com o corpo desatiavdo
            break;
        }
    }
}
void RobotBehavior::kicker_localization_game()                //estado de jogo normal; jogo rolando 
{
    switch (robot.state)
    {
    case searching_ball:
        this->free_neck();
        //RCLCPP_DEBUG(this->get_logger(), "Seaching ball");
        if(ball_is_locked())
            {   //RCLCPP_ERROR(this->get_logger(), "ball locked");
                if(robot.ball_position == center) robot.state = ball_approach;      //anda ate a bola
                else robot.state = aligning_with_the_ball;
            }
        else if(lost_ball_timer.delayNR(MAX_LOST_BALL_TIME)) send_goal(turn_left);        //alinha o corpo com a bola
        break;
    
    case aligning_with_the_ball:
        //RCLCPP_DEBUG(this->get_logger(), "Aligning with the_ball");
        if(robot_align_with_the_ball()) robot.state = ball_approach;
        //else if(ball_is_locked()) robot.state = ball_approach;
        else if(!robot.camera_ball_position.detected) robot.state = searching_ball;
        else if(robot.neck_pos.position19 > 2100) send_goal(turn_left);
        else if(robot.neck_pos.position19 < 1900) send_goal(turn_right);
        break;

    case ball_approach:
        // RCLCPP_DEBUG(this->get_logger(), "ball_approach");
        if((robot.neck_pos.position20 < 1350)& (ball_is_locked())) 
            {
            robot.state = ball_close;
            }         
            
        else if(!robot.camera_ball_position.detected) 
            {
            robot.state = searching_ball;
            }
        else if(!robot_align_with_the_ball()) 
            {
            robot.state = aligning_with_the_ball;
            }
        else 
            {
            send_goal(walk);
            }
        break;

    case ball_close:
        RCLCPP_DEBUG(this->get_logger(), "ball close");
        
            
        delta_yaw = fabs(robot.imu_yaw_rad - yaw_reference_); // estabelece a referencia do "zero" da IMU
        //RCLCPP_INFO(this->get_logger(), "Delta yaw: %f", delta_yaw);
        RCLCPP_INFO(this->get_logger(), "Delta yaw ABS: %f", fabs(delta_yaw));

        if (delta_yaw > M_PI) delta_yaw -= 2*M_PI; 
        if (delta_yaw < -M_PI) delta_yaw += 2*M_PI;
        
        // entrei pelo lado direito
        if (opposite_side == 0)
        {
            if (delta_yaw > 0.1 && delta_yaw <= 0.6 && robot.neck_pos.position20 < 1230) // gol alinado
            {
                robot.state = kick_ball;
            }
            else if (delta_yaw > 0.6 && delta_yaw <= 2)
            {
                send_goal(turn_ball_left);
            }
            else
            {
                send_goal(turn_ball_right);
            }
        }
        if(!robot.camera_ball_position.detected) robot.state = searching_ball;

        break;
    

    case kick_ball:
        // Envia o chute apenas se ele ainda não está em execução
        if(robot.movement != left_kick && robot.movement != right_kick)
        {
            if (robot.neck_pos.position19 >= 2048) 
            {
                send_goal(left_kick);  
            }
            else 
            {
                send_goal(right_kick); 
            }
        }
    
        // Espera o chute terminar
        if(robot.finished_move)
        {
            robot.state = searching_ball;
            lost_ball_timer.reset();
        }
        break;
    }
     
}

void RobotBehavior::bala_localization_game()                //estado de jogo normal; jogo rolando 
{
    //RCLCPP_INFO(this->get_logger(), "BALA LOCALIZATION GAME");
    switch (robot.state)
    {
    case searching_ball:
        this->free_neck();
        //RCLCPP_DEBUG(this->get_logger(), "Seaching ball");
        if(ball_is_locked())
            {   //RCLCPP_ERROR(this->get_logger(), "ball locked");
                if(robot.ball_position == center) robot.state = ball_approach;      //anda ate a bola
                else robot.state = aligning_with_the_ball;
            }
        else if(lost_ball_timer.delayNR(MAX_LOST_BALL_TIME)) send_goal(turn_left);        //alinha o corpo com a bola
        break;
    
    case aligning_with_the_ball:
        //RCLCPP_DEBUG(this->get_logger(), "Aligning with the_ball");
        if(robot_align_with_the_ball()) robot.state = ball_approach;
        //else if(ball_is_locked()) robot.state = ball_approach;
        else if(!robot.camera_ball_position.detected) robot.state = searching_ball;
        else if(robot.neck_pos.position19 > 2100) send_goal(turn_left);
        else if(robot.neck_pos.position19 < 1900) send_goal(turn_right);
        break;

    case ball_approach:
        //RCLCPP_DEBUG(this->get_logger(), "ball_approach");
        if((robot.neck_pos.position20 < 1400)& (ball_is_locked())) 
            {
            robot.state = ball_close;
            }         
            
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
            }
        break;

    case ball_close:
        RCLCPP_DEBUG(this->get_logger(), "ball close");
        
        //RCLCPP_INFO(this->get_logger(), "Z: %f", robot.imu_gyro.vector.z);
            
        delta_yaw = fabs(robot.imu_yaw_rad - yaw_reference_);
        RCLCPP_INFO(this->get_logger(), "Delta yaw ABS: %f", delta_yaw);

        if (delta_yaw > M_PI) delta_yaw -= 2*M_PI;
        if (delta_yaw < -M_PI) delta_yaw += 2*M_PI;
        
        // entrei pelo lado direito
        if (opposite_side == 0)
        {
            if (delta_yaw > 0.1 && delta_yaw <= 0.6)
            {
                robot.state = kick_ball;
            }
            else if (delta_yaw > 0.6 && delta_yaw <= 2)
            {
                send_goal(turn_left);
            }
            else
            {
                send_goal(turn_right);
            }
        }

        if(!robot.camera_ball_position.detected) 
        {
            robot.state = searching_ball;
        }

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
   RCLCPP_INFO(this->get_logger(), "ENTROU AQUI");
   switch (robot.state)
    {
    case searching_ball:
        //RCLCPP_INFO(this->get_logger(), "Searching ball");
        //RCLCPP_ERROR(this->get_logger(), "lost ball timer  %d", lost_ball_timer.delayNR(MAX_LOST_BALL_TIME));

        if(ball_is_locked())
            {   RCLCPP_INFO(this->get_logger(), "ball locked");
                if(robot.ball_position == center) robot.state = ball_approach;      //anda ate a bola
                else robot.state = aligning_with_the_ball;
            }
        else if(lost_ball_timer.delayNR(MAX_LOST_BALL_TIME)) send_goal(turn_left);        //alinha o corpo com a bola
        //else send_goal(gait); // gait
        break;
    
    case aligning_with_the_ball:
        //RCLCPP_INFO(this->get_logger(), "Aligning with the_ball");
        if(robot_align_with_the_ball()) robot.state = ball_approach;
        //else if(ball_is_locked()) robot.state = ball_approach;
        else if(!robot.camera_ball_position.detected) robot.state = searching_ball;
        else if(robot.neck_pos.position19 > 2100) send_goal(turn_left);
        else if(robot.neck_pos.position19 < 1900) send_goal(turn_right);
        break;

    case ball_approach:
        //RCLCPP_ERROR(this->get_logger(), "neck limit %d, ball locked %d, ball close %d", ball_in_close_limit(), ball_is_locked(), robot.camera_ball_position.close);
        //RCLCPP_INFO(this->get_logger(), "ball_approach");
        if((robot.neck_pos.position20 < 1350)& (ball_is_locked())) 
            {
            robot.state = ball_close;
            }         //perdeu a bola
        else if(!robot.camera_ball_position.detected) 
            {
            robot.state = searching_ball;
            } // pode estar bugando
        else if(!robot_align_with_the_ball()) 
            {
            robot.state = aligning_with_the_ball;
            }
        else 
            {
            send_goal(walk);
            }
        break;


    case ball_close:
        //RCLCPP_INFO(this->get_logger(), "ball right %d, ball left %d", robot_align_for_kick_right(), robot_align_for_kick_left());
        //RCLCPP_INFO(this->get_logger(), "ball close");

        if(!robot.camera_ball_position.detected || !robot.camera_ball_position.close) robot.state = searching_ball;
        else if (robot.neck_pos.position20 > 1400) robot.state = aligning_with_the_ball;
        
        if (robot.neck_pos.position19 < 1500)
        {
            send_goal(walk_right);
            RCLCPP_INFO(this->get_logger(), "walking right");
        }
        else if (robot.neck_pos.position19 > 2460)
        {
            send_goal(walk_left);
            RCLCPP_INFO(this->get_logger(), "walking left");
        }
        else
        {
            robot.state = kick_ball;
        }
        //else if(robot_align_for_kick_left()) robot.state = kick_ball;
        break;

    case kick_ball:
        //RCLCPP_INFO(this->get_logger(), "kick");
        if(/*robot.movement != 3 && */robot.movement != 4) 
        {
            // if (robot.neck_pos.position19 >= 2048) 
            //     {   
                    RCLCPP_INFO(this->get_logger(), "kick_left");
                    send_goal(left_kick);
                    robot.state = searching_ball;
                    
                // RCLCPP_INFO(this->get_logger(), "posição do 20: %d", robot.neck_pos.position20);
            //     }
            //     else
            //     {   

            //         RCLCPP_INFO(this->get_logger(), "kick_right");
            //         send_goal(right_kick);
            //         robot.state = searching_ball;
            //     // RCLCPP_INFO(this->get_logger(), "posição do 20: %d", robot.neck_pos.position20);
            // } 
        }
        else if(robot.finished_move)
	    {
            robot.state = searching_ball;
            lost_ball_timer.reset();
        }
        else if(!robot.camera_ball_position.detected || !robot.camera_ball_position.close) robot.state = searching_ball;
        else if (robot.neck_pos.position20 > 1400) robot.state = aligning_with_the_ball;
        //else if(lost_ball_timer.delayNR(2000)) robot.state = searching_ball; //para testar com o corpo desatiavdo
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

bool RobotBehavior::robot_is_align_with_goal()
{
    if(goalCenter.detected == true && robot.neck_pos.position19 <= 2360 && robot.neck_pos.position19 >= 1730){
        is_align_with_goal = true;
    }
    else{
        is_align_with_goal = false;
    }
    return is_align_with_goal;
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
    RCLCPP_FATAL(this->get_logger(), "kicker");
}

bool RobotBehavior::is_bala(int robot_num)
{
    return robot_num == 3;
    RCLCPP_FATAL(this->get_logger(), "bala" );
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

void RobotBehavior::Turn_ball_left_little()
{
    send_goal(turn_ball_left);
    RCLCPP_INFO(this->get_logger(), "Rotacionar pra esquerda um pouco");
    if(Rotate_timer.delay(3000))
    {
        send_goal(stand_still);
        RCLCPP_INFO(this->get_logger(), "Acabou o timer");
        timer_lost_ball.reset();
        robot.state = searching_ball_after_goal;
    }
    

}
void RobotBehavior::Turn_ball_left_lot_noSwitch()
{
   send_goal(turn_ball_left);
    RCLCPP_INFO(this->get_logger(), "Rotacionar para esquerda muito");
    if(Rotate_timer.delay(5000))
    {
        send_goal(stand_still);
        RCLCPP_INFO(this->get_logger(), "Acabou o timer");
    } 
}

void RobotBehavior::Turn_ball_left_lot()
{
    send_goal(turn_ball_left);
    RCLCPP_INFO(this->get_logger(), "Rotacionar para esquerda muito");
    if(Rotate_timer.delay(5000))
    {
        send_goal(stand_still);
        //RCLCPP_INFO(this->get_logger(), "Acabou o timer");
        timer_lost_ball.reset();
        robot.state = searching_ball_after_goal;
    }
    
}

void RobotBehavior::Turn_ball_right_little()
{
    send_goal(turn_ball_right);
    RCLCPP_INFO(this->get_logger(), "Rotacionar para direita pouco");
    if(Rotate_timer.delay(3000))
    {
        send_goal(stand_still);
        //RCLCPP_INFO(this->get_logger(), "Acabou o timer");
        timer_lost_ball.reset();
        robot.state = searching_ball_after_goal;
    }
    
}

void RobotBehavior::Turn_ball_right_lot()
{
    send_goal(turn_ball_right);
    RCLCPP_INFO(this->get_logger(), "Rotacionar para direita muito");
    if(Rotate_timer.delay(5000))
    {
        send_goal(stand_still);
        RCLCPP_INFO(this->get_logger(), "Acabou o timer");
        timer_lost_ball.reset();
        robot.state = searching_ball_after_goal;
    }
    
}

void RobotBehavior::Turn_ball_right_to_goal()
{
    send_goal(turn_ball_right);
    RCLCPP_INFO(this->get_logger(), "Rotacionar para direita ate o gol");
    if(In_kick_angle())
    {
        send_goal(stand_still);
        robot.state = find_oponent_goal;
    }
}

void RobotBehavior::Turn_ball_left_to_goal()
{
    send_goal(turn_ball_left);
    RCLCPP_INFO(this->get_logger(), "Rotacionar para esquerda ate o gol");
    if(In_kick_angle())
    {
        send_goal(stand_still);
        robot.state = find_oponent_goal;
    }
}

double RobotBehavior::normalize_yaw(double angle)
{
    angle = std::fmod(angle + 180.0, 360.0);

    if (angle < 0.0) {
        angle += 360.0;
    }

    return angle - 180.0;
}

bool RobotBehavior::opositeSide()
{
    normalize_max_yaw = normalize_yaw(start_yaw_deg + 90);
    normalize_min_yaw = normalize_yaw(start_yaw_deg - 90);


    //RCLCPP_INFO(this->get_logger(), "limite +90 %f", normalize_max_yaw);
    //RCLCPP_INFO(this->get_logger(), "limite -90 %f", normalize_min_yaw);
    //RCLCPP_INFO(this->get_logger(), "yaw atual %f", yaw_deg);

    if (normalize_min_yaw <= normalize_max_yaw)
    {
        if (yaw_deg >= normalize_min_yaw && yaw_deg <= normalize_max_yaw)
        {
            RCLCPP_INFO(this->get_logger(), "Oposide side True");
            return true;
        }
    }
    else
    {
        if (yaw_deg >= normalize_min_yaw || yaw_deg <= normalize_max_yaw)
        {
            RCLCPP_INFO(this->get_logger(), "Oposide side True");
            return true;
        }
    }
    RCLCPP_INFO(this->get_logger(), "Oposide side False");
    return false;
}


bool RobotBehavior::In_kick_angle()
{
    normalize_max_kick_angle = normalize_yaw(start_yaw_deg + 22);
    normalize_min_kick_angle = normalize_yaw(start_yaw_deg - 22);


    //RCLCPP_INFO(this->get_logger(), "limite +90 %f", normalize_max_kick_angle);
    //RCLCPP_INFO(this->get_logger(), "limite -90 %f", normalize_min_kick_angle);
    //RCLCPP_INFO(this->get_logger(), "yaw atual %f", yaw_deg);

    if (normalize_min_kick_angle <= normalize_max_kick_angle)
    {
        if (yaw_deg >= normalize_min_kick_angle && yaw_deg <= normalize_max_kick_angle)
        {
            RCLCPP_INFO(this->get_logger(), "Kick angle True");
            return true;
        }
    }
    else
    {
        if (yaw_deg >= normalize_min_kick_angle || yaw_deg <= normalize_max_kick_angle)
        {
            RCLCPP_INFO(this->get_logger(), "Kick angle True");
            return true;
        }
    }
    RCLCPP_INFO(this->get_logger(), "Kick angle False");
    return false;
}


bool RobotBehavior::Turn_to_kick_angle()
{
    yaw_error = normalize_yaw(yaw_deg - first_yaw);

    if (yaw_error > 15.0)
    {
        send_goal(turn_ball_right);
        return false;
    }
    else if (yaw_error < -15.0)
    {
        send_goal(turn_ball_left);
        return false;
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Dentro do angulo de chute");
        return true;
    }
}

void RobotBehavior::Turn_ball_left_180()
{
    send_goal(turn_ball_left);
    RCLCPP_INFO(this->get_logger(), "Rotacionando 180 graus");
    if(In_kick_angle())
    {
        send_goal(stand_still);
        RCLCPP_INFO(this->get_logger(), "Acabou o timer");
        robot.state = find_oponent_goal;
    }
    
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

bool RobotBehavior::goalpost_is_locked() //fazer
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

bool RobotBehavior::gol_vision_stable()
{
    if(Gol_in_camera_center() || gol_in_robot_limits())
    {
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

bool RobotBehavior::gol_in_robot_limits()
{
    if(goalCenter.left && ball_in_left_limit())     return true;
    if(goalCenter.right && ball_in_right_limit())    return true;
    if(goalCenter.close && ball_in_close_limit())    return true;
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

bool RobotBehavior::Gol_in_camera_center()
{
    return (goalCenter.center) && goalCenter.med;
}

bool RobotBehavior::is_penalized() // feito
{
    if(gc_info.penalized)
    {
        RCLCPP_DEBUG(this->get_logger(), "Robot Penalized, remain %d seconds", gc_info.seconds_till_unpenalized);

        send_goal(stand_still);

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