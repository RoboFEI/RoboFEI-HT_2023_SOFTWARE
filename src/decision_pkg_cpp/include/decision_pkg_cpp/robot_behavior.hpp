#ifndef ROBOT_BEHAVIOR_HPP
#define ROBOT_BEHAVIOR_HPP

#include "decision_pkg_cpp/decision_node.hpp"
#include "decision_pkg_cpp/attributes.h"
#include "decision_pkg_cpp/utils.h"
#include "decision_pkg_cpp/AssyncTimer.hpp"
#include "std_msgs/msg/int32.hpp"
#include "custom_interfaces/msg/vision.hpp"
#include "sensor_msgs/msg/imu.hpp"

#define ROBOT_NUMBER 2
//2 KICKER
//3 BALA


using namespace std::chrono_literals;
using Int32 = std_msgs::msg::Int32;
using VisionInfo = custom_interfaces::msg::Vision;
using ImuAccelMsg = sensor_msgs::msg::Imu;

struct GoalCenterInformation
{  
  bool detected     = false;  
  bool left         = false;  
  bool center       = false;  
  bool right        = false;  
  bool far          = false;  
  bool med          = false;  
  bool close        = false;
};

class RobotBehavior : public DecisionNode
{
    public:

        GoalCenterInformation goalCenter;

        void players_behavior();
        void normal_game();
        void normal_game_prepair();
        void player_normal_game();
        void goalkeeper_normal_game(); // feito, precisa testar
        void bala_normal_game();
        void kicker_normal_game();
        bool is_goalkeeper(int robot_num); // feito
        bool is_bala(int robot_num);
        bool is_kicker(int robot_num);
        void kicker_localization_game();
        void bala_localization_game();
        bool ball_is_locked();
        bool goalpost_is_locked(); 
        bool vision_stable();
        bool ball_in_camera_center();
        bool ball_in_robot_limits();
        bool ball_in_left_limit();
        bool ball_in_right_limit();
        bool ball_in_close_limit();
        bool ball_in_right_foot();
        bool ball_in_left_foot();
        bool robot_align_with_the_ball();
        bool goalkeeper_align_with_the_ball(); // feito, precisa testar
        bool robot_align_for_kick_right();
        bool robot_align_for_kick_left();
        bool centered_neck();
        void detect_ball_position();
        bool neck_to_left();
        bool neck_to_right();
        void turn_to_ball();
        bool robot_is_align_with_goal();
        void Turn_ball_left_little();
        void Turn_ball_left_lot();
        void Turn_ball_right_little();
        void Turn_ball_right_lot();
        bool Gol_in_camera_center();
        bool gol_vision_stable();
        void Turn_ball_left_lot_noSwitch();
        void Turn_ball_left_180();
        bool In_kick_angle();
        void Turn_ball_left_to_goal();
        void Turn_ball_right_to_goal();

        

        void penalty();
        void player_penalty();
        void goalkeeper_penalty(); // fazer
        bool full_centered_neck();
        bool opositeSide();

        int pos19;
        bool back_to_search;
        int error19;
        float start_yaw_deg;
        bool first_yaw = true;
        double yaw_deg;
        float normalize_max_yaw;
        float normalize_min_yaw;
        float normalize_max_kick_angle;
        float normalize_min_kick_angle;
        float error_yaw;
        RobotBehavior();
        virtual ~RobotBehavior();

        void listener_callback_goal_center_vision(const VisionInfo::SharedPtr msg);
        void listener_callback_imu_yaw(const ImuAccelMsg::SharedPtr msg);
        double normalize_yaw(double angle);
        
    private:

        AssyncTimer lost_ball_timer;
        AssyncTimer look_right_timer;
        AssyncTimer check_goalpost_timer;
        AssyncTimer Rotate_timer;

        void change_state(State new_state);
        bool aligning_with_goal_entry_done = false;

        bool is_penalized();
        void get_up();

        rclcpp::TimerBase::SharedPtr robot_behavior_;
        rclcpp::Publisher<Int32>::SharedPtr publisher_neck_state;
        rclcpp::Subscription<VisionInfo>::SharedPtr vision_goal_center_subscriber;
        rclcpp::Subscription<ImuAccelMsg>::SharedPtr imu_yaw_subscriber;

        
};

#endif // ROBOT_BEHAVIOR_H