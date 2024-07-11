#ifndef ROBOT_BEHAVIOR_HPP
#define ROBOT_BEHAVIOR_HPP

#include "decision_pkg_cpp/decision_node.hpp"
#include "decision_pkg_cpp/attributes.h"
#include "decision_pkg_cpp/utils.h"
#include "decision_pkg_cpp/AssyncTimer.hpp"

#define ROBOT_NUMBER 2

using namespace std::chrono_literals;

class RobotBehavior : public DecisionNode
{
    public:
        void players_behavior();
        void normal_game();
        void player_normal_game();
        bool ball_is_locked();
        bool vision_stable();
        bool ball_in_camera_center();
        bool ball_in_robot_limits();
        bool ball_in_left_limit();
        bool ball_in_right_limit();
        bool ball_in_close_limit();
        bool ball_in_right_foot();
        bool ball_in_left_foot();
        bool robot_align_with_the_ball();
        bool robot_align_for_kick_right();
        bool robot_align_for_kick_left();
        bool centered_neck();
        bool full_centered_neck();
        void detect_ball_position();
        bool neck_to_left();
        bool neck_to_right();
        void turn_to_ball();

        void penalty();
        void player_penalty();

        RobotBehavior();
        virtual ~RobotBehavior();

    private:

        AssyncTimer lost_ball_timer;
        bool is_penalized();
        void get_up();

        rclcpp::TimerBase::SharedPtr robot_behavior_;

        
};

#endif // ROBOT_BEHAVIOR_H