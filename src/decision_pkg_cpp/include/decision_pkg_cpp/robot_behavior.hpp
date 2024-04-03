#include "decision_pkg_cpp/decision_node.hpp"
#include "decision_pkg_cpp/attributes.h"
#include "decision_pkg_cpp/utils.h"

#define ROBOT_NUMBER 2

class RobotBehavior : public DecisionNode
{
    public:

        void players_behavior();
        void normal_game();
        void player_normal_game();
        bool ball_found();

        RobotBehavior();
        virtual ~RobotBehavior();

    private:
        bool is_penalized();
        void get_up();

        rclcpp::TimerBase::SharedPtr robot_behavior_;

        
}