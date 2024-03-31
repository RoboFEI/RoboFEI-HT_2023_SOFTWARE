#include "decision_pkg_cpp/decision_node.hpp"

#define ROBOT_NUMBER 2

class RobotBehavior : public DecisionNode
{
    public:

        void players_behavior();

        RobotBehavior();
        virtual ~RobotBehavior();

    private:
        bool is_penalized();
        void get_up();

        rclcpp::TimerBase::SharedPtr robot_behavior_;

        
}