#ifndef JOY_ROBOT_NODE_HPP
#define JOY_ROBOT_NODE_HPP

#include <chrono>
#include <functional>
#include <memory>
#include <future>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "sensor_msgs/msg/joy.hpp"
#include "custom_interfaces/action/control.hpp"


enum Move
{
    stand_still     = 1,
    greeting        = 2,
    right_kick      = 3,
    left_kick       = 4,
    turn_right      = 5,
    turn_left       = 6,
    goodbye         = 7,
    walk            = 14,
    gait            = 15,
    stand_up_back   = 16,
    stand_up_front  = 17,
    stand_up_side   = 18
};

class JoyRobotNode : public rclcpp::Node
{
    public:
        using JoyMsg = sensor_msgs::msg::Joy;

        using ControlActionMsg = custom_interfaces::action::Control;
        using GoalHandleControl = rclcpp_action::ClientGoalHandle<ControlActionMsg>;

        void listener_callback_joy(const JoyMsg::SharedPtr joy_msg);
        void button_reactions(const JoyMsg old_joy_info, const JoyMsg new_joy_info);
        
        void send_goal(const Move &move);

        JoyRobotNode();
        virtual ~JoyRobotNode();

    private:
        rclcpp::Subscription<JoyMsg>::SharedPtr joy_subscriber_;

        rclcpp_action::Client<ControlActionMsg>::SharedPtr action_client_;
        rclcpp_action::Client<ControlActionMsg>::SendGoalOptions send_goal_options = rclcpp_action::Client<ControlActionMsg>::SendGoalOptions();

        void goal_response_callback(const GoalHandleControl::SharedPtr & goal_handle);
        void feedback_callback(
            GoalHandleControl::SharedPtr,
            const std::shared_ptr<const ControlActionMsg::Feedback> feedback);
        void result_callback(const GoalHandleControl::WrappedResult & result);

        GoalHandleControl::SharedPtr goal_handle_;

        Move last_move = stand_still;
        bool finished_move = true;

        JoyMsg joy_info;
        bool gait_mode = false;

};

#endif