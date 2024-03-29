#ifndef DECISION_NODE_HPP
#define DECISION_NODE_HPP

#include <cstdio>
#include <memory>
#include <string>
#include <future>

#include <unistd.h> //apagar

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "custom_interfaces/msg/humanoid_league_msgs.hpp"
#include "custom_interfaces/msg/neck_position.hpp"
#include "custom_interfaces/action/control.hpp"

#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "sensor_msgs/msg/imu.hpp"

#define FALL_ACCEL_TH 7.0
#define FALSES_FALLEN_TH 30

using namespace std::placeholders;

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

enum FallStatus
{
    NotFallen   = 0,
    FallenFront = 1,
    FallenBack  = 2,
    FallenRight = 3,
    FallenLeft  = 4
};

struct Robot
{
    FallStatus fall = NotFallen;
    Move movement = stand_still;
    bool finished_move = true;
    custom_interfaces::msg::NeckPosition neck_pos;
};



class DecisionNode : public rclcpp::Node
{
    public:
        using GameControllerMsg = custom_interfaces::msg::HumanoidLeagueMsgs;
        using NeckPosMsg = custom_interfaces::msg::NeckPosition;
        using ImuGyroMsg = geometry_msgs::msg::Vector3Stamped;
        using ImuAccelMsg = sensor_msgs::msg::Imu;

        using ControlActionMsg = custom_interfaces::action::Control;
        using GoalHandleControl = rclcpp_action::ClientGoalHandle<ControlActionMsg>;
        

        GameControllerMsg   gc_info;
        ImuGyroMsg          imu_gyro;
        Robot               robot;

        int falses_fallen_counter = 0;

        Move lixo = right_kick;

        void listener_callback_GC(const GameControllerMsg::SharedPtr gc_info);
        void listener_callback_neck_pos(const NeckPosMsg::SharedPtr neck_pos);
        void listener_callback_imu_gyro(const ImuGyroMsg::SharedPtr imu_gyro);
        void listener_callback_imu_accel(const ImuAccelMsg::SharedPtr imu_accel);
        void robot_detect_fallen(const float &robot_accel_x,
                                 const float &robot_accel_y,
                                 const float &robot_accel_z);        
        
        void send_goal(const Move &move);
        
        void body_behavior_callback();


        DecisionNode();
        virtual ~DecisionNode();

    private:
        rclcpp::Subscription<GameControllerMsg>::SharedPtr gc_subscriber_;
        rclcpp::Subscription<NeckPosMsg>::SharedPtr neck_position_subscriber_;
        rclcpp::Subscription<ImuGyroMsg>::SharedPtr imu_gyro_subscriber_;
        rclcpp::Subscription<ImuAccelMsg>::SharedPtr imu_accel_subscriber_;

        rclcpp_action::Client<ControlActionMsg>::SharedPtr action_client_;
        rclcpp_action::Client<ControlActionMsg>::SendGoalOptions send_goal_options = rclcpp_action::Client<ControlActionMsg>::SendGoalOptions();

        rclcpp::TimerBase::SharedPtr body_behavior_;

        void goal_response_callback(const GoalHandleControl::SharedPtr & goal_handle);
        void feedback_callback(
            GoalHandleControl::SharedPtr,
            const std::shared_ptr<const ControlActionMsg::Feedback> feedback);
        void result_callback(const GoalHandleControl::WrappedResult & result);


        GoalHandleControl::SharedPtr goal_handle_;
        std::shared_future<GoalHandleControl::SharedPtr> goal_handle_future_;



};


#endif // DECISION_NODE_H