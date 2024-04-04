#ifndef DECISION_NODE_HPP
#define DECISION_NODE_HPP

#include <cstdio>
#include <memory>
#include <string>
#include <future>

#include "attributes.h"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "custom_interfaces/msg/humanoid_league_msgs.hpp"
#include "custom_interfaces/msg/neck_position.hpp"
#include "custom_interfaces/msg/vision.hpp"
#include "custom_interfaces/action/control.hpp"


#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "sensor_msgs/msg/imu.hpp"

#define FALL_ACCEL_TH 7.0
#define FALSES_FALLEN_TH 30

using namespace std::placeholders;

class DecisionNode : public rclcpp::Node
{
    public:
        using GameControllerMsg = custom_interfaces::msg::HumanoidLeagueMsgs;
        using NeckPosMsg = custom_interfaces::msg::NeckPosition;
        using ImuGyroMsg = geometry_msgs::msg::Vector3Stamped;
        using ImuAccelMsg = sensor_msgs::msg::Imu;
        using VisionMsg = custom_interfaces::msg::Vision;

        using ControlActionMsg = custom_interfaces::action::Control;
        using GoalHandleControl = rclcpp_action::ClientGoalHandle<ControlActionMsg>;
        

        GameControllerMsg   gc_info;
        ImuGyroMsg          imu_gyro;
        Robot               robot;

        int falses_fallen_counter = 0;

        void listener_callback_GC(const GameControllerMsg::SharedPtr gc_info);
        void listener_callback_neck_pos(const NeckPosMsg::SharedPtr neck_pos);
        void listener_callback_imu_gyro(const ImuGyroMsg::SharedPtr imu_gyro);
        void listener_callback_imu_accel(const ImuAccelMsg::SharedPtr imu_accel);
        void robot_detect_fallen(const float &robot_accel_x,
                                 const float &robot_accel_y,
                                 const float &robot_accel_z); 
        void listener_callback_vision(const VisionMsg::SharedPtr vision_info);       
        
        void send_goal(const Move &move);
        
        DecisionNode();
        virtual ~DecisionNode();

    private:
        rclcpp::Subscription<GameControllerMsg>::SharedPtr gc_subscriber_;
        rclcpp::Subscription<NeckPosMsg>::SharedPtr neck_position_subscriber_;
        rclcpp::Subscription<ImuGyroMsg>::SharedPtr imu_gyro_subscriber_;
        rclcpp::Subscription<ImuAccelMsg>::SharedPtr imu_accel_subscriber_;
        rclcpp::Subscription<VisionMsg>::SharedPtr vision_subscriber_;

        rclcpp_action::Client<ControlActionMsg>::SharedPtr action_client_;
        rclcpp_action::Client<ControlActionMsg>::SendGoalOptions send_goal_options = rclcpp_action::Client<ControlActionMsg>::SendGoalOptions();

        void goal_response_callback(const GoalHandleControl::SharedPtr & goal_handle);
        void feedback_callback(
            GoalHandleControl::SharedPtr,
            const std::shared_ptr<const ControlActionMsg::Feedback> feedback);
        void result_callback(const GoalHandleControl::WrappedResult & result);

        GoalHandleControl::SharedPtr goal_handle_;

};


#endif // DECISION_NODE_H