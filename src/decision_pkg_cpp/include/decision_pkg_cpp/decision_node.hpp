#ifndef DECISION_NODE_HPP
#define DECISION_NODE_HPP

#define NECK_TILT_CENTER 2048
#define LIMIT_TH 40

#include <cstdio>
#include <memory>
#include <string>
#include <future>
#include <cmath>

#include "attributes.h"
#include "utils.h"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "custom_interfaces/msg/humanoid_league_msgs.hpp"
// #include "custom_interfaces/msg/neck_position.hpp"
#include "custom_interfaces/msg/joint_state.hpp"
#include "custom_interfaces/msg/vision.hpp"
#include "custom_interfaces/action/control.hpp"

#include "std_msgs/msg/int32.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "sensor_msgs/msg/imu.hpp"

using namespace std::placeholders;

class DecisionNode : public rclcpp::Node
{
    public:
        using GameControllerMsg = custom_interfaces::msg::HumanoidLeagueMsgs;
        // using NeckPosMsg = custom_interfaces::msg::NeckPosition;
        using JointStateMsg = custom_interfaces::msg::JointState;
        using ImuGyroMsg = geometry_msgs::msg::Vector3Stamped;
        using ImuAccelMsg = sensor_msgs::msg::Imu;
        using VisionMsg = custom_interfaces::msg::Vision;
        using intMsg = std_msgs::msg::Int32;

        using ControlActionMsg = custom_interfaces::action::Control;
        using GoalHandleControl = rclcpp_action::ClientGoalHandle<ControlActionMsg>;
        

        GameControllerMsg   gc_info;
        ImuGyroMsg          imu_gyro;
        Robot               robot;

        int falses_fallen_counter = 0;
        int side_penalty = 0;

        void listener_callback_GC(const GameControllerMsg::SharedPtr gc_info);
        void listener_callback_neck_pos(const JointStateMsg::SharedPtr neck_pos);
        void listener_callback_imu_gyro(const ImuGyroMsg::SharedPtr imu_gyro);
        void listener_callback_imu_accel(const ImuAccelMsg::SharedPtr imu_accel);
        void robot_detect_fallen(const float &robot_accel_x,
                                 const float &robot_accel_y,
                                 const float &robot_accel_z); 
        void listener_callback_vision(const VisionMsg::SharedPtr vision_info); 
        void listener_calback_running_move(const intMsg::SharedPtr atualMove);      
        
        void send_goal(const Move &move);

        float FALL_ACCEL_TH;
        int FALSES_FALLEN_TH;

        int NECK_CENTER_TH  = 185;
        int NECK_LEFT_TH = NECK_TILT_CENTER + NECK_CENTER_TH;
        int NECK_RIGHT_TH = NECK_TILT_CENTER - NECK_CENTER_TH;
        int NECK_LEFT_LIMIT;
        int NECK_RIGHT_LIMIT;
        int NECK_CLOSE_LIMIT;
        
        DecisionNode();
        virtual ~DecisionNode();

    private:
        rclcpp::Subscription<GameControllerMsg>::SharedPtr gc_subscriber_;
        rclcpp::Subscription<JointStateMsg>::SharedPtr neck_position_subscriber_;
        rclcpp::Subscription<ImuGyroMsg>::SharedPtr imu_gyro_subscriber_;
        rclcpp::Subscription<ImuAccelMsg>::SharedPtr imu_accel_subscriber_;
        rclcpp::Subscription<VisionMsg>::SharedPtr vision_subscriber_;
        rclcpp::Subscription<intMsg>::SharedPtr running_move_subscriber_;

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