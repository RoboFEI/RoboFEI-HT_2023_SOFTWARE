#ifndef DECISION_NODE_HPP
#define DECISION_NODE_HPP

#include <cstdio>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "custom_interfaces/msg/humanoid_league_msgs.hpp"
#include "custom_interfaces/msg/neck_position.hpp"

#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "sensor_msgs/msg/imu.hpp"

class DecisionNode : public rclcpp::Node
{
    public:
        using GameControllerMsg = custom_interfaces::msg::HumanoidLeagueMsgs;
        using NeckPosMsg = custom_interfaces::msg::NeckPosition;
        using ImuGyroMsg = geometry_msgs::msg::Vector3Stamped;
        using ImuAccelMsg = sensor_msgs::msg::Imu;

        GameControllerMsg   gc_info;
        NeckPosMsg          neck_pos;
        ImuGyroMsg             imu_gyro;

        void listener_callback_GC(const GameControllerMsg::SharedPtr gc_info);
        void listener_callback_neck_pos(const NeckPosMsg::SharedPtr neck_pos);
        void listener_callback_imu_gyro(const ImuGyroMsg::SharedPtr imu_gyro);


        void main_callback();


        DecisionNode();
        virtual ~DecisionNode();

    private:
        rclcpp::Subscription<GameControllerMsg>::SharedPtr gc_subscriber_;
        rclcpp::Subscription<NeckPosMsg>::SharedPtr neck_position_subscriber_;
        rclcpp::Subscription<ImuGyroMsg>::SharedPtr imu_gyro_subscriber_;

        rclcpp::TimerBase::SharedPtr main_timer_;
};


#endif // DECISION_NODE_H