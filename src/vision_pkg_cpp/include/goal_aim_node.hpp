#ifndef LOCALIZATION_NODE_HPP
#define LOCALIZATION_NODE_HPP


#include <cstdio>
#include <memory>
#include <string>
#include <future>
#include <cmath>


#include "custom_interfaces/msg/vision.hpp"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <cmath>
#include <optional>
#include "std_msgs/msg/bool.hpp" 
#include "std_msgs/msg/int32.hpp" 
#include "std_msgs/msg/string.hpp"
#include "custom_interfaces/msg/vision.hpp"

class GoalAimNode : public rclcpp::Node {
    public:
        using ImuAccelMsg = sensor_msgs::msg::Imu;
        using VisionMsg = custom_interfaces::msg::Vision;
        using intMsg = std_msgs::msg::Int32;


        ImuGyroMsg          imu_gyro;
        Robot               robot;


        void listener_callback_goalpost_px(const vision_msgs::msg::Point2D::SharedPtr goalpost_px_position); // posição x das traves  



    private:
        rclcpp::Subscription<ImuGyroMsg>::SharedPtr imu_gyro_subscriber_;
        rclcpp::Subscription<ImuAccelMsg>::SharedPtr imu_accel_subscriber_;
        rclcpp::Subscription<VisionMsg>::SharedPtr vision_subscriber_;
        rclcpp::Subscription<intMsg>::SharedPtr running_move_subscriber_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr localization_subscriber;
        rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr goalpost_count; // quantas traves foram detectadas 
        rclcpp::Subscription<VisionMsg>::SharedPtr goalpost_division_lines; // de acordo com as grades da visao 
        rclcpp::Subscription<vision_msgs::msg::Point2D>::SharedPtr goalpost_px_position;



};


#endif 