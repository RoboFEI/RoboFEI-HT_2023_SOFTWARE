#ifndef DECISION_NODE_HPP
#define DECISION_NODE_HPP

#define NECK_TILT_CENTER 2047
#define LIMIT_TH 80 //40

#include <cstdio>
#include <memory>
#include <string>
#include <future>
#include <cmath>

#include "attributes.h"
#include "utils.h"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp/time.hpp"

#include "custom_interfaces/msg/humanoid_league_msgs.hpp"
// #include "custom_interfaces/msg/neck_position.hpp"
#include "custom_interfaces/msg/joint_state.hpp"
#include "custom_interfaces/msg/vision.hpp"
#include "custom_interfaces/msg/robot_state.hpp"
#include "custom_interfaces/action/control.hpp"

#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/bool.hpp"
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
        using IMU6050 = custom_interfaces::msg::RobotState;
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
        void listener_callback_imu_gyro(const ImuGyroMsg::SharedPtr imu_gyro); // giroscopio da UM7
        void listener_callback_imu_accel(const ImuAccelMsg::SharedPtr imu_accel);
        void listener_callback_imu_6050(const IMU6050::SharedPtr imu_6050);
        void robot_detect_fallen(const float &robot_accel_x,
                                 const float &robot_accel_y,
                                 const float &robot_accel_z); 
        void robot_detect_fallen_6050(const bool fall_front, const bool fall_back);
        void listener_callback_vision(const VisionMsg::SharedPtr vision_info); 
        void listener_calback_running_move(const intMsg::SharedPtr atualMove);  
        void listener_callback_localization_status(const std_msgs::msg::Bool localization_msg); // topico q manda se a localização ta ativa
        void listener_callback_goalpost_lines(const VisionMsg::SharedPtr goalpost_division_lines); // grades davisão em relaão as traves
        void listener_callback_goalpost_count(const std_msgs::msg::Int32::SharedPtr goalpost_count); // quantas traves o robo esta vendo
        void listener_callback_goalpost_px(const vision_msgs::msg::Point2D::SharedPtr goalpost_px_position); // posição x das traves
        void listener_callback_imu_rpy(const geometry_msgs::msg::Vector3Stamped::SharedPtr rpy);
        void set_neck_position(); // fazer o robo olhar pra cima
        void look_right(); // fazer o robo olhar pra direita
        void look_down_to_ball(); // fazer o robo olhar pra baixo
        void publish_neck_position(); 

        // o topico do pescoço recebe informações do neck_control e das funções da decisão. 
        // As funções abaixo definem qual que o robo vai seguir:
        void free_neck(); // quando quiser q o robo faça searching ball
        void lock_neck(); // quando quiser chamar o set_neck_position ou o look_right

        
        void send_goal(const Move &move);

        float FALL_ACCEL_TH;
        int FALSES_FALLEN_TH;

        int NECK_CENTER_TH = get_center_th(robot.neck_pos.position20);
        //int NECK_CENTER_TH  = 150; //185
        int NECK_LEFT_TH = NECK_TILT_CENTER + NECK_CENTER_TH;
        int NECK_RIGHT_TH = NECK_TILT_CENTER - NECK_CENTER_TH;
        int NECK_LEFT_LIMIT;
        int NECK_RIGHT_LIMIT;
        int NECK_CLOSE_LIMIT;

        // Timer pra acompanhar o goalpost - localização por visão
        rclcpp::TimerBase::SharedPtr neck_track_timer_; // para atualizar os valores do pescoço e ele olhar pro gol
        bool neck_timer_active_;  
        float last_pan_ = -1;
        float last_tilt_ = -1;
        int vision_wait_counter = 0;
        bool waiting_look_right = false;
        float previous_goalpost_x = -1.0;
        
        // para a localização com UM7
        float delta_yaw; 
        float yaw_reference_ = 0.0;
        bool yaw_reference_set_ = false;
        bool inverted_z; // saber se robô caiu -> z fica invertido

        int robot_number;
        std_msgs::msg::Bool unlock_msg; //para decidir se vai voltar ao searching_ball

        DecisionNode();
        virtual ~DecisionNode();

        // publicar pra olhar pra direita por 2 segundos
        rclcpp::TimerBase::SharedPtr neck_publish_timer_;
        rclcpp::Time neck_publish_start_time_;

    private:
        rclcpp::Subscription<GameControllerMsg>::SharedPtr gc_subscriber_;
        rclcpp::Subscription<JointStateMsg>::SharedPtr neck_position_subscriber_;
        rclcpp::Subscription<ImuGyroMsg>::SharedPtr imu_gyro_subscriber_;
        rclcpp::Subscription<ImuAccelMsg>::SharedPtr imu_accel_subscriber_;
        rclcpp::Subscription<IMU6050>::SharedPtr imu_6050_subscriber;
        rclcpp::Subscription<VisionMsg>::SharedPtr vision_subscriber_;
        rclcpp::Subscription<intMsg>::SharedPtr running_move_subscriber_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr localization_subscriber;
        rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr goalpost_count; // quantas traves foram detectadas 
        rclcpp::Subscription<VisionMsg>::SharedPtr goalpost_division_lines; // de acordo com as grades da visao 
        rclcpp::Subscription<vision_msgs::msg::Point2D>::SharedPtr goalpost_px_position;
        rclcpp::Publisher<JointStateMsg>::SharedPtr neck_position_publisher_;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr neck_control_lock_pub_; // para publicar posições pro pescoço
        rclcpp_action::Client<ControlActionMsg>::SharedPtr action_client_;
        rclcpp_action::Client<ControlActionMsg>::SendGoalOptions send_goal_options = rclcpp_action::Client<ControlActionMsg>::SendGoalOptions();

        rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr imu_rpy_subscriber_;

        void goal_response_callback(const GoalHandleControl::SharedPtr & goal_handle);
        void feedback_callback(
            GoalHandleControl::SharedPtr,
            const std::shared_ptr<const ControlActionMsg::Feedback> feedback);
        void result_callback(const GoalHandleControl::WrappedResult & result);

        GoalHandleControl::SharedPtr goal_handle_;
};


#endif // DECISION_NODE_H