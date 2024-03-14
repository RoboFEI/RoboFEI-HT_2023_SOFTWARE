#ifndef DECISION_NODE_HPP
#define DECISION_NODE_HPP

#include <cstdio>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "custom_interfaces/msg/vision.hpp"
#include "custom_interfaces/msg/humanoid_league_msgs.hpp"



// struct GameController
// {
    
// };

// enum PrimaryGameState
// {
//     Initial,
//     Ready,
//     Set,
//     Playing,

// }





class DecisionNode : public rclcpp::Node
{
    public:
        using GameControllerMsg = custom_interfaces::msg::HumanoidLeagueMsgs;
        using VisionMsg = custom_interfaces::msg::Vision;
        
        GameControllerMsg   gc_info     ;
        VisionMsg           ball_info   ;

        void listener_callback_GC(const GameControllerMsg::SharedPtr msg);
        void listener_callback_vision(const VisionMsg::SharedPtr ball_info);
        void main_callback();


        DecisionNode();
        virtual ~DecisionNode();

    private:
        rclcpp::Subscription<GameControllerMsg>::SharedPtr gc_subscriber_;
        rclcpp::Subscription<VisionMsg>::SharedPtr vision_subscriber_;

        rclcpp::TimerBase::SharedPtr main_timer_;

    


};


#endif // DECISION_NODE_H