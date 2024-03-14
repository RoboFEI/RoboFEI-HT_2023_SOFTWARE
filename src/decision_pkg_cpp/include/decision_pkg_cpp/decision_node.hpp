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
        
        GameControllerMsg GC_info;

        void listener_callback_GC(const GameControllerMsg::SharedPtr msg);

        DecisionNode();
        virtual ~DecisionNode();

    private:
        rclcpp::Subscription<GameControllerMsg>::SharedPtr gc_subscriber_;
    


};


#endif // DECISION_NODE_H