#include "decision_pkg_cpp/decision_node.hpp"

DecisionNode::DecisionNode() : Node("decision_node")
{
    RCLCPP_INFO(this->get_logger(), "Running Decision Node"); 
    RCLCPP_INFO(this->get_logger(), "Recive %d", this->GC_info.game_state);
   

    // GameController Subscriber
    gc_subscriber_ = this->create_subscription<GameControllerMsg>(
        "gamestate",
        rclcpp::QoS(10),
        std::bind(&DecisionNode::listener_callback_GC, this, std::placeholders::_1));

    // vision_subscriber_ = this->create_subscription<VisionInfo>(
    //     "/ball_position",
    //     rclcpp::QoS(10),
    //     std::bind(this->listener_callback_vision, this, std::placeholders::_1), sub_opt);

    
    

    



    
    





    // vision_subscriber_ = this->create_subscription<VisionInfo>(
    // "/ball_position", 10, std::bind(this->listener_callback_vision, this, std::placeholders::_1), sub_opt);
}

DecisionNode::~DecisionNode()
{
}

void DecisionNode::listener_callback_GC(const GameControllerMsg::SharedPtr msg)
{
    this->GC_info = *msg;
    RCLCPP_INFO(this->get_logger(), "Recive %d", this->GC_info.game_state);
}


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto decision_node = std::make_shared<DecisionNode>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(decision_node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}

