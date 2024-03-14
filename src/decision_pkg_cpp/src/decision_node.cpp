#include "decision_pkg_cpp/decision_node.hpp"

using namespace std::chrono_literals;

DecisionNode::DecisionNode() : Node("decision_node")
{
    RCLCPP_INFO(this->get_logger(), "Running Decision Node"); 
   

    // GameController Subscriber
    gc_subscriber_ = this->create_subscription<GameControllerMsg>(
        "gamestate",
        rclcpp::QoS(10),
        std::bind(&DecisionNode::listener_callback_GC, this, std::placeholders::_1));

    // Ball Position Subscriber
    vision_subscriber_ = this->create_subscription<VisionMsg>(
        "/ball_position",
        rclcpp::QoS(10),
        std::bind(&DecisionNode::listener_callback_vision, this, std::placeholders::_1));
    



    main_timer_ = this->create_wall_timer(
            8ms,
            std::bind(&DecisionNode::main_callback, this));

    // vision_subscriber_ = this->create_subscription<VisionInfo>(
    // "/ball_position", 10, std::bind(this->listener_callback_vision, this, std::placeholders::_1), sub_opt);
}

DecisionNode::~DecisionNode()
{
}

void DecisionNode::listener_callback_GC(const GameControllerMsg::SharedPtr gc_info)
{
    this->gc_info = *gc_info;
    // RCLCPP_INFO(this->get_logger(), "Recive GC Info");
}

void DecisionNode::listener_callback_vision(const VisionMsg::SharedPtr ball_info)
{
    this->ball_info = *ball_info;
    // RCLCPP_INFO(this->get_logger(), "Recive ball info");
}

void DecisionNode::main_callback()
{
    RCLCPP_INFO(this->get_logger(), "ball detected: %d", this->ball_info.detected);
    RCLCPP_INFO(this->get_logger(), "Primary GameState: %d", this->gc_info.game_state);
    

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

