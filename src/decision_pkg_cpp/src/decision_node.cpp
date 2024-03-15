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

    neck_position_subscriber_ = this->create_subscription<NeckPosMsg>(
        "neck_position",
        rclcpp::QoS(10),
        std::bind(&DecisionNode::listener_callback_neck_pos, this, std::placeholders::_1));

    imu_gyro_subscriber_ = this->create_subscription<ImuGyroMsg>(
        "imu/rpy",
        rclcpp::QoS(10),
        std::bind(&DecisionNode::listener_callback_imu_gyro, this, std::placeholders::_1));





    main_timer_ = this->create_wall_timer(
            8ms,
            std::bind(&DecisionNode::main_callback, this));
}

DecisionNode::~DecisionNode()
{
}

void DecisionNode::listener_callback_GC(const GameControllerMsg::SharedPtr gc_info)
{
    this->gc_info = *gc_info;
    // RCLCPP_INFO(this->get_logger(), "Recive GC Info");
}

void DecisionNode::listener_callback_neck_pos(const NeckPosMsg::SharedPtr neck_pos)
{
    this->neck_pos = *neck_pos;
    // RCLCPP_INFO(this->get_logger(), "Recive Neck Pos Info");

}

void DecisionNode::listener_callback_imu_gyro(const ImuGyroMsg::SharedPtr imu_gyro)
{
    this->imu_gyro = *imu_gyro;
    //RCLCPP_INFO(this->get_logger(), "Recive Imu Gyro Info");
}

void DecisionNode::main_callback()
{
    RCLCPP_INFO(this->get_logger(), "Primary GameState: %d", this->gc_info.game_state);
    RCLCPP_INFO(this->get_logger(), "id19: %d / id20: %d", this->neck_pos.position19, this->neck_pos.position20); 

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

