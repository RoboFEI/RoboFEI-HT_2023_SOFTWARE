#include "new_motors_pkg/motors_communication.hpp"

MotorsCommunication::MotorsCommunication()
: Node("read_write_node")
{
  RCLCPP_INFO(this->get_logger(), "Run read write node");

  timer_ = this->create_wall_timer(
  8ms, std::bind(&MotorsCommunication::timer_callback, this));
}

MotorsCommunication::~MotorsCommunication()
{
}

void MotorsCommunication::timer_callback()
{
  RCLCPP_INFO(this->get_logger(), "node fine");

}

int main(int argc, char * argv[])
{ 
  rclcpp::init(argc, argv);

  auto motorscomm = std::make_shared<MotorsCommunication>();
  rclcpp::spin(motorscomm);
  rclcpp::shutdown();

  return 0;
}