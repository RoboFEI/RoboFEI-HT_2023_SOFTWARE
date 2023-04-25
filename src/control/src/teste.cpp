#include "rclcpp/rclcpp.hpp"
#include "custom_interfaces/srv/get_position.hpp"

void add(const std::shared_ptr<custom_interfaces::srv::GetPosition::Request> request,
          std::shared_ptr<custom_interfaces::srv::GetPosition::Response>      response)
{
  response->position = 2048;
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%d] [%d]", (int)request->id, (int)response->position);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("teste");

  rclcpp::Service<custom_interfaces::srv::GetPosition>::SharedPtr service =
    node->create_service<custom_interfaces::srv::GetPosition>("get_position", &add);
  rclcpp::spin(node);
  rclcpp::shutdown();
}
