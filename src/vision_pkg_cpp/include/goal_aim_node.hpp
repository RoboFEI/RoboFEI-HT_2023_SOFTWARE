#ifndef GOAL_AIM_NODE_HPP
#define GOAL_AIM_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <optional>
#include <iostream> 

class GoalAimNode : public rclcpp::Node {
public:
  GoalAimNode();

private:
  // Callbacks
  void onCamInfo(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
  void onImu(const sensor_msgs::msg::Imu::SharedPtr msg);
  void onPosts(const std_msgs::msg::Int32MultiArray::SharedPtr msg);

  // Funções auxiliares
  static double bearingFromPx(int u, double fx, double cx);
  static double normalizeAngle(double a);

  // Subscribers
  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr sub_posts_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_caminfo_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;

  // Publishers
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_mid_bearing_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_goal_yaw_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_yaw_est_;

  // Estado interno
  bool have_caminfo_{false};
  double fx_{0.0}, cx_{0.0};
  std::optional<double> yaw_est_;
  bool use_imu_for_absolute_{false};
  double yaw_lpf_alpha_{0.2};
  std::optional<int> last_left_;
  std::optional<int> last_right_;
};

#endif
