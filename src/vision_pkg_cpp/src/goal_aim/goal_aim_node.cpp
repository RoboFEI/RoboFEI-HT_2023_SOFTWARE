#include "goal_aim_node.hpp"
#include <cmath>
#include <std_msgs/msg/float64.hpp>

GoalAimNode::GoalAimNode() : Node("goal_aim_node")
{
  RCLCPP_INFO(get_logger(), "Iniciando GoalAimNode");
  using std::placeholders::_1;

  sub_posts_ = create_subscription<std_msgs::msg::Int32MultiArray>(
    "/goalpost_px_position", 10,
    std::bind(&GoalAimNode::onPosts, this, _1));

  sub_caminfo_ = create_subscription<sensor_msgs::msg::CameraInfo>(
    "/camera/camera_info", 10,
    std::bind(&GoalAimNode::onCamInfo, this, _1));

  sub_imu_ = create_subscription<sensor_msgs::msg::Imu>(
    "/imu/data", 50,
    std::bind(&GoalAimNode::onImu, this, _1));

  pub_mid_bearing_ = create_publisher<std_msgs::msg::Float32>(
    "/vision/goal_mid_bearing", 10);

  pub_goal_yaw_ = create_publisher<std_msgs::msg::Float64>(
    "/goal_yaw", 10);

  pub_yaw_est_ = create_publisher<std_msgs::msg::Float64>(
    "/yaw_est", 10);  

  declare_parameter<bool>("use_imu_for_absolute", false);
  declare_parameter<double>("yaw_lpf_alpha", 0.2);

  use_imu_for_absolute_ = get_parameter("use_imu_for_absolute").as_bool();
  yaw_lpf_alpha_ = get_parameter("yaw_lpf_alpha").as_double();
}

void GoalAimNode::onCamInfo(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
  fx_ = msg->k[0];
  cx_ = msg->k[2];
  have_caminfo_ = true;
  RCLCPP_INFO(get_logger(), "fx=%f cx=%f", fx_, cx_);
}

void GoalAimNode::onImu(const sensor_msgs::msg::Imu::SharedPtr msg)
{
  double qx = msg->orientation.x;
  double qy = msg->orientation.y;
  double qz = msg->orientation.z;
  double qw = msg->orientation.w;

  // Calcula yaw a partir do quaternion
  double siny_cosp = 2.0 * (qw * qz + qx * qy);
  double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
  double yaw = std::atan2(siny_cosp, cosy_cosp);

  // Filtro LPF (low-pass) para estabilizar yaw
  if (yaw_est_) {
    yaw_est_ = yaw_lpf_alpha_ * yaw + (1.0 - yaw_lpf_alpha_) * (*yaw_est_);
  } else {
    yaw_est_ = yaw;
  }

  //RCLCPP_WARN(get_logger(), "%f yaw  %f yaw est", yaw, yaw_est_.value_or(0.0));

  // Publica yaw_est como Float64
  std_msgs::msg::Float64 yaw_msg;
  yaw_msg.data = yaw_est_.value_or(0.0);
  pub_yaw_est_->publish(yaw_msg);
}

void GoalAimNode::onPosts(const std_msgs::msg::Int32MultiArray::SharedPtr msg)
{
  if (!have_caminfo_) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Sem CameraInfo ainda; ignorando posts");
    return;
  }

  if (msg->data.size() < 2) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Esperava [u_left,u_right] em pixels");
    return;
  }

  int left_px = msg->data[0];
  int right_px = msg->data[1];
  last_left_ = left_px;
  last_right_ = right_px;

  double left_bearing = bearingFromPx(left_px, fx_, cx_);
  double right_bearing = bearingFromPx(right_px, fx_, cx_);
  double mid_bearing = (left_bearing + right_bearing) / 2.0;

  std_msgs::msg::Float32 mid_msg;
  mid_msg.data = mid_bearing;
  pub_mid_bearing_->publish(mid_msg);

  if (yaw_est_) {
    // Target heading: yaw absoluto do robô + direção para o meio do gol
    double goal_yaw = normalizeAngle((*yaw_est_) + mid_bearing);

    // Publica como Float64
    std_msgs::msg::Float64 goal_msg;
    goal_msg.data = goal_yaw;
    pub_goal_yaw_->publish(goal_msg);
  }
}

double GoalAimNode::bearingFromPx(int u, double fx, double cx)
{
  return std::atan2((u - cx), fx);
}

double GoalAimNode::normalizeAngle(double a)
{
  while (a > M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  std::cout << "angulo normalizado: " << a << std::endl;
  return a;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GoalAimNode>());
  rclcpp::shutdown();
  return 0;
}
