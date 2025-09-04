#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <cmath>
#include <optional>

class GoalAimNode : public rclcpp::Node {
public:
  GoalAimNode() : Node("goal_aim_node")
  {
    using std::placeholders::_1;

    sub_posts_ = create_subscription<std_msgs::msg::Int32MultiArray>(
      "/vision/goal_posts_px", 10,
      std::bind(&GoalAimNode::onPosts, this, _1));

    sub_caminfo_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      "/camera/camera_info", 10,
      std::bind(&GoalAimNode::onCamInfo, this, _1));

    sub_imu_ = create_subscription<sensor_msgs::msg::Imu>(
      "/imu/data", 50,
      std::bind(&GoalAimNode::onImu, this, _1));

    pub_mid_bearing_ = create_publisher<std_msgs::msg::Float32>(
      "/vision/goal_mid_bearing", 10);

    pub_heading_target_ = create_publisher<std_msgs::msg::Float32>(
      "/control/heading_target", 10);

    declare_parameter<bool>("use_imu_for_absolute", false);
    declare_parameter<double>("yaw_lpf_alpha", 0.2);

    use_imu_for_absolute_ = get_parameter("use_imu_for_absolute").as_bool();
    yaw_lpf_alpha_ = get_parameter("yaw_lpf_alpha").as_double();
  }

private:
  void onCamInfo(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
  {
    fx_ = msg->k[0];
    cx_ = msg->k[2];
    have_caminfo_ = true;
  }

  void onImu(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    const auto & q = msg->orientation;
    double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    double yaw = std::atan2(siny_cosp, cosy_cosp);

    if (!yaw_est_) yaw_est_ = yaw;
    yaw_est_ = yaw_lpf_alpha_ * yaw + (1.0 - yaw_lpf_alpha_) * yaw_est_.value();
  }

  static double bearingFromPx(int u, double fx, double cx)
  {
    return std::atan2((u - cx) / fx, 1.0);
  }

  void onPosts(const std_msgs::msg::Int32MultiArray::SharedPtr msg)
  {
    if (!have_caminfo_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Sem CameraInfo ainda; ignorando posts");
      return;
    }
    if (msg->data.size() < 2) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "Esperava [u_left,u_right] em pixels");
      return;
    }

    int u_left = msg->data[0];
    int u_right = msg->data[1];

    // Se só uma trave foi detectada, usa o último par válido
    if (u_left <= 0 || u_right <= 0) {
      if (!last_left_ || !last_right_) {
        RCLCPP_WARN(get_logger(), "Só uma trave detectada e não há histórico válido.");
        return;
      }
      u_left = last_left_.value();
      u_right = last_right_.value();
    } else {
      last_left_ = u_left;
      last_right_ = u_right;
    }

    double aL = bearingFromPx(u_left, fx_, cx_);
    double aR = bearingFromPx(u_right, fx_, cx_);
    double aMid = 0.5 * (aL + aR);

    std_msgs::msg::Float32 out;
    out.data = static_cast<float>(aMid);
    pub_mid_bearing_->publish(out);

    if (use_imu_for_absolute_ && yaw_est_) {
      double heading_target = normalizeAngle(yaw_est_.value() + aMid);
      std_msgs::msg::Float32 h;
      h.data = static_cast<float>(heading_target);
      pub_heading_target_->publish(h);

      // Print no terminal
      RCLCPP_INFO(get_logger(), "Yaw atual: %.3f rad | Yaw alvo: %.3f rad",
                  yaw_est_.value(), heading_target);
    } else {
      // Print relativo se não usar IMU
      RCLCPP_INFO(get_logger(), "Bearing relativo ao gol: %.3f rad", aMid);
    }
  }

  static double normalizeAngle(double a)
  {
    while (a > M_PI) a -= 2*M_PI;
    while (a < -M_PI) a += 2*M_PI;
    return a;
  }

  // Subs/Pubs
  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr sub_posts_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_caminfo_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_mid_bearing_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_heading_target_;

  // Estado
  bool have_caminfo_{false};
  double fx_{0.0}, cx_{0.0};
  std::optional<double> yaw_est_;
  bool use_imu_for_absolute_{false};
  double yaw_lpf_alpha_{0.2};

  std::optional<int> last_left_;
  std::optional<int> last_right_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GoalAimNode>());
  rclcpp::shutdown();
  return 0;
}
