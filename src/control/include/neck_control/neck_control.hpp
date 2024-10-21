#ifndef NECK_CONTROL_HPP_
#define NECK_CONTROL_HPP_

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "AssyncTimer.hpp"

#include "custom_interfaces/msg/vision.hpp"
// #include "custom_interfaces/msg/neck_position.hpp"
#include "custom_interfaces/msg/joint_state.hpp"
#include "custom_interfaces/msg/set_position.hpp"
#include "std_msgs/msg/multi_array_dimension.hpp"
#include "vision_msgs/msg/point2_d.hpp"


using namespace std::chrono_literals;

struct BallInformation
{
  bool detected     = false;
  bool left         = false;
  bool center_left  = false;
  bool right        = false;
  bool center_right = false;
  bool far          = false;
  bool med          = false;
  bool close        = false;
};

struct Neck
{
  int pan   = 2048;
  int tilt  = 2048;
};

enum class Side 
{
  left,
  right,
  down,
  up 
};

enum class State
{
  follow_ball,
  search_ball
};

struct BallPositionPx
{
  int x = 0;
  int y = 0;
};


class NeckNode : public rclcpp::Node
{
  public:
    using VisionInfo = custom_interfaces::msg::Vision;
    // using NeckPosition = custom_interfaces::msg::NeckPosition;
    using JointStateMsg = custom_interfaces::msg::JointState;
    using SetPosition = custom_interfaces::msg::SetPosition;
    using MultArrDim = std_msgs::msg::MultiArrayDimension;
    using Point2d = vision_msgs::msg::Point2D;


    BallInformation ball;
    Neck neck;
    State robot_state = State::follow_ball;
    BallPositionPx ball_pos_px;
    int cont_lost_ball = 0;

    int search_ball_delay = 10;
    std::vector<int> search_ball_limits = {2348,2498,2648}; //Não usar valores < 2048
    std::vector<int> search_ball_samples = {75,100,125};
    std::vector<int> search_ball_pos = {2048,2048};

    int search_ball_state = 0;
    int robotNumber;

    AssyncTimer lost_ball_timer;
    AssyncTimer search_ball_timer;

    float x_p_gain;
    float y_p_gain;

    bool neck_activate_;

    int neck_up_limit;
    int neck_down_limit;
    int neck_left_limit;
    int neck_right_limit;
    
    void listener_callback_vision(const VisionInfo::SharedPtr msg);
    void listener_callback_vision_px(const Point2d::SharedPtr msg);
    void listener_callback_neck(const JointStateMsg::SharedPtr msg); 
    void search_ball();
    void main_callback();
       

    NeckNode();
    virtual ~NeckNode();

  private:
    rclcpp::CallbackGroup::SharedPtr callback_group_subscriber_;
    rclcpp::CallbackGroup::SharedPtr main_thread_;

    rclcpp::Subscription<VisionInfo>::SharedPtr vision_subscriber_;
    rclcpp::Subscription<Point2d>::SharedPtr vision_px_subscriber_;
    rclcpp::Subscription<JointStateMsg>::SharedPtr neck_position_subscriber_;
    rclcpp::Publisher<JointStateMsg>::SharedPtr set_neck_position_publisher_;
    rclcpp::TimerBase::SharedPtr main_timer_;
    
};

#endif 

