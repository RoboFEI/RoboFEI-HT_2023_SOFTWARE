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
#include "std_msgs/msg/bool.hpp"
#include "vision_msgs/msg/point2_d.hpp"
#include "std_msgs/msg/int32.hpp"


using namespace std::chrono_literals;


struct BallInformation
{
  bool detected     = false;
  bool left         = false;
  bool center       = false;
  bool right        = false;
  bool far          = false;
  bool med          = false;
  bool close        = false;
};

struct GoalCenterInformation
{  
  bool detected     = false;  
  bool left         = false;  
  bool center       = false;  
  bool right        = false;  
  bool far          = false;  
  bool med          = false;  
  bool close        = false;
};
/*
struct GoalPostInformation
{
  bool detected     = false;  
  bool left         = false;  
  bool center       = false;  
  bool right        = false;  
  bool far          = false;  
  bool med          = false;  
  bool close        = false;
};
*/

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
  follow_ball = 1,
  search_ball = 2,
  search_goalcenter = 3,
  follow_goal_center = 4,  
  //search_goal_post = 5,
};

struct BallPositionPx
{
  int x = 0;
  int y = 0;
};

struct goalpost_center_px
{  
  int x = 0;  
  int y = 0;
};
/*
struct goalpost_px_position
{
  int x = 0;
  int y = 0;
};
*/

class NeckNode : public rclcpp::Node
{
  public:
    using VisionInfo = custom_interfaces::msg::Vision;
    // using NeckPosition = custom_interfaces::msg::NeckPosition;
    using JointStateMsg = custom_interfaces::msg::JointState;
    using SetPosition = custom_interfaces::msg::SetPosition;
    using MultArrDim = std_msgs::msg::MultiArrayDimension;
    using Point2d = vision_msgs::msg::Point2D;
    using Int32 = std_msgs::msg::Int32;
    using Bool = std_msgs::msg::Bool;


    BallInformation ball;
    goalpost_center_px goal_center;  
    //goalpost_px_position goal_post;          
    GoalCenterInformation goalCenter;
    //GoalPostInformation goalPost;
    Neck neck;
    State robot_state = State::follow_ball;
    BallPositionPx ball_pos_px;
    int cont_lost_ball = 0;
    int search_ball_delay = 10;
    std::vector<int> search_ball_limits = {2548,2698,2848}; //Não usar valores < 2048
    std::vector<int> search_ball_samples = {75,100,125};
    std::vector<int> search_ball_pos = {2048,2048};
    float search_ball_state = 0;
    int robotNumber;
    int goal_post_counter;

    AssyncTimer lost_ball_timer;
    AssyncTimer search_ball_timer;

    float x_p_gain;
    float y_p_gain;

    float x_i_gain;
    float y_i_gain;

    float x_d_gain;
    float y_d_gain;

    float last_error_x = 0;
    float last_error_y = 0;

    float acc_error_x;
    float acc_error_y;

    float vel_error_x;
    float vel_error_y;

    float P_x;
    float P_y;

    float I_x;
    float I_y;

    float D_x;
    float D_y;

    float error_x;
    float error_y;

    State new_state;

    bool neck_activate_;
    bool neck_locked_;
    bool received_neck_lock_;

    int neck_up_limit;
    int neck_down_limit;
    int neck_left_limit;
    int neck_right_limit;

    void listener_callback_vision(const VisionInfo::SharedPtr msg);
    void listener_callback_vision_px(const Point2d::SharedPtr msg);
    void listener_callback_neck(const JointStateMsg::SharedPtr msg);
    void neck_lock_callback(const std_msgs::msg::Bool::SharedPtr msg);
    void listener_callback_goal_center(const Point2d::SharedPtr msg);
    void change_neck_state(const Int32::SharedPtr msg);
    void listener_callback_goal_center_vision(const VisionInfo::SharedPtr msg);
    //void listener_callback_goal_post(const Point2d::SharedPtr msg);
    //void listener_callback_goal_post_pos(const VisionInfo::SharedPtr msg);
    //oid listener_callback_goal_post_counter(const Int32::SharedPtr msg);

    void search_goalcenter();
    void search_ball();
    void main_callback();
    //void search_other_goal_post();


    NeckNode();
    virtual ~NeckNode();

  private:
    rclcpp::CallbackGroup::SharedPtr callback_group_subscriber_;
    rclcpp::CallbackGroup::SharedPtr main_thread_;

    rclcpp::Subscription<VisionInfo>::SharedPtr vision_subscriber_;
    rclcpp::Subscription<Point2d>::SharedPtr vision_px_subscriber_;
    rclcpp::Subscription<JointStateMsg>::SharedPtr neck_position_subscriber_;
    rclcpp::Publisher<JointStateMsg>::SharedPtr set_neck_position_publisher_;
    rclcpp::Publisher<Bool>::SharedPtr neck_finished_search_publisher;
    rclcpp::TimerBase::SharedPtr main_timer_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr neck_control_lock_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr neck_state_subscriber;
    rclcpp::Subscription<VisionInfo>::SharedPtr vision_goal_center_subscriber;
    rclcpp::Subscription<Point2d>::SharedPtr vision_subscriber_goal_center;
    //rclcpp::Subscription<Point2d>::SharedPtr vision_subscriber_goal_post;
    //rclcpp::Subscription<VisionInfo>::SharedPtr vision_goal_post_subscriber;
    //rclcpp::Subscription<Int32>::SharedPtr vision_subscriber_goal_post_counter;
};

#endif 

