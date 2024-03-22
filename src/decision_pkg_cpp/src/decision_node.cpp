#include "decision_pkg_cpp/decision_node.hpp"

using namespace std::chrono_literals;

DecisionNode::DecisionNode() : Node("decision_node")
{
    RCLCPP_INFO(this->get_logger(), "Running Decision Node"); 
   

    // GameController Subscriber
    gc_subscriber_ = this->create_subscription<GameControllerMsg>(
        "gamestate",
        rclcpp::QoS(10),
        std::bind(&DecisionNode::listener_callback_GC, this, std::placeholders::_1)
    );

    neck_position_subscriber_ = this->create_subscription<NeckPosMsg>(
        "neck_position",
        rclcpp::QoS(10),
        std::bind(&DecisionNode::listener_callback_neck_pos, this, std::placeholders::_1)
    );

    imu_gyro_subscriber_ = this->create_subscription<ImuGyroMsg>(
        "imu/rpy",
        rclcpp::QoS(10),
        std::bind(&DecisionNode::listener_callback_imu_gyro, this, std::placeholders::_1)
    );

    imu_accel_subscriber_ = this->create_subscription<ImuAccelMsg>(
        "imu/data",
        rclcpp::QoS(10),
        std::bind(&DecisionNode::listener_callback_imu_accel, this, std::placeholders::_1)
    );

    this->action_client_ = rclcpp_action::create_client<ControlActionMsg>(
      this,
      "control_action"
    );



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

void DecisionNode::listener_callback_imu_accel(const ImuAccelMsg::SharedPtr imu_accel)
{
  robot_detect_fallen( imu_accel->linear_acceleration.x,
                      -imu_accel->linear_acceleration.y,
                      -imu_accel->linear_acceleration.z); 

  RCLCPP_INFO(this->get_logger(), "Recive Imu Accel Info");
}

void DecisionNode::robot_detect_fallen(const float &robot_accel_x,
                                       const float &robot_accel_y,
                                       const float &robot_accel_z)
{
  RCLCPP_INFO(this->get_logger(), "Ax: %f\nAy: %f\nAz: %f", robot_accel_x, robot_accel_y, robot_accel_z);

    // VERIFICAR O FUNCIONAMENTO ASSIM
    // if(abs(robot_accel_y) < FALL_ACCEL_TH)
    // {
    //     falses_fallen_counter += 1;
    // }
    // else falses_fallen_counter = 0;


  if( abs(robot_accel_x) > FALL_ACCEL_TH ||
    abs(robot_accel_z) > FALL_ACCEL_TH)
  {
    falses_fallen_counter += 1;
  }
  else
  {
    falses_fallen_counter = 0;
    this->robot.fall = NotFallen;
  } 

  if(falses_fallen_counter > 30)
  {
    if(robot_accel_z > FALL_ACCEL_TH)       this->robot.fall = FallenFront;
    else if(robot_accel_z < -FALL_ACCEL_TH) this->robot.fall = FallenBack;
    else if(robot_accel_x > FALL_ACCEL_TH)  this->robot.fall = FallenLeft;
    else this->robot.fall = FallenRight;  
  }
}

void DecisionNode::send_goal(const Move &move)
{

    // if (!this->action_client_->wait_for_action_server()) {
    //   RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
    //   rclcpp::shutdown();
    // }

    // auto goal_msg = ControlActionMsg::Goal();
    // goal_msg.action_number = int(move);

    // RCLCPP_INFO(this->get_logger(), "Sending goal %d", goal_msg.action_number);

    // auto send_goal_options = rclcpp_action::Client<ControlActionMsg>::SendGoalOptions();
    // send_goal_options.goal_response_callback =
    //   std::bind(&DecisionNode::goal_response_callback, this, _1);

    


    // send_goal_options.feedback_callback =
    //   std::bind(&FibonacciActionClient::feedback_callback, this, _1, _2);
    // send_goal_options.result_callback =
    //   std::bind(&FibonacciActionClient::result_callback, this, _1);
    // this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
}

void DecisionNode::goal_response_callback(std::shared_future<GoalHandleControl::SharedPtr> future)
  {
    // auto goal_handle = future.get();
    // if (!goal_handle) {
    //   RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    // } else {
    //   RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    // }
  }


void DecisionNode::main_callback()
{
    // RCLCPP_INFO(this->get_logger(), "Primary GameState: %d", this->gc_info.game_state);
    // RCLCPP_INFO(this->get_logger(), "id19: %d / id20: %d", this->neck_pos.position19, this->neck_pos.position20);
    // send_goal(Move::gait);
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

