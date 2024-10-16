// Open terminal #2 
// $ ros2 run control control
// $ ros2 run um7 um7_node
// $ ros2 launch control action.launch.py


// ros2 action send_goal  /control_action custom_interfaces/action/Control "{action_number: 1}"

#include <chrono>
#include <memory>
#include <thread>
#include <functional>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <unistd.h>
#include <libgen.h> //dirname
#include <vector>
#include <cstdio>
#include <string.h>
#include <fstream>
#include <filesystem>

#include "MotionManager.h"
#include "Walking.h"
#include "rclcpp/rclcpp.hpp"
// #include "custom_interfaces/msg/set_position.hpp"
#include "custom_interfaces/msg/decision.hpp"
#include "custom_interfaces/msg/walk.hpp"
#include "custom_interfaces/msg/vision.hpp"
#include "sensor_msgs/msg/imu.hpp"
// #include "custom_interfaces/msg/neck_position.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32.hpp"
#include "GaitMove.hpp"
// #include "custom_interfaces/msg/set_position_original.hpp"
#include "custom_interfaces/srv/reset.hpp"
#include "custom_interfaces/action/control.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "json/single_include/nlohmann/json.hpp"
#include "custom_interfaces/msg/joint_state.hpp"

namespace fs = std::filesystem;

using namespace std;
using namespace Robot;
using namespace std::chrono_literals;

using std::placeholders::_1;
using Control_action = custom_interfaces::action::Control;
using GoalHandleAction = rclcpp_action::ServerGoalHandle<Control_action>;

using json = nlohmann::json;

std::string folder_path = fs::current_path();

int movement = 1;
int contador = 0;
bool fase_zero = true;
float sleep_sec = 0.0;
int parameters = true;
int parameter_number = 0;
int walk = 0;
int number_of_mov = 0;
int address_value = 116;
int last_movement = 1;
bool do_gait = false;


std::string address_name = "address";
std::string id_name = "id";
std::string vel_name = "velocity";
std::string position_name = "position";
std::string sleep_name = "sleep";
std::string section = "Stand Still";

class Control : public rclcpp::Node
{
public:
  using JointStateMsg = custom_interfaces::msg::JointState;
  using intMsg = std_msgs::msg::Int32;
  
  int robot_number_;
  int cWalk;

  std::vector<int> stepMoves = {1, 2, 3, 4, 7, 30, 31, 32, 33, 34, 35};
  std::vector<int> paramMoves = {5, 6, 9, 10, 14, 20, 21};

  bool body_activate_;

  json j;

  Control()
  : Node("control")
  {
    RCLCPP_INFO(this->get_logger(), "Running action node");
    subscriber_fase_zero = this->create_subscription<std_msgs::msg::Bool>("/fase_zero", 1, std::bind(&Control::topic_callback_fase, this, _1));
    // publisher_ = this->create_publisher<custom_interfaces::msg::SetPosition>("set_position", 10); 
    // publisher_single = this->create_publisher<custom_interfaces::msg::SetPositionOriginal>("set_position_single", 10);
    pubisher_body_joints_ = this->create_publisher<JointStateMsg>("set_joint_topic", 10);
    publisher_walk = this->create_publisher<custom_interfaces::msg::Walk>("walking", 10);
    publisher_atual_move = this->create_publisher<intMsg>("move_running", 10);

    using namespace std::placeholders;

    this->action_server_ = rclcpp_action::create_server<Control_action>(
    this,
    "control_action",
    std::bind(&Control::handle_goal, this, _1, _2), // Callbacks
    std::bind(&Control::handle_cancel, this, _1),
    std::bind(&Control::handle_accepted, this, _1));

    body_activate_ = this->declare_parameter("body_activate", true);
    robot_number_ = this->declare_parameter("robot_number", 2);
    cWalk = this->declare_parameter("walk_counter", 1);

    RCLCPP_INFO(this->get_logger(), "body activate: %d", body_activate_);
    RCLCPP_INFO(this->get_logger(), "robot number: %d", robot_number_);

    ifstream fJson(folder_path + "/src/control/Data/motion" + std::to_string(robot_number_) + ".json");
    j = json::parse(fJson);
  }

  

  vector<int> allIds { 1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18};
  vector<vector<int>> position;
  
private:

    void topic_callback_fase(const std::shared_ptr<std_msgs::msg::Bool> fase_msg) const
    {
      fase_zero = fase_msg->data;
    }

    // handle new goals, accept or rejects them 
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const Control_action::Goal> goal)
    {       // feedback no terminal do controle numero 1
        RCLCPP_INFO(this->get_logger(), "Received action %d", goal->action_number);
        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    // receives the cancellation and tell the client that it was accepted
    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleAction> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel action");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    // accepts a new goal and process it
    void handle_accepted(const std::shared_ptr<GoalHandleAction> goal_handle)
    {
        using namespace std::placeholders;
        // this needs to return quickly to avoid blocking the executor, so spin up a new thread
        std::thread{std::bind(&Control::execute, this, _1), goal_handle}.detach();
    }

    void choose_movement(int movement){
      switch (movement) {
        case 1: 
          RCLCPP_INFO(this->get_logger(), "Stand Still");
          parameters = false;
          section = "Stand Still";
          break;
        case 2: 
          RCLCPP_INFO(this->get_logger(), "Greeting");
          parameters = false;
          section = "Greeting";
          break;
        case 3: 
          RCLCPP_INFO(this->get_logger(), "Right Kick");
          parameters = false;
          section = "Right Kick";
          break;
        case 4: 
          RCLCPP_INFO(this->get_logger(), "Left Kick");
          parameters = false;
          section = "Left Kick";
          break;
        case 5: 
          RCLCPP_INFO(this->get_logger(), "Turn Right");
          parameters = true;
          parameter_number = 11;  
          break;
        case 6: 
          RCLCPP_INFO(this->get_logger(), "Turn Left");
          parameters = true;
          parameter_number = 12;  
          break;
        case 7: 
          RCLCPP_INFO(this->get_logger(), "Goodbye");
          parameters = false;
          section = "Goodbye";
          break;
        case 8: 
          RCLCPP_INFO(this->get_logger(), "Search Ball");
          parameters = false;
          section = "Search Ball";
          break;
        case 9: //melhor que o 10
          RCLCPP_INFO(this->get_logger(), "Turn Around Ball Clockwise");
          parameters = true;
          parameter_number = 5;  
          break;
        case 10: 
          RCLCPP_INFO(this->get_logger(), "Turn Around Ball Anticlockwise");
          parameters = true;
          parameter_number = 6;  
          break;
        case 11: 
          RCLCPP_INFO(this->get_logger(), "Goalkeeper Fall Left");
          parameters = false;
          section = "Goalkeeper Fall Left";
          break;
        case 12: 
          RCLCPP_INFO(this->get_logger(), "Goalkeeper Fall Right");
          parameters = false;
          section = "Goalkeeper Fall Right";
          break;
        case 13: //
          RCLCPP_INFO(this->get_logger(), "Goalkeeper Middle");
          parameters = false;
          section = "Goalkeeper Middle";
          break;
        case 14: 
          parameters = true;
          RCLCPP_INFO(this->get_logger(), "Walking"); 
          parameter_number = 1;  
          break;
        case 15: 
          parameters = true;
          RCLCPP_INFO(this->get_logger(), "Gait"); 
          parameter_number = 2;  
          break;
        case 16: 
          RCLCPP_INFO(this->get_logger(), "Stand Up Back");
          parameters = false;
          section = "Stand Up Back";
          break;
        case 17: 
          RCLCPP_INFO(this->get_logger(), "Stand Up Front");
          parameters = false;
          section = "Stand Up Front";
          break;
        case 18:
          RCLCPP_INFO(this->get_logger(), "Fallen Side Left");
          parameters = false;
          section = "Fallen Side Left";
          break; 
        case 19:
          RCLCPP_INFO(this->get_logger(), "Fallen Side Right");
          parameters = false;
          section = "Fallen Side Right";
          break;
        case 20: 
          RCLCPP_INFO(this->get_logger(), "Andar esquerda");
          parameters = true;
          parameter_number = 8;  
          break;
        case 21: // OK
          RCLCPP_INFO(this->get_logger(), "Andar direita");
          parameters = true;
          parameter_number = 7;  
          break;
        case 26: // Andar pra trás
          RCLCPP_INFO(this->get_logger(), "Walking Backward");
          parameters = true;
          parameter_number = 9;
          break; 
        case 28: 
          RCLCPP_INFO(this->get_logger(), "Goodbye Loop");
          parameters = false;
          section = "Goodbye Loop";
          break; 
        case 29:
          RCLCPP_INFO(this->get_logger(), "Goalkeeper Searching Ball");
          parameters = false;
          section = "Goalkeeper Searching Ball";
          break;
        case 30: 
          RCLCPP_INFO(this->get_logger(), "Right Kick Penalti");
          parameters = false;
          section = "Right Kick Penalti";
          break;
        case 31: 
          RCLCPP_INFO(this->get_logger(), "Right Kick Variant R");
          parameters = false;
          section = "Right Kick Variant R";
          break;
        case 32: 
          RCLCPP_INFO(this->get_logger(), "Right Kick Variant L");
          parameters = false;
          section = "Right Kick Variant L";
          break;
        case 33: 
          RCLCPP_INFO(this->get_logger(), "Left Kick Variant R");
          parameters = false;
          section = "Left Kick Variant R";
          break;
        case 34: 
          RCLCPP_INFO(this->get_logger(), "Left Kick Variant L");
          parameters = false;
          section = "Left Kick Variant L";
          break;
        case 35: 
          RCLCPP_INFO(this->get_logger(), "Left Kick Penalti");
          parameters = false;
          section = "Left Kick Penalti";
          break;
      }
      auto atualMove = intMsg();
      atualMove.data = movement;
      publisher_atual_move->publish(atualMove);
    }

    bool isParamMove(int move)
    {
      return count(paramMoves.begin(), paramMoves.end(), move) > 0;
    }

    bool isStepMove(int move)
    {
      return count(stepMoves.begin(), stepMoves.end(), move) > 0;
    }

    void execute(const std::shared_ptr<GoalHandleAction> goal_handle)
    {   
        auto message_walk = custom_interfaces::msg::Walk();  
        // auto message = custom_interfaces::msg::SetPosition();   
        // auto message_single = custom_interfaces::msg::SetPositionOriginal();
        
        const auto goal = goal_handle->get_goal();
        movement = goal->action_number;
        //      feedback no terminal do controle numero 2
        //RCLCPP_INFO(this->get_logger(), "Decision: %d", movement);
        auto feedback = std::make_shared<Control_action::Feedback>();
        auto & movements_remaining = feedback->movements_remaining;
        auto result = std::make_shared<Control_action::Result>();

        if(body_activate_ || movement == 1) // when body deactivate only recive move 1
        {
          //      feedback no terminal do controle numero 3
          //RCLCPP_INFO(this->get_logger(), "Executing goal");

          // message_single.id = 254;
          // message_single.address = 112;
          // message_single.position = 128;
          // publisher_single->publish(message_single);
          auto setJointInfoMsg = JointStateMsg();
          setJointInfoMsg.id.push_back(254);
          setJointInfoMsg.info.push_back(112);
          setJointInfoMsg.type.push_back(JointStateMsg::VELOCITY);
          pubisher_body_joints_->publish(setJointInfoMsg);

          int esse = 0;
          if (movement == 14 || movement == 5 || movement == 6 || movement == 9 || movement == 10 || movement == 20 || movement == 21){
            esse = movement;
            if (contador < cWalk){
              movement = 15;   
            }
            contador++;
            if (contador >= cWalk){
              movement = esse;
              contador = 0;
              RCLCPP_INFO(this->get_logger(), "Gait trocando movimentacao %d", contador);
            }
          }
            choose_movement(movement);          

            if (parameters){
              if (goal_handle->is_canceling()) {
                  result->finished = false;
                  goal_handle->canceled(result);
                  RCLCPP_INFO(this->get_logger(), "Goal canceled");
                  return;
              }
              message_walk.walk_number = parameter_number;  
              publisher_walk->publish(message_walk);
            }
          
            else{
              if (fase_zero) {
                message_walk.walk_number = 0; 
                publisher_walk->publish(message_walk);
                // message.id = motors;

                number_of_mov = j[section]["number of movements"];
                
                for (int i = 1; i <= number_of_mov; i++){
                  
                  message_walk.walk_number = 0; 
                  publisher_walk->publish(message_walk);
                  address_name = "address";
                  id_name = "id";
                  vel_name = "velocity";
                  position_name = "position";
                  sleep_name = "sleep";
                  
                  //RCLCPP_INFO(this->get_logger(), " i: %d ",  i);
                  if (goal_handle->is_canceling()) {
                      result->finished = false;
                      goal_handle->canceled(result);
                      RCLCPP_INFO(this->get_logger(), "Goal canceled");
                      return;
                  }

                  address_name = address_name + std::to_string(i);
                  if (j[section][address_name] == 112){
                    //      feedback no terminal do controle 4
                    //RCLCPP_INFO(this->get_logger(), "VELOCIDADE");
                    auto setJointInfoMsg = JointStateMsg();
                    id_name = id_name + std::to_string(i);
                    setJointInfoMsg.id.push_back(j[section][id_name]);
                    // message_single.id = j[section][id_name];
                    vel_name = vel_name + std::to_string(i);

                    setJointInfoMsg.info.push_back(j[section][vel_name]);
                    setJointInfoMsg.type.push_back(JointStateMsg::VELOCITY);
                    pubisher_body_joints_->publish(setJointInfoMsg);

                    // message_single.position = j[section][vel_name];
                    // message_single.address = j[section][address_name];
                    // publisher_single->publish(message_single);
                    usleep(500000);
                  }
                  else if (j[section][address_name] == 116){
                    position_name = position_name + std::to_string(i);
                    position.push_back(j[section][position_name]);

                    auto setJointInfoMsg = JointStateMsg();
                    setJointInfoMsg.info = position.front();
                    setJointInfoMsg.id = allIds;
                    setJointInfoMsg.type = std::vector<std::uint8_t>(20, 0);
    
                    // message.position = position.front();
                    //      feedback no terminal do controle 5
                    //RCLCPP_INFO(this->get_logger(), "POSIÇÃO %d %d", position[0][18], position[0][19]);
                    pubisher_body_joints_->publish(setJointInfoMsg);

                    // publisher_->publish(message);
                    sleep_name = sleep_name + std::to_string(i);
                    sleep_sec = j[section][sleep_name];
                    //      feedback no terminal do controle 6
                    //RCLCPP_INFO(this->get_logger(), "Sleep: %f ", sleep_sec);
                    usleep(sleep_sec*1000000);
                    position.clear();
                  }
                
                  movements_remaining = number_of_mov - i;
                  // Publish feedback
                  goal_handle->publish_feedback(feedback);
                  //      feedback no terminal do controle 7
                  //RCLCPP_INFO(this->get_logger(), "Feedback: %d movements remaining", movements_remaining);
                }
              }
            }
        }
      result->finished = true;
      goal_handle->succeed(result);
      //      feedback no terminal do controle 8
      //RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscriber_fase_zero;
    // rclcpp::Publisher<custom_interfaces::msg::SetPosition>::SharedPtr publisher_; 
    // rclcpp::Publisher<custom_interfaces::msg::SetPositionOriginal>::SharedPtr publisher_single; 
    rclcpp::Publisher<JointStateMsg>::SharedPtr pubisher_body_joints_;
    rclcpp::Publisher<custom_interfaces::msg::Walk>::SharedPtr publisher_walk; 
    rclcpp::Publisher<intMsg>::SharedPtr publisher_atual_move; 
    rclcpp::Client<custom_interfaces::srv::Reset>::SharedPtr client;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp_action::Server<Control_action>::SharedPtr action_server_;
};

void change_current_dir()
{
    char exepath[1024] = {0};
    if(readlink("/proc/self/exe", exepath, sizeof(exepath)) != -1)
        chdir(dirname(exepath));
}

int main(int argc, char * argv[])
{
  change_current_dir();
  //Configurando para prioridade maxima para executar este processo-------
  char string1[50]; //String
  sprintf(string1,"echo 123456| sudo -S renice -20 -p %d", getpid());
  system(string1);//prioridade
  //GaitMove gaitMove(ini);
  rclcpp::init(argc, argv);
  // if(MotionManager::GetInstance()->Initialize() == false)
  //   {
  //     printf("Fail to initialize Motion Manager!\n");
  //     return 0;
  // }
  rclcpp::spin(std::make_shared<Control>());
  rclcpp::shutdown();
  return 0;
}
