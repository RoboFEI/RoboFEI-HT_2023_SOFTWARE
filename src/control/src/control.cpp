// Open terminal #2 
// $ ros2 run control control
// $ ros2 run um7 um7_node
// $ ros2 launch control action.launch.py


// ros2 topic pub -1 /decision custom_interfaces/Decision "{decision: 1}"
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

#include "MotionManager.h"
#include "Walking.h"
#include "rclcpp/rclcpp.hpp"
#include "custom_interfaces/msg/set_position.hpp"
#include "custom_interfaces/msg/decision.hpp"
#include "custom_interfaces/msg/walk.hpp"
#include "custom_interfaces/msg/vision.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "custom_interfaces/msg/neck_position.hpp"
#include "GaitMove.hpp"
#include "custom_interfaces/msg/set_position_original.hpp"
#include "custom_interfaces/srv/reset.hpp"
#include "custom_interfaces/action/control.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "json/single_include/nlohmann/json.hpp"



using namespace std;
using namespace Robot;
using namespace std::chrono_literals;
using std::placeholders::_1;
using Control_action = custom_interfaces::action::Control;
using GoalHandleAction = rclcpp_action::ServerGoalHandle<Control_action>;

using json = nlohmann::json;

int movement = 1;
int contador = 0;
int cont_vision = 2047;
int cont_vision_up = 1300;
int cont_fall_side = 0;
bool stop_gait = true;
uint32_t valor = 246;
uint32_t valor_up = 250;
int neck_sides = 2048;
int neck_up = 2048;
int sleep_sec = 0;
int parameters = false;
int parameter_number = 0;
int walk = 0;
int sidle = 0;
int turn = 0;
int ender = 0;
int value = 0;
int number_of_mov = 0;
std::string address_name = "address0";
std::string id_name = "id0";
std::string vel_name = "velocity0";
std::string position_name = "position0";
std::string sleep_name = "sleep0";
std::string section = "Stand Still";


ifstream fJson("/home/robo/RoboFEI-HT_2022_SOFTWARE/src/control/Data/motion.json");
json j = json::parse(fJson);

#define INI_FILE_PATH       "src/control/Data/motion.json"

class Control : public rclcpp::Node
{
public:
  Control()
  : Node("control")
  {
    RCLCPP_INFO(this->get_logger(), "Running action node");
    subscription_neck = this->create_subscription<custom_interfaces::msg::NeckPosition>(
      "/neck_position", 10, std::bind(&Control::topic_callback_neck, this, _1));
    publisher_ = this->create_publisher<custom_interfaces::msg::SetPosition>("set_position", 10); 
    publisher_single = this->create_publisher<custom_interfaces::msg::SetPositionOriginal>("set_position_single", 10);
    publisher_walk = this->create_publisher<custom_interfaces::msg::Walk>("walking", 10); 

    
    using namespace std::placeholders;

    this->action_server_ = rclcpp_action::create_server<Control_action>(
    this,
    "control_action",
    std::bind(&Control::handle_goal, this, _1, _2), // Callbacks
    std::bind(&Control::handle_cancel, this, _1),
    std::bind(&Control::handle_accepted, this, _1));

    }
  vector<int> motors { 1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20 };
  vector<vector<int>> position;
  
    

private:

    void topic_callback_neck(const std::shared_ptr<custom_interfaces::msg::NeckPosition> neck_msg) const
    {
      neck_sides = neck_msg->position19;
      neck_up = neck_msg->position20;
    }

    // handle new goals, accept or rejects them 
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const Control_action::Goal> goal)
    {
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


    void execute(const std::shared_ptr<GoalHandleAction> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Executing goal");

        auto message_walk = custom_interfaces::msg::Walk();  
        auto message = custom_interfaces::msg::SetPosition();   
        auto message_single = custom_interfaces::msg::SetPositionOriginal();  
        
        const auto goal = goal_handle->get_goal();
        movement = goal->action_number;
        RCLCPP_INFO(this->get_logger(), "Decision: %d", movement);
        auto feedback = std::make_shared<Control_action::Feedback>();
        auto & movements_remaining = feedback->movements_remaining;
        auto result = std::make_shared<Control_action::Result>();

        message_single.id = 254;
        message_single.address = 112;
        message_single.position = 128;
        publisher_single->publish(message_single);

        switch (movement) {
          case 1: // Parado
            parameters = false;
            section = "Stand Still";
            break;
          case 4: // Left kick OK
            RCLCPP_INFO(this->get_logger(), "Left kick");
            parameters = false;
            break;
          case 14: // Walking
            parameters = true;
            RCLCPP_INFO(this->get_logger(), "Walking"); 
            parameter_number = 1;  
            break;
          case 15: // Gait
            parameters = true;
            RCLCPP_INFO(this->get_logger(), "Gait"); 
            parameter_number = 2;  
            break;
          case 17: // Stand up Front
            parameters = false;
            break;
          case 18:
            parameters = false;
            section = "Stand Up Front";
            break;
        }

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
          message_walk.walk_number = 0; 
          publisher_walk->publish(message_walk);
          message.id = motors;

          number_of_mov = j[section]["number of movements"];

          for (int i = 1; i <= number_of_mov; i++){
            if (goal_handle->is_canceling()) {
                result->finished = false;
                goal_handle->canceled(result);
                RCLCPP_INFO(this->get_logger(), "Goal canceled");
                return;
            }
            address_name.pop_back();
            address_name = address_name + std::to_string(i);
            if (j[section][address_name] == 112){
              RCLCPP_INFO(this->get_logger(), "AQUI");
              id_name.pop_back();
              id_name = id_name + std::to_string(i);
              message_single.id = j[section][id_name];
              vel_name.pop_back();
              vel_name = vel_name + std::to_string(i);
              message_single.position = j[section][vel_name];
              message_single.address = j[section][address_name];
              publisher_single->publish(message_single);
            }
            else if (j[section][address_name] == 116){
              position_name.pop_back();
              position_name = position_name + std::to_string(i);
              position.push_back(j[section][position_name]);
              message.position = position.front();
              publisher_->publish(message);
              sleep_name.pop_back();
              sleep_name = sleep_name + std::to_string(i);
              sleep_sec = j[section][sleep_name];
              RCLCPP_INFO(this->get_logger(), "Sleep: %d ", sleep_sec);
              usleep(sleep_sec*1000000);
            }
          
            movements_remaining = number_of_mov - i;
            // Publish feedback
            goal_handle->publish_feedback(feedback);
            RCLCPP_INFO(this->get_logger(), "Feedback: %d movements remaining", movements_remaining);
          }
        }
      
      result->finished = true;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }

    rclcpp::Subscription<custom_interfaces::msg::NeckPosition>::SharedPtr subscription_neck;
    rclcpp::Publisher<custom_interfaces::msg::SetPosition>::SharedPtr publisher_; 
    rclcpp::Publisher<custom_interfaces::msg::SetPositionOriginal>::SharedPtr publisher_single; 
    rclcpp::Publisher<custom_interfaces::msg::Walk>::SharedPtr publisher_walk;      
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