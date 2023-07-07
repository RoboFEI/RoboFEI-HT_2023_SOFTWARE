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

#include "MotionManager.h"
#include "Walking.h"
#include "rclcpp/rclcpp.hpp"
#include "custom_interfaces/msg/set_position.hpp"
#include "custom_interfaces/msg/decision.hpp"
#include "custom_interfaces/msg/walk.hpp"
#include "custom_interfaces/msg/vision.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "custom_interfaces/msg/neck_position.hpp"
#include "std_msgs/msg/bool.hpp"
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

int robot_number = 2;

int movement = 1;
int contador = 0;
int cont_vision = 40;
int cont_vision_up = 40;
int cont_vision_sides = 40;
bool fase_zero = true;
uint32_t valor = 246;
uint32_t valor_up = 250;
int neck_sides = 2048;
int neck_up = 2048;
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


ifstream fJson("./src/control/Data/motion" + std::to_string(robot_number) + ".json");
json j = json::parse(fJson);


class Control : public rclcpp::Node
{
public:
  Control()
  : Node("control")
  {
    RCLCPP_INFO(this->get_logger(), "Running action node");
    subscription_neck = this->create_subscription<custom_interfaces::msg::NeckPosition>(
      "/neck_position", 10, std::bind(&Control::topic_callback_neck, this, _1));
    subscriber_fase_zero = this->create_subscription<std_msgs::msg::Bool>("/fase_zero", 10, std::bind(&Control::topic_callback_fase, this, _1));
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
    
    void topic_callback_fase(const std::shared_ptr<std_msgs::msg::Bool> fase_msg) const
    {
      fase_zero = fase_msg->data;
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
          RCLCPP_INFO(this->get_logger(), "Right kick");
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
          RCLCPP_INFO(this->get_logger(), "Fall Left");
          parameters = false;
          section = "Fall Left";
          break;
        case 12: 
          RCLCPP_INFO(this->get_logger(), "Fall Right");
          parameters = false;
          section = "Fall Right";
          break;
        case 13: //
          RCLCPP_INFO(this->get_logger(), "Agachando");
          parameters = false;
          section = "Agachando";
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
          RCLCPP_INFO(this->get_logger(), "Fallen Side");
          parameters = false;
          section = "Fallen Side";
          break;
        case 19: 
          RCLCPP_INFO(this->get_logger(), "Andar esquerda");
          parameters = true;
          parameter_number = 8;  
          break;
        case 20: // OK
          RCLCPP_INFO(this->get_logger(), "Andar direita");
          parameters = true;
          parameter_number = 7;  
          break;
        case 21: //
          RCLCPP_INFO(this->get_logger(), "Centralizando bola à esquerda");
          parameters = false;
          neck_sides += cont_vision_sides;
          section = "Stand Still";
          break;
        case 22: //
          RCLCPP_INFO(this->get_logger(), "Centralizando bola à direita");
          parameters = false;
          neck_sides -= cont_vision_sides;
          section = "Stand Still";
          break;
        case 23: //
          RCLCPP_INFO(this->get_logger(), "Centralizando bola no pé");
          parameters = false; 
          neck_up -= cont_vision_up;
          section = "Stand Still";
          RCLCPP_INFO(this->get_logger(), "%d", neck_up);
          break;
        case 24: // Centralizar bola acima
          RCLCPP_INFO(this->get_logger(), "Centralizando bola acima");
          parameters = false;
          neck_up += cont_vision_up;
          section = "Stand Still";
          break;
        case 25: // dança
          RCLCPP_INFO(this->get_logger(), "Dance");
          parameters = false;
          section = "Dance";
          break;
        case 26: // chute direito angulado
          RCLCPP_INFO(this->get_logger(), "Left Leg Right Kick");
          parameters = false;
          section = "Left Leg Right Kick";
          break;  
        case 27: // chute esquerdo angulado
          RCLCPP_INFO(this->get_logger(), "Left Leg Left Kick");
          parameters = false;
          section = "Left Leg Left Kick";
          break;
        case 28: // Andar pra trás
          RCLCPP_INFO(this->get_logger(), "Walking Backward");
          parameters = true;
          parameter_number = 9;
          break; 
        case 29: // goleiro penalty
          RCLCPP_INFO(this->get_logger(), "Goalkeeper Penalty");
          parameters = false;
          section = "Goalkeeper Penalty";  
          break; 
        case 30: 
          RCLCPP_INFO(this->get_logger(), "Feather Falling Right");
          parameters = false;
          section = "Feather Falling Right";
          break; 
        case 31: 
          RCLCPP_INFO(this->get_logger(), "Goodbye Loop");
          parameters = false;
          section = "Goodbye Loop";
          break; 
        case 32:
          RCLCPP_INFO(this->get_logger(), "Cheering Up");
          parameters = false;
          section = "Cheering Up";
          break; 
        case 33: // quando o robo ta caido de lado
          RCLCPP_INFO(this->get_logger(), "Fallen Side Left");
          parameters = false;
          section = "Fallen Side Left";
          break; 
        case 34: // quando o robo ta caido de lado
          RCLCPP_INFO(this->get_logger(), "Fallen Side Right");
          parameters = false;
          section = "Fallen Side Right";
          break;
        case 35:
          RCLCPP_INFO(this->get_logger(), "Goalkeeper Searching Ball");
          parameters = false;
          section = "Goalkeeper Searching Ball";
          break;
        case 36: 
          RCLCPP_INFO(this->get_logger(), "Feather Falling Left");
          parameters = false;
          section = "Feather Falling Left";
          break; 
        case 37: 
          RCLCPP_INFO(this->get_logger(), "Quick Left Kick");
          parameters = false;
          section = "Quick Left Kick";
          break;
        case 38: 
          RCLCPP_INFO(this->get_logger(), "Quick Right Kick");
          parameters = false;
          section = "Quick Right Kick";
          break;
      }
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

        if (movement != last_movement && (movement == 5 || movement == 6 || movement == 14)) {
          do_gait = true;
        }

        last_movement = movement;

        if (do_gait){
          movement = 15;
          contador++;
          if (contador >= 5){
            do_gait = false;
            contador = 0;
          }
          RCLCPP_INFO(this->get_logger(), "Gait trocando movimentacao %d", contador);
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
            message.id = motors;

            number_of_mov = j[section]["number of movements"];
            
            for (int i = 1; i <= number_of_mov; i++){
              
              message_walk.walk_number = 0; 
              publisher_walk->publish(message_walk);
              address_name = "address";
              id_name = "id";
              vel_name = "velocity";
              position_name = "position";
              sleep_name = "sleep";
              
              RCLCPP_INFO(this->get_logger(), " i: %d ",  i);
              if (goal_handle->is_canceling()) {
                  result->finished = false;
                  goal_handle->canceled(result);
                  RCLCPP_INFO(this->get_logger(), "Goal canceled");
                  return;
              }

              address_name = address_name + std::to_string(i);
              if (j[section][address_name] == 112){
                RCLCPP_INFO(this->get_logger(), "VELOCIDADE");
                id_name = id_name + std::to_string(i);
                message_single.id = j[section][id_name];
                vel_name = vel_name + std::to_string(i);
                message_single.position = j[section][vel_name];
                message_single.address = j[section][address_name];
                publisher_single->publish(message_single);
                usleep(500000);
              }
              else if (j[section][address_name] == 116){
                position_name = position_name + std::to_string(i);
                position.push_back(j[section][position_name]);
                if (movement == 21 or movement == 22 or movement == 23 or movement == 24){
                  position[0][18] = neck_sides;
                  position[0][19] = neck_up;
                }
                message.position = position.front();

                RCLCPP_INFO(this->get_logger(), "POSIÇÃO %d %d", position[0][18], position[0][19]);
                publisher_->publish(message);
                sleep_name = sleep_name + std::to_string(i);
                sleep_sec = j[section][sleep_name];
                RCLCPP_INFO(this->get_logger(), "Sleep: %f ", sleep_sec);
                usleep(sleep_sec*1000000);
                position.clear();
              }
            
              movements_remaining = number_of_mov - i;
              // Publish feedback
              goal_handle->publish_feedback(feedback);
              RCLCPP_INFO(this->get_logger(), "Feedback: %d movements remaining", movements_remaining);
            }
          }
        }
      
      result->finished = true;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }

    rclcpp::Subscription<custom_interfaces::msg::NeckPosition>::SharedPtr subscription_neck;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscriber_fase_zero;
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
