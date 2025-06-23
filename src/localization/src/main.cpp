#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/imu.hpp"                  
#include "geometry_msgs/msg/vector3_stamped.hpp"    //rpy - imu

// LOCALIZAÇÃO PARA O MUNDIAL DE SALVADOR 2025

// definir aqui se quer q jogue com localização
#define LOCALIZATION_ACTIVE true

using namespace std::placeholders;

class LocalizationNode : public rclcpp::Node
{
public:
    LocalizationNode() : Node("localization_node")
    {   
        gyrosubscriber_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>("imu/rpy",10,
            std::bind(&LocalizationNode::listener_callback_imu_gyro, this, std::placeholders::_1));        //subscriber pra imu/rpy

        //inicializando publisher do tipo Bool para o topico 
        publisher_ = this->create_publisher<std_msgs::msg::Bool>("localization_active", 10);
        // inicalizando timer q publicara a msg a cada 0.5 seg 
        timer = this->create_wall_timer(std::chrono::milliseconds(500),
                                        std::bind(&LocalizationNode::LocalizationStatusPublisher, this));
        
        
        RCLCPP_INFO(this->get_logger(), "Localization package has started");
    }
   

private:

    void LocalizationStatusPublisher()
    {
        auto msg = std_msgs::msg::Bool();
        msg.data = LOCALIZATION_ACTIVE;
        publisher_->publish(msg);
    }

    void listener_callback_imu_gyro(const geometry_msgs::msg::Vector3Stamped rpy_msg)           //prints imu/rpy
    {
        RCLCPP_INFO(this->get_logger(), "X: %f", rpy_msg.vector.x);
        RCLCPP_INFO(this->get_logger(), "Y: %f", rpy_msg.vector.y);
        RCLCPP_INFO(this->get_logger(), "Z: %f", rpy_msg.vector.z);
        
    }

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_;  
    rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr gyrosubscriber_;        //declaração do subscriber
    rclcpp::TimerBase::SharedPtr timer;

    //RCLCPP_INFO(this->get_logger(), "Localization package has started");
    //RCLCPP_WARN(this->get_logger(), "imu %d",  robot_align_for_kick_left());
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LocalizationNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}