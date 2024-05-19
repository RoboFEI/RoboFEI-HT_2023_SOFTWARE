#include "new_motors_pkg/motors_communication.hpp"

// Default setting
#define BAUDRATE 1000000  // Default Baudrate of DYNAMIXEL X series
#define DEVICE_NAME "/dev/motors"  // [Linux]: "/dev/ttyUSB", [Windows]: "COM*"

PortHandler *portHandler = PortHandler::getPortHandler(DEVICE_NAME);

MotorsCommunication::MotorsCommunication()
: Node("read_write_node")
{
    RCLCPP_INFO(this->get_logger(), "Run read write node");

    timer_ = this->create_wall_timer(
    8ms, std::bind(&MotorsCommunication::timer_callback, this));
}

MotorsCommunication::~MotorsCommunication()
{
}

void MotorsCommunication::timer_callback()
{
  RCLCPP_INFO_ONCE(this->get_logger(), "node fine");
}

bool openSerialPort()
{
    dxl_comm_result = portHandler->openPort();
    if (dxl_comm_result == false) {
        RCLCPP_ERROR(rclcpp::get_logger("read_write_node"), "Failed to open the port!");
        return false;
    } else {
        RCLCPP_INFO(rclcpp::get_logger("read_write_node"), "Succeeded to open the port.");
    }

    // Set the baudrate of the serial port (use DYNAMIXEL Baudrate)
    dxl_comm_result = portHandler->setBaudRate(BAUDRATE);
    if (dxl_comm_result == false) {
        RCLCPP_ERROR(rclcpp::get_logger("read_write_node"), "Failed to set the baudrate!");
        return false;
    } else {
        RCLCPP_INFO(rclcpp::get_logger("read_write_node"), "Succeeded to set the baudrate.");
    }

    return true;
}

int main(int argc, char * argv[])
{ 
    char string1[50]; //String
    sprintf(string1,"echo 123456| sudo -S renice -20 -p %d", getpid());
    system(string1);//prioridade

    if(!openSerialPort()) return -1;

    rclcpp::init(argc, argv);

    auto motorscomm = std::make_shared<MotorsCommunication>();
    rclcpp::spin(motorscomm);
    rclcpp::shutdown();

    return 0;
}