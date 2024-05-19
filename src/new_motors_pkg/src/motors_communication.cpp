// $ ros2 topic pub -1 /set_joint_topic custom_interfaces/JointState "{id: {1, 2, 5, 6}, info: {2500, 1900, 2048, 2048}, type: {1, 2, 5, 6}}"


#include "new_motors_pkg/motors_communication.hpp"

#define PROTOCOL_VERSION 2

#if PROTOCOL_VERSION == 2
    #include "new_motors_pkg/protocol2.h"
#else
    #include "new_motors_pkg/protocol1.h"
#endif

// Default setting
#define BAUDRATE 1000000  // Default Baudrate of DYNAMIXEL X series
#define DEVICE_NAME "/dev/motors"  // [Linux]: "/dev/ttyUSB", [Windows]: "COM*"

PortHandler *portHandler = PortHandler::getPortHandler(DEVICE_NAME);

MotorsCommunication::MotorsCommunication()
: Node("read_write_node")
{
    packetHandler = PacketHandler::getPacketHandler(PROTOCOL_VERSION);
    groupSyncWritePos = new dynamixel::GroupSyncWrite(portHandler, packetHandler, ADDR_GOAL_POSITION, LEN_GOAL_POSITION);

    RCLCPP_INFO(this->get_logger(), "Run read write node");   

    joint_state_subscription_ = this->create_subscription<JointStateMsg>(
      "set_joint_topic", 10, std::bind(&MotorsCommunication::joint_state_callback, this, _1));     

    timer_ = this->create_wall_timer(
    8ms, std::bind(&MotorsCommunication::timer_callback, this));
}

MotorsCommunication::~MotorsCommunication()
{
}

void MotorsCommunication::joint_state_callback(const JointStateMsg::SharedPtr joint_state_info)
{
    setJoints(*joint_state_info);
}

void MotorsCommunication::timer_callback()
{
    for(int i=1; i<21; i++)
    {
        RCLCPP_INFO(this->get_logger(), "Position %d | %d",i, joints.position[i]);
    }

    RCLCPP_INFO(this->get_logger(), "-------------------");
}

void MotorsCommunication::setJoints(JointStateMsg jointInfo)
{
    for(int i=0; i<(int)jointInfo.id.size(); i++)
    {
        if(jointInfo.type[i] == JointStateMsg::POSITION) 
            joints.position[jointInfo.id[i]] = jointInfo.info[i];
        // else if(jointInfo.type[i] == JointStateMsg::VELOCITY) setVelocity()
    }
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

    // if(!openSerialPort()) return -1;

    rclcpp::init(argc, argv);

    auto motorscomm = std::make_shared<MotorsCommunication>();
    rclcpp::spin(motorscomm);
    rclcpp::shutdown();

    return 0;
}