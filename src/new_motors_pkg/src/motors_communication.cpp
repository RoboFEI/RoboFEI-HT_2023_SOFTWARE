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

int dxl_comm_result = COMM_TX_FAIL;

PortHandler *portHandler = PortHandler::getPortHandler(DEVICE_NAME);

MotorsCommunication::MotorsCommunication()
: Node("read_write_node")
{
    packetHandler = PacketHandler::getPacketHandler(PROTOCOL_VERSION);
    groupSyncWritePos = new dynamixel::GroupSyncWrite(portHandler, packetHandler, ADDR_GOAL_POSITION, LEN_GOAL_POSITION);
    groupSyncReadPos  = new dynamixel::GroupSyncRead(portHandler, packetHandler, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);
    
    initialMotorsSetup(BROADCAST_ID);

    RCLCPP_INFO(this->get_logger(), "Run read write node"); 

    this->declare_parameter("qos_depth", 10);
    int8_t qos_depth = 0;
    this->get_parameter("qos_depth", qos_depth);
    

    const auto QOS_RKL10V =
        rclcpp::QoS(rclcpp::KeepLast(qos_depth)).reliable().durability_volatile();

    joint_state_subscription_ = this->create_subscription<JointStateMsg>(
      "set_joint_topic", QOS_RKL10V, std::bind(&MotorsCommunication::joint_state_callback, this, _1));     

    timer_ = this->create_wall_timer(
    8ms, std::bind(&MotorsCommunication::timer_callback, this));
}

MotorsCommunication::~MotorsCommunication()
{
}

void MotorsCommunication::initialMotorsSetup(int id)
{
    if(PROTOCOL_VERSION == 2)
    {
        uint8_t dxl_error = 0;
        
        // Use Position Control Mode
        dxl_comm_result = packetHandler->write1ByteTxRx(
        portHandler,
        id,
        11,
        3,
        &dxl_error
        );

        if (dxl_comm_result != COMM_SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "Failed to set Position Control Mode.");
        } else {
            RCLCPP_INFO(this->get_logger(), "Succeeded to set Position Control Mode.");
        }
    }


    // Enable Torque of DYNAMIXEL
    setJointTorque(id, 1);
}

void MotorsCommunication::joint_state_callback(const JointStateMsg::SharedPtr joint_state_info)
{
    setJoints(*joint_state_info);
}

void MotorsCommunication::timer_callback()
{
    setAllJointPos();

    getNoTorquePos();

    for(int i=1; i<21; i++)
    {
        RCLCPP_INFO(this->get_logger(), "Torque %d | %d",i, joints.torque[i]);
    }

    RCLCPP_INFO(this->get_logger(), "-------------------");

        for(int i=1; i<21; i++)
    {
        RCLCPP_INFO(this->get_logger(), "Position %d | %d",i, joints.position[i]);
    }
}

void MotorsCommunication::getNoTorquePos()
{
    int dxl_addparam_result = COMM_TX_FAIL;
    std::vector<int> idPositionReaded;

    for(int i=1; i<21; i++)
    {
        if(joints.torque[i] == 0)
        {
            dxl_addparam_result = groupSyncReadPos->addParam((uint8_t)i);

            if (dxl_addparam_result != true)
            {
                RCLCPP_INFO(this->get_logger(), "[ID:%03d] groupSyncReadPos addparam failed", i);
            }else idPositionReaded.push_back(i);

        }
        
    }
    if(!idPositionReaded.empty())
        dxl_comm_result = groupSyncReadPos->txRxPacket();

    for(auto id : idPositionReaded)
    {
        joints.position[id] = groupSyncReadPos->getData(id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);
    }

    groupSyncReadPos->clearParam();


}

void MotorsCommunication::setAllJointPos()
{
    uint8_t dxl_error = 0;

    for(int i=1; i<21; i++)
    {
        groupSyncWritePos->addParam((uint8_t)i, convertInfo(joints.position[i]));
    }

    dxl_comm_result = groupSyncWritePos->txPacket();

    if (dxl_comm_result != COMM_SUCCESS) {
        RCLCPP_INFO(this->get_logger(), "%s", packetHandler->getTxRxResult(dxl_comm_result));
    } 
    else if (dxl_error != 0) {
        RCLCPP_INFO(this->get_logger(), "%s", packetHandler->getRxPacketError(dxl_error));
    } 
    else {
        //RCLCPP_INFO(this->get_logger(), "Set [ID: {%d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d,%d,%d,%d}] [Goal Position: {%d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d,%d, %d,%d}]", 
        //msg->id[0], msg->id[1], msg->id[2], msg->id[3], msg->id[4], msg->id[5], msg->id[6], msg->id[7], msg->id[8], msg->id[9], msg->id[10], msg->id[11], msg->id[12], msg->id[13], msg->id[14], msg->id[15], msg->id[16], msg->id[17], msg->id[18], msg->id[19],
        //msg->position[0], msg->position[1], msg->position[2], msg->position[3], msg->position[4], msg->position[5], msg->position[6], msg->position[7], msg->position[8], msg->position[9], msg->position[10], msg->position[11], msg->position[12], msg->position[13], 
        //msg->position[14], msg->position[15], msg->position[16], msg->position[17],msg->position[18], msg->position[19]);
    }

    groupSyncWritePos->clearParam();
}

void MotorsCommunication::setJoints(JointStateMsg jointInfo)
{
    for(int i=0; i<(int)jointInfo.id.size(); i++)
    {
        if(jointInfo.type[i] == JointStateMsg::POSITION) 
            joints.position[jointInfo.id[i]] = jointInfo.info[i];

        else if(jointInfo.type[i] == JointStateMsg::VELOCITY)
        {
            setJointVel(jointInfo.id[i], jointInfo.info[i]);
            if(jointInfo.id[i] == BROADCAST_ID)
            {
                joints.velocity = std::vector<std::uint32_t>(21, jointInfo.info[i]);
            } 
            else
            {
                joints.velocity[jointInfo.id[i]] = jointInfo.info[i];
            } 
        }

        else if(jointInfo.type[i] == JointStateMsg::TORQUE)
        {
            setJointTorque(jointInfo.id[i], jointInfo.info[i]);
        }
    }
}

void MotorsCommunication::setJointTorque(int id, int goalTorque)
{
    uint8_t dxl_error = 0;

    dxl_comm_result = packetHandler->write1ByteTxRx(
    portHandler,
    (uint8_t) id,
    ADDR_TORQUE_ENABLE,
    (uint8_t) goalTorque,
    &dxl_error
    );

    if (dxl_comm_result != COMM_SUCCESS) {
        RCLCPP_INFO(this->get_logger(), "%s", packetHandler->getTxRxResult(dxl_comm_result));
        return;
    } else if (dxl_error != 0) {
        RCLCPP_INFO(this->get_logger(), "%s", packetHandler->getRxPacketError(dxl_error));
        return;
    }

    if(id == BROADCAST_ID) joints.torque = std::vector<std::uint8_t>(21, goalTorque);
    else joints.torque[id] = goalTorque;
}

uint8_t *MotorsCommunication::convertInfo(int jointInfo)
{
    uint8_t *info = new uint8_t[LEN_GOAL_POSITION];

    if(LEN_GOAL_POSITION == 4)
    {
        info[0] = DXL_LOBYTE(DXL_LOWORD(jointInfo));
        info[1] = DXL_HIBYTE(DXL_LOWORD(jointInfo));
        info[2] = DXL_LOBYTE(DXL_HIWORD(jointInfo));
        info[3] = DXL_HIBYTE(DXL_HIWORD(jointInfo));
    } else
    {
        info[0] = DXL_LOBYTE(jointInfo);
        info[1] = DXL_HIBYTE(jointInfo);
    }

    return info;

}
void MotorsCommunication::setJointVel(int id, int goalVel)
{
    uint8_t dxl_error = 0;

    if(LEN_PROFILE_VELOCITY == 4)
        dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, (uint8_t) id, ADDR_PROFILE_VELOCITY, (uint32_t) goalVel, &dxl_error);
    
    else
        dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, (uint8_t) id, ADDR_PROFILE_VELOCITY, (uint32_t) goalVel, &dxl_error);


    if (dxl_comm_result != COMM_SUCCESS) {
        RCLCPP_INFO(this->get_logger(), "%s", packetHandler->getTxRxResult(dxl_comm_result));
    } else if (dxl_error != 0) {
        RCLCPP_INFO(this->get_logger(), "%s", packetHandler->getRxPacketError(dxl_error));
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
    sprintf(string1,"echo 123| sudo -S renice -20 -p %d", getpid());
    system(string1);//prioridade

    if(!openSerialPort()) return -1;

    rclcpp::init(argc, argv);

    auto motorscomm = std::make_shared<MotorsCommunication>();
    rclcpp::spin(motorscomm);
    rclcpp::shutdown();

    return 0;
}