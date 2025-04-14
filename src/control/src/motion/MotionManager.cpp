#include "MotionManager.h"
#include <filesystem>
#include "Action.h"
#include "Walking.h"

using namespace Robot;
using namespace std::chrono_literals;
using std::placeholders::_1;
namespace fs = std::filesystem;

// Torque adaption every second
const int TORQUE_ADAPTION_CYCLES = 1000 / MotionModule::TIME_UNIT;
const int DEST_TORQUE = 1023;

int position[20];
int last_movement = 0;
int walk=0;

// 
MotionManager::MotionManager()
: Node("gait_publisher"),
  // inicialização das variáveis
  m_ProcessEnable(false),
  m_IsRunning(false),
  m_IsThreadRunning(false),
  m_IsLogging(false),
  m_torqueAdaptionCounter(TORQUE_ADAPTION_CYCLES),
  m_voltageAdaptionFactor(1.0),

  DEBUG_PRINT(false)
  
{
  // subscribers e publishers
  subscription_imu = this->create_subscription<sensor_msgs::msg::Imu>(
      "imu/data", 10, std::bind(&MotionManager::topic_callback, this, _1));

  subscription_walk = this->create_subscription<custom_interfaces::msg::Walk>(
      "walking", 10, std::bind(&MotionManager::topic_callback_walk, this, _1));

  pubisher_body_joints_ = this->create_publisher<JointStateMsg>("set_joint_topic", 10);
  publisher_fase_zero = this->create_publisher<std_msgs::msg::Bool>("/fase_zero", 10);

  timer_ = this->create_wall_timer(4ms, std::bind(&MotionManager::Process, this));
  
  keep_walking = false;
  
  for (int i = 0; i < JointData::NUMBER_OF_JOINTS; i++)
  
  m_Offset[i] = 0;

  robot_number_ = this->declare_parameter("robot_number", 2);

  std::string iniPath = "src/control/Data/config" + std::to_string(robot_number_) + ".ini";
  ini = new minIni(iniPath);

}

MotionManager::~MotionManager() {}

void MotionManager::update_loop()
{
  while (rclcpp::ok())
  {
    RCLCPP_INFO(this->get_logger(), "Running motion manager");
    this->Process();
  }
}

void MotionManager::topic_callback(const std::shared_ptr<sensor_msgs::msg::Imu> imu_msg_) const
{
  float IMU_GYRO_X = -imu_msg_->angular_velocity.x / 10;
  float IMU_GYRO_Y = -imu_msg_->angular_velocity.y / 10;
  // valores não armazenados ainda — precisa de membro? avaliar uso
}
void MotionManager::GetIniParameter()
{
  if (Walking::GetInstance()->GetCurrentPhase() == 0 || Walking::GetInstance()->GetCurrentPhase() == 2)
  {
    if (walk != last_movement){
			if (walk == 1){
				printf("CALLBACK WALK\n");
				Walking::GetInstance()->LoadINISettings(ini, "Walking Config");
			}
			else if (walk == 2){
				printf("CALLBACK GAIT\n");
				Walking::GetInstance()->LoadINISettings(ini, "Gait");
			}
			else if (walk == 3){
				printf("CALLBACK TURN\n");
				Walking::GetInstance()->LoadINISettings(ini, "Turn Robot");
			}
			else if (walk == 4){
				printf("CALLBACK WALK SLOW\n");
				Walking::GetInstance()->LoadINISettings(ini, "Walk Slow");
			}
			else if (walk == 5){
				printf("CALLBACK TURN BALL RIGHT\n");
				Walking::GetInstance()->LoadINISettings(ini, "Turn Ball Right");
			}
			else if (walk == 6){
				printf("CALLBACK TURN BALL LEFT\n");
				Walking::GetInstance()->LoadINISettings(ini, "Turn Ball Left");
			}
			else if (walk == 7){
				printf("CALLBACK SIDLE RIGHT\n");
				Walking::GetInstance()->LoadINISettings(ini, "Sidle Right");
			}
			else if (walk == 8){
				printf("CALLBACK SIDLE LEFT\n");
				Walking::GetInstance()->LoadINISettings(ini, "Sidle Left");
			}
			else if (walk == 9){
				printf("CALLBACK WALK BACKWARD\n");
				Walking::GetInstance()->LoadINISettings(ini, "Walking Backward");
			}
			else if (walk == 10){
				printf("CALLBACK WALK BACKWARD SLOW\n");
				Walking::GetInstance()->LoadINISettings(ini, "Walking Backward Slow");
			}
			else if (walk == 11){
				printf("CALLBACK TURN ROBOT RIGHT\n");
				Walking::GetInstance()->LoadINISettings(ini, "Turn Robot Right");
			}
			else if (walk == 12){
				printf("CALLBACK TURN ROBOT LEFT\n");
				Walking::GetInstance()->LoadINISettings(ini, "Turn Robot Left");
			}
			else if (walk == 13){
				printf("CALLBACK TURN ROBOT LEFT\n");
				Walking::GetInstance()->LoadINISettings(ini, "Turn Robot Left slow");
			}

      last_movement = walk;
			this->LoadINISettings(ini);
			Action::GetInstance()->Stop();

      this->Initialize();
			Walking::GetInstance()->m_Joint.SetEnableBody(true);
			Action::GetInstance()->m_Joint.SetEnableBody(false);
			MotionStatus::m_CurrentJoints.SetEnableBodyWithoutHead(true);
			this->SetEnable(true);
			Walking::GetInstance()->Start();
			this->keep_walking=true;
    }
  }
}


void MotionManager::topic_callback_walk(const std::shared_ptr<custom_interfaces::msg::Walk> walk_msg_) const
{
  auto message_fase = std_msgs::msg::Bool();
  message_fase.data = false;
walk = walk_msg_->walk_number;

  if (walk != 0)
  {
    message_fase.data = true;
    const_cast<MotionManager*>(this)->GetIniParameter();
  }
  else
  {
    if (!keep_walking && Walking::GetInstance()->GetCurrentPhase() == 0)
    {
      message_fase.data = true;
      Action::GetInstance()->Stop();
      const_cast<MotionManager*>(this)->AddModule((MotionModule*)Walking::GetInstance());
      Walking::GetInstance()->m_Joint.SetEnableBody(false);
      Action::GetInstance()->m_Joint.SetEnableBody(false);
      MotionStatus::m_CurrentJoints.SetEnableBodyWithoutHead(true);
      const_cast<MotionManager*>(this)->SetEnable(true);
      Walking::GetInstance()->Stop();
      const_cast<MotionManager*>(this)->keep_walking = true;
    }
  }

  publisher_fase_zero->publish(message_fase);
}

bool MotionManager::Initialize(bool fadeIn)
{
  int value, error;

	usleep(10000);
  m_Enabled = false;
  m_ProcessEnable = true;

  for (int id = JointData::ID_MIN; id <= JointData::ID_MAX - 2; id++) //diminui tirando a cabeça
  {
    if (DEBUG_PRINT)
      fprintf(stderr, "ID:%d initializing...", id);
      MotionStatus::m_CurrentJoints.SetValue(id, position[id]);
      std::cout << position[id] << std::endl;
  }

  m_fadeIn = fadeIn;
  m_torque_count = 0;
  m_CalibrationStatus = 0;
  m_FBGyroCenter = 512;
  m_RLGyroCenter = 512;

  return true;
}

bool MotionManager::Reinitialize()
{
  m_ProcessEnable = false;
  int value, error;
  for (int id = JointData::ID_MIN; id <= JointData::ID_MAX - 2; id++)
  {
    if (DEBUG_PRINT)
      fprintf(stderr, "ID:%d initializing...", id);
    else
      MotionStatus::m_CurrentJoints.SetEnable(id, false);
  }

  m_ProcessEnable = true;
  return true;
}

void MotionManager::Restartrobot()
{
  m_torque_count = 0;
}

void MotionManager::StopLogging()
{
  m_IsLogging = false;
  m_LogFileStream.close();
}

void MotionManager::StartLogging()
{
  char szFile[32] = {0,};
  int count = 0;
  while (true)
  {
    sprintf(szFile, "Logs/Log%d.csv", count);
    if (0 != access(szFile, F_OK))
      break;
    count++;
    if (count > 256)
      return;
  }

  m_LogFileStream.open(szFile, std::ios::out);
  for (int id = JointData::ID_MIN; id <= JointData::ID_MAX - 2; id++)
    m_LogFileStream << "nID_" << id << "_GP,nID_" << id << "_PP,";
    m_LogFileStream << "GyroFB,GyroRL,AccelFB,AccelRL,L_FSR_X,L_FSR_Y,R_FSR_X,R_FSR_Y," << "\x0d\x0a";

    m_IsLogging = true;
}

void MotionManager::LoadINISettings(minIni *ini)
{
  LoadINISettings(ini, OFFSET_SECTION);
}

void MotionManager::LoadINISettings(minIni *ini, const std::string &section)
{
  int ivalue = INVALID_VALUE;
  for (int i = JointData::ID_MIN; i <= JointData::ID_MAX - 2; i++)
  {
    char key[10];
    sprintf(key, "ID_%.2d", i);
    if ((ivalue = ini->geti(section, key, INVALID_VALUE)) != INVALID_VALUE) 
      m_Offset[i] = ivalue;
  }
  m_angleEstimator.LoadINISettings(ini, section + "_angle");
}

void MotionManager::SaveINISettings(minIni *ini)
{
  SaveINISettings(ini, OFFSET_SECTION);
}

void MotionManager::SaveINISettings(minIni *ini, const std::string &section)
{
  for (int i = JointData::ID_MIN; i <= JointData::ID_MAX - 2; i++)
  {
    char key[10];
    sprintf(key, "ID_%.2d", i);
    ini->put(section, key, m_Offset[i]);
  }
  m_angleEstimator.SaveINISettings(ini, section + "_angle");
}

#define GYRO_WINDOW_SIZE    100
#define ACCEL_WINDOW_SIZE   30
#define MARGIN_OF_SD        2.0

void MotionManager::Process()
{
    // RCLCPP_INFO(this->get_logger(), "FASE PROCESS %d", Walking::GetInstance()->GetCurrentPhase());
    if (Walking::GetInstance()->GetCurrentPhase() == 0 || Walking::GetInstance()->GetCurrentPhase() == 2) {
        auto message_fase = std_msgs::msg::Bool();
        message_fase.data = true;
        publisher_fase_zero->publish(message_fase);
    }

    if (walk != 0 || (walk == 0 && (Walking::GetInstance()->GetCurrentPhase() != 0 || Walking::GetInstance()->GetCurrentPhase() == 2))) {
        if (Walking::GetInstance()->m_Joint.GetEnable(5)) {
            Walking::GetInstance()->Process();
        }

        if (walk != last_movement)
            this->GetIniParameter(); 

        if (m_fadeIn && m_torque_count < DEST_TORQUE) {
            if (m_torque_count < 100)
                m_torque_count += 3;
            else
                m_torque_count = 2047;
        }

        if (!this->m_ProcessEnable || this->m_IsRunning)
            return;

        this->m_IsRunning = true;
        m_CalibrationStatus = 1;

        if (m_CalibrationStatus == 1 && this->GetEnable()) {
            const double GYRO_ALPHA = 0.1;
            int gyroValFB = (int) (IMU_GYRO_Y);
            int gyroValRL = (int) (IMU_GYRO_X);

            MotionStatus::FB_GYRO = (1.0 - GYRO_ALPHA) * MotionStatus::FB_GYRO + GYRO_ALPHA * gyroValFB;
            MotionStatus::RL_GYRO = (1.0 - GYRO_ALPHA) * MotionStatus::RL_GYRO + GYRO_ALPHA * gyroValRL;

            for (int id = JointData::ID_MIN; id <= JointData::ID_MAX - 2; id++) {
                if (Walking::GetInstance()->m_Joint.GetEnable(id)) {
                  MotionStatus::m_CurrentJoints.SetValue(id, (Walking::GetInstance())->m_Joint.GetValue(id));
                  MotionStatus::m_CurrentJoints.SetPGain(id, (Walking::GetInstance())->m_Joint.GetPGain(id));
                  MotionStatus::m_CurrentJoints.SetIGain(id, (Walking::GetInstance())->m_Joint.GetIGain(id));
                  MotionStatus::m_CurrentJoints.SetDGain(id, (Walking::GetInstance())->m_Joint.GetDGain(id));
                }
            }

            int param[JointData::NUMBER_OF_JOINTS * MX28::PARAM_BYTES];
            int joint_num = 0;
            int pos[19];

            for (int id = JointData::ID_MIN; id <= JointData::ID_MAX - 2; id++) 
            {
                param[id] = id;
                pos[id] = MotionStatus::m_CurrentJoints.GetValue(id) + m_Offset[id];

                if (DEBUG_PRINT)
                    fprintf(stderr, "ID[%d] : %d \n", id, MotionStatus::m_CurrentJoints.GetValue(id));
            }

            auto setJointInfoMsg = JointStateMsg();
            setJointInfoMsg.id = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18};
            setJointInfoMsg.info = {
                pos[1], pos[2], pos[3], pos[4], pos[5], pos[6], pos[7], pos[8], pos[9],
                pos[10], pos[11], pos[12], pos[13], pos[14], pos[15], pos[16], pos[17], pos[18]
            };
            setJointInfoMsg.type = std::vector<std::uint8_t>(18, 0);
            pubisher_body_joints_->publish(setJointInfoMsg);
        } else {
            RCLCPP_INFO(this->get_logger(), "nao entrou no segundo if");
        }

        m_IsRunning = false;
    }
}



void MotionManager::SetEnable(bool enable)
{
  m_Enabled = enable;
}

void MotionManager::AddModule(MotionModule *module)
{
  module->Initialize();
  m_Modules.push_back(module);
}

void MotionManager::RemoveModule(MotionModule *module)
{
  m_Modules.remove(module);
}

void MotionManager::SetJointDisable(int index)
{
  if (m_Modules.size() != 0)
  {
    for (auto &module : m_Modules)
    {
      for(std::list<MotionModule*>::iterator i = m_Modules.begin(); i != m_Modules.end(); i++)
      (*i)->m_Joint.SetEnable(index, false);
    }
  }
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MotionManager>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
