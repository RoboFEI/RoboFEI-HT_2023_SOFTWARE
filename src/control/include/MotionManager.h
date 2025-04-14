/*
 *   MotionManager.h
 *
 *   Author: ROBOTIS
 *
 */

 #ifndef _MOTION_MANGER_H_
 #define _MOTION_MANGER_H_
 
 #include <list>
 #include <thread>
 #include <fstream>
 #include <iostream>
 #include "MotionStatus.h"
 #include "MotionModule.h"
 #include "MX28.h"
 #include "minIni.h"
 #include "AngleEstimator.h"
 #include "sensor_msgs/msg/imu.hpp"
 #include "rclcpp/rclcpp.hpp"
 #include "Action.h"
 #include "Walking.h"
 
 #include "custom_interfaces/msg/joint_state.hpp"
 #include "custom_interfaces/srv/get_position.hpp"
 #include "custom_interfaces/msg/walk.hpp"
 #include "custom_interfaces/msg/neck_position.hpp"
 #include "std_msgs/msg/bool.hpp"
 
 #define OFFSET_SECTION "Offset"
 #define INVALID_VALUE   -1024.0
 
 namespace Robot
 {
	 class MotionManager : public rclcpp::Node
	 {
	 private:
		 using JointStateMsg = custom_interfaces::msg::JointState;
		 std::list<MotionModule*> m_Modules;
		 static bool start;
		 bool m_ProcessEnable;
		 bool m_Enabled;
		 int m_FBGyroCenter;
		 int m_RLGyroCenter;
		 int m_CalibrationStatus;
 
		 bool m_IsRunning;
		 bool m_IsThreadRunning;
		 bool m_IsLogging;
		 float IMU_GYRO_X;
		 float IMU_GYRO_Y;
		 int robot_number_;
 
		 rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_imu;
		 rclcpp::Subscription<custom_interfaces::msg::Walk>::SharedPtr subscription_walk;
		 rclcpp::Publisher<JointStateMsg>::SharedPtr pubisher_body_joints_;
		 rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_fase_zero; 
 
		 rclcpp::Client<custom_interfaces::srv::GetPosition>::SharedPtr client;
		
		 static JointData m_CurrentJoints;
		 
		 rclcpp::TimerBase::SharedPtr timer_;
		 std::ofstream m_LogFileStream;
 
		 AngleEstimator m_angleEstimator;
		 bool m_fadeIn;
		 int m_torque_count;
 
		 FILE* m_voltageLog;
 
		 void topic_callback(const std::shared_ptr<sensor_msgs::msg::Imu> imu_msg_) const;
		 void topic_callback_walk(const std::shared_ptr<custom_interfaces::msg::Walk> walk_msg_) const;
		 void topic_callback_positions(const std::shared_ptr<JointStateMsg> position_msg_) const;
		 unsigned int m_torqueAdaptionCounter;
		 double m_voltageAdaptionFactor;
		 std::thread update_thread_;
		 void update_loop(void);
 
		 void GetIniParameter();
 
	 public:
		 bool DEBUG_PRINT;
		 int m_Offset[JointData::NUMBER_OF_JOINTS];
		 int *memBB;
 
		 minIni* ini;
		  bool keep_walking;
 
		 ~MotionManager();
		 MotionManager();
 
		 bool Initialize(bool fadeIn = true);
		 bool Reinitialize();
		 void Restartrobot();
		 void Process();
		 void SetEnable(bool enable);
		 bool GetEnable()				{ return m_Enabled; }
		 void AddModule(MotionModule *module);
		 void RemoveModule(MotionModule *module);
 
		 void ResetGyroCalibration() { m_CalibrationStatus = 0; m_FBGyroCenter = 512; m_RLGyroCenter = 512; }
		 int GetCalibrationStatus() { return m_CalibrationStatus; }
		 void SetJointDisable(int index);
 
		 void StartLogging();
		 void StopLogging();
 
		 void LoadINISettings(minIni* ini);
		 void LoadINISettings(minIni* ini, const std::string &section);
		 void SaveINISettings(minIni* ini);
		 void SaveINISettings(minIni* ini, const std::string &section);
 
		 inline AngleEstimator* angleEstimator()		{ return &m_angleEstimator; }
 };
 }
 
 #endif