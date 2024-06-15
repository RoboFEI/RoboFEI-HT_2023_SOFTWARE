#pragma once

#include <QWidget>
#include <QMainWindow>
#include <QVector>
#include <QLineEdit>
#include <QLabel>
#include <QShortcut>
#include <QPushButton>
#include <string>
#include <filesystem>

#include "json.hpp"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int8.hpp>

// #include "custom_interfaces/msg/set_position_original.hpp"
#include "custom_interfaces/msg/joint_state.hpp"
#include "custom_interfaces/msg/humanoid_league_msgs.hpp"

using std::placeholders::_1;

QT_BEGIN_NAMESPACE
namespace Ui
{
class MainWindow;
}
QT_END_NAMESPACE

class MainWindow : public QMainWindow, public rclcpp::Node
{
  Q_OBJECT

public:
  using JointStateMsg     = custom_interfaces::msg::JointState;
  using GameControllerMsg = custom_interfaces::msg::HumanoidLeagueMsgs;

  int robot_number_;

  Json motions;
  
  void jointPositionCallback(const JointStateMsg::SharedPtr all_joints_position);

  explicit MainWindow(
    const rclcpp::NodeOptions & node_options,
    QWidget * parent = nullptr);
  ~MainWindow() override;

private:
  Ui::MainWindow * ui_;

  int mode = 0; //0 = Position mode    1 = Velocity mode
  int atualStep = 0;

  QVector<QLabel*> allPosLabel;
  QVector<QLineEdit*> allPosLineEdit;
  QVector<QPushButton*> gameStateButtons;
  std::vector<int> lastVelocitys = std::vector<int>(20, 32);
  std::vector<int> lastPositions = std::vector<int>(20, 2048);
  std::vector<std::vector<std::vector<int>>> atualMovesList;

  
  
  rclcpp::Publisher<JointStateMsg>::SharedPtr joint_state_publisher_;
  rclcpp::Publisher<GameControllerMsg>::SharedPtr fakeGameControlerPublisher_;
  rclcpp::Subscription<JointStateMsg>::SharedPtr position_subscriber_;

  rclcpp::TimerBase::SharedPtr timer_;

  const std::string prefix_;

  void torque_checkbox_changed();
  void gameStateVal();
  void send_torque_info(int id, int torque);
  void sendGameControllerInfo();
  void publishJointStates();
  void getAllPositions();
  void sendSingleInfo();
  void printPos();
  void setMotionEditorScreen(bool arg);

private slots:
  void on_loadMoves_button_released();
  void on_movesList_currentTextChanged(const QString &arg1);
  void on_pos_button_clicked();
  void on_vel_button_clicked();


};