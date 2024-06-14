// Copyright 2022 HarvestX Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <QApplication>
#include "main_window.hpp"
#include "ui_main_window.h"

std::string folder_path = fs::current_path();

MainWindow::MainWindow(
  const rclcpp::NodeOptions & node_options,
  QWidget * parent
)
: QMainWindow(parent),
  rclcpp::Node("robot_gui", node_options),
  ui_(new Ui::MainWindow),
  prefix_(this->declare_parameter("prefix", ""))
{
  robot_number_ = this->declare_parameter("robot_number", 2);

  motions.openJson(folder_path + "/src/control/Data/motion" + std::to_string(robot_number_) + ".json");
  motions.printJson(); 
  using namespace std::chrono_literals;
 
  this->timer_ = this->create_wall_timer(
    100ms, std::bind(&MainWindow::publishJointStates, this));

  joint_state_publisher_      = this->create_publisher<JointStateMsg>("set_joint_topic", 10);
  fakeGameControlerPublisher_ = this->create_publisher<GameControllerMsg>("gamestate", 10);

  position_subscriber_ = this->create_subscription<JointStateMsg>(
    "all_joints_position", 10,
    std::bind(&MainWindow::jointPositionCallback, this, _1) 
  );

  this->ui_->setupUi(this);

  allPosLabel << ui_->label_1  << ui_->label_2  << ui_->label_3  << ui_->label_4  << ui_->label_5 ;
  allPosLabel << ui_->label_6  << ui_->label_7  << ui_->label_8  << ui_->label_9  << ui_->label_10;
  allPosLabel << ui_->label_11 << ui_->label_12 << ui_->label_13 << ui_->label_14 << ui_->label_15;
  allPosLabel << ui_->label_16 << ui_->label_17 << ui_->label_18 << ui_->label_19 << ui_->label_20;

  allPosLineEdit << ui_->pos_id_1  << ui_->pos_id_2  << ui_->pos_id_3  << ui_->pos_id_4  << ui_->pos_id_5;
  allPosLineEdit << ui_->pos_id_6  << ui_->pos_id_7  << ui_->pos_id_8  << ui_->pos_id_9  << ui_->pos_id_10;
  allPosLineEdit << ui_->pos_id_11 << ui_->pos_id_12 << ui_->pos_id_13 << ui_->pos_id_14 << ui_->pos_id_15;
  allPosLineEdit << ui_->pos_id_16 << ui_->pos_id_17 << ui_->pos_id_18 << ui_->pos_id_19 << ui_->pos_id_20;

  gameStateButtons << ui_->initial_button << ui_->playing_button << ui_->ready_button << ui_->penalized_button;

  for(auto button : gameStateButtons)
  {
    this->connect( button, &QPushButton::clicked, this, &MainWindow::gameStateVal);
  }

  this->connect( ui_->normal_game_button, &QPushButton::clicked, this, &MainWindow::gameStateVal);
  this->connect( ui_->penalty_button    , &QPushButton::clicked, this, &MainWindow::gameStateVal);
  

  QShortcut *sC1 = new QShortcut(QKeySequence("Ctrl+Space"), this);
  this->connect(sC1, &QShortcut::activated, this, &MainWindow::getAllPositions);

  QShortcut *sC2 = new QShortcut(QKeySequence("Ctrl+Return"), this);
  this->connect(sC2, &QShortcut::activated, this, &MainWindow::printPos);

  for(auto PosLineEdit : allPosLineEdit)
  {
    this->connect(PosLineEdit, &QLineEdit::returnPressed, this, &MainWindow::sendSinglePos);
  }


  for (auto checkBox : findChildren<QCheckBox *>()) {
    this->connect(
      checkBox, &QCheckBox::stateChanged,
      this, &MainWindow::torque_checkbox_changed);
  }

}

MainWindow::~MainWindow()
{
  delete this->ui_;
}

void MainWindow::gameStateVal()
{
  QPushButton* pushBottonCaller = qobject_cast<QPushButton*>(sender());

  if(!pushBottonCaller->isChecked() && pushBottonCaller != ui_->penalized_button) pushBottonCaller->setChecked(true);
  else if(pushBottonCaller == ui_->penalized_button);
  else if(pushBottonCaller == ui_->normal_game_button)  ui_->penalty_button->setChecked(false);
  else if(pushBottonCaller == ui_->penalty_button)      ui_->normal_game_button->setChecked(false);
  else
  {
    for(auto button : gameStateButtons)
    {
      if(pushBottonCaller != button && button != ui_->penalized_button) button->setChecked(false);
    }
  }

  sendGameControllerInfo();

}

void MainWindow::sendGameControllerInfo()
{
  auto gameControllerInfo = GameControllerMsg();

  if(ui_->initial_button->isChecked()) gameControllerInfo.game_state = GameControllerMsg::GAMESTATE_INITAL;
  else if(ui_->ready_button->isChecked()) gameControllerInfo.game_state = GameControllerMsg::GAMESTATE_READY;
  
  if(ui_->penalized_button->isChecked()) gameControllerInfo.penalized = true;
  else gameControllerInfo.penalized = false;

  if(ui_->penalty_button->isChecked()) gameControllerInfo.secondary_state = GameControllerMsg::STATE_PENALTYSHOOT;

  fakeGameControlerPublisher_->publish(gameControllerInfo);

}

void MainWindow::sendSinglePos()
{
  QLineEdit* posLineEdit = qobject_cast<QLineEdit*>(sender());
  int id = posLineEdit->objectName().remove("pos_id_").toInt();
  RCLCPP_INFO(this->get_logger(), "Sending for id %d", id);

  auto pos = JointStateMsg();
  pos.id.push_back(id);
  pos.type.push_back(JointStateMsg::POSITION);
  pos.info.push_back(posLineEdit->text().toInt());

  joint_state_publisher_->publish(pos);

}

void MainWindow::printPos()
{  
  QString saida = "[";
 
  for(int i=0; i<20; i++)
  {
    saida.append(allPosLineEdit[i]->text());
    saida.append(",");
  }
  saida.chop(1);
  saida.append("],");

  RCLCPP_INFO(this->get_logger(), saida.toUtf8().constData());

  // ui_->label_saida->setText(saida);
}

void MainWindow::getAllPositions()
{
  for(int i=0; i<20; i++)
  {
    allPosLineEdit[i]->setText(allPosLabel[i]->text());
  }
  RCLCPP_INFO(this->get_logger(), "Teste");
}

void MainWindow::jointPositionCallback(const JointStateMsg::SharedPtr all_joints_position)
{
  for(int i=0; i<20; i++)
  { 
    allPosLabel[i]->setText(QString("%1").arg(all_joints_position->info[i+1]));
  }
}

void MainWindow::torque_checkbox_changed()
{
  QCheckBox* torque_checkbox = qobject_cast<QCheckBox*>(sender());
  int id = torque_checkbox->objectName().remove("torque_id_").toInt();

  send_torque_info(id, torque_checkbox->isChecked());  
}

void MainWindow::publishJointStates()
{
  // RCLCPP_INFO(this->get_logger(), "Teste");
}

void MainWindow::send_torque_info(int id, int torque)
{
  RCLCPP_INFO(this->get_logger(), "Torque %d | id: %d", torque, id);

  auto torque_info = JointStateMsg();
  torque_info.id.push_back(id);
  torque_info.info.push_back(torque);
  torque_info.type.push_back(JointStateMsg::TORQUE);

  joint_state_publisher_ ->publish(torque_info);
}
