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
  allPosLabel << ui_->label_16 << ui_->label_17 << ui_->label_18;

  allPosLineEdit << ui_->pos_id_1  << ui_->pos_id_2  << ui_->pos_id_3  << ui_->pos_id_4  << ui_->pos_id_5;
  allPosLineEdit << ui_->pos_id_6  << ui_->pos_id_7  << ui_->pos_id_8  << ui_->pos_id_9  << ui_->pos_id_10;
  allPosLineEdit << ui_->pos_id_11 << ui_->pos_id_12 << ui_->pos_id_13 << ui_->pos_id_14 << ui_->pos_id_15;
  allPosLineEdit << ui_->pos_id_16 << ui_->pos_id_17 << ui_->pos_id_18;

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

  QShortcut *sC3 = new QShortcut(QKeySequence("Ctrl+S"), this);
  this->connect(sC3, &QShortcut::activated, this, &MainWindow::on_saveStep_button_clicked);

  for(auto PosLineEdit : allPosLineEdit)
  {
    this->connect(PosLineEdit, &QLineEdit::returnPressed, this, &MainWindow::sendSingleInfo);
  }


  for (auto checkBox : findChildren<QCheckBox *>()) {
    this->connect(
      checkBox, &QCheckBox::stateChanged,
      this, &MainWindow::torque_checkbox_changed);
  }

  checkUnsavedTimer = new QTimer(this);
  this->connect(checkUnsavedTimer, &QTimer::timeout, this, &MainWindow::checkUnsaved);
  checkUnsavedTimer->start(10);

  on_loadMoves_button_released();
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

void MainWindow::sendSingleInfo()
{
  QLineEdit* posLineEdit = qobject_cast<QLineEdit*>(sender());
  int id = posLineEdit->objectName().remove("pos_id_").toInt();
  RCLCPP_INFO(this->get_logger(), "Sending for id %d", id);

  auto pos = JointStateMsg();
  pos.id.push_back(id);
  
  if(mode == 0)
  {
    pos.type.push_back(JointStateMsg::POSITION);
  }
  else
  {
    pos.type.push_back(JointStateMsg::VELOCITY);
    lastVelocitys[id-1] = posLineEdit->text().toInt();
    allPosLabel[id-1]->setText(QString("%1").arg(lastVelocitys[id-1]));
  }
  
  pos.info.push_back(posLineEdit->text().toInt());

  joint_state_publisher_->publish(pos);

}

void MainWindow::printPos()
{  
  if(mode == 1) return;

  sendJointPos(getAllPositions());

  QString saida = "[";
 
  for(int i=0; i<18; i++)
  {
    saida.append(allPosLineEdit[i]->text());
    saida.append(",");
  }
  saida.chop(1);
  saida.append("],");

  RCLCPP_INFO(this->get_logger(), saida.toUtf8().constData());
}

std::vector<int> MainWindow::getAllPositions()
{
  std::vector<int> allPos;
  for(int i=0; i<18; i++)
  {
    allPosLineEdit[i]->setText(allPosLabel[i]->text());
    allPos.push_back(allPosLabel[i]->text().toInt());
  }

  return allPos;
}

void MainWindow::jointPositionCallback(const JointStateMsg::SharedPtr all_joints_position)
{
    for(int i=0; i<18; i++)
    {
      if(mode == 0) allPosLabel[i]->setText(QString("%1").arg(all_joints_position->info[i+1]));
      lastPositions[i] = all_joints_position->info[i+1];
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

void MainWindow::on_loadMoves_button_released()
{
  motions.openJson(folder_path + "/src/control/Data/motion" + std::to_string(robot_number_) + ".json");

  ui_->movesList->clear();  
  ui_->movesList->addItem("No Move");

  for(auto move : motions.getKeys())
  {
    ui_->movesList->addItem(QString::fromStdString(move));
  }
}

void MainWindow::on_pos_button_clicked()
{
  if(!ui_->pos_button->isChecked()) ui_->pos_button->setChecked(true);
  else 
  {
    ui_->vel_button->setChecked(false);
    mode = 0;

    for(int i=0; i<18; i++)
    { 
      allPosLabel[i]->setText(QString("%1").arg(lastPositions[i]));
    }
  }  
}

void MainWindow::on_vel_button_clicked()
{
  if(!ui_->vel_button->isChecked()) ui_->vel_button->setChecked(true);
  else 
  {
    ui_->pos_button->setChecked(false);
    mode = 1;
    for(int i=0; i<18; i++)
    { 
      allPosLabel[i]->setText(QString("%1").arg(lastVelocitys[i]));
    }
  }
}


void MainWindow::on_movesList_currentTextChanged(const QString &arg1)
{
  if(arg1.compare("No Move") && arg1.compare(""))
  {
    atualMovesList = motions.getMove(arg1.toStdString());
    setMotionEditorScreen(true);
  }
  else
  {
    setMotionEditorScreen(false);
  }
}

void MainWindow::setMotionEditorScreen(bool arg)
{
  if(arg)
  {
    atualStep = 1;
    ui_->motion_frame->setEnabled(true);
    ui_->statusbar->showMessage(QString("%1 of %2").arg(atualStep).arg(atualMovesList.size()));
    displayStepInfo();
  }
  else
  {
    atualStep = 0;
    ui_->motion_frame->setEnabled(false);
    ui_->statusbar->clearMessage();
  }
}

void MainWindow::displayStepInfo() //mudar para enviar pros motores as prosições 
{
  lastPositions = atualMovesList[atualStep-1][0];
  lastVelocitys = atualMovesList[atualStep-1][1];
  
  sendJointVel(lastVelocitys);
  sendJointPos(lastPositions);

  if(mode == 0) on_pos_button_clicked();
  else on_vel_button_clicked(); 

  ui_->statusbar->showMessage(QString("%1 of %2").arg(atualStep).arg(atualMovesList.size())); 
}

void MainWindow::on_nextStep_button_clicked()
{
  if(atualStep < atualMovesList.size())
  {
    atualStep++;
    displayStepInfo();
  }
}

void MainWindow::on_prevStep_button_clicked()
{
  if(atualStep > 1)
  {
    atualStep--;
    displayStepInfo();
  }
}

void MainWindow::sendJointVel(std::vector<int> jointsVel)
{
  auto jointVel = JointStateMsg();

  if(count(jointsVel.begin(), jointsVel.end(), jointsVel[0]) == jointsVel.size())
  {
    jointVel.id.push_back(254);
    jointVel.info.push_back(jointsVel[0]);
    jointVel.type.push_back(JointStateMsg::VELOCITY);
    joint_state_publisher_ ->publish(jointVel);
  }
  else
  {
    for(int i=0; i<18; i++)
    {
      jointVel.id.push_back(i+1);
      jointVel.info.push_back(jointsVel[i]);
      jointVel.type.push_back(JointStateMsg::VELOCITY);
    }
    joint_state_publisher_ ->publish(jointVel); 
  }
}

void MainWindow::sendJointPos(std::vector<int> jointsPos)
{
  auto jointPos = JointStateMsg();

  for(int i=0; i<18; i++)
  {
    jointPos.id.push_back(i+1);
    jointPos.info.push_back(jointsPos[i]);
    jointPos.type.push_back(JointStateMsg::POSITION);
  }
  joint_state_publisher_ ->publish(jointPos);
}

void MainWindow::on_saveStep_button_clicked()
{
  if(atualStep == 0) return;
  
  atualMovesList[atualStep-1][0] = lastPositions;
  atualMovesList[atualStep-1][1] = lastVelocitys;
}

void MainWindow::checkUnsaved()
{
  if(atualStep == 0)
  {
    for (auto label : allPosLabel)
    {
      label->setStyleSheet("QLabel {background-color: none; color : black; }");
    }
    
    return;
  }

  for(int i=0; i<18; i++)
  {
    auto infoForVerify = lastPositions;
    int idx = 0;
    if(mode == 1)
    {
      infoForVerify = lastVelocitys;
      idx = 1;  
    }


    if(atualMovesList[atualStep-1][idx][i] != infoForVerify[i] )
    {
      allPosLabel[i]->setStyleSheet("QLabel {background-color: yellow; color : none; }");
    }
    else allPosLabel[i]->setStyleSheet("QLabel {background-color: none; color : black; }");
  
  }
}

void MainWindow::on_playMove_button_clicked()
{
  QFuture<void> future = QtConcurrent::run(this, &MainWindow::runMove, true);
}

void MainWindow::on_playUntilMove_button_clicked()
{
  QFuture<void> future = QtConcurrent::run(this, &MainWindow::runMove, false);
}

void MainWindow::runMove(bool all)
{
  moveIsRunning = true;

  int limiter = atualMovesList.size();
  if(!all) limiter = atualStep;

  for(int i=0; i<limiter; i++)
  {
    sendJointVel(atualMovesList[i][1]);
    sendJointPos(atualMovesList[i][0]);
    usleep(atualMovesList[i][2][0]*1e3);

    if(stoppingMove) break;
  }

  moveIsRunning = false;
  stoppingMove = false;
} 

void MainWindow::on_stop_button_clicked()
{
  if(!moveIsRunning) return;
  stoppingMove = true;
}

void MainWindow::on_deletStep_button_clicked()
{
  if(atualStep == 0 || atualMovesList.size() == 1) return;
  atualMovesList.erase(atualMovesList.begin()+atualStep-1);
  atualStep = atualStep - 2;
  if(atualStep == -1) atualStep = 0;
  on_nextStep_button_clicked();
}

void MainWindow::on_newStep_button_clicked()
{
  if(atualStep == 0) return;
  std::vector<std::vector<int>> newMove = atualMovesList[atualStep-1];
  atualMovesList.emplace(atualMovesList.begin()+atualStep-1, newMove);
  atualStep += 1;
  displayStepInfo();
}

