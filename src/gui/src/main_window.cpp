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

MainWindow::MainWindow(
  const rclcpp::NodeOptions & node_options,
  QWidget * parent
)
: QMainWindow(parent),
  rclcpp::Node("robot_gui", node_options),
  ui_(new Ui::MainWindow),
  prefix_(this->declare_parameter("prefix", ""))
{
  using namespace std::chrono_literals;
 
  this->timer_ = this->create_wall_timer(
    100ms, std::bind(&MainWindow::publishJointStates, this));

  torque_publisher_ = this->create_publisher<JointStateMsg>("set_joint_topic", 10);

  position_subscriber_ = this->create_subscription<JointStateMsg>(
    "all_joints_position", 10,
    std::bind(&MainWindow::jointPositionCallback, this, _1) 
  );

  this->ui_->setupUi(this);

  allPosLineEdit << ui_->pos_id_1  << ui_->pos_id_2  << ui_->pos_id_3  << ui_->pos_id_4  << ui_->pos_id_5;
  allPosLineEdit << ui_->pos_id_6  << ui_->pos_id_7  << ui_->pos_id_8  << ui_->pos_id_9  << ui_->pos_id_10;
  allPosLineEdit << ui_->pos_id_11 << ui_->pos_id_12 << ui_->pos_id_13 << ui_->pos_id_14 << ui_->pos_id_15;
  allPosLineEdit << ui_->pos_id_16 << ui_->pos_id_17 << ui_->pos_id_18 << ui_->pos_id_19 << ui_->pos_id_20;

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

void MainWindow::jointPositionCallback(const JointStateMsg::SharedPtr all_joints_position)
{
  for(int i=0; i<20; i++)
  { 
    allPosLineEdit[i]->setText(QString("%1").arg(all_joints_position->info[i+1]));
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

  torque_publisher_ ->publish(torque_info);
}
