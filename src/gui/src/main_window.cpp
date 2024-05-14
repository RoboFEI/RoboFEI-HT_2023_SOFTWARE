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
#include <QIntValidator>

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

  torque_publisher_ = this->create_publisher<SinglePositionMsg>("set_position_single", 10);

  this->ui_->setupUi(this);


  for (auto checkBox : findChildren<QCheckBox *>()) {
    this->connect(
      checkBox, &QCheckBox::stateChanged,
      this, &MainWindow::torque_checkbox_changed);
  }
  
  for(auto lineEdit : findChildren<QLineEdit *>())
  {
    lineEdit->setValidator(new QIntValidator(0, 4095, lineEdit));
  }

}

MainWindow::~MainWindow()
{
  delete this->ui_;
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

  auto torque_info = SinglePositionMsg();
  torque_info.address = 64;
  torque_info.id = (uint8_t) id;
  torque_info.position = torque;

  torque_publisher_ ->publish(torque_info);
}
