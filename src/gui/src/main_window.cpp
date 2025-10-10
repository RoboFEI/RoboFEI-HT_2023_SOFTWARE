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

  QShortcut *sC4 = new QShortcut(QKeySequence("Ctrl+Z"), this);
  this->connect(sC4, &QShortcut::activated, this, &MainWindow::on_lockAllTorques_button_clicked);

  QShortcut *sC5 = new QShortcut(QKeySequence("Ctrl+X"), this);
  this->connect(sC5, &QShortcut::activated, this, &MainWindow::on_unlockAllTorques_button_clicked);

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
  atualStep = 0;
  updateStepDisplay();
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
  atualStep = 0;
  updateStepDisplay();
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
    updateStepDisplay();
  }
  else
  {
    atualStep = 0;
    ui_->motion_frame->setEnabled(false);
    ui_->statusbar->clearMessage();
    updateStepDisplay();
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
  if(atualStep < (int)atualMovesList.size())
  {
    atualStep++;
    displayStepInfo();
    updateStepDisplay();
  }
}

void MainWindow::on_prevStep_button_clicked()
{
  if(atualStep > 1)
  {
    atualStep--;
    displayStepInfo();
    updateStepDisplay();
  }
}

void MainWindow::sendJointVel(std::vector<int> jointsVel)
{
  auto jointVel = JointStateMsg();

  if (std::count(jointsVel.begin(), jointsVel.end(), jointsVel[0]) == static_cast<int>(jointsVel.size()))
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
  joint_state_publisher_->publish(jointPos);
}

void MainWindow::on_saveStep_button_clicked()
{
  if (atualStep == 0) return;

  // 1) Atualiza seu vetor interno
  lastPositions = getAllPositions();
  
  // Verificação de bounds para atualMovesList
  if (atualStep - 1 < static_cast<int>(atualMovesList.size())) {
    atualMovesList[atualStep - 1][0] = lastPositions;
    atualMovesList[atualStep - 1][1] = lastVelocitys;
  } else {
    RCLCPP_ERROR(this->get_logger(), "❌ Erro: step %d fora dos limites do atualMovesList", atualStep);
    return;
  }

  // 2) Obtém o JSON do movimento
  std::string moveName = ui_->movesList->currentText().toStdString();
  if (!motions.contains(moveName)) {
    RCLCPP_WARN(this->get_logger(),
                "Movimento %s não encontrado no JSON",
                moveName.c_str());
    return;
  }
  json &moveJson = motions.getMoveJson(moveName);

  // 3) Conta quantos blocos de posição existem para determinar o máximo
  int totalSuffix = moveJson["number of movements"];
  int posCount = 0;
  int targetI = -1;

  // Primeiro: contar todos os blocos de posição para saber o máximo
  for (int i = 1; i <= totalSuffix; ++i) {
    std::string addrKey = "address" + std::to_string(i);
    if (moveJson.contains(addrKey) && moveJson[addrKey] == 116) {
      posCount++;
    }
  }

  // Segundo: encontrar o sufixo correspondente ao atualStep
  int currentPosCount = 0;
  for (int i = 1; i <= totalSuffix; ++i) {
    std::string addrKey = "address" + std::to_string(i);
    if (moveJson.contains(addrKey) && moveJson[addrKey] == 116) {
      currentPosCount++;
      if (currentPosCount == atualStep) {
        targetI = i;
        break;
      }
    }
  }

  if (targetI < 0) {
    RCLCPP_WARN(this->get_logger(),
                "Não encontrei o %dº bloco de posição em %s",
                atualStep, moveName.c_str());
    return;
  }

  // 4) Obtém o valor do sleep da interface
  bool is_valid;
  QString sleep_text = ui_->sleep_value->text();
  double sleep_value = sleep_text.toDouble(&is_valid);
  if (!is_valid) {
      RCLCPP_INFO(this->get_logger(), "Erro! Valor de sleep inválido");
      sleep_value = 1.0;
  }

  // 5) Sobrescreve position e sleep para esse sufixo
  std::string suf = std::to_string(targetI);
  moveJson["position" + suf] = lastPositions;
  moveJson["sleep"    + suf] = sleep_value;

  // 6) Grava e pós-processa
  std::string jsonFilePath =
    folder_path +
    "/src/control/Data/motion" +
    std::to_string(robot_number_) +
    ".json";
  motions.saveJson(jsonFilePath);
  postProcessFile(jsonFilePath);

  RCLCPP_INFO(this->get_logger(),
              "✅ Step %d/%d (sufixo %d) salvo com sucesso em %s",
              atualStep, posCount, targetI, moveName.c_str());
  
  // 7) Incrementa o step atual de forma segura
  if (atualStep < posCount) {
    atualStep++;
    RCLCPP_INFO(this->get_logger(), "➡️  Movendo para step %d/%d", atualStep, posCount);
  } else {
    RCLCPP_INFO(this->get_logger(), "🛑 Último step (%d/%d) alcançado", atualStep, posCount);
  }
  
  updateStepDisplay();
}

void MainWindow::on_saveIntoNextStep_button_clicked() {
    // 1) Obtém o JSON do movimento
    std::string moveName = ui_->movesList->currentText().toStdString();
    if (!motions.contains(moveName)) {
        RCLCPP_WARN(this->get_logger(), "Movimento %s não encontrado no JSON", moveName.c_str());
        return;
    }
    json &moveJson = motions.getMoveJson(moveName);

    // 2) Conta apenas os blocos de posição (address == 116)
    int posCount = 0;
    std::vector<int> positionIndices; // Armazena os índices reais dos blocos de posição
    
    int totalSuffix = moveJson["number of movements"];
    for (int i = 1; i <= totalSuffix; ++i) {
        std::string addrKey = "address" + std::to_string(i);
        if (moveJson.contains(addrKey) && moveJson[addrKey] == 116) {
            posCount++;
            positionIndices.push_back(i); // Guarda o índice real
        }
    }

    if (posCount == 0) {
        RCLCPP_WARN(this->get_logger(), "Nenhum bloco de posição encontrado em %s", moveName.c_str());
        return;
    }

    // 3) Determina onde salvar
    int stepToSave = atualStep + 1;
    
    // Se quer salvar além do número de steps disponíveis, vai para o último
    if (stepToSave > posCount) {
        RCLCPP_WARN(this->get_logger(), "⚠️  Não há próximo step! Salvando no último step %d", posCount);
        stepToSave = posCount;
    }

    // 4) Verifica bounds do atualMovesList
    if (stepToSave - 1 >= static_cast<int>(atualMovesList.size())) {
        RCLCPP_ERROR(this->get_logger(), "❌ Erro: step %d fora dos limites do atualMovesList (size: %zu)", 
                    stepToSave, atualMovesList.size());
        return;
    }

    // 5) Atualiza vetor interno
    lastPositions = getAllPositions();
    atualMovesList[stepToSave - 1][0] = lastPositions;
    atualMovesList[stepToSave - 1][1] = lastVelocitys;

    // 6) Obtém o índice real no JSON usando nosso mapeamento
    int targetI = positionIndices[stepToSave - 1]; // -1 porque positionIndices é 0-based
    std::string suf = std::to_string(targetI);

    // 7) Sobrescreve position/sleep
    bool is_valid;
    QString sleep_text = ui_->sleep_value->text();
    double sleep_value = sleep_text.toDouble(&is_valid);
    if (!is_valid) {
        RCLCPP_INFO(this->get_logger(), "Valor de sleep inválido, usando 1.0");
        sleep_value = 1.0;
    }

    moveJson["position" + suf] = lastPositions;
    moveJson["sleep" + suf] = sleep_value;

    // 8) Grava e pós-processa
    std::string jsonFilePath = folder_path + "/src/control/Data/motion" + std::to_string(robot_number_) + ".json";
    motions.saveJson(jsonFilePath);
    postProcessFile(jsonFilePath);
    if (atualStep < posCount) {
      atualStep++;
    }
    if (atualStep == posCount) {
        RCLCPP_INFO(this->get_logger(), "Último step (%d) alcançado", posCount);
    }
    RCLCPP_INFO(this->get_logger(), "✅ Step %d/%d (sufixo %d) salvo em %s", 
                stepToSave, posCount, targetI, moveName.c_str());
    
    updateStepDisplay();
    
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
  updateStepDisplay();
}

void MainWindow::on_newStep_button_clicked() {
    if (atualStep == 0) return;
    
    std::string moveName = ui_->movesList->currentText().toStdString();
    if (!motions.contains(moveName)) {
        RCLCPP_WARN(this->get_logger(), "Movimento não encontrado");
        return;
    }
    
    std::string filePath = folder_path + "/src/control/Data/motion" + 
                          std::to_string(robot_number_) + ".json";
    
    // ✅ CHAMA A FUNÇÃO UNIFICADA QUE TRATA TODOS OS CENÁRIOS
    insertNewStepInFile(filePath, moveName, atualStep);
    
    // Recarrega e atualiza
    on_loadMoves_button_released();
    
    // Atualiza a lista interna
    if (atualStep < static_cast<int>(atualMovesList.size())) {
        std::vector<std::vector<int>> newMove = atualMovesList[atualStep - 1];
        atualMovesList.emplace(atualMovesList.begin() + atualStep, newMove);
    }
    
    atualStep += 1;
    displayStepInfo();
    
    RCLCPP_INFO(this->get_logger(), "Novo step %d criado", atualStep);
}

void MainWindow::sendAllTorques(bool torque_state)
{
    RCLCPP_INFO(this->get_logger(), "Setting all torques to: %d", torque_state);
    
    auto torque_info = JointStateMsg();
    for(int i = 1; i <= 18; i++){
        torque_info.id.push_back(i);
        torque_info.info.push_back(torque_state);
        torque_info.type.push_back(JointStateMsg::TORQUE);
    }
    joint_state_publisher_->publish(torque_info);
}

void MainWindow::update_all_torque_checkboxes(bool checked)
{
    for(int i = 1; i <= 18; i++){
        QCheckBox* checkbox = findChild<QCheckBox*>(QString("torque_id_%1").arg(i));
        if(checkbox && checkbox != nullptr){
            checkbox->blockSignals(true);
            checkbox->setChecked(checked);
            checkbox->blockSignals(false);
        }
    }
}

void MainWindow::on_lockAllTorques_button_clicked(){
    try {
        sendAllTorques(true);
        update_all_torque_checkboxes(true);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error locking torques: %s", e.what());
    }
}

void MainWindow::on_unlockAllTorques_button_clicked(){
    try {
        sendAllTorques(false);
        update_all_torque_checkboxes(false);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Error unlocking torques: %s", e.what());
    }
}

void MainWindow::updateStepDisplay()
{
    if (atualStep == 0 || atualMovesList.empty()) {
        ui_->atual_pos_label->setText("No move selected");
        ui_->atual_pos_label->setStyleSheet("QLabel { color: gray; font-style: italic; }");
    } else {
        // Formatação mais elaborada
        QString stepText = QString("Step: <b>%1</b> / %2").arg(atualStep).arg(atualMovesList.size());
        ui_->atual_pos_label->setText(stepText);
        
        // Muda a cor baseado na posição
        if (atualStep == 1) {
            ui_->atual_pos_label->setStyleSheet("QLabel { color: blue; font-weight: bold; }");
        } else if (atualStep == static_cast<int>(atualMovesList.size())) {
            ui_->atual_pos_label->setStyleSheet("QLabel { color: green; font-weight: bold; }");
        } else {
            ui_->atual_pos_label->setStyleSheet("QLabel { color: black; font-weight: bold; }");
        }
    }
    
    // Atualiza também o status bar se quiser
    if (atualStep > 0) {
        ui_->statusbar->showMessage(QString("Current step: %1 of %2").arg(atualStep).arg(atualMovesList.size()));
    } else {
        ui_->statusbar->clearMessage();
    }
}


//Implementar

/*

void MainWindow::on_newStep_button_clicked()
{
    try {
        RCLCPP_INFO(this->get_logger(), "Clicado new step button");
        
        if (atualStep == 0) {
            RCLCPP_WARN(this->get_logger(), "Nenhum movimento selecionado");
            return;
        }
        
        std::string moveName = ui_->movesList->currentText().toStdString();
        if (moveName.empty() || moveName == "No Move") {
            RCLCPP_WARN(this->get_logger(), "Movimento inválido");
            return;
        }
        
        if (!motions.contains(moveName)) {
            RCLCPP_WARN(this->get_logger(), "Movimento %s não encontrado", moveName.c_str());
            return;
        }
        
        // Encontrar o sufixo real correspondente ao step atual
        json &moveJson = motions.getMoveJson(moveName);
        int totalSuffix = moveJson["number of movements"].get<int>();
        int currentSuffix = 0;
        int posCount = 0;
        
        for (int i = 1; i <= totalSuffix; ++i) {
            std::string addrKey = "address" + std::to_string(i);
            if (moveJson.contains(addrKey) && moveJson[addrKey].get<int>() == 116) {
                posCount++;
                if (posCount == atualStep) {
                    currentSuffix = i;
                    break;
                }
            }
        }
        
        if (currentSuffix == 0) {
            RCLCPP_ERROR(this->get_logger(), "Não encontrou sufixo para step %d", atualStep);
            return;
        }
        
        std::string filePath = folder_path + "/src/control/Data/motion" + 
                              std::to_string(robot_number_) + ".json";
        
        RCLCPP_INFO(this->get_logger(), "Inserindo novo step após sufixo %d", currentSuffix);
        
        // ✅ CHAMADA LIMPA - a função se encarrega de tudo
        insertNewStepInFile(filePath, moveName, currentSuffix);
        
        // Recarregar e atualizar
        on_loadMoves_button_released();
        
        // Atualizar lista interna
        if (!atualMovesList.empty() && atualStep >= 1 && atualStep <= static_cast<int>(atualMovesList.size())) {
            std::vector<std::vector<int>> newMove = atualMovesList[atualStep - 1];
            atualMovesList.emplace(atualMovesList.begin() + atualStep, newMove);
            atualStep += 1;
            displayStepInfo();
            
            RCLCPP_INFO(this->get_logger(), "Novo step %d criado com sucesso", atualStep);
        }
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Exceção em newStep: %s", e.what());
    }
}

*/