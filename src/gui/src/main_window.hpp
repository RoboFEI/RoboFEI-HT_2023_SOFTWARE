#pragma once

#include <QWidget>
#include <QMainWindow>


#include <string>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int8.hpp>

#include "custom_interfaces/msg/set_position_original.hpp"

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
  using SinglePositionMsg = custom_interfaces::msg::SetPositionOriginal;
  

  explicit MainWindow(
    const rclcpp::NodeOptions & node_options,
    QWidget * parent = nullptr);
  ~MainWindow() override;

private:
  Ui::MainWindow * ui_;

  rclcpp::Publisher<SinglePositionMsg>::SharedPtr torque_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  const std::string prefix_;

  void torque_checkbox_changed();
  void send_torque_info(int id, int torque);
  void publishJointStates();
};