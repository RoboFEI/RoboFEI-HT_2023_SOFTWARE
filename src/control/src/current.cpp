#include <rclcpp/rclcpp.hpp>
#include <dynamixel_sdk/dynamixel_sdk.h>

#include <vector>
#include <string>
#include <map>
#include <chrono>

// ---------------- CONFIG ----------------
#define PROTOCOL_VERSION 2.0
#define BAUDRATE 1000000

#define CURRENT_ADDR 126
#define CURRENT_LENGTH 2

#define ID_MIN 1
#define ID_MAX 20

// ----------------------------------------

using namespace std::chrono_literals;

struct MotorInfo
{
  int id;
  dynamixel::PortHandler *port;
};

class CurrentReader : public rclcpp::Node
{
public:
  CurrentReader() : Node("current_reader")
  {
    packet_handler_ = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    open_ports();
    detect_motors();

    timer_ = this->create_wall_timer(
      500ms, std::bind(&CurrentReader::read_currents, this));
  }

  ~CurrentReader()
  {
    for (auto &p : ports_)
    {
      p.second->closePort();
    }
  }

private:
  std::map<std::string, dynamixel::PortHandler *> ports_;
  std::vector<MotorInfo> motors_;

  dynamixel::PacketHandler *packet_handler_;
  rclcpp::TimerBase::SharedPtr timer_;

  // -----------------------------
  void open_ports()
  {
    for (int i = 0; i < 4; i++)
    {
      std::string port_name = "/dev/ttyUSB" + std::to_string(i);
      auto *port = dynamixel::PortHandler::getPortHandler(port_name.c_str());

      if (!port->openPort())
      {
        RCLCPP_WARN(this->get_logger(), "Failed to open %s", port_name.c_str());
        continue;
      }

      if (!port->setBaudRate(BAUDRATE))
      {
        RCLCPP_WARN(this->get_logger(), "Failed baudrate on %s", port_name.c_str());
        port->closePort();
        continue;
      }

      ports_[port_name] = port;
      RCLCPP_INFO(this->get_logger(), "Opened %s", port_name.c_str());
    }
  }

  // -----------------------------
  void detect_motors()
  {
    uint8_t dxl_error = 0;

    for (auto &p : ports_)
    {
      auto *port = p.second;

      for (int id = ID_MIN; id <= ID_MAX; id++)
      {
        int dxl_comm_result = packet_handler_->ping(port, id, &dxl_error);

        if (dxl_comm_result == COMM_SUCCESS)
        {
          motors_.push_back({id, port});
          RCLCPP_INFO(this->get_logger(),
                      "Motor ID %d detected on %s",
                      id, p.first.c_str());
        }
      }
    }

    if (motors_.empty())
    {
      RCLCPP_ERROR(this->get_logger(), "No motors detected!");
    }
  }

  // -----------------------------
  void read_currents()
  {
    for (auto &m : motors_)
    {
      uint8_t dxl_error = 0;
      int16_t current_raw = 0;

      int dxl_comm_result = packet_handler_->read2ByteTxRx(
          m.port,
          m.id,
          CURRENT_ADDR,
          reinterpret_cast<uint16_t *>(&current_raw),
          &dxl_error);
          usleep(10000);  // 2ms

      if (dxl_comm_result != COMM_SUCCESS)
      {
        RCLCPP_WARN(this->get_logger(),
                    "Read failed ID %d: %s",
                    m.id,
                    packet_handler_->getTxRxResult(dxl_comm_result));
        continue;
      }

      double current_mA = current_raw * 2.69;

      RCLCPP_INFO(this->get_logger(),
                  "ID %02d | Current: %6.1f mA",
                  m.id, current_mA);
    }

    RCLCPP_INFO(this->get_logger(), "----------------------------");
  }
};

// -----------------------------
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CurrentReader>());
  rclcpp::shutdown();
  return 0;
}