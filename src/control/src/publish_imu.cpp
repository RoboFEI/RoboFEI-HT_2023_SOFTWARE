#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <string>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <functional>
#include <cerrno>

#include <fcntl.h>
#include <unistd.h>
#include <termios.h>

class PublishImu : public rclcpp::Node
{
public:
    PublishImu()
        : Node("publish_imu")
    {
        // Publicador da mensagem IMU
        imu_publisher_ =
            this->create_publisher<sensor_msgs::msg::Imu>(
                "/imu/data", 10);

        // Abre a porta serial da IMU
        serial_fd_ = open(
            "/dev/ttyUSB0",
            O_RDONLY | O_NOCTTY | O_NONBLOCK
        );

        if (serial_fd_ < 0)
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "Erro ao abrir /dev/ttyUSB0"
            );

            throw std::runtime_error(
                "Nao foi possivel abrir a porta serial da IMU"
            );
        }

        // Configuração da comunicação serial
        struct termios tty {};

        if (tcgetattr(serial_fd_, &tty) != 0)
        {
            close(serial_fd_);
            serial_fd_ = -1;

            throw std::runtime_error(
                "Erro ao obter configuracao da porta serial"
            );
        }

        cfmakeraw(&tty);

        // Velocidade: 115200 baud
        cfsetispeed(&tty, B115200);
        cfsetospeed(&tty, B115200);

        // Configuração: 8 bits, sem paridade, 1 stop bit
        tty.c_cflag &= ~PARENB;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8;

        // Habilita recebimento e conexão local
        tty.c_cflag |= CREAD | CLOCAL;

        // Sem controle de fluxo por hardware
        tty.c_cflag &= ~CRTSCTS;

        // Leitura não bloqueante
        tty.c_cc[VMIN] = 0;
        tty.c_cc[VTIME] = 0;

        if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0)
        {
            close(serial_fd_);
            serial_fd_ = -1;

            throw std::runtime_error(
                "Erro ao configurar a porta serial"
            );
        }

        // Descarta dados antigos do buffer serial
        tcflush(serial_fd_, TCIFLUSH);

        RCLCPP_INFO(
            this->get_logger(),
            "IMU conectada em /dev/ttyUSB0 a 115200 baud"
        );

        // Timer para ler a serial periodicamente
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&PublishImu::read_serial, this)
        );
    }

    ~PublishImu()
    {
        if (serial_fd_ >= 0)
        {
            close(serial_fd_);
        }
    }

private:
    // Publicador ROS 2
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr
        imu_publisher_;

    rclcpp::TimerBase::SharedPtr timer_;

    // Porta serial
    int serial_fd_ = -1;

    // Buffer para montar linhas recebidas pela serial
    std::string serial_buffer_;

    // Valores recebidos da IMU
    double roll_ = 0.0;
    double pitch_ = 0.0;
    double yaw_ = 0.0;

    double acc_x_ = 0.0;
    double acc_y_ = 0.0;
    double acc_z_ = 0.0;

    double gyro_x_ = 0.0;
    double gyro_y_ = 0.0;
    double gyro_z_ = 0.0;

    // Flags para verificar se todos os dados foram recebidos
    bool has_roll_ = false;
    bool has_pitch_ = false;
    bool has_yaw_ = false;

    bool has_acc_x_ = false;
    bool has_acc_y_ = false;
    bool has_acc_z_ = false;

    bool has_gyro_x_ = false;
    bool has_gyro_y_ = false;
    bool has_gyro_z_ = false;

    // Conversão de graus para radianos
    static constexpr double PI = 3.14159265358979323846;

    double degrees_to_radians(double angle)
    {
        return angle * PI / 180.0;
    }

    // Lê os dados disponíveis na porta serial
    void read_serial()
    {
        char buffer[512];

        while (true)
        {
            const ssize_t bytes_read = read(
                serial_fd_,
                buffer,
                sizeof(buffer)
            );

            if (bytes_read > 0)
            {
                serial_buffer_.append(
                    buffer,
                    static_cast<std::size_t>(bytes_read)
                );

                // Evita crescimento excessivo caso falte '\n'
                if (serial_buffer_.size() > 8192)
                {
                    RCLCPP_WARN(
                        this->get_logger(),
                        "Buffer serial muito grande; limpando buffer"
                    );

                    serial_buffer_.clear();
                }

                // Processa todas as linhas completas
                std::size_t newline_position;

                while ((newline_position =
                            serial_buffer_.find('\n')) !=
                       std::string::npos)
                {
                    std::string line =
                        serial_buffer_.substr(
                            0,
                            newline_position
                        );

                    serial_buffer_.erase(
                        0,
                        newline_position + 1
                    );

                    // Remove '\r', se existir
                    if (!line.empty() &&
                        line.back() == '\r')
                    {
                        line.pop_back();
                    }

                    process_line(line);
                }
            }
            else
            {
                // Não há mais dados disponíveis neste momento
                if (bytes_read < 0 &&
                    errno != EAGAIN &&
                    errno != EWOULDBLOCK &&
                    errno != EINTR)
                {
                    RCLCPP_ERROR(
                        this->get_logger(),
                        "Erro ao ler a porta serial"
                    );
                }

                break;
            }
        }
    }

    // Extrai o valor numérico de uma linha
    bool extract_value(
        const std::string &line,
        const std::string &prefix,
        double &value)
    {
        if (line.rfind(prefix, 0) != 0)
        {
            return false;
        }

        try
        {
            std::size_t chars_used = 0;

            const std::string number =
                line.substr(prefix.size());

            value = std::stod(number, &chars_used);

            // Ignora caracteres extras depois do número,
            // mas não aceita uma linha sem número válido.
            return chars_used > 0;
        }
        catch (const std::exception &)
        {
            RCLCPP_WARN(
                this->get_logger(),
                "Valor invalido recebido: %s",
                line.c_str()
            );

            return false;
        }
    }

    // Interpreta cada linha recebida pela serial
    void process_line(const std::string &line)
    {
        if (line.empty())
        {
            return;
        }

        double value;

        // ROLL indica o início de um novo conjunto de dados.
        if (extract_value(line, "ROLL=", value))
        {
            reset_flags();

            roll_ = value;
            has_roll_ = true;
        }
        else if (extract_value(line, "PITCH=", value))
        {
            pitch_ = value;
            has_pitch_ = true;
        }
        else if (extract_value(line, "YALL=", value) ||
                 extract_value(line, "YAW=", value))
        {
            // Aceita YALL, como informado na saída da IMU,
            // e também YAW, caso o formato seja alterado.
            if (extract_value(line, "YALL=", value) ||
                extract_value(line, "YAW=", value))
            {
                yaw_ = value;
                has_yaw_ = true;
            }
        }
        else if (extract_value(line, "acc_x=", value))
        {
            acc_x_ = value;
            has_acc_x_ = true;
        }
        else if (extract_value(line, "acc_y=", value))
        {
            acc_y_ = value;
            has_acc_y_ = true;
        }
        else if (extract_value(line, "acc_z=", value))
        {
            acc_z_ = value;
            has_acc_z_ = true;
        }
        else if (extract_value(line, "gyro_x=", value))
        {
            gyro_x_ = value;
            has_gyro_x_ = true;
        }
        else if (extract_value(line, "gyro_y=", value))
        {
            gyro_y_ = value;
            has_gyro_y_ = true;
        }
        else if (extract_value(line, "gyro_z=", value))
        {
            gyro_z_ = value;
            has_gyro_z_ = true;
        }

        // Publica somente quando todos os nove valores
        // do conjunto atual tiverem sido recebidos.
        if (all_data_received())
        {
            publish_imu();

            // Aguarda um novo conjunto completo
            reset_flags();
        }
    }

    // Limpa as flags para receber um novo conjunto
    void reset_flags()
    {
        has_roll_ = false;
        has_pitch_ = false;
        has_yaw_ = false;

        has_acc_x_ = false;
        has_acc_y_ = false;
        has_acc_z_ = false;

        has_gyro_x_ = false;
        has_gyro_y_ = false;
        has_gyro_z_ = false;
    }

    // Verifica se todos os valores foram recebidos
    bool all_data_received() const
    {
        return
            has_roll_ &&
            has_pitch_ &&
            has_yaw_ &&
            has_acc_x_ &&
            has_acc_y_ &&
            has_acc_z_ &&
            has_gyro_x_ &&
            has_gyro_y_ &&
            has_gyro_z_;
    }

    // Calcula o quaternion e publica a mensagem IMU
    void publish_imu()
    {
        sensor_msgs::msg::Imu msg;

        msg.header.stamp = this->now();
        msg.header.frame_id = "imu_link";

        // Converte Roll, Pitch e YALL de graus para radianos
        const double roll = degrees_to_radians(roll_);
        const double pitch = degrees_to_radians(pitch_);
        const double yaw = degrees_to_radians(yaw_);

        // Cálculo do quaternion
        const double cr = std::cos(roll / 2.0);
        const double sr = std::sin(roll / 2.0);

        const double cp = std::cos(pitch / 2.0);
        const double sp = std::sin(pitch / 2.0);

        const double cy = std::cos(yaw / 2.0);
        const double sy = std::sin(yaw / 2.0);

        msg.orientation.w =
            cr * cp * cy + sr * sp * sy;

        msg.orientation.x =
            sr * cp * cy - cr * sp * sy;

        msg.orientation.y =
            cr * sp * cy + sr * cp * sy;

        msg.orientation.z =
            cr * cp * sy - sr * sp * cy;

        // A velocidade angular é preenchida com os valores
        // recebidos da serial, sem conversão de unidades.
        msg.angular_velocity.x = gyro_x_;
        msg.angular_velocity.y = gyro_y_;
        msg.angular_velocity.z = gyro_z_;

        // A aceleração linear também é preenchida diretamente.
        msg.linear_acceleration.x = acc_x_;
        msg.linear_acceleration.y = acc_y_;
        msg.linear_acceleration.z = acc_z_;

        // Covariâncias zeradas: ainda não foram calibradas.
        // Como a orientação é fornecida, não usamos -1
        // na primeira posição da covariância da orientação.

        imu_publisher_->publish(msg);

        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            2000,
            "IMU publicada | Roll: %.2f | Pitch: %.2f | YALL: %.2f",
            roll_,
            pitch_,
            yaw_
        );
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    try
    {
        auto node = std::make_shared<PublishImu>();
        rclcpp::spin(node);
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(
            rclcpp::get_logger("publish_imu"),
            "%s",
            e.what()
        );

        rclcpp::shutdown();
        return 1;
    }

    rclcpp::shutdown();

    return 0;
}