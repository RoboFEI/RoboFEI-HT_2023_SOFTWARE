import os
import json
import serial
import serial.tools.list_ports
import numpy as np
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.logging import LoggingSeverity

from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import Twist
from custom_interfaces.msg import RobotState


class Ros2Serial(Node):
    ser = None

    def __init__(self, node_name="ros2serial_node"):
        super().__init__(node_name=node_name)

        self.get_logger().set_level(LoggingSeverity.INFO)

        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('serial_port', '/dev/imu_ard')  # symlink ex: /dev/imu_ard

        self.baud_rate = self.get_parameter('baud_rate').value
        self.serial_port = self.get_parameter('serial_port').value

        # não chamar reset_input_buffer antes de ter uma conexão válida
        self.ser = self.connect_to_serial(self.serial_port)
        if self.ser is not None:
            try:
                self.ser.reset_input_buffer()
                self.ser.reset_output_buffer()
            except Exception as e:
                self.get_logger().warn(f"Não foi possível resetar buffers: {e}")

        qos_profile = QoSProfile(depth=10)

        self.robot_state = self.create_publisher(RobotState, '/robot_state', qos_profile)
        self.imu = self.create_publisher(Int32MultiArray, '/imu', qos_profile)

        self.publish_thread = threading.Thread(target=self.publish_messages, daemon=True)
        self.publish_thread.start()

    def check_port_available(self, port_name):
        """
        Retorna (True, resolved_path) se a porta/symlink existir e for detectada
        pelo system/pyserial; caso contrário (False, None).
        """
        try:
            resolved = os.path.realpath(port_name)  # resolve symlink (se houver)
        except Exception:
            resolved = port_name

        # Se o arquivo não existir no FS, não adianta
        if not os.path.exists(resolved):
            return False, None

        # Lista de portas detectadas pelo sistema
        ports = list(serial.tools.list_ports.comports())
        # comparar resolved com port.device (ex: '/dev/ttyUSB0')
        for p in ports:
            # p.device é o caminho real; alguns ambientes podem usar apenas basename
            if resolved == p.device or os.path.basename(resolved) == os.path.basename(p.device):
                return True, resolved
            # também podemos comparar pelo HWID/serial number (se disponível)
            if hasattr(p, 'hwid') and p.hwid and os.path.basename(port_name) in str(p.hwid):
                return True, p.device

        # se o resolved existe mas não está em comports, ainda assim podemos tentar abrir
        # (às vezes comports não mostra imediatamente). Retornar True com resolved para tentar.
        return True, resolved

    def robotState(self, valor):
        robot = RobotState()
        robot.fallen_forward = False
        robot.fallen_backwards = False
        robot.fallen_side = False

        try:
            if valor[1] > -3000 and valor[2] > 0:
                robot.fallen_forward = True
            elif valor[2] < -10000:
                robot.fallen_backwards = True
        except Exception:
            # evitar crash se `valor` não for indexável do jeito esperado
            self.get_logger().warn("robotState recebeu 'valor' inesperado")

        self.robot_state.publish(robot)

    def imuSensor(self, valor):
        msg = Int32MultiArray()
        try:
            msg.data = [int(valor[i]) for i in range(6)]
        except Exception:
            # fallback se tiver menos de 6 valores
            msg.data = [int(x) for x in valor]
        self.imu.publish(msg)

    def connect_to_serial(self, port):
        retry_delay = 0.5
        while rclpy.ok():
            ok, resolved = self.check_port_available(port)
            if not ok or resolved is None:
                # log informativo com portas detectadas para debugging
                available = [f"{p.device} (hwid={p.hwid})" for p in serial.tools.list_ports.comports()]
                self.get_logger().error(f"A porta especificada não está disponível: {port}. Detectadas: {available}")
                time.sleep(retry_delay)
                continue

            try:
                # abrir usando o caminho resolvido (realpath) e timeouts para evitar bloqueios
                self.get_logger().info(f"Tentando abrir porta serial: pedido='{port}' resolvido='{resolved}'")
                self.ser = serial.Serial(resolved, self.baud_rate, timeout=1, write_timeout=1)
                # alguns Arduinos resetam ao abrir a serial — aguardar um pouquinho
                time.sleep(0.2)
                try:
                    self.ser.write(b'\x05')
                except serial.SerialTimeoutException:
                    self.get_logger().warn("Timeout ao escrever no dispositivo logo após abrir a porta")
                self.get_logger().info('\033[92m' + f"Conexão bem-sucedida com a porta serial {resolved}" + '\033[0m')
                return self.ser
            except serial.SerialException as e:
                # registrar informações adicionais sobre o symlink alvo e portas detectadas
                self.get_logger().error(f"Falha ao conectar com a porta serial '{resolved}': {e}")
                try:
                    if os.path.islink(port):
                        target = os.readlink(port)
                        self.get_logger().info(f"Symlink {port} -> {target}")
                except OSError:
                    pass

                available = [f"{p.device} (hwid={p.hwid})" for p in serial.tools.list_ports.comports()]
                self.get_logger().info(f"Portas detectadas agora: {available}")

            time.sleep(retry_delay)

        # se rclpy não OK, retornar None
        return None

    def publish_messages(self):
        time.sleep(1)
        while rclpy.ok():
            try:
                if self.ser is not None and self.ser.is_open and rclpy.ok():
                    try:
                        try:
                            line = self.ser.readline()
                        except (serial.SerialException, serial.SerialTimeoutException, serial.serialutil.SerialException):
                            self.get_logger().warn("Conexão com a porta serial perdida. Tentando reconectar...")
                            try:
                                self.ser.close()
                            except Exception:
                                pass
                            self.ser = self.connect_to_serial(self.serial_port)
                            continue  # volta para o loop
                        # se linha vazia, continua
                        if not line:
                            continue

                        try:
                            data = json.loads(line)
                        except json.JSONDecodeError:
                            self.get_logger().warn(f"Recebeu linha não-JSON: {line!r}")
                            continue

                        if 'encoders' in data and rclpy.ok():
                            try:
                                valor = data['encoders']
                                self.robotState(valor)
                                self.imuSensor(valor)
                                self.get_logger().info(f"Encoders: {valor}")
                            except Exception as e:
                                self.get_logger().error(f"Erro ao processar encoders: {e}")
                        elif rclpy.ok():
                            self.get_logger().info("Não recebeu os valores dos encoders")
                    except TypeError as e:
                        self.get_logger().error(f"Falha ao comunicar com a porta serial: {e}")
            except (serial.SerialException, serial.SerialTimeoutException):
                self.get_logger().info("Conexão com a porta serial perdida. Tentando reconectar...")
                try:
                    self.ser.close()
                except Exception:
                    pass
                self.ser = self.connect_to_serial(self.serial_port)
            except Exception as e:
                self.get_logger().error(f"Ocorreu uma exceção não tratada: {e}")
        if self.ser is not None:
            try:
                self.ser.close()
            except Exception:
                pass


def main(args=None):
    rclpy.init(args=args)
    ros2serial = Ros2Serial()
    try:
        rclpy.spin(ros2serial)
    except KeyboardInterrupt:
        print('Interrupção por teclado capturada.')
    finally:
        if ros2serial.ser is not None:
            try:
                ros2serial.ser.close()
            except Exception:
                pass
        ros2serial.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
