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
        self.declare_parameter('serial_port', '/dev/ttyUSB0') # Substitua pelo nome da porta correta - '/dev/ttyACM0'  ou '/dev/ttyUSB0'

        self.baud_rate = self.get_parameter('baud_rate').value
        self.serial_port = self.get_parameter('serial_port').value
        
        self.ser = self.connect_to_serial(self.serial_port)
        self.ser.reset_input_buffer() # Esvaziando o buffer de recepção
        self.ser.reset_output_buffer() # Esvaziando o buffer de transmissão

        qos_profile = QoSProfile(depth=10)
        
        self.robot_state = self.create_publisher(RobotState, '/robot_state', qos_profile)

        self.imu = self.create_publisher(Int32MultiArray, '/imu', qos_profile)

        
        self.publish_thread = threading.Thread(target=self.publish_messages)
        self.publish_thread.start()



    def check_port_available(self, port_name):
        port_list = list(serial.tools.list_ports.comports())
        for port in port_list:
            if port_name in port.device:
                return True
        return False
        

    
    def robotState(self, valor):
        robot = RobotState()
        robot.fallen_forward = False
        robot.fallen_backwards = False
        robot.fallen_side = False

        if valor[1] > -5 and valor[1] < 5:
            if valor[0] < -5:
                robot.fallen_forward = True
            elif valor[0] > 5:
                robot.fallen_backwards = True
        else:
            robot.fallen_side = True
        
        


        # self.get_logger().info(valor)
        # msg.data = [int(valor[0]), int(valor[1])]
        self.robot_state.publish(robot)
        # self.get_logger().info('Robot falling foward!' + msg.fallen_backwards)

    
    def imuSensor(self, valor):
        msg = Int32MultiArray()
        msg.data = [int(valor[0]), int(valor[1]), int(valor[2])]
        self.imu.publish(msg)
        # self.get_logger().info('Robot falling foward!' + msg.fallen_backwards)
        

        
    def connect_to_serial(self, port):
        while rclpy.ok():
            if self.check_port_available(port):
                try:
                    self.ser = serial.Serial(port, self.baud_rate)
                    self.get_logger().info('\033[92m' + "Conexão bem-sucedida com a porta serial " + str(port) + '\033[0m')
                    self.ser.write(b'\x05')  #Envia o caractere "Enquiry" (0x05) que informa ao Arduino que a conexão foi estabelecida
                    return self.ser
                except serial.SerialException as e:
                    self.get_logger().error("Falha ao conectar com a porta serial: " + str(e) )
            else:
                self.get_logger().error("A porta serial especificada não está disponível")
            time.sleep(0.5)  # Espera meio segundo antes de tentar conectar novamente
            
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
                            self.ser.close()
                            self.ser = self.connect_to_serial(self.serial_port)
                        data = json.loads(line)
                        if 'encoders' in data and rclpy.ok():
                            try:
                                valor = data['encoders']
                                self.robotState(valor)
                                self.imuSensor(valor)
                                self.get_logger().debug(f"Encoders: {valor}")
                            except rclpy.handle.InvalidHandle as e:
                                self.get_logger().error(f"Provavelmente o nó foi finalizado: {e}")
                            except:
                                self.get_logger().error("Ocorreu um erro na leitura dos encoders")
                        elif rclpy.ok():
                            self.get_logger().info("Não recebeu os valores dos encoders")
                    except TypeError as e:
                        self.get_logger().error(f"Falha ao comunicar com a porta serial: {e}" )
            except (serial.SerialException, serial.SerialTimeoutException):
                self.get_logger().info("Conexão com a porta serial perdida. Tentando reconectar...")
                self.ser.close()
                self.ser = self.connect_to_serial(self.serial_port)
            except Exception as e:
                self.get_logger().error(f"Ocorreu uma excesão não tratada: {e}")
            #time.sleep(0.001) #Atrasa 1 milissegundo para não consumir muito processamento da CPU
        if self.ser is not None:
            self.ser.close()

 
def main(args=None):
    rclpy.init(args=args)
    ros2serial = Ros2Serial()
    try:
        rclpy.spin(ros2serial)
    except KeyboardInterrupt:
        print('Interrupção por teclado capturada.')
    finally:
        ros2serial.ser.close()
        ros2serial.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()