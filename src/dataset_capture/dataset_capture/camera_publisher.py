#!/usr/bin/env python3
"""
camera_publisher.py LELEK

No dedicado apenas para abrir a camera e publicar o que ela esta vendo
em um topico ROS2 (sensor_msgs/Image). Este no roda continuamente e
NAO salva nada em disco - ele so serve como "olho" para outros nos,
incluindo o image_saver.py deste mesmo pacote.

Parametros (ros2 run dataset_capture camera_publisher --ros-args -p nome:=valor):
  device        (string) caminho/indice da camera. Aceita tanto um symlink
                         udev como "/dev/camera" (usado no vision_pkg) quanto
                         um indice numerico como "0" para /dev/video0
                         (padrao: /dev/camera)
  topic_name    (string) topico onde a imagem sera publicada
                         (padrao: /dataset_capture/image_raw)
  frame_rate    (double) taxa de publicacao em Hz (padrao: 15.0)
  frame_width   (int)    largura desejada do frame (0 = usa padrao da camera)
  frame_height  (int)    altura desejada do frame (0 = usa padrao da camera)

IMPORTANTE: o dispositivo da camera so pode ser aberto por UM processo
por vez. Se o no de visao (detect.py / image_node, do vision_pkg) estiver
rodando e com a camera aberta, pare ele antes de rodar este no - caso
contrario a abertura da camera aqui vai falhar.
"""

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class CameraPublisher(Node):

    def __init__(self):
        super().__init__('camera_publisher')

        self.declare_parameter('device', '/dev/camera')
        self.declare_parameter('topic_name', '/dataset_capture/image_raw')
        self.declare_parameter('frame_rate', 15.0)
        self.declare_parameter('frame_width', 0)
        self.declare_parameter('frame_height', 0)

        device = self.get_parameter('device').value
        topic_name = self.get_parameter('topic_name').value
        frame_rate = self.get_parameter('frame_rate').value
        frame_width = self.get_parameter('frame_width').value
        frame_height = self.get_parameter('frame_height').value

        self.publisher_ = self.create_publisher(Image, topic_name, 10)

        # Aceita tanto um indice numerico ("0") quanto um path/symlink
        # udev (ex: "/dev/camera"), igual ao vision_pkg/detect.py.
        camera_source = int(device) if str(device).isdigit() else device
        self.cap = cv2.VideoCapture(camera_source, cv2.CAP_ANY)
        if frame_width:
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, frame_width)
        if frame_height:
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, frame_height)

        if not self.cap.isOpened():
            self.get_logger().error(
                f'Nao foi possivel abrir a camera "{device}". Verifique se '
                f'o /dev/camera (regra udev) existe, se as permissoes estao '
                f'ok, e se nenhum outro no (ex: detect.py do vision_pkg) '
                f'ja esta com a camera aberta.'
            )
            raise RuntimeError('Falha ao abrir a camera')

        timer_period = 1.0 / frame_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info(
            f'CameraPublisher iniciado: device={device}, '
            f'topico="{topic_name}", frame_rate={frame_rate} Hz'
        )

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('Falha ao ler frame da camera.')
            return

        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_frame'
        msg.height = frame.shape[0]
        msg.width = frame.shape[1]
        msg.encoding = 'bgr8'
        msg.is_bigendian = 0
        msg.step = frame.shape[1] * frame.shape[2]
        msg.data = np.ascontiguousarray(frame).tobytes()
        self.publisher_.publish(msg)

    def destroy_node(self):
        if self.cap.isOpened():
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
