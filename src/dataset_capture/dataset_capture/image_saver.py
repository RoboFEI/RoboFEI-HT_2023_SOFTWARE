#!/usr/bin/env python3
"""
image_saver.py LELEK

No responsavel por SALVAR as imagens em uma pasta local do robo.
As fotos so comecam a ser tiradas a partir do momento em que este
no e executado (dar "ros2 run" ou "ros2 launch" nele) - o
camera_publisher pode ficar publicando o tempo todo, mas nada e
gravado em disco enquanto este no nao estiver rodando.

Parametros (ros2 run dataset_capture image_saver --ros-args -p nome:=valor):
  image_topic   (string) topico de onde a imagem sera lida
                         (padrao: /dataset_capture/image_raw)
  output_dir    (string) pasta onde as fotos serao salvas
                         (padrao: ~/dataset_images)
  num_images    (int)    quantidade total de fotos a capturar (padrao: 1000)
  interval      (double) intervalo minimo em segundos entre capturas
                         no modo automatico (padrao: 0.5)
  mode          (string) 'auto'   -> captura sozinho a cada "interval" segundos
                         'manual' -> so captura quando voce aperta ENTER
                                     no terminal (padrao: auto)
  prefix        (string) prefixo do nome do arquivo (padrao: img)
  image_format  (string) extensao/formato da imagem: jpg ou png (padrao: jpg)

Exemplo:
  ros2 run dataset_capture image_saver --ros-args \
      -p output_dir:=/home/robo/Pictures/Webcam \
      -p num_images:=1000 \
      -p mode:=manual OU auto
"""

import os
import threading

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class ImageSaver(Node):

    def __init__(self):
        super().__init__('image_saver')

        self.declare_parameter('image_topic', '/dataset_capture/image_raw')
        self.declare_parameter('output_dir', os.path.expanduser('~/dataset_images'))
        self.declare_parameter('num_images', 1000)
        self.declare_parameter('interval', 1.5)
        self.declare_parameter('mode', 'auto')
        self.declare_parameter('prefix', 'img')
        self.declare_parameter('image_format', 'jpg')

        self.output_dir = self.get_parameter('output_dir').value
        self.num_images = self.get_parameter('num_images').value
        self.interval = self.get_parameter('interval').value
        self.mode = self.get_parameter('mode').value
        self.prefix = self.get_parameter('prefix').value
        self.image_format = self.get_parameter('image_format').value
        image_topic = self.get_parameter('image_topic').value

        os.makedirs(self.output_dir, exist_ok=True)

        self.latest_frame = None
        self.count = self._count_existing_images()
        self.target_count = self.count + self.num_images
        self.lock = threading.Lock()
        self.capture_requested = False
        self.done = False

        self.subscription = self.create_subscription(
            Image, image_topic, self.image_callback, 10
        )

        self.get_logger().info(
            f'ImageSaver iniciado. Salvando em "{self.output_dir}", '
            f'modo="{self.mode}", meta={self.num_images} fotos '
            f'(ja existem {self.count} nesta pasta).'
        )

        if self.mode == 'manual':
            self.get_logger().info(
                'Modo MANUAL: pressione ENTER no terminal para tirar cada foto. '
                'Ctrl+C para encerrar antes do fim.'
            )
            self._start_manual_input_thread()
        else:
            self.get_logger().info(
                f'Modo AUTO: uma foto a cada {self.interval:.2f}s.'
            )
            self.timer = self.create_timer(self.interval, self._auto_capture_callback)

    def _count_existing_images(self):
        if not os.path.isdir(self.output_dir):
            return 0
        existing = [
            f for f in os.listdir(self.output_dir)
            if f.startswith(self.prefix) and f.endswith(self.image_format)
        ]
        return len(existing)

    def _start_manual_input_thread(self):
        def worker():
            while rclpy.ok() and not self.done:
                try:
                    input()
                except EOFError:
                    break
                with self.lock:
                    self.capture_requested = True

        thread = threading.Thread(target=worker, daemon=True)
        thread.start()

    def image_callback(self, msg):
        channels = 1 if 'mono' in msg.encoding else 3
        cv_image = np.frombuffer(msg.data, dtype=np.uint8).reshape(
            msg.height, msg.width, channels
        )
        self.latest_frame = cv_image

        if self.mode == 'manual':
            with self.lock:
                should_capture = self.capture_requested
                self.capture_requested = False
            if should_capture:
                self._save_frame(cv_image)

    def _auto_capture_callback(self):
        if self.latest_frame is not None:
            self._save_frame(self.latest_frame)

    def _save_frame(self, frame):
        if self.done:
            return

        filename = f'{self.prefix}_{self.count:05d}.{self.image_format}'
        filepath = os.path.join(self.output_dir, filename)
        cv2.imwrite(filepath, frame)
        self.count += 1

        self.get_logger().info(f'[{self.count}/{self.target_count}] Salvo: {filename}')

        if self.count >= self.target_count:
            self.done = True
            self.get_logger().info(
                f'Meta atingida! {self.num_images} fotos salvas em "{self.output_dir}". '
                f'Encerrando o no.'
            )
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = ImageSaver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
