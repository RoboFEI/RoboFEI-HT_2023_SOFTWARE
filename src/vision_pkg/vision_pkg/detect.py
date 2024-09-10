import rclpy
from rclpy.node import Node

import cv2
from ultralytics import YOLO
import os
import numpy as np
from math import hypot
import time
import datetime
from cv_bridge import CvBridge
import torch

from custom_interfaces.msg import Vision
from vision_msgs.msg import Point2D
from sensor_msgs.msg import Image

from .submodules.utils          import draw_lines, position, resize_image
from .submodules.ClassConfig    import *
from .submodules.Client         import Client

class BallDetection(Node):
    def __init__(self):

        super().__init__("image_node")

        #PARAMS
        #===============================================================================================================
        # \ \ / / _ \| |   / _ \ 
        #  \ V / | | | |  | | | |
        #   | || |_| | |__| |_| |
        #   |_| \___/|_____\___/ 

        self.declare_parameter("device", "cpu")
        self.device = self.get_parameter("device").get_parameter_value().string_value
        
        self.declare_parameter("model", f"{os.path.dirname(os.path.realpath(__file__))}/weights/best_openvino_model/")
        self.model = YOLO(self.get_parameter("model").get_parameter_value().string_value) #Load Model

        #   ____                               
        #  / ___|__ _ _ __ ___   ___ _ __ __ _ 
        # | |   / _` | '_ ` _ \ / _ \ '__/ _` |
        # | |__| (_| | | | | | |  __/ | | (_| |
        #  \____\__,_|_| |_| |_|\___|_|  \__,_|

        self.declare_parameter("img_qlty", 100) 
        self.img_qlty = self.get_parameter("img_qlty").get_parameter_value().integer_value

        self.declare_parameter("image_width", 1280)
        self.img_width = self.get_parameter("image_width").get_parameter_value().integer_value

        self.declare_parameter("image_height", 720)
        self.img_height = self.get_parameter("image_height").get_parameter_value().integer_value

        #  ____
        # / ___|  ___ _ ____   _____ _ __ 
        # \___ \ / _ \ '__\ \ / / _ \ '__|
        #  ___) |  __/ |   \ V /  __/ |    
        # |____/ \___|_|    \_/ \___|_|   

        self.declare_parameter("enable_udp", True)
        self.enable_udp = self.get_parameter("enable_udp").get_parameter_value().bool_value

        self.declare_parameter("server_ip", "192.168.7.10")
        self.server_ip = self.get_parameter("server_ip").get_parameter_value().string_value

        self.declare_parameter("server_port", 5050)
        self.server_port = self.get_parameter("server_port").get_parameter_value().integer_value

        #   ___  _   _                   
        #  / _ \| |_| |__   ___ _ __ ___ 
        # | | | | __| '_ \ / _ \ '__/ __|
        # | |_| | |_| | | |  __/ |  \__ \
        #  \___/ \__|_| |_|\___|_|  |___/

        self.declare_parameter("show_divisions", True) # Show division lines and center of ball in output image
        self.show_divisions = self.get_parameter("show_divisions").get_parameter_value().bool_value

        self.declare_parameter("get_image", False) #
        self.get_image = self.get_parameter("get_image").get_parameter_value().bool_value
        
        self.declare_parameter("fps_save", 2) 
        self.fps_save = self.get_parameter("fps_save").get_parameter_value().integer_value
        #===============================================================================================================
        

        self.original_dim = np.array([self.img_width, self.img_height])
        self.redued_dim = self.original_dim * self.img_qlty / 100
        self.value_classes = self.get_classes()
        
        self.bridge = CvBridge()
        self.raw_image_sub_ = self.create_subscription(Image, "/image_raw", self.image_callback, 1)
        self.raw_image_sub_

        self.ball_position_publisher_ = self.create_publisher(Vision, '/ball_position', 2)
        self.ball_position_publisher_
        
        self.ball_px_position_publisher_ = self.create_publisher(Point2D, '/ball_px_position', 2)
        self.ball_px_position_publisher_

        #recive data from config.ini using the ClassConfig submodule
        self.config = classConfig()

        self.results = None
        self.img = None

        self.ball_pos = position() #apagar
        self.filtered_ball_position = Point2D()

        self.ball_pos_area = Vision()
        
        self.cont_real_detections = 0

        self.old_time = time.time()
        self.foto_count = 0

        if self.enable_udp:
            self.client = Client(self.server_ip, self.server_port)

        if self.get_image: 
            today = datetime.datetime.now()
            self.vision_log_path = f'vision_log/{today.day}_{today.month}_{today.year}-{today.hour}_{today.minute}'
            os.makedirs(self.vision_log_path, exist_ok=True)

    def __del__(self):
        self.client.close_socket()

    def get_classes(self): #function for list all classes and the respective number in a dictionary
        classes = self.model.names
        value_classes = {value: key for key, value in classes.items()}
        return value_classes   
    
    def image_callback(self, msg):
        self.img = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        if self.get_image:
            self.save_image(self.img)

        self.results = self.predict_image(resize_image(self.img, self.img_qlty)) # predict image 

        if self.show_divisions:
            self.img = draw_lines(self.img, self.config)  #Draw camera divisions

        self.ball_detection()

        if self.enable_udp:
            self.client.send_image(self.img)
             
        cv2.imshow('Ball', self.img) # Show image
        cv2.waitKey(1)

    
    def predict_image(self, img):
        results = self.model(img, device=self.device, conf=0.3, max_det=3, verbose=True)        
        return results[0]

    def ball_detection(self):
        ball_px_pos = self.find_ball() # recive x and y position (Point2D)

        new_ball_pos_area = Vision()

        if ball_px_pos != -1: #if ball was finded
            ball_px_pos = self.ball_px_position_filter(ball_px_pos, 0)
            new_ball_pos_area = self.get_ball_pos_area(ball_px_pos)

        self.ball_pos_area_filter(new_ball_pos_area, 1)

    def find_ball(self):
        ball_detection = (self.results.boxes.cls == self.value_classes['ball']).nonzero(as_tuple=True)[0].numpy()
        
        if ball_detection.size > 0:
            ball_pos = Point2D()

            ball_box_xywh = self.results.boxes[ball_detection[0]].xywhn.numpy() #get the most conf ball detect box in xyxy format
            array_box_xywh = np.reshape(ball_box_xywh, -1)  #convert matriz to array

            ball_pos.x = float(array_box_xywh[0] * self.img.shape[1])
            ball_pos.y = float(array_box_xywh[1] * self.img.shape[0])
            
            raio_ball       = int((array_box_xywh[2] * self.img.shape[1] +array_box_xywh[3] * self.img.shape[0]) / 4)

            cv2.circle(self.img, (int(ball_pos.x), int(ball_pos.y)), abs(raio_ball), (255, 0, 0), 2)
            cv2.circle(self.img, (int(ball_pos.x), int(ball_pos.y)), 5, (255, 0, 0), -1)
            return ball_pos

        return -1

    def ball_px_position_filter(self, not_filtered_ball_pos, opt):
        
        ball_px_pos = not_filtered_ball_pos

        if opt == 0:
            pass
        
        self.ball_px_position_publisher_.publish(ball_px_pos)
        return ball_px_pos

    def ball_pos_area_filter(self, not_filtered_ball_pos, opt):
        if opt == 0:
            self.ball_pos_area = not_filtered_ball_pos
        
        elif opt == 1:
            if not_filtered_ball_pos == self.ball_pos_area:
                self.cont_real_detections += 1
            else:
                self.cont_real_detections = 0
                self.ball_pos_area = not_filtered_ball_pos

            
            if self.cont_real_detections > 2:
                self.ball_position_publisher_.publish(self.ball_pos_area)

    def get_ball_pos_area(self, ball_px_pos):
        ball_pos = Vision()
        ball_pos.detected = True

        # identify the ball position in Y axis
        if (ball_px_pos.x <= self.config.x_left):     #ball to the left
            ball_pos.left = True
            self.get_logger().debug("Bola à Esquerda")

        elif (ball_px_pos.x < self.config.x_center):  #ball to the center left
            ball_pos.center_left = True
            self.get_logger().debug("Bola Centralizada a Esquerda")

        elif (ball_px_pos.x < self.config.x_right):   #ball to the center right
            ball_pos.center_right = True
            self.get_logger().debug("Bola Centralizada a Direita")

        else:                                            #ball to the right
            ball_pos.right = True
            self.get_logger().debug("Bola à Direita")
        

        # identify the ball position in Y axis
        if (ball_px_pos.y > self.config.y_chute):     #ball near
            ball_pos.close = True
            self.get_logger().debug("Bola Perto")
        
        elif (ball_px_pos.y <= self.config.y_longe):  #ball far
            ball_pos.far = True
            self.get_logger().debug("Bola Longe")

        else:                                           #Bola middle
            ball_pos.med = True
            self.get_logger().debug("Bola ao Centro")

        return ball_pos
        
    def ball_delta_position_threshold(self, new_position, threshold):
        dp = position()
        dp.x = abs(new_position.x - self.ball_pos.x)
        dp.y = abs(new_position.y - self.ball_pos.y)

        return hypot(dp.x, dp.y) < threshold

    def save_image(self, img):
        if time.time() - self.old_time > 1.0/self.fps_save:
            self.old_time = time.time()
            file_name = f"/ball_photo{self.foto_count:04d}.jpg"
            cv2.imwrite(self.vision_log_path+file_name, img)
            self.foto_count += 1
            
def main(args=None):
    rclpy.init(args=args)


    ball_detection = BallDetection()
    rclpy.spin(ball_detection)

    ball_detection.destroy_node()
    rclpy.shutdown()

    cv2.destroyAllWindows()
 

if __name__ == '__main__':
    main()


# #!/usr/bin/python3
# import rclpy
# from rclpy.node import Node
# import cv2
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge

# class ImageSubscriber(Node):
#     def __init__(self):
#         super().__init__("image_subscriber")
#         self.bridge = CvBridge()

#         self.sub = self.create_subscription(Image, "/image_raw", self.image_callback, 1)
#         self.sub

#     def image_callback(self, msg):
#         cv2.imshow('Imagem', self.bridge.imgmsg_to_cv2(msg, "bgr8"))
#         cv2.waitKey(1)

# def main(args=None):
#     rclpy.init(args=args)

#     image_subscriber = ImageSubscriber()
#     rclpy.spin(image_subscriber)

#     image_subscriber.destroy_node()
#     rclpy.shutdown()

#     cv2.destroyAllWindows()

# if __name__ == '__main__':
#     main()