import rclpy
from rclpy.node import Node

import cv2
from ultralytics import YOLO
import os
import numpy as np
from math import hypot
import time
    
from custom_interfaces.msg import Vision
from vision_msgs.msg import Point2D

from .submodules.utils          import draw_lines, position, recise_image
from .submodules.ClassConfig    import *
from .submodules.Client         import Client

class BallDetection(Node):
    def __init__(self):

        super().__init__("image_node")

        #params
        self.declare_parameter("device", "cpu")
        self.device = self.get_parameter("device").get_parameter_value().string_value

        self.declare_parameter("show_divisions", True) # Show division lines and center of ball in output image
        self.show_divisions = self.get_parameter("show_divisions").get_parameter_value().bool_value

        self.declare_parameter("get_image", True) #
        self.get_image = self.get_parameter("get_image").get_parameter_value().bool_value

        self.declare_parameter("img_qlty", 70) # 
        self.img_qlty = self.get_parameter("img_qlty").get_parameter_value().integer_value

        self.declare_parameter("server_ip", "192.168.7.10")
        self.server_ip = self.get_parameter("server_ip").get_parameter_value().string_value

        self.declare_parameter("server_port", 5050)
        self.server_port = self.get_parameter("server_port").get_parameter_value().integer_value

        self.declare_parameter("model", f"{os.path.dirname(os.path.realpath(__file__))}/weights/best.pt")
        self.model = YOLO(self.get_parameter("model").get_parameter_value().string_value) #Load Model
        
        self.original_dim = np.array([640, 480])
        self.redued_dim = self.original_dim * self.img_qlty / 100
        self.value_classes = self.get_classes()

        self.cap = cv2.VideoCapture("/dev/video0")
        self.cap.set(3, self.original_dim[0])
        self.cap.set(4, self.original_dim[1])
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
        self.cap.set(cv2.CAP_PROP_EXPOSURE, -4) 
        

        self.timer = self.create_timer(0.008, self.main_callbalck)

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

        self.client = Client(self.server_ip, self.server_port)

    def __del__(self):
        self.client.close_socket()

    def get_classes(self): #function for list all classes and the respective number in a dictionary
        fake_image = np.zeros((640,480,3), dtype=np.uint8)
        result = self.model(fake_image, device=self.device, verbose=False, imgsz=list(self.redued_dim))
        classes = result[0].names
        value_classes = {value: key for key, value in classes.items()}
        return value_classes   

    def main_callbalck(self):

        ret, self.img = self.cap.read()


        if(ret):
            if(self.get_image and (time.time() - self.old_time > 0.5)):
                self.get_logger().info("entrou")
                self.old_time = time.time()
                cv2.imwrite(f"/home/robo/Desktop/novo_dataset/new_ball{self.foto_count:04d}.jpg", self.img)
                self.foto_count += 1

            self.results = self.predict_image(recise_image(self.img, self.img_qlty)) # predict image 

            if self.show_divisions:
                self.img = draw_lines(self.img, self.config)  #Draw camera divisions

            self.ball_detection()
            
            self.client.send_image(self.img)
            #cv2.imshow('Ball', self.img) # Show image
            #cv2.waitKey(1)
    
    def predict_image(self, img):
        results = self.model(img, device=self.device, conf=0.7, max_det=3, verbose=False, imgsz=img.shape[:2])
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
            self.get_logger().info("Bola à Esquerda")

        elif (ball_px_pos.x < self.config.x_center):  #ball to the center left
            ball_pos.center_left = True
            self.get_logger().info("Bola Centralizada a Esquerda")

        elif (ball_px_pos.x < self.config.x_right):   #ball to the center right
            ball_pos.center_right = True
            self.get_logger().info("Bola Centralizada a Direita")

        else:                                            #ball to the right
            ball_pos.right = True
            self.get_logger().info("Bola à Direita")
        

        # identify the ball position in Y axis
        if (ball_px_pos.y > self.config.y_chute):     #ball near
            ball_pos.close = True
            self.get_logger().info("Bola Perto")
        
        elif (ball_px_pos.y <= self.config.y_longe):  #ball far
            ball_pos.far = True
            self.get_logger().info("Bola Longe")

        else:                                           #Bola middle
            ball_pos.med = True
            self.get_logger().info("Bola ao Centro")

        return ball_pos
        
    def ball_delta_position_threshold(self, new_position, threshold):
        dp = position()
        dp.x = abs(new_position.x - self.ball_pos.x)
        dp.y = abs(new_position.y - self.ball_pos.y)

        return hypot(dp.x, dp.y) < threshold
            
def main(args=None):
    rclpy.init(args=args)


    ball_detection = BallDetection()
    rclpy.spin(ball_detection)

    ball_detection.destroy_node()
    rclpy.shutdown()

    cv2.destroyAllWindows()
 

if __name__ == '__main__':
    main()
