import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge
from ultralytics import YOLO
import os
import numpy as np
from math import hypot

from sensor_msgs.msg import Image
from custom_interfaces.msg import Vision

from .submodules.utils import draw_lines, position
from .submodules.ClassConfig import *

class BallDetection(Node):
    def __init__(self):

        super().__init__("image_subscriber")
        self.bridge = CvBridge()

        self.sub = self.create_subscription(Image, "/image", self.predict_callbalck, 10)
        self.sub

        self.publisher_ = self.create_publisher(Vision, '/ball_position', 10)
        self.publisher_

        #recive data from config.ini using the ClassConfig submodule
        self.config = classConfig()
        
        #params
        self.declare_parameter("device", "cpu")
        self.device = self.get_parameter("device").get_parameter_value().string_value

        self.declare_parameter("show_divisions", True) # Show division lines and center of ball in output image
        self.show_divisions = self.get_parameter("show_divisions").get_parameter_value().bool_value

        self.declare_parameter("model", f"{os.path.dirname(os.path.realpath(__file__))}/weights/best.pt")
        self.model = YOLO(self.get_parameter("model").get_parameter_value().string_value) #Load Model
        self.value_classes = self.get_classes()

        self.results = None
        self.img = None

        self.ball_pos = position()
        self.filtered_ball_position = position()

        self.ball_info_msg = Vision()
        
        self.cont_falses_lost_ball = 0 
        self.cont_real_detections = 0

    def get_classes(self): #function for list all classes and the respective number in a dictionary
        fake_image = np.zeros((640,480,3), dtype=np.uint8)
        result = self.model(fake_image, device=self.device, verbose=False)
        classes = result[0].names
        value_classes = {value: key for key, value in classes.items()}
        return value_classes        


    def find_ball(self):
        ball_detection = (self.results.boxes.cls == self.value_classes['ball']).nonzero(as_tuple=True)[0].numpy()
        
        
        if ball_detection.size > 0:
            ball_pos = position()

            ball_box_xyxy = self.results.boxes[ball_detection[0]].xyxy.numpy() #get the most conf ball detect box in xyxy format
            array_box_xyxy = np.reshape(ball_box_xyxy, -1)  #convert matriz to array

            ball_pos.x = int((array_box_xyxy[0] + array_box_xyxy[2]) / 2)
            ball_pos.y = int((array_box_xyxy[1] + array_box_xyxy[3]) / 2)
            
            raio_ball       = int((array_box_xyxy[2] - array_box_xyxy[0]) / 2)

            cv2.circle(self.img, (ball_pos.x, ball_pos.y), abs(raio_ball), (255, 0, 0), 2)
            cv2.circle(self.img, (ball_pos.x, ball_pos.y), 5, (255, 0, 0), -1)
            return ball_pos

        return -1

    def ball_delta_position_threshold(self, new_position, threshold):
        dp = position()
        dp.x = abs(new_position.x - self.ball_pos.x)
        dp.y = abs(new_position.y - self.ball_pos.y)

        return hypot(dp.x, dp.y) < threshold

    def ball_info(self):
        ball_pos = self.find_ball()

        if ball_pos != -1: #if ball was finded
            if self.ball_delta_position_threshold(ball_pos, 50): #verify if is a false detection
                self.cont_real_detections += 1
            else:
                self.cont_real_detections = 0


            if self.cont_real_detections > 5:
                self.cont_falses_lost_ball = self.config.max_count_lost_frame
                self.cont_real_detections = 0

                self.filtered_ball_position.x = ball_pos.x
                self.filtered_ball_position.y = ball_pos.y

                self.ball_info_msg.detected = True

            self.ball_pos.x = ball_pos.x
            self.ball_pos.y = ball_pos.y

        elif self.cont_falses_lost_ball > 0: #after some detection with ball lost keep seting ball detected
            self.cont_falses_lost_ball -= 1

            self.ball_info_msg.detected = True
            self.get_logger().info(f'Bola Detectada, Falso negativo: {self.cont_falses_lost_ball}')
        else:
            self.ball_info_msg = Vision()

        # tentar colocar isso antes de tudo e atribuir o vision depois
        if self.ball_info_msg.detected:
            self.ball_info_msg = Vision()
            self.ball_info_msg.detected = True

            # identify the ball position in Y axis
            if (self.filtered_ball_position.x <= self.config.x_left):     #ball to the left
                self.ball_info_msg.left = True
                self.get_logger().info("Bola à Esquerda")

            elif (self.filtered_ball_position.x < self.config.x_center):  #ball to the center left
                self.ball_info_msg.center_left = True
                self.get_logger().info("Bola Centralizada a Esquerda")

            elif (self.filtered_ball_position.x < self.config.x_right):   #ball to the center right
                self.ball_info_msg.center_right = True
                self.get_logger().info("Bola Centralizada a Direita")

            else:                                            #ball to the right
                self.ball_info_msg.right = True
                self.get_logger().info("Bola à Direita")
            

            # identify the ball position in Y axis
            if (self.filtered_ball_position.y > self.config.y_chute):     #ball near
                self.ball_info_msg.close = True
                self.get_logger().info("Bola Perto")
            
            elif (self.filtered_ball_position.y <= self.config.y_longe):  #ball far
                self.ball_info_msg.far = True
                self.get_logger().info("Bola Longe")

            else:                                           #Bola middle
                self.ball_info_msg.med = True
                self.get_logger().info("Bola ao Centro")
            


    def publish_ball_info(self):
        self.ball_info()
        self.publisher_.publish(self.ball_info_msg)


    def predict_callbalck(self, msg):
        self.img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        self.results = self.model(self.img, device=self.device, conf=0.5, max_det=3, verbose=False)
        self.results = self.results[0]

    
        if self.show_divisions:
            self.img = draw_lines(self.img, self.config)  #Draw camera divisions

        self.publish_ball_info()
        
        
        cv2.imshow('Ball', self.img) # Draw divisions of camera
        cv2.waitKey(1)
        

def main(args=None):
    rclpy.init(args=args)


    ball_detection = BallDetection()
    rclpy.spin(ball_detection)

    ball_detection.destroy_node()
    rclpy.shutdown()

    cv2.destroyAllWindows()
 

if __name__ == '__main__':
    main()
