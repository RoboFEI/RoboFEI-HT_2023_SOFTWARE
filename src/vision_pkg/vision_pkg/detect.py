import rclpy
from rclpy.node import Node
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
import os
import numpy as np

from .submodules.utils import draw_boxes, draw_lines, locate_ball
from .submodules.ClassConfig import *

class BallDetection(Node):
    def __init__(self):

        super().__init__("image_subscriber")
        self.bridge = CvBridge()

        self.sub = self.create_subscription(Image, "/image", self.predict_callbalck, 10)
        self.sub
        
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

    def get_classes(self): #function for list all classes and the respective number in a dictionary
        fake_image = np.zeros((640,480,3), dtype=np.uint8)
        result = self.model(fake_image, device=self.device, verbose=False)
        classes = result[0].names
        value_classes = {value: key for key, value in classes.items()}
        return value_classes


    def label_img(self): #Draw lines and object boxes detected
        img_lined = draw_lines(self.img, self.config)
        labeled_img = draw_boxes(img_lined, self.results[0], self.value_classes)
        return labeled_img


    def predict_callbalck(self, msg):
        self.img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        self.results = self.model(self.img, device=self.device, conf=0.5, max_det=3, verbose=False)


        if self.show_divisions:
            self.img = self.label_img()
        else:
            self.img = draw_boxes(self.img, self.results[0])

        locate_ball(self.img, self.results)

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
