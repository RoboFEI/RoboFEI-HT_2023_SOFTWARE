#!/usr/bin/python3
import rclpy
from rclpy.node import Node
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class CameraPublisher(Node):
    def __init__(self):
        super().__init__("camera_publisher")

        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(2)

        self.pub = self.create_publisher(Image, "/image", 10)

    def run(self):
        while True:
            try:
                r, frame = self.cap.read()
                self.get_logger().info(f'Publishing', once=True)
                if not r:
                    return
                self.pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))
                
            except CvBridgeError as e:
                print(e)
        self.cap.release()

def main(args=None):
    rclpy.init(args=args)

    camera_publisher = CameraPublisher()
    camera_publisher.run()

if __name__ == '__main__':
    main()