#!/usr/bin/env python3
import numpy as np
import torch
import cv2
import matplotlib.pyplot as plt

import sys
sys.path.insert(0, './src/vision_vitdet/vision_vitdet/detectron2')

from detectron2.checkpoint import DetectionCheckpointer
from detectron2.config import LazyConfig, instantiate
from detectron2.data.detection_utils import read_image
from detectron2.data import MetadataCatalog
from detectron2.utils.visualizer import ColorMode
from detectron2.data.datasets import register_coco_instances
from contextlib import ExitStack, contextmanager
from detectron2.engine.defaults import create_ddp_model

import glob
import time

import rclpy
from rclpy.node import Node
from custom_interfaces.msg import Vision

from matplotlib.animation import FuncAnimation

from .camvideostream import WebcamVideoStream

from .ClassConfig import *
try:
    """There are differences in versions of the config parser
    For versions > 3.0 """
    from configparser import ConfigParser
except ImportError:
    """For versions < 3.0 """
    from configparser import ConfigParser 

threshold = 0.45

cfg = LazyConfig.load("./src/vision_vitdet/vision_vitdet/detectron2/projects/ViTDet/configs/COCO/mask_rcnn_vitdet_b_100ep.py")

# edit the config to utilize common Batch Norm
cfg.model.backbone.norm = "BN"
cfg.model.roi_heads.num_classes = 2

register_coco_instances("ball_robot", {},"./src/vision_vitdet/vision_vitdet/detectron2/Telstar_Mechta/test/_annotations.coco.json", "./src/vision_vitdet/vision_vitdet/detectron2/Telstar_Mechta/test")
MetadataCatalog.get("ball_robot").thing_classes = ['ball', 'robot']
MetadataCatalog.get("ball_robot").set(thing_classes=['ball', 'robot'])

cfg.train.device = "cpu"
model = instantiate(cfg.model)
model.to(cfg.train.device)
model = create_ddp_model(model)
DetectionCheckpointer(model).load("./src/vision_vitdet/vision_vitdet/detectron2/model_final_mask.pth")  # load a file, usually from cfg.MODEL.WEIGHTS


class Detect(Node):

    def __init__(self, config):
        super().__init__('detect')
        self.get_logger().info('Running Vision VitDet Node')
        self.config = config
        self.publisher_ = self.create_publisher(Vision, '/ball_position', 10)
        self.publisher_robot = self.create_publisher(Vision, '/robot_position', 10)
        self.ball = False
        self.robot = False
        self.vcap = WebcamVideoStream(src="/dev/camera").start() # Abrindo camera
        self.timer=self.create_timer(0.008,self.timer_callback)

    def timer_callback(self):
        print("Inside detect()")
        msg_ball=Vision()
        msg_robot=Vision()

        start_timer = time.time()
        img = self.vcap.read()
        img = cv2.resize(img, (1024, 1024))
        
        img2 = img
        img2 = torch.from_numpy(np.ascontiguousarray(img))
        img2 = img2.permute(2, 0, 1)  # HWC -> CHW
        inputs = [{"image": img2}]

        # run the model
        model.eval()
        with torch.no_grad():
            predictions_ls = model(inputs)
        predictions = predictions_ls[0]

        indices = predictions['instances'].get_fields()['pred_classes'].to("cpu").numpy()
        classes = MetadataCatalog.get("ball_robot").thing_classes
        labels = [classes[idx] for idx in indices] 

        self.ball = False
        self.robot = False
        msg_ball.detected =False
        msg_ball.left = False
        msg_ball.center_left = False
        msg_ball.center_right = False
        msg_ball.right = False
        msg_ball.med = False
        msg_ball.far = False
        msg_ball.close = False
        msg_robot.detected =False
        msg_robot.left = False
        msg_robot.center_left = False
        msg_robot.center_right = False
        msg_robot.right = False
        msg_robot.med = False
        msg_robot.far = False
        msg_robot.close = False

        for i in range(len(predictions["instances"])):
            score = predictions["instances"][i].get("scores").item()
            if (score>threshold):
                if labels[i]=='ball':
                    self.ball = True
                    box_ball = predictions["instances"][i].get("pred_boxes")
                    box_tensor_ball= box_ball.tensor[0]
                    x_inicial_ball = box_tensor_ball.data[0].item()
                    y_inicial_ball = box_tensor_ball.data[1].item()
                    x_final_ball = box_tensor_ball.data[2].item()
                    y_final_ball = box_tensor_ball.data[3].item()
                    start_ball = (int(x_inicial_ball), int(y_inicial_ball))
                    final_ball = (int(x_final_ball), int(y_final_ball))
                    cv2.rectangle(img, start_ball, final_ball, (255, 0, 0), 3)
                    img = cv2.putText(img, labels[i], (int(x_inicial_ball), int(y_inicial_ball)-4), cv2.FONT_HERSHEY_SIMPLEX, 
                        0.5, (255, 0, 0), 2, cv2.LINE_AA)
                    img = cv2.putText(img, str(round(score,3)), (int(x_inicial_ball), int(y_final_ball)+14), cv2.FONT_HERSHEY_SIMPLEX, 
                        0.5, (255, 0, 0), 2, cv2.LINE_AA)
                    x_ball = (int((x_final_ball + x_inicial_ball)/2))
                    y_ball = (int(y_final_ball + y_inicial_ball)/2)
                
                    
                if labels[i]=='robot':
                    self.robot = True
                    box_robot = predictions["instances"][i].get("pred_boxes")
                    box_tensor_robot= box_robot.tensor[0]
                    x_inicial_robot = box_tensor_robot.data[0].item()
                    y_inicial_robot = box_tensor_robot.data[1].item()
                    x_final_robot = box_tensor_robot.data[2].item()
                    y_final_robot = box_tensor_robot.data[3].item()
                    start_robot = (int(x_inicial_robot), int(y_inicial_robot))
                    final_robot = (int(x_final_robot), int(y_final_robot))
                    cv2.rectangle(img, start_robot, final_robot, (255, 0, 0), 3)
                    img = cv2.putText(img, labels[i], (int(x_inicial_robot), int(y_inicial_robot)-4), cv2.FONT_HERSHEY_SIMPLEX, 
                        0.5, (255, 0, 0), 2, cv2.LINE_AA)
                    img = cv2.putText(img, str(round(score,3)), (int(x_inicial_robot), int(y_final_robot)+14), cv2.FONT_HERSHEY_SIMPLEX, 
                        0.5, (255, 0, 0), 2, cv2.LINE_AA)
                    x_robot = (int(x_final_robot + x_inicial_robot)/2)
                    y_robot = (int(y_final_robot + y_inicial_robot)/2)
                
                if self.ball:
                    msg_ball.detected = True
                    print("Bola detectada '%s'" % msg_ball.detected)
                    #Bola a esquerda
                    if (int(x_ball) <= self.config.x_left):
                        msg_ball.left = True
                        self.publisher_.publish(msg_ball)
                        print("Bola à Esquerda")

                    #Bola centro esquerda
                    elif (int(x_ball) > self.config.x_left and int(x_ball) < self.config.x_center):
                        msg_ball.center_left = True
                        self.publisher_.publish(msg_ball)
                        print("Bola Centralizada a Esquerda")

                    #Bola centro direita
                    elif (int(x_ball) < self.config.x_right and int(x_ball) > self.config.x_center):
                        msg_ball.center_right = True
                        self.publisher_.publish(msg_ball)
                        print("Bola Centralizada a Direita")

                    #Bola a direita
                    else:
                        msg_ball.right = True
                        self.publisher_.publish(msg_ball)
                        print("Bola à Direita")
                    
                    #Bola Perto
                    if (int(y_ball) > self.config.y_chute):
                        msg_ball.close = True
                        self.publisher_.publish(msg_ball)
                        print("Bola Perto")

                    #Bola Longe
                    elif (int(y_ball) <= self.config.y_longe):
                        msg_ball.far = True
                        self.publisher_.publish(msg_ball)
                        print("Bola Longe")

                    #Bola ao centro
                    # elif (int(y) > self.config.y_longe and int(y) < self.config.y_chute):
                    else:
                        msg_ball.med = True
                        self.publisher_.publish(msg_ball)
                        print("Bola ao Centro")

                if self.robot:
                    msg_robot.detected = True
                    print("Robô detectada '%s'" % msg_robot.detected)
                        #Bola a esquerda
                    if (int(x_robot) <= self.config.x_left):
                        msg_robot.left = True
                        self.publisher_robot.publish(msg_robot)
                        print("Robô à Esquerda")

                    #Bola centro esquerda
                    elif (int(x_robot) > self.config.x_left and int(x_robot) < self.config.x_center):
                        msg_robot.center_left = True
                        self.publisher_robot.publish(msg_robot)
                        print("Robô Centralizada a Esquerda")

                    #Bola centro direita
                    elif (int(x_robot) < self.config.x_right and int(x_robot) > self.config.x_center):
                        msg_robot.center_right = True
                        self.publisher_robot.publish(msg_robot)
                        print("Robô Centralizada a Direita")

                    #Bola a direita
                    else:
                        msg_robot.right = True
                        self.publisher_robot.publish(msg_robot)
                        print("Robô à Direita")
                    
                    #Bola Perto
                    if (int(y_robot) > self.config.y_chute):
                        msg_robot.close = True
                        self.publisher_robot.publish(msg_robot)
                        print("Robô Perto")

                    #Bola Longe
                    elif (int(y_robot) <= self.config.y_longe):
                        msg_robot.far = True
                        self.publisher_robot.publish(msg_robot)
                        print("Robô Longe")

                    #Bola ao centro
                    # elif (int(y) > self.config.y_longe and int(y) < self.config.y_chute):
                    else:
                        msg_robot.med = True
                        self.publisher_robot.publish(msg_robot)
                        print("Robô ao Centro")
                    
        self.publisher_.publish(msg_ball)
        self.publisher_robot.publish(msg_robot)

        print(f'Time total: {time.time() - start_timer}')
        cv2.imshow("RoboFEI",img)
        
        
        if cv2.waitKey(25) and 0xFF == ord('q'):
            print("FINISHED SUCCESSFULLY!")
            cam.release()
            cv2.destroyWindow("RoboFEI")
        

def main(args=None):
    rclpy.init(args=args)

    config = classConfig()
    
    detection = Detect(config)
    
    rclpy.spin(detection)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    detection.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()


if __name__ == '__main__':
    main()



