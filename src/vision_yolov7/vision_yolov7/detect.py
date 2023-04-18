####################################################################################################################################
# ros2 run vision_pkg vision
#
# Para ver o que a camera esta vendo:
# ros2 run vision_yolov7 detect --source 2 
#
# ros2 topic pub -1 /neck_position custom_interfaces/NeckPosition "{position19: 2047, position20: 1050}"
####################################################################################################################################
from telnetlib import NOP
import rclpy
from rclpy.node import Node

#from std_msgs.msg import String
from custom_interfaces.msg import Vision
from custom_interfaces.msg import VisionRobot


import sys
sys.path.insert(0, './src/vision_yolov7/vision_yolov7')
import numpy as np
import cv2
import ctypes
from math import log,exp,tan,radians
#import imutils

from .ClassConfig import *

try:
    """There are differences in versions of the config parser
    For versions > 3.0 """
    from configparser import ConfigParser
except ImportError:
    """For versions < 3.0 """
    from configparser import ConfigParser 

import argparse
import time
from pathlib import Path

import torch
import torch.backends.cudnn as cudnn
from numpy import random


from experimental import attempt_load
from utils.datasets import LoadStreams, LoadImages
from utils.general import check_img_size, check_requirements, check_imshow, non_max_suppression, apply_classifier, \
    scale_coords, xyxy2xywh, strip_optimizer, set_logging, increment_path
from utils.plots import plot_one_box
from utils.torch_utils import select_device, load_classifier, time_synchronized, TracedModel
from models import *

PATH_TO_WEIGHTS = 'src/vision_yolov7/vision_yolov7/peso_tiny/best.pt'
THRESHOLD = 0.45
OUR_ROBOT_COLOR = "B" # B para azul

parser = argparse.ArgumentParser()
parser.add_argument('--weights', nargs='+', type=str, default='yolov7.pt', help='model.pt path(s)')
parser.add_argument('--source', type=str, default='/dev/camera', help='source')  # file/folder, 0 for webcam
parser.add_argument('--img-size', type=int, default=640, help='inference size (pixels)')
parser.add_argument('--conf-thres', type=float, default=0.25, help='object confidence threshold')
parser.add_argument('--iou-thres', type=float, default=0.45, help='IOU threshold for NMS')
parser.add_argument('--device', default='cpu', help='cuda device, i.e. 0 or 0,1,2,3 or cpu')
parser.add_argument('--view-img', action='store_true', help='display results')
parser.add_argument('--save-txt', action='store_true', help='save results to *.txt')
parser.add_argument('--save-conf', action='store_true', help='save confidences in --save-txt labels')
parser.add_argument('--nosave', action='store_true', help='do not save images/videos')
parser.add_argument('--classes', nargs='+', type=int, help='filter by class: --class 0, or --class 0 2 3')
parser.add_argument('--agnostic-nms', action='store_true', help='class-agnostic NMS')
parser.add_argument('--augment', action='store_true', help='augmented inference')
parser.add_argument('--update', action='store_true', help='update all models')
# parser.add_argument('--project', default='runs/detect', help='save results to project/name')
parser.add_argument('--name', default='exp', help='save results to project/name')
parser.add_argument('--exist-ok', action='store_true', help='existing project/name ok, do not increment')
parser.add_argument('--no-trace', action='store_true', help='don`t trace model')
opt = parser.parse_args()
print(opt)


class ballStatus(Node):

    def __init__(self, config):
        super().__init__('detect')
        self.config = config
        self.publisher_ = self.create_publisher(Vision, '/ball_position', 10)
        self.publisher_robot_enemy = self.create_publisher(VisionRobot, '/robot_position_enemy', 10)
        self.publisher_robot_friend = self.create_publisher(VisionRobot, '/robot_position_friend', 10)
        timer_period = 0.008  # seconds
        self.cont_vision = 0
        self.weights = PATH_TO_WEIGHTS
        self.timer = self.create_timer(timer_period, self.detect())
        self.update = True


    def detect(self):
        msg_ball = Vision()
        msg_robot_friend = VisionRobot()
        msg_robot_enemy = VisionRobot()
        source, weights, view_img, save_txt, imgsz, trace = opt.source, self.weights, opt.view_img, opt.save_txt, opt.img_size, not opt.no_trace

        set_logging()
        device = select_device(opt.device)
        half = False

     # Load model
        model = attempt_load(weights, map_location=device)  # load FP32 model
        stride = int(model.stride.max())  # model stride
        imgsz = check_img_size(imgsz, s=stride)  # check img_size

        if trace:
            model = TracedModel(model, device, opt.img_size)

        if half:
            model.half()  # to FP16

        # Second-stage classifier
        classify = False
        if classify:
            modelc = load_classifier(name='resnet101', n=2)  # initialize
            modelc.load_state_dict(torch.load('weights/resnet101.pt', map_location=device)['model']).to(device).eval()

        # if webcam:
        view_img = check_imshow()
        cudnn.benchmark = True  # set True to speed up constant image size inference
        dataset = LoadStreams(source, img_size=imgsz, stride=stride)

        # Get names and colors
        names = model.module.names if hasattr(model, 'module') else model.names
        colors = [[random.randint(0, 255) for _ in range(3)] for _ in names]

        # Run inference
        if device.type != 'cpu':
            model(torch.zeros(1, 3, imgsz, imgsz).to(device).type_as(next(model.parameters())))  # run once
        old_img_w = old_img_h = imgsz
        old_img_b = 1

        t0 = time.time()

        for path, img, im0s, vid_cap in dataset:
            self.left = []
            img = torch.from_numpy(img).to(device)
            img = img.half() if half else img.float()  # uint8 to fp16/32
            img /= 255.0  # 0 - 255 to 0.0 - 1.0
            if img.ndimension() == 3:
                img = img.unsqueeze(0)

            # Warmup
            if device.type != 'cpu' and (old_img_b != img.shape[0] or old_img_h != img.shape[2] or old_img_w != img.shape[3]):
                old_img_b = img.shape[0]
                old_img_h = img.shape[2]
                old_img_w = img.shape[3]
                for i in range(3):
                    model(img, augment=opt.augment)[0]

            # Inference
            t1 = time_synchronized()
            pred = model(img, augment=opt.augment)[0]
            t2 = time_synchronized()

            # Apply NMS
            pred = non_max_suppression(pred, opt.conf_thres, opt.iou_thres, classes=opt.classes, agnostic=opt.agnostic_nms)
            t3 = time_synchronized()

            # Apply Classifier
            if classify:
                pred = apply_classifier(pred, modelc, img, im0s)

            # Process detections
            im0, frame = im0s[0].copy(), dataset.count
            
            det = pred[0]
            # Directories
            p = path[0]
            p = Path(p)  # to Path
            save_img = False
            save_txt = False
            # txt_path = str(save_dir / 'labels' / p.stem) + ('' if dataset.mode == 'image' else f'_{frame}')  # img.txt

            gn = torch.tensor(im0.shape)[[1, 0, 1, 0]]  # normalization gain whwh
            s=0
            msg_robot_friend.detected =False
            msg_robot_friend.left = 0
            msg_robot_friend.center_left = 0
            msg_robot_friend.center_right = 0
            msg_robot_friend.right = 0
            msg_robot_friend.med = 0
            msg_robot_friend.far = 0
            msg_robot_friend.close = 0
            msg_robot_enemy.detected =False
            msg_robot_enemy.left = 0
            msg_robot_enemy.center_left = 0
            msg_robot_enemy.center_right = 0
            msg_robot_enemy.right = 0
            msg_robot_enemy.med = 0
            msg_robot_enemy.far = 0
            msg_robot_enemy.close = 0
            msg_ball.detected = False
            msg_ball.left = False
            msg_ball.center_left = False
            msg_ball.center_right = False
            msg_ball.right = False
            msg_ball.med = False
            msg_ball.far = False
            msg_ball.close = False
            img_robots = list()
            conf_list = list()
            label_list = dict()

            if len(det): # Entra aqui se detectou a bola ou robô
                # Rescale boxes from img_size to im0 size
                det[:, :4] = scale_coords(img.shape[2:], det[:, :4], im0.shape).round()
                s=1
                # Write results
                print("AAAAAAAAAAAAAAAAAAA")
                for *xyxy, conf, cls in reversed(det):
                    print("BBBBBBBBBBBBBBBBB")
                    if save_txt:  # Write to file
                        xywh = (xyxy2xywh(torch.tensor(xyxy).view(1, 4)) / gn).view(-1).tolist()  # normalized xywh
                        line = (cls, *xywh, conf) if opt.save_conf else (cls, *xywh)  # label format
                        with open(txt_path + '.txt', 'a') as f:
                            f.write(('%g ' * len(line)).rstrip() % line + '\n')

                    if save_img or view_img:  # Add bbox to image
                        label = f'{names[int(cls)]} {conf:.2f}'
                        label_name = names[int(cls)]
                        
                        if conf>THRESHOLD:
                            if label_name == "ball":
                                # plot_one_box(xyxy, im0, label=label, color=colors[int(cls)], line_thickness=1)

                                label_list['ball'] = [xyxy, im0]

                                c1_ball = (xyxy[0] + xyxy[2]) / 2
                                c2_ball = (xyxy[1] +  xyxy[3]) / 2
                                raio_ball = (xyxy[2] - xyxy[0]) / 2
                                cv2.circle(im0, (int(c1_ball), int(c2_ball)), int(abs(raio_ball)), (255, 0, 0), 2)

                                msg_ball.detected = True
                                print("Bola detectada '%s'" % msg_ball.detected)
                                    #Bola a esquerda
                                if (int(c1_ball) <= self.config.x_left):
                                    msg_ball.left = True
                                    print("Bola à Esquerda")

                                #Bola centro esquerda
                                elif (int(c1_ball) > self.config.x_left and int(c1_ball) < self.config.x_center):
                                    msg_ball.center_left = True
                                    print("Bola Centralizada a Esquerda")

                                #Bola centro direita
                                elif (int(c1_ball) < self.config.x_right and int(c1_ball) > self.config.x_center):
                                    msg_ball.center_right = True
                                    print("Bola Centralizada a Direita")

                                #Bola a direita
                                else:
                                    msg_ball.right = True
                                    print("Bola à Direita")
                                
                                #Bola Perto
                                if (int(c2_ball) > self.config.y_chute):
                                    msg_ball.close = True
                                    print("Bola Perto")

                                #Bola Longe
                                elif (int(c2_ball) <= self.config.y_longe):
                                    msg_ball.far = True
                                    print("Bola Longe")

                                #Bola ao centro
                                # elif (int(c2) > self.config.y_longe and int(c2) < self.config.y_chute):
                                else:
                                    msg_ball.med = True
                                    print("Bola ao Centro")


                            if label_name == "robot":
                                print("OOOOOOOOOOOO")
                                label_list.setdefault("robot", [])
                                # plot_one_box(xyxy, im0, label=label, color=colors[int(cls)], line_thickness=1) 
                                c1_robot = (xyxy[0] + xyxy[2]) / 2
                                c2_robot = (xyxy[1] +  xyxy[3]) / 2
                                raio_robot = (xyxy[2] - xyxy[0]) / 2

                                robot_img = im0[int(xyxy[1]):int(xyxy[3]), int(xyxy[0]):int(xyxy[2])]

                                hsv = cv2.cvtColor(robot_img, cv2.COLOR_BGR2HSV) 
                                lower_blue = np.array([85, 100, 100])
                                upper_blue = np.array([130, 255, 255])
                                lower_red = np.array([160, 100, 100])
                                upper_red = np.array([179, 255, 255])

                                mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)
                                mask_red = cv2.inRange(hsv, lower_red, upper_red)
                                
                                mask_blue = np.array(mask_blue/mask_blue.max())
                                mask_red = np.array(mask_red/mask_red.max())

                                mask_blue[np.isnan(mask_blue)] = 0
                                mask_red[np.isnan(mask_red)] = 0 

                                blue_sum = np.sum(mask_blue) 
                                red_sum = np.sum(mask_red)

                                img_robots.append([mask_blue, mask_red])
                                conf_list.append(conf)
                                
                                print("BLUE: " + str(blue_sum) + "CONF: " + str(conf))
                                print("RED: " + str(red_sum) + "CONF: " + str(conf))
                                

                                if (blue_sum > red_sum and OUR_ROBOT_COLOR.upper() == "B") or (red_sum>blue_sum and OUR_ROBOT_COLOR.upper() != "B"):
                                    msg_robot_friend.detected = True
                                    
                                    label_list['robot'].append([xyxy,im0, f'friend robot {conf:.2f}'])

                                    print("Robô Amigo detectado '%s'" % msg_robot_friend.detected)
                                        #Bola a esquerda
                                    if (int(c1_robot) <= self.config.x_left):
                                        msg_robot_friend.left+=1
                                        print("Robô Amigo à Esquerda")

                                    #Bola centro esquerda
                                    elif (int(c1_robot) > self.config.x_left and int(c1_robot) < self.config.x_center):
                                        msg_robot_friend.center_left+=1
                                        print("Robô Amigo Centralizada à Esquerda")

                                    #Bola centro direita
                                    elif (int(c1_robot) < self.config.x_right and int(c1_robot) > self.config.x_center):
                                        msg_robot_friend.center_right+=1
                                        print("Robô Amigo Centralizada à Direita")

                                    #Bola a direita
                                    else:
                                        msg_robot_friend.right+=1
                                        print("Robô Amigo à Direita")
                                    
                                    #Bola Perto
                                    if (int(c2_robot) > self.config.y_chute):
                                        msg_robot_friend.close+=1
                                        print("Robô Amigo Perto")

                                    #Bola Longe
                                    elif (int(c2_robot) <= self.config.y_longe):
                                        msg_robot_friend.far+=1
                                        print("Robô Amigo Longe")

                                    #Bola ao centro
                                    # elif (int(c2) > self.config.y_longe and int(c2) < self.config.y_chute):
                                    else:
                                        msg_robot_friend.med+=1
                                        print("Robô Amigo ao Centro")
                                else:
                                    msg_robot_enemy.detected = True
                                    
                                    label_list['robot'].append([xyxy,im0, f'enemy robot {conf:.2f}'])

                                    print("Robô Inimigo detectado '%s'" % msg_robot_enemy.detected)
                                        #Bola a esquerda
                                    if (int(c1_robot) <= self.config.x_left):
                                        msg_robot_enemy.left+=1
                                        print("Robô Inimigo à Esquerda")

                                    #Bola centro esquerda
                                    elif (int(c1_robot) > self.config.x_left and int(c1_robot) < self.config.x_center):
                                        msg_robot_enemy.center_left+=1
                                        print("Robô Inimigo Centralizada à Esquerda")

                                    #Bola centro direita
                                    elif (int(c1_robot) < self.config.x_right and int(c1_robot) > self.config.x_center):
                                        msg_robot_enemy.center_right+=1
                                        print("Robô Inimigo Centralizada à Direita")

                                    #Bola a direita
                                    else:
                                        msg_robot_enemy.right+=1
                                        print("Robô Inimigo à Direita")
                                    
                                    #Bola Perto
                                    if (int(c2_robot) > self.config.y_chute):
                                        msg_robot_enemy.close+=1
                                        print("Robô Inimigo Perto")

                                    #Bola Longe
                                    elif (int(c2_robot) <= self.config.y_longe):
                                        msg_robot_enemy.far+=1
                                        print("Robô Inimigo Longe")

                                    #Bola ao centro
                                    else:
                                        msg_robot_enemy.med+=1
                                        print("Robô Inimigo ao Centro")
                                    

                                
            self.publisher_.publish(msg_ball)
                
            self.publisher_robot_enemy.publish(msg_robot_enemy)
            self.publisher_robot_friend.publish(msg_robot_friend)

            # Print time (inference + NMS)
            print(f'{s}Done. ({(1E3 * (t2 - t1)):.1f}ms) Inference, ({(1E3 * (t3 - t2)):.1f}ms) NMS')

            # Stream results
            # winName = 'Prometheus'
            # cv2.namedWindow(winName, cv2.WINDOW_NORMAL)
            # cv2.setWindowProperty(winName, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
            if view_img:
                # im0 = cv2.resize(im0, (700, 900))
                if 'ball' in label_list:
                    plot_one_box(label_list["ball"][0], label_list["ball"][1], label='ball', color=colors[int(cls)], line_thickness=1)
                if 'robot' in label_list:
                    for i in range(len(label_list["robot"])):
                        plot_one_box(label_list["robot"][i][0], label_list["robot"][i][1], label=label_list["robot"][i][2], color=colors[int(cls)], line_thickness=1)
                cv2.imshow("RoboFEI", im0)
                # if len(img_robots) > 0:
                #     for i in range(len(img_robots)):
                #         print("BLUE AREA : " + str(np.sum(img_robots[i][0])) + "CONF: " + str(conf_list[i]))
                #         print("RED AREA: " + str(np.sum(img_robots[i][1])) + "CONF: " + str(conf_list[i]))
                #         name_blue = 'BLUE ' + str(i+1)
                #         name_red = 'RED ' + str(i+1)
                #         cv2.imshow(name_blue, img_robots[i][0])
                #         cv2.imshow(name_red, img_robots[i][1])

                if cv2.waitKey(25) & 0xFF == ord('q'):
                    NOP
                    
            # print(self.status)

            #print(f'Done. ({time.time() - t0:.3f}s)')

            

def main(args=None):
    rclpy.init(args=args)

    config = classConfig()
    
    ballS = ballStatus(config)
    
    rclpy.spin(ballS)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    ballS.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
