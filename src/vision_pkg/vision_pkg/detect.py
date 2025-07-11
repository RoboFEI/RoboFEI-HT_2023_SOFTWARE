import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32 
from collections import deque


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

from .submodules.utils          import draw_lines, position
from .submodules.ClassConfig    import *
from .submodules.Client         import Client
from .submodules.ImageGetter    import ImageGetter
from .submodules.image          import findBall, findGoalpost, resize_image


class BallDetection(Node):
    def __init__(self):

        super().__init__("image_node")

        #PARAMS
        #==============================================================================================================#
        # __   _____  _     ___  
        # \ \ / / _ \| |   / _ \ 
        #  \ V / | | | |  | | | |
        #   | || |_| | |__| |_| |
        #   |_| \___/|_____\___/ 


        self.declare_parameter("device", "cpu")
        self.device = self.get_parameter("device").get_parameter_value().string_value
        
        self.declare_parameter("model", f"{os.path.dirname(os.path.realpath(__file__))}/weights/best_openvino_model/")
        self.model = YOLO(self.get_parameter("model").get_parameter_value().string_value) # Load model
        self.value_classes = self.get_classes()  # define antes de usar
        self.get_logger().info(f"CLASSES DO MODELO: {self.value_classes}")


        #   ____                               
        #  / ___|__ _ _ __ ___   ___ _ __ __ _ 
        # | |   / _ | '_  _ \ / _ \ '__/ _ |
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

        self.declare_parameter("server_ip", "10.42.0.1")
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
        fps_save = self.get_parameter("fps_save").get_parameter_value().integer_value
        #==============================================================================================================#
        

        self.original_dim = np.array([self.img_width, self.img_height])
        self.redued_dim = self.original_dim * self.img_qlty / 100
        self.value_classes = self.get_classes()
        
        self.bridge = CvBridge()
        self.raw_image_sub_ = self.create_subscription(Image, "/image_raw", self.image_callback, 1)
        self.raw_image_sub_

        self.ball_position_publisher_ = self.create_publisher(Vision, '/ball_position', 2)
        self.ball_position_publisher_

        self.goalpost_position_publisher_ = self.create_publisher(Vision, '/goalpost_position', 10)
        self.goalpost_position_publisher_
        
        self.goalpost_px_position_publisher_ = self.create_publisher(Point2D, '/goalpost_px_position', 2)
        self.goalpost_px_position_publisher_

        self.ball_px_position_publisher_ = self.create_publisher(Point2D, '/ball_px_position', 2)
        self.ball_px_position_publisher_

        self.goalpost_center_publisher_ = self.create_publisher(Point2D, '/goalpost_center_px', 2)
        self.goalpost_center_publisher_

        self.goalpost_count_publisher_ = self.create_publisher(Int32, '/goalpost_count', 2)
        self.goalpost_count_publisher_

        self.cont_real_ball_detections = 0
        self.cont_real_goalpost_detections = 0

        self.kick_ready_streak = 0
        self.kick_fail_tolerance = 0


        self.cont_frames_kick_ready = 0
        self.last_ball_positions = deque(maxlen=5)  # Para suavizar posição da bola
        self.last_goalpost_positions = deque(maxlen=5) 


        #recive data from config.ini using the ClassConfig submodule
        self.config = classConfig()

        self.results = None
        self.img = None

        self.ball_pos = position() #apagar
        self.filtered_ball_position = Point2D()

        self.ball_pos_area = Vision()
        self.goalpost_pos_area = Vision()
        
        self.cont_real_detections = 0

        if self.enable_udp:
            self.client = Client(self.server_ip, self.server_port)

        if self.get_image: 
            self.imageGetter = ImageGetter('vision_log', fps_save)


    def __del__(self):
        if hasattr(self, "enable_udp") and self.enable_udp:
            try:
                self.client.close_socket()
            except:
                pass


    def get_classes(self): #function for list all classes and the respective number in a dictionary
        classes = self.model.names
        value_classes = {value: key for key, value in classes.items()}
        return value_classes   
    

    def image_callback(self, msg):
        try:
            self.img = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            if self.get_image:
                self.imageGetter.save(self.img)

            self.results = self.predict_image(resize_image(self.img, self.img_qlty)) # predict image 

            if self.show_divisions:
                self.img = draw_lines(self.img, self.config)  #Draw camera divisions

            self.img = self.ball_detection(self.img, self.results)
            self.img = self.goalpost_detection(self.img, self.results)

            try:
                if self.filtered_ball_position.size != 0:
                    goalposts = getattr(self, 'goalpost_px_positions_for_decision', [])
                    self.decide_kick(self.filtered_ball_position, goalposts)

            except:
                pass
            
            if self.enable_udp:
                try:
                    self.client.send_image(self.img)
                except:
                    pass
                    self.get_logger().debug("Não está publicando no servidor udp")

            cv2.imshow('Ball', self.img) # Show image
            cv2.waitKey(1)
        except:
            
            pass


    def predict_image(self, img):
        results = self.model(img, device=self.device, conf=0.5, max_det=3, verbose=False)        
        return results[0]

    def goalpost_detection(self, img, results):
        img_cp = img.copy()

        try:
            img_cp, goalpost_px_positions = findGoalpost(img_cp, results, self.value_classes)

            if not goalpost_px_positions or len(goalpost_px_positions) == 0: # se a trave nn for detectada
                goalpost_area = Vision()
                goalpost_area.detected = False
                self.goalpost_position_publisher_.publish(goalpost_area)
                self.goalpost_px_positions_for_decision = []  # ISSO É OBRIGATÓRIO

                #self.get_logger().warn("❌ Nenhuma trave detectada — limpando para decisão.")
                goalpost_count_msg = Int32()
                goalpost_count_msg.data = len(goalpost_px_positions)
                self.goalpost_count_publisher_.publish(goalpost_count_msg)
                return img_cp

            
            # Publica cada trave detectada
            for goalpost_px_pos in goalpost_px_positions:
                goalpost_px_pos_msg = Point2D()
                goalpost_px_pos_msg.x = float(goalpost_px_pos[0])
                goalpost_px_pos_msg.y = float(goalpost_px_pos[1])
                self.goalpost_px_position_publisher_.publish(goalpost_px_pos_msg)

                new_goalpost_pos_area = self.get_goalpost_pos_area(goalpost_px_pos)
                self.goalpost_pos_area_filter(new_goalpost_pos_area, 1)

                goalpost_count_msg = Int32()
                goalpost_count_msg.data = len(goalpost_px_positions)
                self.goalpost_count_publisher_.publish(goalpost_count_msg)


            # calcula o centro do gol se detectou pelo menos 2 traves
            if len(goalpost_px_positions) >= 0:
                # Ordena traves pela posição x (esquerda para direita)
                goalpost_px_positions.sort(key=lambda pos: pos[0])

                left_post = goalpost_px_positions[0]
                right_post = goalpost_px_positions[1]

                center_x = (left_post[0] + right_post[0]) / 2
                center_y = (left_post[1] + right_post[1]) / 2

                center_goalpost_msg = Point2D()
                center_goalpost_msg.x = float(center_x)
                center_goalpost_msg.y = float(center_y)

                # Publica o centro do gol
                self.goalpost_center_publisher_.publish(center_goalpost_msg)

                self.goalpost_px_positions_for_decision = goalpost_px_positions  # salva para decisão

                # Desenha o centro do gol na imagem
                cv2.circle(img_cp, (int(center_x), int(center_y)), 8, (0, 0, 255), -1)  # vermelho

        except Exception as e:
            self.get_logger().error(f"Erro na goalpost_detection: {str(e)}")

        return img_cp

    
    def ball_detection(self, img, results):
        img_cp = img.copy()

        img_cp, ball_px_pos = findBall(img_cp, results, self.value_classes) #image, [x, y]

        new_ball_pos_area = Vision()
        ball_px_pos_msg = Point2D()

        if ball_px_pos.size != 0: #if ball was finded
            ball_px_pos_msg.x = ball_px_pos[0]
            ball_px_pos_msg.y = ball_px_pos[1]
            self.ball_px_position_publisher_.publish(ball_px_pos_msg)
            new_ball_pos_area = self.get_ball_pos_area(ball_px_pos)

            self.filtered_ball_position = ball_px_pos  # salva a posição filtrada da bola
        
        self.ball_pos_area_filter(new_ball_pos_area, 1)
        return img_cp


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
                self.cont_real_ball_detections += 1
            else:
                self.cont_real_ball_detections = 0
                self.ball_pos_area = not_filtered_ball_pos

            if self.cont_real_ball_detections > 2:
                self.ball_position_publisher_.publish(self.ball_pos_area)

    
    def goalpost_pos_area_filter(self, not_filtered_goalpost_pos, opt):
        if opt == 0:
            self.goalpost_pos_area = not_filtered_goalpost_pos
        
        elif opt == 1:
            if not_filtered_goalpost_pos == self.goalpost_pos_area:
                self.cont_real_goalpost_detections += 1
            else:
                self.cont_real_goalpost_detections = 0
                self.goalpost_pos_area = not_filtered_goalpost_pos

            if self.cont_real_goalpost_detections > 2:
                self.goalpost_position_publisher_.publish(self.goalpost_pos_area)



    def get_ball_pos_area(self, ball_px_pos):
        ball_pos = Vision()
        ball_pos.detected = True

        # identify the ball position in Y axis
        if (ball_px_pos[0] <= self.config.x_left):     #ball to the left
            ball_pos.left = True
            self.get_logger().debug("Bola à Esquerda")

        elif (ball_px_pos[0] < self.config.x_center):  #ball to the center left
            ball_pos.center = True
            self.get_logger().debug("Bola Centralizada")

        else:                                            #ball to the right
            ball_pos.right = True
            self.get_logger().debug("Bola à Direita")
        

        # identify the ball position in Y axis
        if (ball_px_pos[1] > self.config.y_chute):     #ball near
            ball_pos.close = True
            self.get_logger().debug("Bola Perto")
        
        elif (ball_px_pos[1] <= self.config.y_longe):  #ball far
            ball_pos.far = True
            self.get_logger().debug("Bola Longe")

        else:                                           #Bola middle
            ball_pos.med = True
            self.get_logger().debug("Bola ao Centro")

        return ball_pos
    
    def get_goalpost_pos_area(self, goalpost_px_pos):
        goalpost_pos = Vision()

        # Resetando tudo
        goalpost_pos.detected = True
        goalpost_pos.left = False
        goalpost_pos.center = False
        goalpost_pos.right = False
        goalpost_pos.close = False
        goalpost_pos.med = False
        goalpost_pos.far = False

        x = goalpost_px_pos[0]
        y = goalpost_px_pos[1]

        # Lado horizontal
        if x < self.config.x_left:
            goalpost_pos.left = True
            self.get_logger().debug("Trave à Esquerda")
        elif x > self.config.x_center:
            goalpost_pos.right = True
            self.get_logger().debug("Trave à Direita")
        else:
            goalpost_pos.center = True
            self.get_logger().debug("Trave Centralizada")

        # Distância vertical (y = mais alto = mais baixo na tela)
        if y < self.config.y_longe:  # mais alto → mais longe
            goalpost_pos.far = True
            self.get_logger().debug("Trave Longe")
        elif y > self.config.y_chute:  # mais baixo → mais perto
            goalpost_pos.close = True
            self.get_logger().debug("Trave Perto")
        else:
            goalpost_pos.med = True
            self.get_logger().debug("Trave ao Centro")

        return goalpost_pos
    
            

    def ball_delta_position_threshold(self, new_position, threshold):
        dp = position()
        dp.x = abs(new_position.x - self.ball_pos.x)
        dp.y = abs(new_position.y - self.ball_pos.y)

        return hypot(dp.x, dp.y) < threshold


def main(args=None):
    rclpy.init(args=args)
    node = None  # inicializa como None

    try:
        node = BallDetection()
        rclpy.spin(node)
    except Exception as e:
        print(f"[ERRO] {e}")
    finally:
        if node is not None:
            node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.try_shutdown()

 

if __name__ == '__main__':
    main()