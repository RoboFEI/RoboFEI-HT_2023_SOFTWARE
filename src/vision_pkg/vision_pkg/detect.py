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
from cv_bridge import CvBridge
import torch

from custom_interfaces.msg import Vision
from vision_msgs.msg import Point2D
from sensor_msgs.msg import Image

from .submodules.utils          import draw_lines, position
from .submodules.ClassConfig    import *
from .submodules.Client         import Client
from .submodules.ImageGetter    import ImageGetter
from .submodules.image          import findBall, findGoalpost, findL, findT, findX, resize_image


class BallDetection(Node):
    def __init__(self):

        super().__init__("image_node")

        # CAMERA / OPENCV
        self.declare_parameter("camera_device", 0)
        self.camera_device = self.get_parameter("camera_device").get_parameter_value().integer_value

        self.declare_parameter("camera_fps", 30)
        self.camera_fps = self.get_parameter("camera_fps").get_parameter_value().integer_value

        # PARAMS (kept from original)
        self.declare_parameter("device", "cpu")
        self.device = self.get_parameter("device").get_parameter_value().string_value

        self.declare_parameter("model", f"{os.path.dirname(os.path.realpath(__file__))}/weights/LTX.pt") # changed - era: /best_openvino_model/
        self.model = YOLO(self.get_parameter("model").get_parameter_value().string_value) # Load model
        self.value_classes = self.get_classes()  # define antes de usar
        self.get_logger().info(f"CLASSES DO MODELO: {self.value_classes}")

        self.declare_parameter("img_qlty", 100) 
        self.img_qlty = self.get_parameter("img_qlty").get_parameter_value().integer_value

        self.declare_parameter("image_width", 1280)
        self.img_width = self.get_parameter("image_width").get_parameter_value().integer_value

        self.declare_parameter("image_height", 720)
        self.img_height = self.get_parameter("image_height").get_parameter_value().integer_value

        self.declare_parameter("enable_udp", True)
        self.enable_udp = self.get_parameter("enable_udp").get_parameter_value().bool_value

        self.declare_parameter("server_ip", "10.42.0.1")
        self.server_ip = self.get_parameter("server_ip").get_parameter_value().string_value

        self.declare_parameter("server_port", 5050)
        self.server_port = self.get_parameter("server_port").get_parameter_value().integer_value

        self.declare_parameter("show_divisions", True) # Show division lines and center of ball in output image
        self.show_divisions = self.get_parameter("show_divisions").get_parameter_value().bool_value

        self.declare_parameter("get_image", False)
        self.get_image = self.get_parameter("get_image").get_parameter_value().bool_value
        
        self.declare_parameter("fps_save", 2) 
        fps_save = self.get_parameter("fps_save").get_parameter_value().integer_value

        # derived dims
        self.original_dim = np.array([self.img_width, self.img_height])
        self.redued_dim = self.original_dim * self.img_qlty / 100
        self.value_classes = self.get_classes()
        
        self.bridge = CvBridge()

        # Publishers (same as original)
        self.ball_position_publisher_ = self.create_publisher(Vision, '/ball_position', 2)
        self.goalpost_position_publisher_ = self.create_publisher(Vision, '/goalpost_position', 10)
        self.goalpost_px_position_publisher_ = self.create_publisher(Point2D, '/goalpost_px_position', 2)
        self.ball_px_position_publisher_ = self.create_publisher(Point2D, '/ball_px_position', 2)
        self.goalpost_center_publisher_ = self.create_publisher(Point2D, '/goalpost_center_px', 2)
        self.goalpost_count_publisher_ = self.create_publisher(Int32, '/goalpost_count', 2)
        # Added:
        self.l_position_publisher_ = self.create_publisher(Vision, '/l_position', 10)
        self.l_px_position_publisher_ = self.create_publisher(Point2D, '/l_px_position', 2)
        self.l_count_publisher_ = self.create_publisher(Int32, '/l_count', 2)

        self.t_position_publisher_ = self.create_publisher(Vision, '/t_position', 10)
        self.t_px_position_publisher_ = self.create_publisher(Point2D, '/t_px_position', 2)
        self.t_count_publisher_ = self.create_publisher(Int32, '/t_count', 2)

        self.x_position_publisher_ = self.create_publisher(Vision, '/x_position', 10)
        self.x_px_position_publisher_ = self.create_publisher(Point2D, '/x_px_position', 2)
        self.x_count_publisher_ = self.create_publisher(Int32, '/x_count', 2)

        # state vars
        self.cont_real_ball_detections = 0
        self.cont_real_goalpost_detections = 0

        self.kick_ready_streak = 0
        self.kick_fail_tolerance = 0

        self.cont_frames_kick_ready = 0
        self.last_ball_positions = deque(maxlen=5)  # Para suavizar posição da bola
        self.last_goalpost_positions = deque(maxlen=5) 

        # load config
        self.config = classConfig()

        self.results = None
        self.img = None

        self.ball_pos = position() #apagar
        self.filtered_ball_position = Point2D()

        self.ball_pos_area = Vision()
        self.goalpost_pos_area = Vision()
        # Added:
        self.l_pos_area = Vision()
        self.t_pos_area = Vision()
        self.x_pos_area = Vision()
        
        self.cont_real_detections = 0
        # Added: 
        self.cont_real_l_detections = 0
        self.cont_real_t_detections = 0
        self.cont_real_x_detections = 0


        if self.enable_udp:
            self.client = Client(self.server_ip, self.server_port)

        if self.get_image: 
            self.imageGetter = ImageGetter('vision_log', fps_save)

        # OpenCV VideoCapture
        self.cap = cv2.VideoCapture("/dev/camera", cv2.CAP_ANY)
        if not self.cap.isOpened():
            self.get_logger().error(f"Não foi possível abrir a câmera device={self.camera_device}")
        else:
            # try to set resolution if desired
            try:
                self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.img_width)
                self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.img_height)
                # fps attempt
                self.cap.set(cv2.CAP_PROP_FPS, float(self.camera_fps))
            except Exception:
                pass

        # Timer to read frames with requested fps
        timer_period = 1.0 / max(1.0, float(self.camera_fps))
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def __del__(self):
        # close UDP client if any
        if hasattr(self, "enable_udp") and self.enable_udp:
            try:
                self.client.close_socket()
            except:
                pass
        # release camera
        try:
            if hasattr(self, "cap") and self.cap is not None:
                self.cap.release()
        except:
            pass
        cv2.destroyAllWindows()

    def get_classes(self): #function for list all classes and the respective number in a dictionary
        classes = self.model.names
        value_classes = {value: key for key, value in classes.items()}
        return value_classes   

    def timer_callback(self):
        # Read frame from OpenCV
        if not hasattr(self, "cap") or self.cap is None or not self.cap.isOpened():
            return

        ret, frame = self.cap.read()
        if not ret or frame is None:
            # camera read failure
            self.get_logger().warn("Falha ao ler frame da câmera.")
            return

        # emulate receiving a sensor_msgs/Image if other parts expect it: we process the cv image directly
        try:
            if self.get_image:
                self.imageGetter.save(frame)

            # predict & process
            self.results = self.predict_image(resize_image(frame, self.img_qlty))  # predict image

            if self.show_divisions:
                frame = draw_lines(frame, self.config)  # Draw camera divisions

            frame = self.ball_detection(frame, self.results)
            frame = self.goalpost_detection(frame, self.results)
            # Added:
            frame = self.l_detection(frame, self.results)
            frame = self.t_detection(frame, self.results)
            frame = self.x_detection(frame, self.results)

            try:
                # The original code checks filtered_ball_position.size != 0 — keep similar behavior.
                if hasattr(self.filtered_ball_position, 'size') and self.filtered_ball_position.size != 0:
                    goalposts = getattr(self, 'goalpost_px_positions_for_decision', [])
                    self.decide_kick(self.filtered_ball_position, goalposts)
            except Exception:
                pass

            if self.enable_udp:
                try:
                    self.client.send_image(frame)
                except Exception:
                    self.get_logger().debug("Não está publicando no servidor udp")

            # Show
            cv2.imshow('Ball', frame)
            cv2.waitKey(1)

            # If you want to also publish the original frame as sensor_msgs/Image, uncomment:
            # try:
            #     ros_img = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            #     # publish to some topic if desired
            # except Exception as e:
            #     self.get_logger().error(f"Erro convertendo frame para Image msg: {e}")

        except Exception as e:
            self.get_logger().error(f"Erro no timer_callback: {str(e)}")

    # changed - era: 
    # def predict_image(self, img):
    #     results = self.model(img, device=self.device, conf=0.5, max_det=3, verbose=False)        
    #     return results[0]

    def predict_image(self, img):
        # Aumenta max_det para 15-20 detecções
        results = self.model(img, device=self.device, conf=0.35, max_det=15, verbose=False)
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

                goalpost_count_msg = Int32()
                goalpost_count_msg.data = 0
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
            if len(goalpost_px_positions) >= 2:
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

        # note: findBall may return numpy array or Point2D-like; original code used .size
        try:
            if hasattr(ball_px_pos, 'size') and ball_px_pos.size != 0: #if ball was finded
                ball_px_pos_msg.x = float(ball_px_pos[0])
                ball_px_pos_msg.y = float(ball_px_pos[1])
                self.ball_px_position_publisher_.publish(ball_px_pos_msg)
                new_ball_pos_area = self.get_ball_pos_area(ball_px_pos)

                self.filtered_ball_position = ball_px_pos  # salva a posição filtrada da bola
        except Exception:
            # fallback: if findBall returns None or similar
            pass
        
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

        # identify the ball position in X axis
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

        # Distância vertical (y = mais alto = mais longe na tela)
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

    # If you used decide_kick somewhere else in your project, keep it. Otherwise
    # add your decide_kick implementation here or import it.

    # Added: ________________________________________________________________________________________________________________________________________________________
    def l_detection(self, img, results):
        img_cp = img.copy()
        try:
            img_cp, l_px_positions = findL(img_cp, results, self.value_classes)
            
            if not l_px_positions or len(l_px_positions) == 0:
                l_area = Vision()
                l_area.detected = False
                self.l_position_publisher_.publish(l_area)
                
                l_count_msg = Int32()
                l_count_msg.data = 0
                self.l_count_publisher_.publish(l_count_msg)
                return img_cp
            
            # Publica cada L detectado
            for l_px_pos in l_px_positions:
                l_px_pos_msg = Point2D()
                l_px_pos_msg.x = float(l_px_pos[0])
                l_px_pos_msg.y = float(l_px_pos[1])
                self.l_px_position_publisher_.publish(l_px_pos_msg)
                
                new_l_pos_area = self.get_landmark_pos_area(l_px_pos)  # Reutiliza função genérica
                self.l_pos_area_filter(new_l_pos_area, 1)
            
            l_count_msg = Int32()
            l_count_msg.data = len(l_px_positions)
            self.l_count_publisher_.publish(l_count_msg)
            
        except Exception as e:
            self.get_logger().error(f"Erro na l_detection: {str(e)}")
        
        return img_cp

    def l_pos_area_filter(self, not_filtered_l_pos, opt):
        if opt == 0:
            self.l_pos_area = not_filtered_l_pos
        
        elif opt == 1:
            if not_filtered_l_pos == self.l_pos_area:
                self.cont_real_l_detections += 1
            else:
                self.cont_real_l_detections = 0
                self.l_pos_area = not_filtered_l_pos
            
            if self.cont_real_l_detections > 2:
                self.l_position_publisher_.publish(self.l_pos_area)



    def t_detection(self, img, results):
        img_cp = img.copy()
        
        try:
            img_cp, t_px_positions = findT(img_cp, results, self.value_classes)
            
            if not t_px_positions or len(t_px_positions) == 0:
                t_area = Vision()
                t_area.detected = False
                self.t_position_publisher_.publish(t_area)
                
                t_count_msg = Int32()
                t_count_msg.data = 0
                self.t_count_publisher_.publish(t_count_msg)
                return img_cp
            
            # Publica cada T detectado
            for t_px_pos in t_px_positions:
                t_px_pos_msg = Point2D()
                t_px_pos_msg.x = float(t_px_pos[0])
                t_px_pos_msg.y = float(t_px_pos[1])
                self.t_px_position_publisher_.publish(t_px_pos_msg)
                
                new_t_pos_area = self.get_landmark_pos_area(t_px_pos)  # Reutiliza função genérica
                self.t_pos_area_filter(new_t_pos_area, 1)
            
            t_count_msg = Int32()
            t_count_msg.data = len(t_px_positions)
            self.t_count_publisher_.publish(t_count_msg)
            
        except Exception as e:
            self.get_logger().error(f"Erro na t_detection: {str(e)}")
        
        return img_cp
        
    def t_pos_area_filter(self, not_filtered_t_pos, opt):
        if opt == 0:
            self.t_pos_area = not_filtered_t_pos
        
        elif opt == 1:
            if not_filtered_t_pos == self.t_pos_area:
                self.cont_real_t_detections += 1
            else:
                self.cont_real_t_detections = 0
                self.t_pos_area = not_filtered_t_pos
            
            if self.cont_real_t_detections > 2:
                self.t_position_publisher_.publish(self.t_pos_area)
    
        
    def x_detection(self, img, results):
        img_cp = img.copy()
        
        try:
            img_cp, x_px_positions = findX(img_cp, results, self.value_classes)
            
            if not x_px_positions or len(x_px_positions) == 0:
                x_area = Vision()
                x_area.detected = False
                self.x_position_publisher_.publish(x_area)
                
                x_count_msg = Int32()
                x_count_msg.data = 0
                self.x_count_publisher_.publish(x_count_msg)
                return img_cp
            
            # Publica cada X detectado
            for x_px_pos in x_px_positions:
                x_px_pos_msg = Point2D()
                x_px_pos_msg.x = float(x_px_pos[0])
                x_px_pos_msg.y = float(x_px_pos[1])
                self.x_px_position_publisher_.publish(x_px_pos_msg)
                
                new_x_pos_area = self.get_landmark_pos_area(x_px_pos)  # Reutiliza função genérica
                self.x_pos_area_filter(new_x_pos_area, 1)
            
            x_count_msg = Int32()
            x_count_msg.data = len(x_px_positions)
            self.x_count_publisher_.publish(x_count_msg)
            
        except Exception as e:
            self.get_logger().error(f"Erro na x_detection: {str(e)}")
        
        return img_cp

    def x_pos_area_filter(self, not_filtered_x_pos, opt):
        if opt == 0:
            self.x_pos_area = not_filtered_x_pos
        
        elif opt == 1:
            if not_filtered_x_pos == self.x_pos_area:
                self.cont_real_x_detections += 1
            else:
                self.cont_real_x_detections = 0
                self.x_pos_area = not_filtered_x_pos
            
            if self.cont_real_x_detections > 2:
                self.x_position_publisher_.publish(self.x_pos_area)

    def get_landmark_pos_area(self, landmark_px_pos):
        """Função genérica para calcular posição de landmarks (L, T, X)"""
        landmark_pos = Vision()
        landmark_pos.detected = True
        landmark_pos.left = False
        landmark_pos.center = False
        landmark_pos.right = False
        landmark_pos.close = False
        landmark_pos.med = False
        landmark_pos.far = False

        x = landmark_px_pos[0]
        y = landmark_px_pos[1]

        # Lado horizontal
        if x < self.config.x_left:
            landmark_pos.left = True
        elif x > self.config.x_center:
            landmark_pos.right = True
        else:
            landmark_pos.center = True

        # Distância vertical
        if y < self.config.y_longe:
            landmark_pos.far = True
        elif y > self.config.y_chute:
            landmark_pos.close = True
        else:
            landmark_pos.med = True

        return landmark_pos

    # _________________________________________________________________________________________________________________________________________________________

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
            try:
                node.destroy_node()
            except Exception:
                pass
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()