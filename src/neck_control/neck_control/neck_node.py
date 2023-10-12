import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from time import sleep

from std_msgs.msg import String
from custom_interfaces.msg import Decision 
from custom_interfaces.msg import Vision
from custom_interfaces.msg import HumanoidLeagueMsgs as GC
from custom_interfaces.msg import NeckPosition
from custom_interfaces.msg import VisionRobot
from std_msgs.msg import Bool
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3Stamped
from custom_interfaces.action import Control

class NeckNode(Node):

    def __init__(self):
        super().__init__('neck_node')
        self.get_logger().info('Running Neck Node')

        self.BALL_DETECTED = False
        self.BALL_LEFT = False
        self.BALL_CENTER_LEFT = False
        self.BALL_RIGHT = False
        self.BALL_CENTER_RIGHT = False
        self.BALL_FAR = False # Bola longe, usada no callback da visão para atualizar aonde a bola se encontra
        self.BALL_CLOSE = False # Bola perto, usada no callback da visão para atualizar aonde a bola se encontra
        self.BALL_MED = False # Bola centralizada, usada no callback da visão para atualizar aonde a bola se encontra
        self.ROBOT_DETECTED = False
        self.ready_robot=False
        self.gamestate = 0 # Initial state - Robô parado em pé esperando mudar de estado
        self.secstate = 0
        self.secteam = 0
        self.penalized = False
        self.cont_falses = 0
        self.go_ball = 0
        self.has_kick_off = False
        self.game_already_started = True
        self.last_movement = 0
        self.movement = 1
        self.contador_imu = 0
        self.contador = 0
        self.contador_turn_search_ball = 0
        self.fallen = False
        self.fallenFront = False
        self.fallen_side = False
        self.fallenRight = False
        self.cont_fall_side = 0
        self.cancel = False
        self.finished = True # Variável usada na action para verificar se o controle terminou a ação que estava fazendo
        self.save_ball_left = False
        self.save_ball_right = False
        self.save_need_stand_still = False
        self.cont_turn = 0

        # Subscriber da visão 
        self.subscription_vision = self.create_subscription(
            Vision, 
            '/ball_position',
            self.listener_callback_vision,
            10)
        self.subscription_vision
        
        #Variaveis da Visão (BOLA)
        self.BALL_DETECTED = False
        self.BALL_LEFT = False
        self.BALL_CENTER_LEFT = False
        self.BALL_RIGHT = False
        self.BALL_CENTER_RIGHT = False
        self.BALL_FAR = False # Bola longe, usada no callback da visão para atualizar aonde a bola se encontra
        self.BALL_CLOSE = False # Bola perto, usada no callback da visão para atualizar aonde a bola se encontra
        self.BALL_MED = False # Bola centralizada, usada no callback da visão para atualizar aonde a bola se encontra

        
        # Subscriber da posição dos motores do pescoço
        self.subscription_neck = self.create_subscription(
            NeckPosition, 
            '/neck_position',
            self.listener_callback_neck,
            10) 
        self.subscription_neck


        self.publisher_set_neck_position = self.create_publisher(
            NeckPosition, 
            '/set_neck_position', 
            10)
        self.publisher_set_neck_position
        self.old_neck_position = [2048, 2048]

        self.timer=self.create_timer(0.008,self.main_timer_callback)

        self.falses_not_detected = 0



    def main_timer_callback(self):
        new_neck_position = NeckPosition()

        new_neck_position.position19 = self.old_neck_position[0]
        new_neck_position.position20 = self.old_neck_position[1]
            

        if(self.BALL_DETECTED):
            if(self.BALL_LEFT and self.old_neck_position[0] < 2650):
                new_neck_position.position19 = self.move_head('left', self.old_neck_position[0])
            elif(self.BALL_RIGHT and self.old_neck_position[0] > 1350):
                new_neck_position.position19 = self.move_head('right', self.old_neck_position[0])
            elif (self.BALL_CLOSE):
                self.get_logger().info('BALL CLOSE')
                if(self.old_neck_position[1]>1340):
                    new_neck_position.position20 = self.move_head('down', self.old_neck_position[1])
        
        self.old_neck_position = [new_neck_position.position19, new_neck_position.position20]

        self.publisher_set_neck_position.publish(new_neck_position)

                    

    def listener_callback_vision(self, msg):
        # print("Vision Callback")
        self.BALL_DETECTED = msg.detected
        self.get_logger().info('BALL "%s"' % self.BALL_DETECTED)
        self.BALL_LEFT = msg.left
        self.BALL_CENTER_LEFT = msg.center_left
        self.BALL_RIGHT = msg.right
        self.BALL_CENTER_RIGHT = msg.center_right
        self.BALL_FAR = msg.far
        self.BALL_MED = msg.med
        self.BALL_CLOSE = msg.close
    
    #Calback da posição lida dos motores do pescoço
    def listener_callback_neck(self, msg):
        self.old_neck_position = [msg.position19, msg.position20]

    def move_head(self, side, neck_position):
        side = side.lower()
        if(side == 'left'):
            return neck_position + cont_vision_sides
        elif(side == 'right'):    
            return neck_position + cont_vision_sides
        elif(side == 'down'):    
            return neck_position - cont_vision_up


def main(args=None):
    rclpy.init(args=args)

    neckNode = NeckNode()

    rclpy.spin(neckNode)
    neckNode.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()





        