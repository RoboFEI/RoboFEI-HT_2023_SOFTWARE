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

# ros2 run decision_pkg decision_node
# ros2 topic pub -1 /gamestate custom_interfaces/msg/HumanoidLeagueMsgs "{game_state: 3}"
# ros2 topic pub -1 /gamestate custom_interfaces/msg/HumanoidLeagueMsgs "{game_state: 3, secondary_state: 1, has_kick_off: True}"
# ros2 topic pub -1 /position std_msgs/Bool "data: True"
# ros2 topic pub -1 /neck_position custom_interfaces/msg/NeckPosition "{position19: 2048, position20: 2048}"

TEAM_ROBOFEI = 7
ROBOT_NUMBER = 2 # Goleiro = 1, jogadores != 1
LADO = 0 # 0 vira para o lado DIREITO e 1 para o lado ESQUERDO, depende de que lado o nosso time vai começar

# ros2 topic echo /imu/data
# ros2 topic echo /imu/rpy


class DecisionNode(Node):

    def __init__(self):
        super().__init__('decision_node')
        self.get_logger().info('Running Decision Node')
        # Subscriber do Game Controller
        self.subscription = self.create_subscription(
            GC,
            'gamestate',
            self.listener_callback,
            10)
        # Subscriber da visão 
        self.subscription_vision = self.create_subscription(
            Vision, 
            '/ball_position',
            self.listener_callback_vision,
            10)
        # Subscriber da visão para detecção de robôs inimigos
        self.subscription_robot = self.create_subscription(
            VisionRobot, 
            '/robot_position_enemy',
            self.listener_callback_robot,
            10)
        # Subscriber da posição dos motores do pescoço
        self.subscription_neck = self.create_subscription(
            NeckPosition, 
            '/neck_position',
            self.listener_callback_neck,
            10) 
        self.subscription_imu_gyro = self.create_subscription(
            Vector3Stamped, 
            'imu/rpy',
            self.listener_callback_imu_gyro,
            10) 
        self.subscription_imu_accel = self.create_subscription(
            Imu, 
            'imu/data',
            self.listener_callback_imu_accel,
            10) 
        self.timer=self.create_timer(0.008,self.timer_callback)
        self._action_client = ActionClient(self, Control, 'control_action')
        self.subscription  
        self.subscription_vision
        self.subscription_neck
        self.subscription_imu_gyro
        self.subscription_imu_accel
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
        self.contador = 0
        self.contador_imu = 0
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
        

    def listener_callback_neck(self, msg):
        self.neck_position = [msg.position19, msg.position20]

    def listener_callback_imu_gyro(self, msg):
        self.gyro_z = msg.vector.z
    
    def listener_callback_imu_accel(self, msg):
        self.accel_z = msg.linear_acceleration.z
        self.accel_x = msg.linear_acceleration.x

        if (self.accel_z > 7 or self.accel_z < -7):
            self.contador_imu+=1
        else:
            self.contador_imu = 0
        self.get_logger().info('Contador IMU: "%d"' % self.contador_imu)
      
        if(self.contador_imu>=30):
            self.fallen = True # Robô caido
            if(self.accel_z < 0):  # Robô caido de frente
                self.fallenFront = True
                self.get_logger().info('Caido de frente')
            else: # Robô caido de costas
                self.fallenFront = False
                self.get_logger().info('Caido de costas')
        
        if (self.accel_x > 7 or self.accel_x < -7):
            self.cont_fall_side+=1
        else:
            self.cont_fall_side = 0
        self.get_logger().info('Contador IMU fall side: "%d"' % self.cont_fall_side)
      
        if(self.cont_fall_side>=30):
            self.fallen_side = True # Robô caido
            if(self.accel_x < 0):  # Robô caido de frente
                self.fallenRight = False
                self.get_logger().info('Caido para a direita')
            else: # Robô caido de costas
                self.fallenRight = True
                self.get_logger().info('Caido para a esquerda')
            self.get_logger().info('Caido de lado')
            

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

    def listener_callback_robot(self, msg):
        self.ROBOT_DETECTED = msg.detected
        self.get_logger().info('ROBOT "%s"' % self.ROBOT_DETECTED)
        self.ROBOT_LEFT = msg.left
        self.ROBOT_CENTER_LEFT = msg.center_left
        self.ROBOT_RIGHT = msg.right
        self.ROBOT_CENTER_RIGHT = msg.center_right
        self.ROBOT_FAR = msg.far
        self.ROBOT_MED = msg.med
        self.ROBOT_CLOSE = msg.close

    def listener_callback(self, msg):
        self.get_logger().info('GAME STATE: "%s"' % msg.game_state)
        self.gamestate = msg.game_state
        self.secstate = msg.secondary_state
        self.secteam = msg.secondary_state_team
        # self.get_logger().info('PENALTY: "%s"' % msg.secondary_state_team)
        self.penalized = msg.penalized
        self.has_kick_off = msg.has_kick_off
        self.penaltyshoot_mode = msg.secondary_state_mode

    def send_goal(self, order):
        goal_msg = Control.Goal()
        goal_msg.action_number = order
        self._action_client.wait_for_server()

        if order != self.last_movement: # Se tiver que mudar o movimento
            if (order == 16 or order == 17 or order == 18 or self.last_movement == 8): # Robô caído: prioridade é levantar, por isso ele cancela o que estiver fazendo
                self._send_goal_future = self.goal_handle.cancel_goal_async()
                self._send_goal_future.add_done_callback(self.cancel_done)
                self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
                self._send_goal_future.add_done_callback(self.goal_response_callback)
                self.last_movement = order
                self.get_logger().info(f'Last movement {self.last_movement}')

            else: # Robô não caído: espera ele terminar o que ele tava fazendo
                if self.finished == True:
                    self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
                    self._send_goal_future.add_done_callback(self.goal_response_callback)
                    self.last_movement = order
            self.finished = False

        else: # Mandando a mesma movimentação para o server
            if self.finished == True:
                self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
                self.last_movement = order
                self._send_goal_future.add_done_callback(self.goal_response_callback)
                self.finished = False


    def cancel_done(self, future):
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info('Goal successfully canceled')
        else:
            self.get_logger().info('Goal failed to cancel')

    def goal_response_callback(self, future):
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return
        
        self.get_logger().info('Goal accepted :)')

        self._get_result_future = self.goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.finished = result.finished

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: %d' % feedback.movements_remaining)


    def timer_callback(self):
        if self.fallen:
            if self.fallenFront:
                self.stand_up_front()
                self.fallenFront = False
            else:
                self.stand_up_back()
            self.fallen = False
        elif self.fallen_side:
            if self.fallenRight:
                self.stand_right()
                self.fallenRight = False
            else:
                self.stand_left()
            self.fallen_side = False
        else:
            if(ROBOT_NUMBER != 1): # JOGADOR
                if(self.gamestate == 0): # Initial state - Robô parado em pé
                    self.ready_robot = False
                    self.gait()
                    self.get_logger().info('INITIAL: Initial State')

                elif(self.gamestate == 1 and not self.ready_robot): # Robô vai para a posição inicial    
                    self.gait()
                    
                elif(self.gamestate == 2): # Espera o jogo começar
                    self.get_logger().info('SET: Keep ready')
                    self.gait()


                elif(self.gamestate == 3):

                    if(self.secstate == 6): # Penalti do oponente
                        self.gait()

                    elif(self.secstate == 3): # Timeout
                        self.gait()
                    
                    elif(self.secstate == 1 and self.has_kick_off == True): # Penalti nosso 
                        if ((self.save_ball_left or self.save_ball_right)):
                            if (not self.ROBOT_DETECTED and self.neck_position[1]<1700):
                                self.search_goalkeeper()
                            else:
                                if((self.ROBOT_CENTER_RIGHT or self.ROBOT_RIGHT) and self.save_ball_left):
                                    self.open_left_kick()
                                elif ((self.ROBOT_CENTER_LEFT or self.ROBOT_LEFT) and self.save_ball_right):
                                    self.open_right_kick()
                                elif (self.save_ball_left):
                                    self.kick_left()
                                else:
                                    self.kick_right()
                        elif (not self.BALL_DETECTED):
                            self.search_ball()
                        else:
                            if(self.BALL_LEFT and self.neck_position[0] < 2650):
                                self.turn_head_left()
                            elif(self.BALL_RIGHT and self.neck_position[0] > 1350):
                                self.turn_head_right()
                            elif (self.neck_position[0] < 1700):
                                self.turn_right()
                            elif (self.neck_position[0] > 2300):
                                self.turn_left()
                            elif(self.BALL_FAR or self.BALL_MED):
                                self.walking()
                            elif (self.BALL_CLOSE):
                                if(self.neck_position[1]>1300):
                                    self.turn_head_down()
                                else:
                                    if (self.BALL_CENTER_LEFT or self.BALL_LEFT):
                                        self.save_ball_left = True
                                    else:
                                        self.save_ball_right = True
                    
                            
                    elif(self.secstate == 1 and self.has_kick_off == False): # Penalti do outro time
                        self.gait()

                    elif(self.secstate == 4 and self.secteam != TEAM_ROBOFEI): # Direct freekick do oponente
                        self.gait()

                    elif(self.secstate == 5 and self.secteam != TEAM_ROBOFEI): # Indirect freekick do oponente
                        self.gait()

                    elif(self.secstate == 7 and self.secteam != TEAM_ROBOFEI): # Escanteio do oponente
                        self.gait()
                    
                    elif(self.secstate == 8): # Tiro de meta do goleiro
                        self.gait()

                    elif(self.secstate == 9 and self.secteam != TEAM_ROBOFEI): # Lateral do oponente
                        self.gait()
                            
                    elif(self.penalized==True):
                        self.gait()

                    else: 
                        if(self.BALL_DETECTED == False):
                            self.get_logger().info('BALL NOT FOUND %d' %self.contador)
                            self.get_logger().info('CONT FALSES %d' %self.cont_falses)
                            if (self.save_ball_left and self.cont_turn < 150):
                                self.turn_left()
                                self.cont_turn += 1
                            elif (self.save_ball_right and self.cont_turn < 150):
                                self.turn_right()
                                self.cont_turn += 1
                            elif (self.contador >= 150):
                                if (self.cont_falses >=2900 ):
                                    self.cont_falses = 0
                                # elif (self.cont_falses >=2700 and (self.gyro_z < 1.57 and self.gyro_z > -1.57)):
                                elif (self.cont_falses >=2700):
                                    self.walking()
                                    self.cont_falses += 1
                                    self.get_logger().info('WALKING INSIDE SEARCH BALL')
                                elif(self.cont_falses>=1800):
                                    self.turn_left()
                                    self.cont_falses += 1
                                    self.get_logger().info('TURNING INSIDE SEARCH BALL')
                                else:
                                    self.get_logger().info('PROCURANDOOOO')
                                    self.cont_falses += 1
                                    self.search_ball() # Procura a bola
                            else:
                                self.contador += 1
                            
                        else:
                            self.contador = 0
                            self.cont_falses = 0
                            self.cont_turn = 0
                            self.save_ball_left = False
                            self.save_ball_right = False
                            self.get_logger().info('BALL DETECTED')
                            if(self.BALL_LEFT and self.neck_position[0] < 2650):
                                self.turn_head_left()
                                self.save_ball_left = True
                            elif(self.BALL_RIGHT and self.neck_position[0] > 1350):
                                self.turn_head_right()
                                self.save_ball_right = True
                            elif (self.neck_position[0] < 1700):
                                self.turn_right()
                            elif (self.neck_position[0] > 2300):
                                self.turn_left()
                            elif(self.BALL_FAR or self.BALL_MED):
                                self.walking()
                            elif (self.BALL_CLOSE):
                                self.get_logger().info('BALL CLOSE')
                                if(self.neck_position[1]>1300):
                                    self.turn_head_down()
                                else:
                                    self.get_logger().info('BALL KICK')
                                    # if (self.gyro_z < 1.57 and self.gyro_z > -1.57): 
                                        # self.get_logger().info('FACING THE OPPONENT GOAL')
                                    if (self.BALL_CENTER_LEFT or self.BALL_LEFT):
                                        self.kick_left()
                                    else:
                                        self.kick_right()
                                    # else: 
                                        # self.get_logger().info('FACING THE OUR GOAL: TURNING')
                                        # self.turn_around_clockwise()


                elif(self.gamestate == 4): # Jogo terminou, robô sai do campo
                    self.stand_still()
            
            else: # GOLEIRO
                if(self.gamestate == 0): # Initial state - Robô parado em pé
                    self.stand_still()
                    self.get_logger().info('INITIAL: Initial State')

                elif(self.gamestate == 1): # Robô vai para a posição inicial
                    self.get_logger().info('READY: Go to start position')
                    self.stand_still()

                elif(self.gamestate == 2): # Espera o jogo começar
                    self.get_logger().info('Keep ready')
                    self.stand_still()

                elif(self.gamestate == 3): # Jogo começou
                    if(self.BALL_DETECTED == False):
                        self.get_logger().info('BALL NOT FOUND')
                        self.search_ball() # Procura a bola
                    else:
                        self.get_logger().info('BALL DETECTED')
                        if (self.BALL_FAR and self.BALL_RIGHT):
                            self.goalkeeper_penalty_feather_falling_right() 
                        elif (self.BALL_FAR and self.BALL_LEFT):
                            self.goalkeeper_penalty_feather_falling_left() # Precisa do outro movimento para a esquerda (para defender de longe)
                        elif ((self.BALL_CLOSE or self.BALL_MED) and self.BALL_LEFT):
                            self.goalkeeper_fall_left()
                        elif ((self.BALL_CLOSE or self.BALL_MED) and self.BALL_RIGHT):
                            self.goalkeeper_fall_right()
                        elif ((self.BALL_FAR or self.BALL_MED or self.BALL_CLOSE) and (self.BALL_CENTER_LEFT or self.BALL_CENTER_RIGHT)):
                            self.goalkeeper_squat()
                elif(self.gamestate == 4): # Jogo terminou, robô sai do campo
                    self.stand_still()


    def stand_still(self): # Robô em pé parado
        self.send_goal(1)
        self.get_logger().info('Stand still')

    def stand_up_front(self):
        self.send_goal(17)
        self.get_logger().info('Caído de frente')

    def stand_up_back(self):
        self.send_goal(16)
        self.get_logger().info('Caído de costas')

    def stand_up_side(self):
        self.send_goal(18)
        self.get_logger().info('Caído de lado')

    def kick_right(self):
        self.send_goal(3)
        self.get_logger().info('Right kick')

    def kick_left(self):
        self.send_goal(4)
        self.get_logger().info('Left kick')

    def walking(self):
        self.send_goal(14) 
        self.get_logger().info('Walking')

    def gait(self):
        self.send_goal(15) 
        self.get_logger().info('Gait')

    def search_ball(self):
        self.send_goal(8)
        self.get_logger().info('Searching ball')

    def turn_right(self): # gira no seu próprio eixo (direita)
        self.send_goal(5)
        self.get_logger().info('Turn right')

    def turn_left(self): # gira no seu próprio eixo (esquerda)
        self.send_goal(6)
        self.get_logger().info('Turn left')

    def turn_around_clockwise(self): # gira em torno da bola sentido horário
        self.send_goal(9)
        self.get_logger().info('Turning around ball clockwise')
            
    def turn_around_anti_clockwise(self): # gira em torno da bola sentido antihorário
        self.send_goal(10)
        self.get_logger().info('Turning around ball anti-clockwise')

    def goalkeeper_squat(self):
        self.send_goal(29)
        self.get_logger().info('Squat')

    def goalkeeper_fall_left(self):
        self.send_goal(11)
        self.get_logger().info('Falling left')

    def goalkeeper_fall_right(self):
        self.send_goal(12)
        self.get_logger().info('Falling right')

    def turn_head_left(self):
        self.send_goal(21)
        self.get_logger().info('Turning head left')
        
    def turn_head_right(self):
        self.send_goal(22)
        self.get_logger().info('Turning head right')

    def turn_head_down(self):
        self.send_goal(23)
        self.get_logger().info('Turning head down')

    def open_right_kick(self):
        self.send_goal(26)
        self.get_logger().info('Chute direito angulado')

    def open_left_kick(self):
        self.send_goal(27)
        self.get_logger().info('Chute esquerdo angulado')

    def search_goalkeeper(self): #centralizando acima da bola
        self.send_goal(24)
        self.get_logger().info('Procurando o goleiro')

    def sidle_left(self):
        self.send_goal(19)
        self.get_logger().info('Andando pra esquerda')

    def sidle_right(self):
        self.send_goal(20)
        self.get_logger().info('Andando pra direita')

    def goalkeeper_penalty_feather_falling_right(self):
        self.send_goal(30)
        self.get_logger().info('Goalkeeper Feather Falling Right')

    def goalkeeper_penalty_feather_falling_left(self):
        self.send_goal(1)
        self.get_logger().info('Goalkeeper Feather Falling Left')

    def goalkeeper_search_ball(self):
        self.send_goal(38)
        self.get_logger().info('Goalkeeper Feather Falling Left')
    
    def goalkeeper_centralize(self):
        self.send_goal(39)
        self.get_logger().info('Goalkeeper Feather Falling Left')

    def stand_left(self):
        self.send_goal(33)
        self.get_logger().info('Stand up left')

    def stand_right(self):
        self.send_goal(34)
        self.get_logger().info('Stand up right')



def main(args=None):
    rclpy.init(args=args)

    decisionNode = DecisionNode()

    rclpy.spin(decisionNode)
    decisionNode.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()