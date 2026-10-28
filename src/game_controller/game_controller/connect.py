import socket
import time
import rclpy

from construct import Container, ConstError
from rclpy import logging
from rclpy.node import Node
from std_msgs.msg import Bool
from game_controller.gamestate import GameState, ReturnData, GAME_CONTROLLER_RESPONSE_VERSION
from custom_interfaces.msg import HumanoidLeagueMsgs as GameStateMsg

logger = logging.get_logger('humanoid_league_game_controller')


class GameStateReceiver(Node):
    """ This class puts up a simple UDP Server which receives the
    *addr* parameter to listen to the packages from the game_controller.
    If it receives a package it will be interpreted with the construct data
    structure and the :func:`on_new_gamestate` will be called with the content.
    After this we send a package back to the GC """

    def __init__(self):
        super().__init__('game_controller')

        self.declare_parameter('team_number', 65)
        self.declare_parameter('robot_number', 1)
        self.team = self.get_parameter('team_number').get_parameter_value().integer_value
        self.player_number = self.get_parameter('robot_number').get_parameter_value().integer_value
        logger.info('We are playing as player {} in team {}'.format(self.player_number, self.team))

        self.state_publisher = self.create_publisher(GameStateMsg, 'gamestate', 1)

        self.man_penalize = False
        self.game_controller_lost_time = 20
        self.game_controller_connected_publisher = self.create_publisher(Bool, 'game_controller_connected', 1)
        #logger.info("Declarado publisher do GameController quando conectado")

        # The address listening on and the port for sending back the robots meta data
        self.declare_parameter('listen_host', '0.0.0.0')
        self.declare_parameter('listen_port',3838)
        listen_host = self.get_parameter('listen_host').value
        listen_port = self.get_parameter('listen_port').value
        self.declare_parameter('answer_port',3939)
        #logger.info("Declarado portas")


        self.addr = (listen_host, listen_port)
        self.answer_port = self.get_parameter('answer_port').value

        # The state and time we received last form the GC
        self.state = None
        self.time = time.time()

        # The socket and whether it is still running
        self.socket = None
        self.running = True

        self._open_socket()

    def _open_socket(self):
        """ Creates the socket """
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket.bind(self.addr)
        self.socket.settimeout(2)
        self.socket2 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        self.socket2.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        #logger.info("Socket aberta")

    def receive_forever(self):
        """ Waits in a loop that is terminated by setting self.running = False """
        while True:
            try:
                self.receive_once()
            except IOError as e:
                logger.warn("Error while sending keepalive: " + str(e))

    def receive_once(self):
        """ Receives a package and interprets it.
            Calls :func:`on_new_gamestate`
            Sends an answer to the GC """
        try:
            #logger.info("Tentando receber e interpretar o pacote")
            data, peer = self.socket.recvfrom(GameState.sizeof())

            # Throws a ConstError if it doesn't work
            #logger.info("Package recebido, tentando interpretar")
            parsed_state = GameState.parse(data)

            # Assign the new package after it parsed successful to the state
            #logger.info("Sla, mas ta funcionando")
            self.state = parsed_state
            self.time = time.time()

            # Publish that game controller received message
            #logger.info("Criando publisher para o package")
            msg = Bool()
            msg.data = True
            self.game_controller_connected_publisher.publish(msg)

            # Call the handler for the package
            #logger.info("Chamando o handler para o package")
            self.on_new_gamestate(self.state)

            # Answer the GameController
            #logger.info("Respondendo o GameController")
            self.answer_to_gamecontroller(peer)

        except AssertionError as ae:
            logger.error(ae)
        except socket.timeout:
            logger.info("No GameController message received (socket timeout)", throttle_duration_sec=5)
        except ConstError:
            logger.warn("Parse Error: Probably using an old protocol!")
        finally:
            if self.get_time_since_last_package() > self.game_controller_lost_time:
                self.time += 5  # Resend message every five seconds
                logger.info("No GameController message received, allowing robot to move", throttle_duration_sec=5)
                msg = GameStateMsg()
                msg.state = 3  # PLAYING
                self.state_publisher.publish(msg)
                msg2 = Bool()
                msg2.data = False
                self.game_controller_connected_publisher.publish(msg2)

    def answer_to_gamecontroller(self, peer):
        """ Sends a life sign to the game controller """

        #logger.info("Enviando data package de volta para o GameController")
        data = Container(header=b"RGrt",
                         version=GAME_CONTROLLER_RESPONSE_VERSION,
                         player=self.player_number,
                         team=self.team,
                         fallen=0,
                         pose=[0,0,0],
                         ballAge=0,
                         ball=[0,0])
        try:
            destination = peer[0], self.answer_port
            logger.debug('Sending answer to {} port {}'.format(destination[0], destination[1]))
            self.socket.sendto(ReturnData.build(data), destination)
        except Exception as e:
            logger.error("Network Error: %s" % str(e))

    def on_new_gamestate(self, state):
        """ Is called with the new game state after receiving a package.
            The information is processed and published as a standard message to a ROS topic.
            :param state: Game State
        """

        #logger.info("Chamando novo GameState ou sla oq isso faz")
        is_own_team = lambda number: number == self.team
        own_team = self.select_team_by(is_own_team, state.teams)

        is_not_own_team = lambda number: number != self.team
        rival_team = self.select_team_by(is_not_own_team, state.teams)

        if not own_team or not rival_team:
            logger.error('Team {} not playing, only {} and {}'.format(self.team, state.teams[0].team_number,
                                                                      state.teams[1].team_number))
            return

        try:
            me = own_team.players[self.player_number - 1]
        except IndexError:
            logger.error('Robot {} not playing'.format(self.player_number))
            return
       
        msg = GameStateMsg()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.game_phase = state.game_phase.intvalue
        msg.state = state.state.intvalue
        msg.set_play = state.set_play.intvalue
        msg.first_half = state.first_half
        msg.own_score = own_team.score
        msg.rival_score = rival_team.score
        msg.seconds_remaining = state.seconds_remaining
        msg.secondary_time = state.secondary_time
        msg.has_kick_off = state.kicking_team == self.team
        msg.penalized = True if int(me.penalty) >0 else False
        msg.seconds_till_unpenalized = me.seconds_till_unpenalized
        msg.team_color = own_team.team_color.intvalue
        # msg.drop_in_team = state.drop_in_team
        # msg.drop_in_time = state.drop_in_time
        msg.penalty_shot = own_team.penalty_shot
        msg.single_shots = own_team.single_shots
        # msg.coach_message = own_team.coach_message
        penalties = []
        red_cards = []
        for i in range(min(state.players_per_team, len(own_team.players))):
            penalties.append(int(own_team.players[i].penalty) != 0)
            red_cards.append(int(own_team.players[i].penalty) == 12)
        msg.team_mates_with_penalty = penalties
        msg.team_mates_with_red_card = red_cards
        msg.team_number = own_team.team_number
        self.state_publisher.publish(msg)

    def get_last_state(self):
        return self.state, self.time

    def get_time_since_last_package(self):
        return time.time() - self.time

    def stop(self):
        self.running = False

    def set_manual_penalty(self, flag):
        self.man_penalize = flag

    def select_team_by(self, predicate, teams):
        selected = [team for team in teams if predicate(team.team_number)]
        return next(iter(selected), None)


def main(args=None):
    rclpy.init(args=args)
    receiver = GameStateReceiver()

    try:
        receiver.receive_forever()

    except KeyboardInterrupt:
        receiver.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
    
