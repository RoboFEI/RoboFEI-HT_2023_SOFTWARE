# Copyright 1996-2021 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Adapted from from https://github.com/RoboCup-Humanoid-TC/GameController/blob/master/protocols/python/gamestate.py

from construct import Array, Byte, Bytes, Const, Enum, Flag, Int16sl, Int16ul, Struct, Float32l
try:
    from construct import PaddedString
except ImportError:
    # construct < 2.9.39
    from construct import String as PaddedString

Short = Int16ul

RobotInfo = "robot_info" / Struct(
    # NONE                                        0
    # PENALTY_HL_KID_BALL_MANIPULATION            1
    # PENALTY_HL_KID_PHYSICAL_CONTACT             2
    # PENALTY_HL_KID_ILLEGAL_ATTACK               3
    # PENALTY_HL_KID_ILLEGAL_DEFENSE              4
    # PENALTY_HL_KID_REQUEST_FOR_PICKUP           5
    # PENALTY_HL_KID_REQUEST_FOR_SERVICE          6
    # PENALTY_HL_KID_REQUEST_FOR_PICKUP_2_SERVICE 7
    # MANUAL                                      15
    "penalty" / Enum(Byte,
                        PENALTY_NONE = 0,
                        PENALTY_ILLEGAL_POSITIONING = 1,
                        PENALTY_MOTION_IN_SET = 2,
                        PENALTY_MOTION_IN_STOP = 3,
                        PENALTY_LOCAL_GAME_STUCK = 4,
                        PENALTY_INCAPABLE_ROBOT = 5,
                        PENALTY_PICK_UP = 6,
                        PENALTY_BALL_HOLDING = 7,
                        PENALTY_LEAVING_THE_FIELD = 8, 
                        PENALTY_PLAYING_WITH_ARMS_HANDS = 9,
                        PENALTY_PUSHING = 10,
                        PENALTY_CAUTIONED = 11,
                        PENALTY_SENT_OFF = 12,
                        PENALTY_SUBSTITUTE = 13),
    "seconds_till_unpenalized" / Byte,
    "cautions" / Byte
)

TeamInfo = "team" / Struct(
    "team_number" / Byte,
    "team_color" / Enum(Byte,
                        BLUE=0,
                        RED=1,
                        YELLOW=2,
                        BLACK=3,
                        WHITE=4,
                        GREEN=5,
                        ORANGE=6,
                        PURPLE=7,
                        BROWN=8,
                        GRAY=9),
    "goalkeeperColour" / Enum(Byte,
                        BLUE=0,
                        RED=1,
                        YELLOW=2,
                        BLACK=3,
                        WHITE=4,
                        GREEN=5,
                        ORANGE=6,
                        PURPLE=7,
                        BROWN=8,
                        GRAY=9),
    "goalkeeper" / Byte,
    "score" / Byte,
    "penalty_shot" / Byte,  # penalty shot counter
    "single_shots" / Short,  # bits represent penalty shot success
    "messageBudget" / Short,
    "players" / Array(20, RobotInfo)
)

GameState = "gamedata" / Struct(
    "header" / Const(b'RGme'),
    "version" / Const(20, Byte),
    "packet_number" / Byte,
    "players_per_team" / Byte,
    "competition_type" / Byte,
    "stopped" / Byte,
    "game_phase" / Enum(Byte,
                        GAME_PHASE_NORMAL = 0,
                        GAME_PHASE_PENALTY_SHOOT_OUT = 1,
                        GAME_PHASE_EXTRA_TIME = 2,
                        GAME_PHASE_TIMEOUT = 3),
    "state" / Enum(Byte,
                        STATE_INITIAL=0,
                        STATE_READY=1,      # Tempo para o time que tem kick-off jogar (10 segundos no inicio de tempo)
                        STATE_SET=2,        # 
                        STATE_PLAYING=3,    # Estado de jogo normal
                        STATE_FINISHED=4),  # game over
    "set_play" / Enum(Byte,
                        SET_PLAY_NONE = 0,
                        SET_PLAY_DIRECT_FREE_KICK = 1,
                        SET_PLAY_INDIRECT_FREE_KICK = 2,
                        SET_PLAY_PENALTY_KICK = 3,
                        SET_PLAY_THROW_IN = 4,
                        SET_PLAY_GOAL_KICK = 5,
                        SET_PLAY_CORNER_KICK = 6),
    "first_half" / Flag,
    "kicking_team" / Byte,
    "seconds_remaining" / Int16sl,
    "secondary_time" / Int16sl,
    "teams" / Array(2, TeamInfo)
)

GAME_CONTROLLER_RESPONSE_VERSION = 4

ReturnData = Struct(
    "header" / Const(b"RGrt"),
    "version" / Byte,
    "player" / Byte,
    "team" / Byte,
    "fallen" / Byte,
    "pose" / Array(3, Float32l),
    "ballAge" / Float32l,
    "ball" / Array(2, Float32l)
)
