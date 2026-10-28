# This message provides all information from the game controller
# for additional information see documentation of the game controller
# https://github.com/bhuman/GameController

# Informações recebidas do GameController

std_msgs/Header header

# Fases da partida:

uint8 GAME_PHASE_NORMAL=0
uint8 GAME_PHASE_PENALTY_SHOOT_OUT=1
uint8 GAME_PHASE_EXTRA_TIME=2
uint8 GAME_PHASE_TIMEOUT=3

uint8 game_phase

# Estados da partida:

uint8 STATE_INITIAL=0
uint8 STATE_READY=1
uint8 STATE_SET=2
uint8 STATE_PLAYING=3
uint8 STATE_FINISHED=4

uint8 state 3

# Jogadas especiais:

uint8 SET_PLAY_NONE=0
uint8 SET_PLAY_DIRECT_FREE_KICK=1
uint8 SET_PLAY_INDIRECT_FREE_KICK=2
uint8 SET_PLAY_PENALTY_KICK=3
uint8 SET_PLAY_THROW_IN=4
uint8 SET_PLAY_GOAL_KICK=5
uint8 SET_PLAY_CORNER_KICK=6

uint8 set_play

# Estados de penalty:

uint8 PENALTY_NONE=0
uint8 PENALTY_ILLEGAL_POSITIONING=1
uint8 PENALTY_MOTION_IN_SET=2
uint8 PENALTY_MOTION_IN_STOP=3
uint8 PENALTY_LOCAL_GAME_STUCK=4
uint8 PENALTY_INCAPABLE_ROBOT=5
uint8 PENALTY_PICK_UP=6
uint8 PENALTY_BALL_HOLDING=7
uint8 PENALTY_LEAVING_THE_FIELD=8
uint8 PENALTY_PLAYING_WITH_ARMS_HANDS=9
uint8 PENALTY_PUSHING=10
uint8 PENALTY_CAUTIONED=11
uint8 PENALTY_SENT_OFF=12
uint8 PENALTY_SUBSTITUTE=13

uint8 penalty

# penaltyshoot is penalty shootout at the end of the game,
# penaltykick is a kick during the game

# Estado de interrupção da partida:

uint8 stopped

# Tipos de competição:

uint8 COMPETITION_TYPE_SMALL=0
uint8 COMPETITION_TYPE_MIDDLE=1
uint8 COMPETITION_TYPE_LARGE=2

uint8 competition_type

# Informações gerais da partida:

uint8 packet_number
uint8 players_per_team
bool first_half
uint8 kicking_team

# Tempos da partida (Em segundos):

int16 seconds_remaining
int16 secondary_time

# Identificação das equipes:

uint8 team_number
uint8 own_score
uint8 rival_score

# Informações da equipe:

uint8 BLUE=0
uint8 RED=1
uint8 YELLOW=2
uint8 BLACK=3
uint8 WHITE=4
uint8 GREEN=5
uint8 ORANGE=6
uint8 PURPLE=7
uint8 BROWN=8
uint8 GRAY=9

uint8 team_color
uint8 goalkeeper_color
uint8 goalkeeper
uint16 message_budget

# Informações derivadas para o comportamento do robô:

bool has_kick_off
bool penalized
uint8 seconds_till_unpenalized

# Informações de cobrança para pênaltis:

uint8 penalty_shot
uint16 single_shots

# Situação dos jogadores da equipe:

bool[] team_mates_with_penalty
bool[] team_mates_with_red_card