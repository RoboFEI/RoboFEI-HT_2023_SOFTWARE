#ifndef ATTRIBUTES_H
#define ATTRIBUTES_H

#include "custom_interfaces/msg/neck_position.hpp"
#include "custom_interfaces/msg/vision.hpp"
#include "custom_interfaces/msg/humanoid_league_msgs.hpp"
#include "std_msgs/msg/bool.hpp" 
#include "std_msgs/msg/int32.hpp" 
#include "std_msgs/msg/string.hpp" 
#include "custom_interfaces/msg/joint_state.hpp"
#include "custom_interfaces/msg/set_position.hpp"
#include "vision_msgs/msg/point2_d.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp" 

enum State
{
    searching_ball = 1,
    aligning_with_the_ball = 2,
    ball_approach = 3,
    ball_close = 4,
    kick_ball = 5
};

enum FallStatus
{
    NotFallen   = 0,
    FallenFront = 1,
    FallenBack  = 2,
    FallenRight = 3,
    FallenLeft  = 4
};

enum Move
{
    stand_still         = 1,
    greeting            = 2,
    right_kick          = 3,
    left_kick           = 4,
    right_kick_to_right = 31,
    //right_kick_to_left  = 32,
    left_kick_to_left   = 34, 
    //lefk_kick_to_right  = 33,
    turn_right          = 5,
    turn_left           = 6,
    goodbye             = 7,
    squat               = 13,
    walk                = 14,
    gait                = 15,
    stand_up_back       = 16,
    stand_up_front      = 17,
    stand_up_side       = 18,
    walk_left           = 20,
    walk_right          = 24,
    turn_ball_left      = 9,
    turn_ball_right      = 10
};

enum ReadyEtapa
        {
            ETAPA_WALK,
            ETAPA_TURN,
            ETAPA_PARAR
        };

ReadyEtapa ready_etapa = ETAPA_WALK;

enum RobotBallPosition
{
    left,   // 0
    center, // 1
    right   // 2
};

struct NeckPosition
{
    int position19;
    int position20;
};


struct Robot
{
    FallStatus fall = NotFallen;
    Move movement = stand_still;

    bool finished_move = true;
    State state = searching_ball;
    NeckPosition neck_pos;
    custom_interfaces::msg::Vision camera_ball_position;
    RobotBallPosition ball_position = center;
    std_msgs::msg::Bool localization_msg;
    std_msgs::msg::Int32 goalpost_count;
    custom_interfaces::msg::Vision goalpost_division_lines;
    vision_msgs::msg::Point2D goalpost_px_position;
    geometry_msgs::msg::Vector3Stamped imu_gyro;
    float imu_yaw_rad = 0.0;
    

};

#endif // ATTRIBUTES_H
