#ifndef ATTRIBUTES_H
#define ATTRIBUTES_H

#include "custom_interfaces/msg/neck_position.hpp"
#include "custom_interfaces/msg/humanoid_league_msgs.hpp"


struct Robot
{
    FallStatus fall = NotFallen;
    Move movement = stand_still;
    bool finished_move = true;
    custom_interfaces::msg::NeckPosition neck_pos;
};


enum Move
{
    stand_still     = 1,
    greeting        = 2,
    right_kick      = 3,
    left_kick       = 4,
    turn_right      = 5,
    turn_left       = 6,
    goodbye         = 7,
    walk            = 14,
    gait            = 15,
    stand_up_back   = 16,
    stand_up_front  = 17,
    stand_up_side   = 18
};



enum FallStatus
{
    NotFallen   = 0,
    FallenFront = 1,
    FallenBack  = 2,
    FallenRight = 3,
    FallenLeft  = 4
};


















#endif // ATTRIBUTES_H