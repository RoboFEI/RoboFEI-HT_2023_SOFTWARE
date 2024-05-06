#ifndef UTILS_H
#define UTILS_H

#include "attributes.h"

bool is_goalkeeper(const int &robot_number)
{
    return robot_number == 1;
}

bool robot_fallen(const Robot &robot)
{
    return robot.fall != NotFallen;
}

int get_center_th(const int &pos20)
{
    int left_limit = (int) 1.0474e-3 * pow(pos20, 2) - 4.0864 * pos20 + 6160;
    return left_limit; 
}

#endif // UTILS_H