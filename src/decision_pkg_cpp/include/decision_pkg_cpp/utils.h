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
    int left_limit = (int) (9.0363e-3 * pow(pos20, 2) - 3.3280 * pos20 + 5.0661e3);
    return left_limit-2048; 
}

#endif // UTILS_H