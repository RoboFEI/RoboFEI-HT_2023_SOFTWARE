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
    int left_limit = (int) (3.0761e-4 * pow(pos20, 2) - 1.44906 * pos20 + 3.7726e3);
    return left_limit-2048; 
}

#endif // UTILS_H
