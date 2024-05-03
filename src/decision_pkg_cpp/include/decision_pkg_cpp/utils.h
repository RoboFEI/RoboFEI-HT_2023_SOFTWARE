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

#endif // UTILS_H