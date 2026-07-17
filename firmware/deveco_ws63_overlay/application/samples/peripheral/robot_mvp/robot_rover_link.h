#ifndef ROBOT_MVP_ROVER_LINK_H
#define ROBOT_MVP_ROVER_LINK_H

#include <stdbool.h>
#include <stdint.h>

bool robot_rover_link_init(void);
bool robot_rover_link_apply(int8_t left_percent, int8_t right_percent);
bool robot_rover_link_stop(void);

#endif
