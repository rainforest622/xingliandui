#ifndef ROBOT_MVP_ROBOT_MOTOR_H
#define ROBOT_MVP_ROBOT_MOTOR_H

#include <stdbool.h>
#include <stdint.h>

bool robot_motor_init(void);
bool robot_motor_apply(int8_t left_percent, int8_t right_percent);
bool robot_motor_stop(void);

#endif
