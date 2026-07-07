#ifndef ROBOT_MVP_DUT_MOTOR_H
#define ROBOT_MVP_DUT_MOTOR_H

#include <stdbool.h>
#include <stdint.h>

bool dut_motor_init(void);
bool dut_motor_apply(int8_t left_percent, int8_t right_percent);
bool dut_motor_stop(void);

#endif
