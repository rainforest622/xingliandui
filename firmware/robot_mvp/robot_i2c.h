#ifndef ROBOT_MVP_I2C_H
#define ROBOT_MVP_I2C_H

#include <stdbool.h>
#include <stdint.h>

#define ROBOT_I2C_BUS 1
#define ROBOT_I2C_BAUDRATE 100000U

bool robot_i2c_ensure_init(void);

#endif
