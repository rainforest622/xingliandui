#ifndef ROBOT_MVP_ENV_H
#define ROBOT_MVP_ENV_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    bool valid;
    int16_t temperature_deci_c;
    uint16_t humidity_deci_percent;
} robot_env_data_t;

bool robot_env_read(robot_env_data_t *env);

#endif
