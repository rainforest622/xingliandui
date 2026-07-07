#ifndef ROBOT_MVP_CONTROL_H
#define ROBOT_MVP_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

#include "robot_env.h"
#include "robot_obstacle.h"
#include "robot_protocol.h"

typedef struct {
    uint32_t uptime_ms;
    bool motor_ready;
    bool moving;
    uint8_t last_command;
    uint8_t last_sequence;
    uint32_t command_age_ms;
} robot_mvp_state_t;

typedef struct {
    bool active;
    uint8_t phase;
    robot_status_t status;
    robot_obstacle_data_t obstacle;
} robot_mvp_avoid_result_t;

robot_status_t robot_mvp_control_motion(
    robot_command_code_t command,
    uint8_t speed,
    uint8_t *sequence,
    bool *moving
);
bool robot_mvp_control_motor_init(bool *ready);
bool robot_mvp_control_oled_init(bool *ready);
bool robot_mvp_control_read_env(robot_env_data_t *env);
bool robot_mvp_control_read_obstacle(robot_obstacle_data_t *obstacle);
robot_status_t robot_mvp_control_start_avoid(robot_mvp_avoid_result_t *result);
void robot_mvp_control_get_state(robot_mvp_state_t *state);

#endif
