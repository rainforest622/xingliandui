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
    uint32_t test_duration_ms;
} robot_mvp_avoid_result_t;

typedef struct {
    bool active;
    uint8_t phase;
    robot_status_t status;
    uint8_t leg_index;
    uint32_t loop_count;
    uint32_t alarm_flags;
    robot_obstacle_data_t obstacle;
} robot_mvp_patrol_result_t;

typedef struct {
    uint32_t forward_ms;
    uint32_t turn_ms;
    uint8_t speed;
    uint32_t max_loops;
} robot_mvp_patrol_config_t;

typedef enum {
    ROBOT_MONITOR_ALARM_NONE = 0,
    ROBOT_MONITOR_ALARM_ENV_INVALID = 1U << 0,
    ROBOT_MONITOR_ALARM_OBSTACLE_INVALID = 1U << 1,
    ROBOT_MONITOR_ALARM_OBSTACLE_BLOCKED = 1U << 2,
    ROBOT_MONITOR_ALARM_TEMP_HIGH = 1U << 3,
    ROBOT_MONITOR_ALARM_HUMIDITY_HIGH = 1U << 4
} robot_mvp_monitor_alarm_t;

typedef struct {
    robot_mvp_state_t state;
    robot_env_data_t env;
    robot_obstacle_data_t obstacle;
    uint32_t alarm_flags;
    uint32_t sample_count;
    uint32_t env_age_ms;
    uint32_t obstacle_age_ms;
} robot_mvp_monitor_t;

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
robot_status_t robot_mvp_control_start_avoid_test(robot_mvp_avoid_result_t *result);
bool robot_mvp_control_config_patrol(const robot_mvp_patrol_config_t *config);
void robot_mvp_control_get_patrol_config(robot_mvp_patrol_config_t *config);
robot_status_t robot_mvp_control_start_patrol(robot_mvp_patrol_result_t *result);
void robot_mvp_control_get_patrol(robot_mvp_patrol_result_t *result);
void robot_mvp_control_get_state(robot_mvp_state_t *state);
bool robot_mvp_control_get_monitor(robot_mvp_monitor_t *monitor);

#endif
