#ifndef ROBOT_MVP_OBSTACLE_H
#define ROBOT_MVP_OBSTACLE_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    ROBOT_OBSTACLE_REASON_OK = 0,
    ROBOT_OBSTACLE_REASON_NOT_READY = 1,
    ROBOT_OBSTACLE_REASON_ECHO_IDLE_HIGH = 2,
    ROBOT_OBSTACLE_REASON_NO_ECHO_RISE = 3,
    ROBOT_OBSTACLE_REASON_NO_ECHO_FALL = 4,
    ROBOT_OBSTACLE_REASON_INVALID_PULSE = 5,
    ROBOT_OBSTACLE_REASON_OUT_OF_RANGE = 6,
} robot_obstacle_reason_t;

typedef struct {
    bool enabled;
    bool valid;
    bool blocked;
    uint16_t distance_mm;
    uint16_t threshold_mm;
    robot_obstacle_reason_t reason;
} robot_obstacle_data_t;

bool robot_obstacle_init(void);
bool robot_obstacle_read(robot_obstacle_data_t *output);
uint16_t robot_obstacle_threshold_mm(void);

#endif
