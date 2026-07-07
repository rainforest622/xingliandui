#ifndef ROBOT_MVP_OBSTACLE_H
#define ROBOT_MVP_OBSTACLE_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    bool enabled;
    bool valid;
    bool blocked;
    uint16_t distance_mm;
    uint16_t threshold_mm;
} robot_obstacle_data_t;

bool robot_obstacle_init(void);
bool robot_obstacle_read(robot_obstacle_data_t *output);
uint16_t robot_obstacle_threshold_mm(void);

#endif
