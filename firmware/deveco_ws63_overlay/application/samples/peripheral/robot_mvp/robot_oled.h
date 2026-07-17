#ifndef ROBOT_MVP_OLED_H
#define ROBOT_MVP_OLED_H

#include <stdbool.h>
#include <stdint.h>

bool robot_oled_init(void);
bool robot_oled_is_ready(void);
void robot_oled_render(
    bool motor_ready,
    bool moving,
    uint8_t last_command,
    uint8_t last_sequence,
    uint32_t uptime_ms,
    bool env_valid,
    int16_t temperature_deci_c,
    uint16_t humidity_deci_percent
);

#endif
