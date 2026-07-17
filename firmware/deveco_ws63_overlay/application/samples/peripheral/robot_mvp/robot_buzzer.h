#ifndef ROBOT_MVP_BUZZER_H
#define ROBOT_MVP_BUZZER_H

#include <stdbool.h>
#include <stdint.h>

bool robot_buzzer_init(void);
bool robot_buzzer_ready(void);
bool robot_buzzer_alarm_led_ready(void);
void robot_buzzer_update(uint32_t alarm_flags, uint64_t now_ms);
void robot_buzzer_off(void);
bool robot_buzzer_test(void);

#endif
