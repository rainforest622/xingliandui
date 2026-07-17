#include "robot_rover_link.h"

#include <stdio.h>

#include "uart.h"

#define ROBOT_ROVER_UART_BUS UART_BUS_0
#define ROBOT_ROVER_MAX_SPEED_CENTI 50
#define ROBOT_ROVER_LEFT_WHEEL_SCALE_NUM 100
#define ROBOT_ROVER_LEFT_WHEEL_SCALE_DEN 100

static int8_t clamp_percent(int8_t value)
{
    if (value > 100) {
        return 100;
    }
    if (value < -100) {
        return -100;
    }
    return value;
}

static int8_t calibrate_left_wheel(int8_t percent)
{
    int16_t value = percent;
    int16_t magnitude = (value < 0) ? (int16_t)(-value) : value;

    magnitude = (int16_t)(((magnitude * ROBOT_ROVER_LEFT_WHEEL_SCALE_NUM) +
        (ROBOT_ROVER_LEFT_WHEEL_SCALE_DEN / 2)) / ROBOT_ROVER_LEFT_WHEEL_SCALE_DEN);
    return (int8_t)((value < 0) ? -magnitude : magnitude);
}

static int16_t percent_to_centi_speed(int8_t percent)
{
    int32_t scaled = (int32_t)clamp_percent(percent) * ROBOT_ROVER_MAX_SPEED_CENTI;

    if (scaled >= 0) {
        scaled += 50;
    } else {
        scaled -= 50;
    }
    return (int16_t)(scaled / 100);
}

static bool write_json_speed(int16_t left_centi, int16_t right_centi)
{
    char buffer[48];
    int len;
    int32_t written;
    uint16_t left_abs = (uint16_t)((left_centi < 0) ? -left_centi : left_centi);
    uint16_t right_abs = (uint16_t)((right_centi < 0) ? -right_centi : right_centi);
    const char *left_sign = (left_centi < 0) ? "-" : "";
    const char *right_sign = (right_centi < 0) ? "-" : "";

    len = snprintf(buffer, sizeof(buffer),
        "{\"T\":1,\"L\":%s0.%02u,\"R\":%s0.%02u}\n",
        left_sign, left_abs, right_sign, right_abs);
    if ((len <= 0) || ((uint32_t)len >= sizeof(buffer))) {
        return false;
    }

    written = uapi_uart_write(ROBOT_ROVER_UART_BUS, (uint8_t *)buffer, (uint32_t)len, 0U);
    return written == len;
}

bool robot_rover_link_apply(int8_t left_percent, int8_t right_percent)
{
    int16_t left_centi = percent_to_centi_speed(calibrate_left_wheel(left_percent));
    int16_t right_centi = percent_to_centi_speed(right_percent);

    return write_json_speed(left_centi, right_centi);
}

bool robot_rover_link_stop(void)
{
    return robot_rover_link_apply(0, 0);
}

bool robot_rover_link_init(void)
{
    return robot_rover_link_stop();
}
