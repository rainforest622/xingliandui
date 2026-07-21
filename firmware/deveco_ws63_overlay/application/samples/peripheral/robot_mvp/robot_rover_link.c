#include "robot_rover_link.h"

#include <stdio.h>

#include "soc_osal.h"

#define ROBOT_ROVER_MAX_SPEED_CENTI 50
#define ROBOT_ROVER_WHEEL_SCALE_MIN 70U
#define ROBOT_ROVER_WHEEL_SCALE_MAX 120U
#define ROBOT_ROVER_WHEEL_SCALE_DEFAULT 100U

static uint8_t g_left_wheel_scale = ROBOT_ROVER_WHEEL_SCALE_DEFAULT;
static uint8_t g_right_wheel_scale = ROBOT_ROVER_WHEEL_SCALE_DEFAULT;

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

static int8_t calibrate_wheel(int8_t percent, uint8_t scale)
{
    int16_t value = percent;
    int16_t magnitude = (value < 0) ? (int16_t)(-value) : value;

    magnitude = (int16_t)(((magnitude * scale) + 50) / 100);
    if (magnitude > 100) {
        magnitude = 100;
    }
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
    uint16_t left_abs = (uint16_t)((left_centi < 0) ? -left_centi : left_centi);
    uint16_t right_abs = (uint16_t)((right_centi < 0) ? -right_centi : right_centi);
    const char *left_sign = (left_centi < 0) ? "-" : "";
    const char *right_sign = (right_centi < 0) ? "-" : "";

    len = snprintf(buffer, sizeof(buffer),
        "{\"T\":1,\"L\":%s0.%02u,\"R\":%s0.%02u}",
        left_sign, left_abs, right_sign, right_abs);
    if ((len <= 0) || ((uint32_t)len >= sizeof(buffer))) {
        return false;
    }

    /* The board routes the stable Type-C output through osal_printk. Emit a
     * standalone JSON line so the Pi bridge can use its normal line parser. */
    osal_printk("\r\n%s\r\n", buffer);
    return true;
}

bool robot_rover_link_apply(int8_t left_percent, int8_t right_percent)
{
    int16_t left_centi = percent_to_centi_speed(calibrate_wheel(left_percent, g_left_wheel_scale));
    int16_t right_centi = percent_to_centi_speed(calibrate_wheel(right_percent, g_right_wheel_scale));

    return write_json_speed(left_centi, right_centi);
}

bool robot_rover_link_stop(void)
{
    return robot_rover_link_apply(0, 0);
}

bool robot_rover_link_request_map_patrol(void)
{
    /* The Pi owns route timing and obstacle avoidance; this is a control event,
     * not a motor command. */
    osal_printk("\r\n{\"mode\":\"auto_map\",\"route_action\":\"start\"}\r\n");
    return true;
}

bool robot_rover_link_init(void)
{
    return robot_rover_link_stop();
}

bool robot_rover_link_set_wheel_scale(uint8_t left_percent, uint8_t right_percent)
{
    if ((left_percent < ROBOT_ROVER_WHEEL_SCALE_MIN) || (left_percent > ROBOT_ROVER_WHEEL_SCALE_MAX) ||
        (right_percent < ROBOT_ROVER_WHEEL_SCALE_MIN) || (right_percent > ROBOT_ROVER_WHEEL_SCALE_MAX)) {
        return false;
    }
    g_left_wheel_scale = left_percent;
    g_right_wheel_scale = right_percent;
    return true;
}

void robot_rover_link_get_wheel_scale(uint8_t *left_percent, uint8_t *right_percent)
{
    if (left_percent != NULL) {
        *left_percent = g_left_wheel_scale;
    }
    if (right_percent != NULL) {
        *right_percent = g_right_wheel_scale;
    }
}
