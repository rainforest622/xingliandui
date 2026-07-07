#include "robot_obstacle.h"

#include "gpio.h"
#include "pinctrl.h"
#include "soc_osal.h"
#include "tcxo.h"

#define ROBOT_OBSTACLE_ENABLE_HCSR04 1
#define ROBOT_OBSTACLE_TRIG_PIN 0
#define ROBOT_OBSTACLE_ECHO_PIN 1
#define ROBOT_OBSTACLE_PIN_MODE 0
#define ROBOT_OBSTACLE_ECHO_PULL PIN_PULL_TYPE_DOWN
#define ROBOT_OBSTACLE_THRESHOLD_MM 250U
#define ROBOT_OBSTACLE_ECHO_TIMEOUT_US 30000U
#define ROBOT_OBSTACLE_IDLE_TIMEOUT_US 1000U
#define ROBOT_OBSTACLE_TRIGGER_US 10U
#define ROBOT_OBSTACLE_SETTLE_US 2U
#define ROBOT_OBSTACLE_MIN_DISTANCE_MM 20U
#define ROBOT_OBSTACLE_MAX_DISTANCE_MM 4000U
#define ROBOT_OBSTACLE_SOUND_NUMERATOR 343U
#define ROBOT_OBSTACLE_SOUND_DENOMINATOR 2000U

static bool g_obstacle_ready;

uint16_t robot_obstacle_threshold_mm(void)
{
    return ROBOT_OBSTACLE_THRESHOLD_MM;
}

bool robot_obstacle_init(void)
{
#if ROBOT_OBSTACLE_ENABLE_HCSR04
    uapi_gpio_init();
    if (uapi_pin_set_mode(ROBOT_OBSTACLE_TRIG_PIN, ROBOT_OBSTACLE_PIN_MODE) != ERRCODE_SUCC) {
        g_obstacle_ready = false;
        return false;
    }
    if (uapi_pin_set_mode(ROBOT_OBSTACLE_ECHO_PIN, ROBOT_OBSTACLE_PIN_MODE) != ERRCODE_SUCC) {
        g_obstacle_ready = false;
        return false;
    }
    (void)uapi_pin_set_pull(ROBOT_OBSTACLE_ECHO_PIN, ROBOT_OBSTACLE_ECHO_PULL);
    if (uapi_gpio_set_dir(ROBOT_OBSTACLE_TRIG_PIN, GPIO_DIRECTION_OUTPUT) != ERRCODE_SUCC) {
        g_obstacle_ready = false;
        return false;
    }
    if (uapi_gpio_set_dir(ROBOT_OBSTACLE_ECHO_PIN, GPIO_DIRECTION_INPUT) != ERRCODE_SUCC) {
        g_obstacle_ready = false;
        return false;
    }
    (void)uapi_gpio_set_val(ROBOT_OBSTACLE_TRIG_PIN, GPIO_LEVEL_LOW);
    g_obstacle_ready = true;
    return true;
#else
    g_obstacle_ready = false;
    return false;
#endif
}

#if ROBOT_OBSTACLE_ENABLE_HCSR04
static bool wait_for_echo(gpio_level_t target, uint32_t timeout_us, uint64_t *timestamp_us)
{
    uint64_t start_us = uapi_tcxo_get_us();
    if (timestamp_us == NULL) {
        return false;
    }
    while ((uapi_tcxo_get_us() - start_us) < timeout_us) {
        if (uapi_gpio_get_val(ROBOT_OBSTACLE_ECHO_PIN) == target) {
            *timestamp_us = uapi_tcxo_get_us();
            return true;
        }
    }
    return false;
}
#endif

bool robot_obstacle_read(robot_obstacle_data_t *output)
{
    if (output == NULL) {
        return false;
    }

    output->enabled = g_obstacle_ready;
    output->valid = false;
    output->blocked = false;
    output->distance_mm = 0U;
    output->threshold_mm = ROBOT_OBSTACLE_THRESHOLD_MM;
    output->reason = g_obstacle_ready ? ROBOT_OBSTACLE_REASON_NO_ECHO_RISE : ROBOT_OBSTACLE_REASON_NOT_READY;

#if ROBOT_OBSTACLE_ENABLE_HCSR04
    uint64_t echo_start_us = 0U;
    uint64_t echo_end_us = 0U;
    uint32_t pulse_us;
    uint32_t distance_mm;

    if (!g_obstacle_ready) {
        return false;
    }

    if (uapi_gpio_get_val(ROBOT_OBSTACLE_ECHO_PIN) == GPIO_LEVEL_HIGH) {
        if (!wait_for_echo(GPIO_LEVEL_LOW, ROBOT_OBSTACLE_IDLE_TIMEOUT_US, &echo_end_us)) {
            output->reason = ROBOT_OBSTACLE_REASON_ECHO_IDLE_HIGH;
            return false;
        }
    }

    (void)uapi_gpio_set_val(ROBOT_OBSTACLE_TRIG_PIN, GPIO_LEVEL_LOW);
    (void)uapi_tcxo_delay_us(ROBOT_OBSTACLE_SETTLE_US);
    (void)uapi_gpio_set_val(ROBOT_OBSTACLE_TRIG_PIN, GPIO_LEVEL_HIGH);
    (void)uapi_tcxo_delay_us(ROBOT_OBSTACLE_TRIGGER_US);
    (void)uapi_gpio_set_val(ROBOT_OBSTACLE_TRIG_PIN, GPIO_LEVEL_LOW);

    if (!wait_for_echo(GPIO_LEVEL_HIGH, ROBOT_OBSTACLE_ECHO_TIMEOUT_US, &echo_start_us)) {
        output->reason = ROBOT_OBSTACLE_REASON_NO_ECHO_RISE;
        return false;
    }
    if (!wait_for_echo(GPIO_LEVEL_LOW, ROBOT_OBSTACLE_ECHO_TIMEOUT_US, &echo_end_us)) {
        output->reason = ROBOT_OBSTACLE_REASON_NO_ECHO_FALL;
        return false;
    }
    if (echo_end_us <= echo_start_us) {
        output->reason = ROBOT_OBSTACLE_REASON_INVALID_PULSE;
        return false;
    }

    pulse_us = (uint32_t)(echo_end_us - echo_start_us);
    distance_mm = (uint32_t)(((uint64_t)pulse_us * ROBOT_OBSTACLE_SOUND_NUMERATOR) /
        ROBOT_OBSTACLE_SOUND_DENOMINATOR);
    if (distance_mm < ROBOT_OBSTACLE_MIN_DISTANCE_MM) {
        distance_mm = ROBOT_OBSTACLE_MIN_DISTANCE_MM;
    } else if (distance_mm > ROBOT_OBSTACLE_MAX_DISTANCE_MM) {
        distance_mm = ROBOT_OBSTACLE_MAX_DISTANCE_MM;
    }

    output->valid = true;
    output->distance_mm = (uint16_t)distance_mm;
    output->blocked = output->distance_mm < ROBOT_OBSTACLE_THRESHOLD_MM;
    output->reason = ROBOT_OBSTACLE_REASON_OK;
    return true;
#else
    output->reason = ROBOT_OBSTACLE_REASON_NOT_READY;
    return false;
#endif
}
