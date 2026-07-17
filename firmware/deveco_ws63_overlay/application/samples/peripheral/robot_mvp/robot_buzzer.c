#include "robot_buzzer.h"

#include "gpio.h"
#include "pinctrl.h"
#include "robot_mvp_control.h"
#include "soc_osal.h"
#include "tcxo.h"

#define ROBOT_BUZZER_PIN 4
#define ROBOT_BUZZER_PIN_MODE 0
#define ROBOT_BUZZER_ON_LEVEL GPIO_LEVEL_HIGH
#define ROBOT_BUZZER_OFF_LEVEL GPIO_LEVEL_LOW
#define ROBOT_ALARM_LED_R_PIN 9
#define ROBOT_ALARM_LED_PIN_MODE 0
#define ROBOT_ALARM_LED_ON_LEVEL GPIO_LEVEL_HIGH
#define ROBOT_ALARM_LED_OFF_LEVEL GPIO_LEVEL_LOW
#define ROBOT_BUZZER_OBSTACLE_PERIOD_MS 600U
#define ROBOT_BUZZER_ENV_PERIOD_MS 1600U
#define ROBOT_BUZZER_TONE_HALF_PERIOD_US 185U
#define ROBOT_BUZZER_ALARM_TONE_SLICE_MS 18U
#define ROBOT_BUZZER_TEST_DC_MS 700U
#define ROBOT_BUZZER_TEST_TONE_MS 700U
#define ROBOT_BUZZER_TEST_GAP_MS 160U

static bool g_buzzer_ready;
static bool g_buzzer_on;
static bool g_alarm_led_ready;
static bool g_alarm_led_on;

static void buzzer_tone(uint32_t duration_ms);

static bool output_pin_init(pin_t pin, pin_mode_t mode)
{
    if (uapi_pin_set_mode(pin, mode) != ERRCODE_SUCC) {
        return false;
    }
    (void)uapi_pin_set_pull(pin, PIN_PULL_TYPE_DISABLE);
    (void)uapi_pin_set_ds(pin, PIN_DS_7);
    if (uapi_gpio_set_dir(pin, GPIO_DIRECTION_OUTPUT) != ERRCODE_SUCC) {
        return false;
    }
    return true;
}

static void buzzer_set(bool on)
{
    if (!g_buzzer_ready) {
        return;
    }
    if (g_buzzer_on == on) {
        return;
    }
    (void)uapi_gpio_set_val(ROBOT_BUZZER_PIN, on ? ROBOT_BUZZER_ON_LEVEL : ROBOT_BUZZER_OFF_LEVEL);
    g_buzzer_on = on;
}

static void alarm_led_set(bool on)
{
    if (!g_alarm_led_ready) {
        return;
    }
    if (g_alarm_led_on == on) {
        return;
    }
    (void)uapi_gpio_set_val(ROBOT_ALARM_LED_R_PIN, on ? ROBOT_ALARM_LED_ON_LEVEL : ROBOT_ALARM_LED_OFF_LEVEL);
    g_alarm_led_on = on;
}

bool robot_buzzer_init(void)
{
    uapi_gpio_init();

    g_buzzer_ready = output_pin_init(ROBOT_BUZZER_PIN, ROBOT_BUZZER_PIN_MODE);
    g_alarm_led_ready = output_pin_init(ROBOT_ALARM_LED_R_PIN, ROBOT_ALARM_LED_PIN_MODE);

    g_buzzer_on = true;
    buzzer_set(false);
    g_alarm_led_on = true;
    alarm_led_set(false);
    return g_buzzer_ready;
}

bool robot_buzzer_ready(void)
{
    return g_buzzer_ready;
}

bool robot_buzzer_alarm_led_ready(void)
{
    return g_alarm_led_ready;
}

void robot_buzzer_off(void)
{
    buzzer_set(false);
    alarm_led_set(false);
}

void robot_buzzer_update(uint32_t alarm_flags, uint64_t now_ms)
{
    uint32_t slot_ms;
    bool envelope_on = false;
    bool obstacle_alarm = (alarm_flags & ROBOT_MONITOR_ALARM_OBSTACLE_BLOCKED) != 0U;
    bool env_alarm = (alarm_flags & (ROBOT_MONITOR_ALARM_TEMP_HIGH | ROBOT_MONITOR_ALARM_HUMIDITY_HIGH)) != 0U;

    if (!g_buzzer_ready && !g_alarm_led_ready) {
        return;
    }
    if (obstacle_alarm) {
        slot_ms = (uint32_t)(now_ms % ROBOT_BUZZER_OBSTACLE_PERIOD_MS);
        envelope_on = slot_ms < 120U || (slot_ms >= 180U && slot_ms < 300U) ||
            (slot_ms >= 360U && slot_ms < 480U);
    } else if (env_alarm) {
        slot_ms = (uint32_t)(now_ms % ROBOT_BUZZER_ENV_PERIOD_MS);
        envelope_on = slot_ms < 180U || (slot_ms >= 300U && slot_ms < 480U);
    }

    if (envelope_on) {
        alarm_led_set(true);
        buzzer_tone(ROBOT_BUZZER_ALARM_TONE_SLICE_MS);
        return;
    }
    buzzer_set(false);
    alarm_led_set(false);
}

static void buzzer_tone(uint32_t duration_ms)
{
    uint64_t end_us = uapi_tcxo_get_us() + ((uint64_t)duration_ms * 1000U);

    while (uapi_tcxo_get_us() < end_us) {
        buzzer_set(true);
        uapi_tcxo_delay_us(ROBOT_BUZZER_TONE_HALF_PERIOD_US);
        buzzer_set(false);
        uapi_tcxo_delay_us(ROBOT_BUZZER_TONE_HALF_PERIOD_US);
    }
    buzzer_set(false);
}

static void buzzer_dc(uint32_t duration_ms)
{
    buzzer_set(true);
    osal_msleep(duration_ms);
    buzzer_set(false);
}

bool robot_buzzer_test(void)
{
    if ((!g_buzzer_ready && !g_alarm_led_ready) && !robot_buzzer_init()) {
        return false;
    }

    for (uint8_t i = 0; i < 3U; ++i) {
        alarm_led_set(true);
        buzzer_dc(ROBOT_BUZZER_TEST_DC_MS);
        osal_msleep(ROBOT_BUZZER_TEST_GAP_MS);
        buzzer_tone(ROBOT_BUZZER_TEST_TONE_MS);
        alarm_led_set(false);
        osal_msleep(ROBOT_BUZZER_TEST_GAP_MS);
    }
    return g_buzzer_ready;
}
