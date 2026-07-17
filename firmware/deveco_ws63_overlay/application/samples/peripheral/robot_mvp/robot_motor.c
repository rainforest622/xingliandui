#include "robot_motor.h"

#include "i2c.h"
#include "robot_i2c.h"

#define ROBOT_MOTOR_PWM_I2C_ADDR 0x5AU
#define ROBOT_MOTOR_PWM_LIMIT 999U
#define ROBOT_MOTOR_PWM_FREQ_1KHZ 0x16U
#define ROBOT_MOTOR_LEFT_WHEEL_SCALE_NUM 100
#define ROBOT_MOTOR_LEFT_WHEEL_SCALE_DEN 100

static bool robot_pwm_write(const uint8_t *buffer, uint32_t length)
{
    i2c_data_t data = {
        .send_buf = (uint8_t *)buffer,
        .send_len = length,
    };
    return uapi_i2c_master_write(ROBOT_I2C_BUS, ROBOT_MOTOR_PWM_I2C_ADDR, &data) == ERRCODE_SUCC;
}

static bool robot_pwm_channel(uint8_t channel, uint16_t pwm_value)
{
    uint8_t packet[3] = {
        channel,
        (uint8_t)(pwm_value >> 8),
        (uint8_t)(pwm_value & 0xFFU),
    };
    return robot_pwm_write(packet, sizeof(packet));
}

static uint16_t percent_to_pwm_value(int8_t percent)
{
    uint8_t magnitude = (uint8_t)((percent < 0) ? -percent : percent);
    return (uint16_t)(((uint32_t)magnitude * ROBOT_MOTOR_PWM_LIMIT) / 100U);
}

static bool set_right_wheel(int8_t percent)
{
    uint16_t pwm_value = percent_to_pwm_value(percent);
    if (percent >= 0) {
        return robot_pwm_channel(0x70U, 0U) &&
               robot_pwm_channel(0x80U, pwm_value);
    }
    return robot_pwm_channel(0x80U, 0U) &&
           robot_pwm_channel(0x70U, pwm_value);
}

static bool set_left_wheel(int8_t percent)
{
    uint16_t pwm_value = percent_to_pwm_value(percent);
    if (percent >= 0) {
        return robot_pwm_channel(0xA0U, 0U) &&
               robot_pwm_channel(0x90U, pwm_value);
    }
    return robot_pwm_channel(0x90U, 0U) &&
           robot_pwm_channel(0xA0U, pwm_value);
}

static int8_t calibrate_left_wheel(int8_t percent)
{
    int16_t value = percent;
    int16_t magnitude = (value < 0) ? (int16_t)(-value) : value;
    magnitude = (int16_t)(((magnitude * ROBOT_MOTOR_LEFT_WHEEL_SCALE_NUM) +
        (ROBOT_MOTOR_LEFT_WHEEL_SCALE_DEN / 2)) / ROBOT_MOTOR_LEFT_WHEEL_SCALE_DEN);
    return (int8_t)((value < 0) ? -magnitude : magnitude);
}

bool robot_motor_stop(void)
{
    bool right_ok = set_right_wheel(0);
    bool left_ok = set_left_wheel(0);
    return right_ok && left_ok;
}

bool robot_motor_apply(int8_t left_percent, int8_t right_percent)
{
    bool left_ok = set_left_wheel(calibrate_left_wheel(left_percent));
    bool right_ok = set_right_wheel(right_percent);
    if (!left_ok || !right_ok) {
        (void)robot_motor_stop();
        return false;
    }
    return true;
}

bool robot_motor_init(void)
{
    uint8_t frequency = ROBOT_MOTOR_PWM_FREQ_1KHZ;

    if (!robot_i2c_ensure_init()) {
        return false;
    }

    if (!robot_pwm_write(&frequency, sizeof(frequency))) {
        return false;
    }
    return robot_motor_stop();
}
