#include "dut_motor.h"

#include "i2c.h"
#include "robot_i2c.h"

#define DUT_PWM_I2C_ADDR 0x5AU
#define DUT_PWM_LIMIT 999U
#define DUT_PWM_FREQ_1KHZ 0x16U

static bool dut_pwm_write(const uint8_t *buffer, uint32_t length)
{
    i2c_data_t data = {
        .send_buf = (uint8_t *)buffer,
        .send_len = length,
    };
    return uapi_i2c_master_write(ROBOT_I2C_BUS, DUT_PWM_I2C_ADDR, &data) == ERRCODE_SUCC;
}

static bool dut_pwm_channel(uint8_t channel, uint16_t duty)
{
    uint8_t packet[3] = {
        channel,
        (uint8_t)(duty >> 8),
        (uint8_t)(duty & 0xFFU),
    };
    return dut_pwm_write(packet, sizeof(packet));
}

static uint16_t percent_to_duty(int8_t percent)
{
    uint8_t magnitude = (uint8_t)((percent < 0) ? -percent : percent);
    return (uint16_t)(((uint32_t)magnitude * DUT_PWM_LIMIT) / 100U);
}

static bool set_right_wheel(int8_t percent)
{
    uint16_t duty = percent_to_duty(percent);
    if (percent >= 0) {
        return dut_pwm_channel(0x70U, 0U) &&
               dut_pwm_channel(0x80U, duty);
    }
    return dut_pwm_channel(0x80U, 0U) &&
           dut_pwm_channel(0x70U, duty);
}

static bool set_left_wheel(int8_t percent)
{
    uint16_t duty = percent_to_duty(percent);
    if (percent >= 0) {
        return dut_pwm_channel(0xA0U, 0U) &&
               dut_pwm_channel(0x90U, duty);
    }
    return dut_pwm_channel(0x90U, 0U) &&
           dut_pwm_channel(0xA0U, duty);
}

bool dut_motor_stop(void)
{
    bool right_ok = set_right_wheel(0);
    bool left_ok = set_left_wheel(0);
    return right_ok && left_ok;
}

bool dut_motor_apply(int8_t left_percent, int8_t right_percent)
{
    bool left_ok = set_left_wheel(left_percent);
    bool right_ok = set_right_wheel(right_percent);
    if (!left_ok || !right_ok) {
        (void)dut_motor_stop();
        return false;
    }
    return true;
}

bool dut_motor_init(void)
{
    uint8_t frequency = DUT_PWM_FREQ_1KHZ;

    if (!robot_i2c_ensure_init()) {
        return false;
    }

    if (!dut_pwm_write(&frequency, sizeof(frequency))) {
        return false;
    }
    return dut_motor_stop();
}
