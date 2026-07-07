#include "robot_env.h"

#include "i2c.h"
#include "robot_i2c.h"
#include "soc_osal.h"

#define AHT20_I2C_ADDR 0x38U
#define AHT20_CMD_RESET 0xBAU
#define AHT20_CMD_INIT 0xBEU
#define AHT20_CMD_TRIGGER 0xACU
#define AHT20_STATUS_BUSY 0x80U
#define AHT20_RESET_WAIT_MS 20U
#define AHT20_CALIBRATE_WAIT_MS 40U
#define AHT20_MEASURE_WAIT_MS 90U
#define AHT20_BUSY_RETRY_WAIT_MS 10U
#define AHT20_BUSY_RETRIES 10U

static bool g_aht20_initialized;

static bool aht20_write(const uint8_t *buffer, uint32_t length)
{
    i2c_data_t data = {
        .send_buf = (uint8_t *)buffer,
        .send_len = length,
    };
    return uapi_i2c_master_write(ROBOT_I2C_BUS, AHT20_I2C_ADDR, &data) == ERRCODE_SUCC;
}

static bool aht20_read(uint8_t *buffer, uint32_t length)
{
    i2c_data_t data = {
        .receive_buf = buffer,
        .receive_len = length,
    };
    return uapi_i2c_master_read(ROBOT_I2C_BUS, AHT20_I2C_ADDR, &data) == ERRCODE_SUCC;
}

static bool aht20_init(void)
{
    uint8_t reset_cmd[1] = {AHT20_CMD_RESET};
    uint8_t init_cmd[3] = {AHT20_CMD_INIT, 0x08U, 0x00U};

    if (g_aht20_initialized) {
        return true;
    }
    if (!robot_i2c_ensure_init()) {
        return false;
    }

    if (!aht20_write(reset_cmd, sizeof(reset_cmd))) {
        return false;
    }
    osal_msleep(AHT20_RESET_WAIT_MS);

    if (!aht20_write(init_cmd, sizeof(init_cmd))) {
        return false;
    }
    osal_msleep(AHT20_CALIBRATE_WAIT_MS);

    g_aht20_initialized = true;
    return true;
}

bool robot_env_read(robot_env_data_t *env)
{
    uint8_t trigger_cmd[3] = {AHT20_CMD_TRIGGER, 0x33U, 0x00U};
    uint8_t raw[6] = {0};
    uint32_t attempt;
    uint32_t humidity_raw;
    uint32_t temperature_raw;

    if (env == NULL) {
        return false;
    }

    env->valid = false;
    env->temperature_deci_c = 0;
    env->humidity_deci_percent = 0U;

    if (!aht20_init()) {
        return false;
    }
    if (!aht20_write(trigger_cmd, sizeof(trigger_cmd))) {
        return false;
    }

    osal_msleep(AHT20_MEASURE_WAIT_MS);

    for (attempt = 0U; attempt < AHT20_BUSY_RETRIES; ++attempt) {
        if (!aht20_read(raw, sizeof(raw))) {
            return false;
        }
        if ((raw[0] & AHT20_STATUS_BUSY) == 0U) {
            humidity_raw = ((uint32_t)raw[1] << 12) | ((uint32_t)raw[2] << 4) | ((uint32_t)raw[3] >> 4);
            temperature_raw = (((uint32_t)raw[3] & 0x0FU) << 16) | ((uint32_t)raw[4] << 8) | raw[5];

            env->humidity_deci_percent = (uint16_t)(((uint64_t)humidity_raw * 1000U) >> 20);
            env->temperature_deci_c = (int16_t)((((int64_t)temperature_raw * 2000) >> 20) - 500);
            env->valid = true;
            return true;
        }
        osal_msleep(AHT20_BUSY_RETRY_WAIT_MS);
    }
    return false;
}
