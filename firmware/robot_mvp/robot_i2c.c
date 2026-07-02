#include "robot_i2c.h"

#include "i2c.h"
#include "pinctrl.h"

#define ROBOT_I2C_SCL_PIN 15
#define ROBOT_I2C_SDA_PIN 16
#define ROBOT_I2C_PIN_MODE 2

static bool g_robot_i2c_initialized;

bool robot_i2c_ensure_init(void)
{
    if (g_robot_i2c_initialized) {
        return true;
    }

    uapi_pin_init();
    uapi_pin_set_mode(ROBOT_I2C_SCL_PIN, ROBOT_I2C_PIN_MODE);
    uapi_pin_set_mode(ROBOT_I2C_SDA_PIN, ROBOT_I2C_PIN_MODE);
    if (uapi_i2c_master_init(ROBOT_I2C_BUS, ROBOT_I2C_BAUDRATE, 0U) != ERRCODE_SUCC) {
        return false;
    }

    g_robot_i2c_initialized = true;
    return true;
}
