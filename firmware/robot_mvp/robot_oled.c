#include "robot_oled.h"

#include <stdio.h>
#include <stdlib.h>
#include "robot_i2c.h"
#include "../helloworld_oled/ssd1306.h"
#include "../helloworld_oled/ssd1306_fonts.h"

static bool g_robot_oled_ready;

static char command_to_char(uint8_t command)
{
    switch (command) {
        case 1:
            return 'F';
        case 2:
            return 'B';
        case 3:
            return 'L';
        case 4:
            return 'R';
        case 5:
            return 'S';
        default:
            return '-';
    }
}

bool robot_oled_init(void)
{
    if (!robot_i2c_ensure_init()) {
        g_robot_oled_ready = false;
        return false;
    }

    ssd1306_Init();
    g_robot_oled_ready = true;
    robot_oled_render(false, false, 0U, 0U, 0U, false, 0, 0U);
    return true;
}

bool robot_oled_is_ready(void)
{
    return g_robot_oled_ready;
}

void robot_oled_render(
    bool motor_ready,
    bool moving,
    uint8_t last_command,
    uint8_t last_sequence,
    uint32_t uptime_ms,
    bool env_valid,
    int16_t temperature_deci_c,
    uint16_t humidity_deci_percent
)
{
    char line[24];
    int16_t temp_abs;

    if (!g_robot_oled_ready) {
        return;
    }

    ssd1306_Fill(Black);

    ssd1306_SetCursor(0, 0);
    ssd1306_DrawString("WS63 ROBOT MVP", Font_7x10, White);

    (void)snprintf(line, sizeof(line), "RDY:%u MOV:%u", motor_ready ? 1U : 0U, moving ? 1U : 0U);
    ssd1306_SetCursor(0, 15);
    ssd1306_DrawString(line, Font_7x10, White);

    (void)snprintf(line, sizeof(line), "CMD:%c SEQ:%03u", command_to_char(last_command), last_sequence);
    ssd1306_SetCursor(0, 30);
    ssd1306_DrawString(line, Font_7x10, White);

    if (env_valid) {
        temp_abs = (int16_t)abs((int)temperature_deci_c);
        (void)snprintf(
            line,
            sizeof(line),
            "T:%c%d.%d H:%u.%u",
            temperature_deci_c < 0 ? '-' : '+',
            temp_abs / 10,
            temp_abs % 10,
            humidity_deci_percent / 10U,
            humidity_deci_percent % 10U
        );
    } else {
        (void)snprintf(line, sizeof(line), "UP:%us ENV:--", (unsigned int)(uptime_ms / 1000U));
    }
    ssd1306_SetCursor(0, 45);
    ssd1306_DrawString(line, Font_7x10, White);

    ssd1306_UpdateScreen();
}
