#include <assert.h>
#include <stdint.h>
#include <stdio.h>

#include "../../ws63_liteos/app/robot_app.h"
#include "../../ws63_liteos/protocol/robot_protocol.h"
#include "../../ws63_liteos/sensor/environment_sensor.h"

static int8_t applied_left;
static int8_t applied_right;

static void fake_motor_apply(int8_t left, int8_t right)
{
    applied_left = left;
    applied_right = right;
}

static bool fake_environment_read(robot_environment_t *output)
{
    static const uint8_t frame[ROBOT_AHT20_FRAME_SIZE] = {
        0x18U, 0x94U, 0xFDU, 0xF6U, 0x06U, 0x25U
    };
    return robot_aht20_decode_measurement(frame, output);
}

static void make_command(
    uint8_t command,
    uint8_t speed_left,
    uint8_t speed_right,
    uint8_t sequence,
    uint8_t output[ROBOT_COMMAND_SIZE])
{
    output[0] = ROBOT_COMMAND_HEADER;
    output[1] = command;
    output[2] = speed_left;
    output[3] = speed_right;
    output[4] = sequence;
    output[5] = robot_checksum(output, ROBOT_COMMAND_SIZE - 1U);
}

int main(void)
{
    robot_app_t app;
    uint8_t command[ROBOT_COMMAND_SIZE];
    uint8_t ack[ROBOT_ACK_SIZE];
    uint8_t state_packet[ROBOT_STATE_SIZE];
    robot_state_t state;

    robot_app_init(&app, fake_motor_apply, 0U);
    assert((applied_left == 0) && (applied_right == 0));
    robot_app_set_environment_reader(&app, fake_environment_read);
    assert(robot_app_read_environment(&app));
    assert(robot_app_environment(&app)->temperature_deci_c == 253);
    assert(robot_app_environment(&app)->humidity_deci_percent == 582U);

    make_command(ROBOT_CMD_FORWARD, 40U, 50U, 7U, command);
    robot_app_process_command(&app, command, false, 10U, ack);
    assert((applied_left == 40) && (applied_right == 50));
    assert((ack[0] == ROBOT_ACK_HEADER) && (ack[1] == 7U));
    assert(ack[2] == ROBOT_STATUS_OK);
    assert(robot_app_build_state(&app, 87U, 1234U, state_packet) ==
           ROBOT_STATUS_OK);
    assert(robot_state_decode(state_packet, &state) == ROBOT_STATUS_OK);
    assert(state.sequence == 0U);
    assert(state.battery == 87U);
    assert(state.humidity == 58U);
    assert(state.distance_mm == 1234U);
    assert(state.motor_state == ROBOT_MOTOR_STATE_FORWARD);

    assert(!robot_app_tick(&app, 509U));
    assert(robot_app_tick(&app, 510U));
    assert((applied_left == 0) && (applied_right == 0));

    make_command(ROBOT_CMD_FORWARD, 30U, 30U, 8U, command);
    robot_app_process_command(&app, command, true, 600U, ack);
    assert(ack[2] == ROBOT_STATUS_OBSTACLE_STOP);
    assert((applied_left == 0) && (applied_right == 0));

    command[5] ^= 1U;
    robot_app_process_command(&app, command, false, 700U, ack);
    assert(ack[2] == ROBOT_STATUS_CHECKSUM_ERROR);

    puts("C robot core tests: OK");
    return 0;
}
