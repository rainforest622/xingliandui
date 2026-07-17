#include "robot_protocol.h"

#include <stddef.h>

uint8_t robot_checksum(const uint8_t *data, uint32_t length)
{
    uint8_t value = 0U;
    uint32_t index;
    for (index = 0U; index < length; ++index) {
        value ^= data[index];
    }
    return value;
}

robot_status_t robot_command_decode(
    const uint8_t frame[ROBOT_COMMAND_SIZE],
    robot_command_t *command)
{
    if ((frame == NULL) || (command == NULL)) {
        return ROBOT_STATUS_INVALID_COMMAND;
    }
    if (frame[0] != ROBOT_COMMAND_HEADER) {
        return ROBOT_STATUS_INVALID_COMMAND;
    }
    if (robot_checksum(frame, ROBOT_COMMAND_SIZE - 1U) != frame[5]) {
        return ROBOT_STATUS_CHECKSUM_ERROR;
    }
    if ((frame[1] < ROBOT_CMD_FORWARD) || (frame[1] > ROBOT_CMD_STOP) ||
        (frame[2] > 100U) || (frame[3] > 100U)) {
        return ROBOT_STATUS_INVALID_COMMAND;
    }

    command->command = (robot_command_code_t)frame[1];
    command->speed_left = frame[2];
    command->speed_right = frame[3];
    command->sequence = frame[4];
    return ROBOT_STATUS_OK;
}

void robot_ack_encode(
    uint8_t sequence,
    robot_status_t status,
    uint8_t ack[ROBOT_ACK_SIZE])
{
    ack[0] = ROBOT_ACK_HEADER;
    ack[1] = sequence;
    ack[2] = (uint8_t)status;
    ack[3] = robot_checksum(ack, ROBOT_ACK_SIZE - 1U);
}
