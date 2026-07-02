#ifndef ROBOT_MVP_PROTOCOL_H
#define ROBOT_MVP_PROTOCOL_H

#include <stdbool.h>
#include <stdint.h>

#define ROBOT_COMMAND_HEADER 0xAAU
#define ROBOT_ACK_HEADER 0x55U
#define ROBOT_COMMAND_SIZE 6U
#define ROBOT_ACK_SIZE 4U

typedef enum {
    ROBOT_CMD_FORWARD = 1,
    ROBOT_CMD_BACKWARD = 2,
    ROBOT_CMD_LEFT = 3,
    ROBOT_CMD_RIGHT = 4,
    ROBOT_CMD_STOP = 5
} robot_command_code_t;

typedef enum {
    ROBOT_STATUS_OK = 0,
    ROBOT_STATUS_OBSTACLE_STOP = 1,
    ROBOT_STATUS_LOW_BATTERY = 2,
    ROBOT_STATUS_CHECKSUM_ERROR = 3,
    ROBOT_STATUS_INVALID_COMMAND = 4,
    ROBOT_STATUS_MOTOR_ERROR = 5
} robot_status_t;

typedef struct {
    robot_command_code_t command;
    uint8_t speed_left;
    uint8_t speed_right;
    uint8_t sequence;
} robot_command_t;

uint8_t robot_checksum(const uint8_t *data, uint32_t length);
robot_status_t robot_command_decode(
    const uint8_t frame[ROBOT_COMMAND_SIZE],
    robot_command_t *command
);
void robot_ack_encode(
    uint8_t sequence,
    robot_status_t status,
    uint8_t ack[ROBOT_ACK_SIZE]
);

#endif
