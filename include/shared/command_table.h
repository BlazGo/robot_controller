#ifndef COMMAND_TABLE_H
#define COMMAND_TABLE_H

#include <Arduino.h>

enum CommandId{
    CMD_UNKNOWN = -1,
    CMD_PING,
    CMD_MOVE_JOINT,
    CMD_MOVE_JOINT_VEL,
    CMD_MOVE_CART,
    CMD_MOVE_CART_LIN,
    CMD_START_HOMING,
    CMD_SET_CALIB_FROM_ENC,
    CMD_GET_ENC_ANGLE,
    CMD_GET_MAX_JOINT_VEL,
    CMD_GET_MAX_JOINT_ACCEL,
    CMD_GET_MAX_JOINT_ANGLE,
    CMD_GET_MIN_JOINT_ANGLE,
    NUM_COMMANDS
};

struct CommandDefinition{
    const char* cmd_name;
    CommandId cmd_id;
    uint8_t cmd_num_params;
};

extern const CommandDefinition COMMAND_TABLE[NUM_COMMANDS];

#endif // COMMAND_TABLE_H
