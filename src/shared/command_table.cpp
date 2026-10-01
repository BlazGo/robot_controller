#include "shared/command_table.h"

const CommandDefinition COMMAND_TABLE[NUM_COMMANDS] = {
    {"PING",                CMD_PING,                   0},
    {"MOVE_JOINT",          CMD_MOVE_JOINT,             6},
    {"MOVE_JOINT_VEL",      CMD_MOVE_JOINT_VEL,         6},
    {"MOVE_CART",           CMD_MOVE_CART,              6},
    {"MOVE_CART_LIN",       CMD_MOVE_CART_LIN,          6},
    {"START_HOMING",        CMD_START_HOMING,           0},
    {"SET_CALIB_FROM_ENC",  CMD_SET_CALIB_FROM_ENC,     0},
    {"GET_ENC_ANGLE",       CMD_GET_ENC_ANGLE,          0},
    {"GET_MAX_JOINT_VEL",   CMD_GET_MAX_JOINT_VEL,      0},
    {"GET_MAX_JOINT_ACCEL", CMD_GET_MAX_JOINT_ACCEL,    0},
    {"GET_MAX_JOINT_ANGLE", CMD_GET_MAX_JOINT_ANGLE,    0},
    {"GET_MIN_JOINT_ANGLE", CMD_GET_MIN_JOINT_ANGLE,    0}
};
