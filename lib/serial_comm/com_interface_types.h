#ifndef COM_INTERFACE_TYPES_H
#define COM_INTERFACE_TYPES_H

#define MAX_MSG_REC_LEN 128
#define MAX_MSG_SEND_LEN 128
#define MAX_PARAMS 6

typedef enum{
    CMD_JOINT_MOVE,
    CMD_CART_MOVE,
    CMD_ENABLE,
    CMD_DISABLE,
    CMD_SET_MAX_JOINT_SPEED,
    CMD_SET_MAX_JOINT_ACCELERATION,
    CMD_SET_SPEED_CONTROL_PARADIGM,
    CMD_SET_POSITION_CONTROL_PARADIGM,
    CMD_SET_CART_CONTROL_MODE,
    CMD_SET_JOINT_CONTROL_MODE,
    CMD_EMERGENCY_STOP
}cmd_type_t;

struct CommandInfo {
    const char* name;
    cmd_type_t type;
    uint8_t param_count;
};

// map the strings to commands and num of parameters
const CommandInfo command_table[] = {
    {"CMD_JOINT_MOVE", CMD_JOINT_MOVE, 6},
    {"CMD_CART_MOVE", CMD_CART_MOVE, 6},
    {"CMD_ENABLE", CMD_ENABLE, 0},
    {"CMD_DISABLE", CMD_DISABLE, 0},
    {"CMD_SET_MAX_JOINT_SPEED", CMD_SET_MAX_JOINT_SPEED, 6},
    {"CMD_SET_MAX_JOINT_ACCELERATION", CMD_SET_MAX_JOINT_ACCELERATION, 6},
    {"CMD_SET_SPEED_CONTROL_PARADIGM", CMD_SET_SPEED_CONTROL_PARADIGM, 0},
    {"CMD_SET_POSITION_CONTROL_PARADIGM", CMD_SET_POSITION_CONTROL_PARADIGM, 0},
    {"CMD_SET_CART_CONTROL_MODE", CMD_SET_CART_CONTROL_MODE, 0},
    {"CMD_SET_JOINT_CONTROL_MODE", CMD_SET_JOINT_CONTROL_MODE, 0},
    {"CMD_EMERGENCY_STOP", CMD_EMERGENCY_STOP, 0},
};

// count the size of the command table -> num of commands
const uint8_t NUM_COMMANDS = sizeof(command_table)/sizeof(CommandInfo);

typedef struct{
    cmd_type_t type;
    float params[MAX_PARAMS];
    uint8_t param_count;
} cmd_robot_t;

#endif
