#ifndef ROBOT_TYPES_H
#define ROBOT_TYPES_H

#include <Arduino.h>
#include "config/robot_config.h"
#include "command_table.h"

static constexpr uint8_t MAX_CMD_NAME_LEN = 17;  // 16 chars + null terminator
static constexpr uint8_t MAX_CMD_PARAM_NUM = 6; // covers 6 cartesian pose values or 6 joint values

struct ParsedCommand {
    CommandId id;
    char cmd_name[MAX_CMD_NAME_LEN];
    float params[MAX_CMD_PARAM_NUM];
    uint8_t num_params;
};

struct RobotConfig {
    float gear_ratio = 0.0f;
    float motor_steps_per_joint_rev = 0.0f;

    float min_q[JOINT_NUM] = {};
    float max_q[JOINT_NUM] = {};
    float max_q_dot[JOINT_NUM] = {};
    float max_q_ddot[JOINT_NUM] = {};
};

struct RobotState {
    float q[JOINT_NUM] = {};
    float q_dot[JOINT_NUM] = {};

    bool max_limits[JOINT_NUM] = {};
    bool min_limits[JOINT_NUM] = {};
    bool is_moving = false;
    bool is_enabled = false;
/*
    float q_encoder_rad[JOINT_NUM] = {};
    bool q_encoder_valid[JOINT_NUM] = {};
    unsigned long q_encoder_age_ms[JOINT_NUM] = {};
*/    
    float update_frequency = 0.0f;
    uint32_t timestamp = 0u;
};

#endif // ROBOT_TYPES_H
