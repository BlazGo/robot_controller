#ifndef ROBOT_STATE_SHARED_H
#define ROBOT_STATE_SHARED_H

#include <Arduino.h>
#include "config.h"
#include "robot_types.h"

typedef struct {
    float q_rad[JOINT_NUM];
    float q_target_rad[JOINT_NUM];
    float q_dot_rad[JOINT_NUM];
    float q_dot_target_rad[JOINT_NUM];
    
    float x_mm[3];
    float x_rad[3];
    float x_target_mm[3];
    float x_target_rad[3];

    bool moving;
    robot_error_state_t robot_error_state;
    robot_motion_control_paradigm_t robot_motion_mode;  // 0-cart, 1-joint, ...

    uint32_t timestamp_us;
    uint32_t update_counter;
}RobotSharedState;

extern RobotSharedState g_robot_shared_state;

#endif
