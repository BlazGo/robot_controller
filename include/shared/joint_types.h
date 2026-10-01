#ifndef JOINT_TYPES_H
#define JOINT_TYPES_H

#include <Arduino.h>

struct JointConfig {
    float gear_ratio = 0.0f;
    float motor_steps_per_joint_rev = 0.0f;
    float rads_per_step = 0.0f;
    float steps_per_rad = 0.0f;
    
    float min_angle_rad = -PI/4;
    float max_angle_rad = PI/4;
};

struct JointState {
    float currentPositionRad = 0.0f;
    float currentSpeedRad = 0.0f;
    float currentAccelerationRad = 0.0f;
    
    bool isMoving = false;
    bool min_limit = false;
    bool max_limit = false;
};

#endif // JOINT_TYPES_H
