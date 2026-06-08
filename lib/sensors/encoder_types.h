#ifndef ENCODER_TYPES_H
#define ENCODER_TYPES_H

#include <Arduino.h>
#include "config.h"

struct JointMeasurement{
    float angle_rad;
    bool valid;
};

struct EncoderFrame{
    JointMeasurement joints[JOINT_NUM];
    uint32_t frame_idx;
    uint32_t timestamp_us;
};

#endif
