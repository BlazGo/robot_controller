#ifndef MOTOR_TYPES_H
#define MOTOR_TYPES_H

#include <Arduino.h>

struct MotorConfig {
    uint8_t stepPin = 0U;
    uint8_t directionPin = 0U;
    uint8_t microsteps = 1U;
    bool dirInverted = false; // 0 for CW 1 for CCW
    long stepsPerRevolution = 0L;
    
    float maxSpeedSteps = 0.0f;
};

#endif
