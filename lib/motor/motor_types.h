#ifndef MOTOR_TYPES_H
#define MOTOR_TYPES_H

struct MotorConfig {
    uint8_t stepPin = 0U;
    uint8_t directionPin = 0U;
    uint8_t microsteps = 1U;
    long stepsPerRevolution = 0L;
    float maxSpeedSteps = 0.0f;
    float maxAccelerationSteps = 0.0f;
};

struct MotorState {
    long currentPositionSteps = 0L;
    float currentSpeedSteps = 0.0f;
    float targetSpeedSteps = 0.0f;
    bool isMoving = false;
};

#endif
