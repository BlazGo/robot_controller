#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>
#include <AccelStepper.h>
#include "motor_types.h"

static constexpr long STEPPER_STEPS_PER_REV = 200L;
static constexpr long MAX_SPEED_STEPS_PER_S = 4000L;
static constexpr long MAX_ACCELERATION_STEPS_PER_S2 = 8000L;
static constexpr float kSpeedEpsilon = 0.0025f;


class Motor {
    public:
    Motor(uint8_t stepPin, uint8_t directionPin, uint8_t microsteps, bool dir_inverted);

    void initialize(void);
    void update(void);
    void stop(void);
    void emergencyStop(void);
    bool isMoving(void) const;

    void setCurrentPositionSteps(long currentPosition);
    void setTargetSpeedSteps(float targetSpeed);
    void setMaxSpeedSteps(float maxSpeed);
    void setMaxAccelerationSteps(float maxAcceleration);

    long getCurrentPositionSteps(void) const;
    float getCurrentSpeedSteps(void) const;
    float getMaxSpeedSteps(void) const;
    float getMaxAccelerationSteps(void) const;
    long getStepsPerRevolution(void) const;

private:
    uint32_t _lastUpdateUs;

    MotorConfig _config;
    MotorState _state;

    AccelStepper _stepper;
};

#endif