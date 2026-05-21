#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>
#include <AccelStepper.h>

static constexpr long STEPPER_STEPS_PER_REV = 200L;
static constexpr long MAX_SPEED_STEPS_PER_S = 100L;
static constexpr long MAX_ACCELERATION_STEPS_PER_S2 = 500L;
static constexpr float kSpeedEpsilon = 0.01f;

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

class Motor {
public:
    Motor(uint8_t stepPin, uint8_t directionPin, uint8_t microsteps);

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