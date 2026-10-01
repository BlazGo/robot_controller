#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>
#include <AccelStepper.h>
#include "shared/motor_types.h"

static constexpr long STEPPER_STEPS_PER_REV = 200L;
static constexpr long MAX_DEFAULT_SPEED_STEPS_PER_S = 1000L;
static constexpr long MAX_DEFAULT_ACCELERATION_STEPS_PER_S2 = 4000L;

enum class ControlMode : uint8_t {
    POSITION,
    SPEED
};

class Motor {
public:
    Motor(uint8_t step_pin, uint8_t direction_pin,
          uint8_t microsteps, bool dir_inverted);

    void initialize();
    void update();

    // Reference reset, not a motion command. Accepted only while stopped.
    bool setPositionSteps(long position_steps);
    bool moveToSteps(long target_steps);
    bool setSpeedSteps(float speed_steps_s);
    void setMaxSpeedSteps(float max_speed_steps_s);
    void setAccelerationSteps(float acceleration_steps_s2);
    void stopPositionMove();

    long getStepsPerRev();
    long getPositionSteps();
    float getSpeedSteps();
    bool isMoving();
    ControlMode getControlMode();

private:
    bool selectControlMode(ControlMode control_mode);

    MotorConfig _config;
    AccelStepper _stepper;
    ControlMode _control_mode;
};

#endif // MOTOR_H
