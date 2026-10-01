#include "motor.h"
#include <math.h>

Motor::Motor(uint8_t step_pin, uint8_t direction_pin,
             uint8_t microsteps, bool dir_inverted)
    : _config{},
      _stepper(AccelStepper::DRIVER, step_pin, direction_pin),
      _control_mode(ControlMode::SPEED)
{
    _config.stepPin = step_pin;
    _config.directionPin = direction_pin;
    _config.microsteps = (microsteps < 1U) ? 1U : microsteps;
    _config.dirInverted = dir_inverted;
    _config.stepsPerRevolution =
        STEPPER_STEPS_PER_REV * static_cast<long>(_config.microsteps);
    _config.maxSpeedSteps = MAX_DEFAULT_SPEED_STEPS_PER_S;
}

void Motor::initialize()
{
    _control_mode = ControlMode::SPEED;
    _stepper.setPinsInverted(_config.dirInverted, false, false);
    _stepper.setCurrentPosition(0);
    setMaxSpeedSteps(MAX_DEFAULT_SPEED_STEPS_PER_S);
    setAccelerationSteps(MAX_DEFAULT_ACCELERATION_STEPS_PER_S2);
    _stepper.setSpeed(0.0f);
}

void Motor::update()
{
    if (_control_mode == ControlMode::POSITION) {
        _stepper.run();
    } else {
        _stepper.runSpeed();
    }
}

bool Motor::selectControlMode(ControlMode control_mode)
{
    if (_control_mode == control_mode) {
        return true;
    }
    if (isMoving()) {
        return false;
    }

    // Clear the old target and AccelStepper's profile state while stationary.
    _stepper.setCurrentPosition(_stepper.currentPosition());
    _control_mode = control_mode;
    return true;
}

bool Motor::setPositionSteps(long position_steps)
{
    if (isMoving()) {
        return false;
    }
    _stepper.setCurrentPosition(position_steps);
    return true;
}

bool Motor::moveToSteps(long target_steps)
{
    if (!selectControlMode(ControlMode::POSITION)) {
        return false;
    }
    _stepper.moveTo(target_steps);
    return true;
}

bool Motor::setSpeedSteps(float speed_steps_s)
{
    if (!isfinite(speed_steps_s) ||
        !selectControlMode(ControlMode::SPEED)) {
        return false;
    }
    _stepper.setSpeed(speed_steps_s);
    return true;
}

void Motor::setMaxSpeedSteps(float max_speed_steps_s)
{
    if (isfinite(max_speed_steps_s) && max_speed_steps_s > 0.0f) {
        _stepper.setMaxSpeed(max_speed_steps_s);
        _config.maxSpeedSteps = max_speed_steps_s;
    }
}

void Motor::setAccelerationSteps(float acceleration_steps_s2)
{
    if (isfinite(acceleration_steps_s2) && acceleration_steps_s2 > 0.0f) {
        _stepper.setAcceleration(acceleration_steps_s2);
    }
}

void Motor::stopPositionMove()
{
    if (_control_mode == ControlMode::POSITION && isMoving()) {
        _stepper.stop();
    }
}

long Motor::getStepsPerRev()
{
    return _config.stepsPerRevolution;
}

long Motor::getPositionSteps()
{
    return _stepper.currentPosition();
}

float Motor::getSpeedSteps()
{
    return _stepper.speed();
}

bool Motor::isMoving()
{
    if (_control_mode == ControlMode::POSITION) {
        return _stepper.isRunning();
    }
    return _stepper.speed() != 0.0f;
}

ControlMode Motor::getControlMode()
{
    return _control_mode;
}
