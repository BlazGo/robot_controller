#include "motor.h"
#include "utils.h"


Motor::Motor(uint8_t stepPin, uint8_t directionPin, uint8_t microsteps, bool dir_inverted)
:   _lastUpdateUs(0U),
    _config{},
    _state{},
    _stepper(AccelStepper::DRIVER, stepPin, directionPin)
{
    _config.stepPin = stepPin;
    _config.directionPin = directionPin;
    _config.dir_inverted = dir_inverted;
    _config.microsteps = (microsteps < 1U) ? 1U : microsteps;
}

void Motor::initialize() {
    setCurrentPositionSteps(0);
    _state.currentSpeedSteps = _stepper.speed();
    _state.targetSpeedSteps = _state.currentSpeedSteps;
    _state.isMoving = _stepper.isRunning();

    _config.stepsPerRevolution = STEPPER_STEPS_PER_REV * static_cast<long>(_config.microsteps);

    if (_config.dir_inverted = false) {
        _stepper.setPinsInverted(false); 
    }
    else {
        _stepper.setPinsInverted(true); 
    }

    setMaxSpeedSteps(MAX_SPEED_STEPS_PER_S);
    setMaxAccelerationSteps(MAX_ACCELERATION_STEPS_PER_S2);

    _lastUpdateUs = micros();
}

void Motor::update() {
    const uint32_t nowUs = micros();
    const uint32_t deltaTimeUs = nowUs - _lastUpdateUs;
    _lastUpdateUs = nowUs;

    float deltaTimeSeconds = static_cast<float>(deltaTimeUs) * 1.0e-6f;
    deltaTimeSeconds = clampAbsFloat(deltaTimeSeconds, 0.001f);

    float maxSpeedDelta = _config.maxAccelerationSteps * deltaTimeSeconds;
    _state.currentSpeedSteps = moveTowards(
        _state.currentSpeedSteps,
        _state.targetSpeedSteps,
        maxSpeedDelta
    );

    _stepper.setSpeed(_state.currentSpeedSteps);
    _stepper.runSpeed();

    _state.currentPositionSteps = _stepper.currentPosition();
    _state.isMoving = (fabsf(_state.currentSpeedSteps) > kSpeedEpsilon);
}

bool Motor::isMoving(void) const {
    return _state.isMoving;
}

void Motor::setCurrentPositionSteps(long currentPosition) {
    _stepper.setCurrentPosition(currentPosition);
    _state.currentPositionSteps = currentPosition;
}

void Motor::setTargetSpeedSteps(float targetSpeed) {
    _state.targetSpeedSteps = clampAbsFloat(targetSpeed, _config.maxSpeedSteps);
}

void Motor::setMaxSpeedSteps(float maxSpeed) {
    if (maxSpeed < 0.0f) {
        maxSpeed = -maxSpeed;
    }

    _stepper.setMaxSpeed(maxSpeed);
    _config.maxSpeedSteps = maxSpeed;
}

void Motor::setMaxAccelerationSteps(float maxAcceleration) {
    if (maxAcceleration < 0.0f) {
        maxAcceleration = -maxAcceleration;
    }

    _config.maxAccelerationSteps = maxAcceleration;
}

long Motor::getCurrentPositionSteps(void) const {
    return _state.currentPositionSteps;
}

float Motor::getCurrentSpeedSteps(void) const {
    return _state.currentSpeedSteps;
}

float Motor::getMaxSpeedSteps(void) const {
    return _config.maxSpeedSteps;
}

float Motor::getMaxAccelerationSteps(void) const {
    return _config.maxAccelerationSteps;
}

void Motor::stop(void) {
    setTargetSpeedSteps(0);
}

void Motor::emergencyStop(void) {
    _state.targetSpeedSteps = 0.0f;
    _state.currentSpeedSteps = 0.0f;
    _stepper.setSpeed(0.0f);
    _state.isMoving = false;
}

long Motor::getStepsPerRevolution(void) const {
    return _config.stepsPerRevolution;
}