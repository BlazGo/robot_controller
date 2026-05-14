#include "motor.h"
#include "utils.h"

#define SPEED_EPS 0.01f

Motor::Motor(uint8_t step_pin, uint8_t dir_pin, uint8_t microsteps)
  : _step_pin(step_pin),
    _dir_pin(dir_pin),
    _microsteps(microsteps),
    _stepper(AccelStepper::DRIVER, step_pin, dir_pin)
{
}

// ─────────────────────────────────────────────────────────────
// Public functions
// ─────────────────────────────────────────────────────────────

void Motor::init() {  
  _curr_position = _stepper.currentPosition();
  _curr_speed = _stepper.speed();
  _target_speed = 0.0f;
  _max_speed = 500;
  _max_acceleration = 100;
  
  _steps_per_rev = FULL_STEPS_PER_REV * static_cast<long>(_microsteps);

  setMaxSpeed(500);
  setMaxAcceleration(100);
  _last_update_us = micros();
  _is_moving = false;
}

void Motor::update() {
  // Update the time
  const uint32_t now_us = micros();
  // Get elapsed time from the last update
  const uint32_t dt_us = now_us - _last_update_us;
  // And save the new time as the old time
  _last_update_us = now_us;

  // convert to [s] from [ms] for proper calculations
  const float dt_s = static_cast<float>(dt_us) * 1.0e-6f;

  float max_delta = _max_acceleration * dt_s;
  _target_speed = clampAbsFloat(_target_speed, _max_speed);
  _curr_speed = moveTowards(_curr_speed, _target_speed, max_delta);

  _stepper.setSpeed(_curr_speed);
  _stepper.runSpeed();

  _curr_position = _stepper.currentPosition();

  _is_moving = (fabsf(_curr_speed) > SPEED_EPS);
}

bool Motor::isMoving(void) const {
  return _is_moving;
}

void Motor::setCurrPosition(long curr_position){
  _stepper.setCurrentPosition(curr_position);
  _curr_position = curr_position;
}

void Motor::setTargetSpeed(float target_speed){
  _target_speed = target_speed;
}

void Motor::setMaxSpeed(float max_speed){
  // force it positive since it's a limit not a direction/quantity
  if (max_speed < 0.0f){
    max_speed = -max_speed;
  }
  _stepper.setMaxSpeed(max_speed);
  _max_speed = max_speed;
}

void Motor::setMaxAcceleration(float max_acceleration){
  // force it positive
  if (max_acceleration < 0.0f){
    max_acceleration = -max_acceleration;
  }
  // Not used with only speed control we manually accelerate
  // _stepper.setAcceleration(max_acceleration);
  _max_acceleration = max_acceleration;
}

long Motor::getCurrPosition(void) const {
  return _curr_position;
}

float Motor::getCurrSpeed(void) const {
  return _curr_speed;
}

float Motor::getMaxSpeed(void) const {
  return _max_speed;
}

float Motor::getMaxAcceleration(void) const {
  return _max_acceleration;
}

void Motor::stop(void){
  setTargetSpeed(0);
}

void Motor::emergencyStop(void) {
  _target_speed = 0.0f;
  _curr_speed = 0.0f;
  _stepper.setSpeed(0.0f);
  _is_moving = false;
}

long Motor::getStepsPerRev(void) const {
  return _steps_per_rev;
}

// ─────────────────────────────────────────────────────────────
// Private functions
// ─────────────────────────────────────────────────────────────

void Motor::setCurrSpeed(float curr_speed){
  _stepper.setSpeed(curr_speed);
  _curr_speed = curr_speed;
}
