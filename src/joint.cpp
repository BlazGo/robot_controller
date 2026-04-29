#include "joint.h"
#include "utils.h"
#include <cmath>

Joint::Joint(uint8_t step_pin, uint8_t dir_pin, uint8_t microsteps, float gear_ratio, float min_angle_rad, float max_angle_rad)
  : _min_angle_rad(min_angle_rad),
    _max_angle_rad(max_angle_rad),
    _gear_ratio(gear_ratio),
    _motor(step_pin, dir_pin, microsteps) {
}

// ─────────────────────────────────────────────────────────────
// Public functions
// ─────────────────────────────────────────────────────────────

void Joint::init() {
  _motor.init();
  _steps_per_joint_rev = _gear_ratio * _motor.getStepsPerRev();

  _curr_angle = 0.0f;
  _curr_speed = 0.0f;
  
  _start_angle = 0.0f;
  _target_angle = 0.0f;
  _target_speed = 0.0f;
  _max_speed = 0.0f;
  _max_acceleration = 0.0f;
  _is_moving = false;

  _position_tolerance = 0.005f;

  setMaxSpeed(1.0f);
  setMaxAcceleration(2.5f);
}

void Joint::update(void) {
  // TODO: add angle checks for max joint limits
   // Refresh current state
  _curr_angle = stepsToAngleRad(_motor.getCurrPosition());
  _curr_speed = stepsPerSecToAngleVelRad(_motor.getCurrSpeed());

  if (_motion_control_paradigm == POSITION_CONTROL){
    float remaining = _target_angle - _curr_angle;
    float dir = (remaining >= 0) ? 1.0f : -1.0f;

    float abs_remaining = fabsf(remaining);

    // stopping distance from current velocity
    float d_stop = (_curr_speed * _curr_speed) / (2 * _max_acceleration);
    // acceleration zone threshold
    float d_acc = (_max_speed * _max_speed) / (2 * _max_acceleration);
    
    float cmd_speed = 0.0f;
    if (fabsf(_curr_angle - _target_angle) < _position_tolerance){
      cmd_speed = 0.0;
    }
    else{
      if (abs_remaining <= d_stop) {
          // must decelerate
          cmd_speed = sqrtf(2 * _max_acceleration * abs_remaining);
      }
      else if (abs_remaining < d_acc) {
          // still can accelerate toward max speed
          cmd_speed = _max_speed;
      }
      else {
          // cruise
          cmd_speed = _max_speed;
      }
    }
    cmd_speed = cmd_speed * dir;
    setTargetSpeed(cmd_speed);
  }
  else if (_motion_control_paradigm == SPEED_CONTROL){
    float d_stop = (_curr_speed * _curr_speed) / (2 * _max_acceleration);
    // stopping distance from current velocity
    clampAbsFloat(_curr_speed, _max_speed);
    // check if we can stop until reaching the limit
    if (((fabs(_curr_angle) + d_stop) < _min_angle_rad) || ((fabs(_curr_angle) + d_stop) > _max_angle_rad)){
      setTargetSpeed(0);
    }
  }
  //Serial.printf("d_acc: %.2f | d_left: %.2f | d_curr: %.2f | d_target: %.2f | v_cmd: %.2f\n", d_acc, remaining, _curr_angle, _target_angle, cmd_speed)

  _motor.update();
  _is_moving = _motor.isMoving();
}

bool Joint::isMoving(void){
  return _is_moving;
}

void Joint::moveToAngle(float angle){
  angle = clampAngleRad(angle);
  _target_angle = angle;
  _target_steps = angleRadToSteps(_target_angle);
  _start_angle = _curr_angle;
}

void Joint::setTargetSpeed(float angular_speed){
  _motor.setTargetSpeed(angleVelRadToStepsPerSec(angular_speed));
  _target_speed = angular_speed;
}

void Joint::setMaxSpeed(float max_angular_speed){
  // force it positive since it's a limit not a direction/quantity
  if (max_angular_speed < 0.0f){
    max_angular_speed = -max_angular_speed;
  }
  _motor.setMaxSpeed(angleVelRadToStepsPerSec(max_angular_speed));
  _max_speed = max_angular_speed;
}

void Joint::setMaxAcceleration(float max_angular_acceleration){
  // force it positive since it's a limit not a direction/quantity
  if (max_angular_acceleration < 0.0f){
    max_angular_acceleration = -max_angular_acceleration;
  }
  _motor.setMaxAcceleration(angleAccelRadToStepsPerSec2(max_angular_acceleration));
  _max_acceleration = max_angular_acceleration;
}

float Joint::getAngle(void){
  return _curr_angle;
}

float Joint::getTargetAngle(void){
  return _target_angle;
}

float Joint::getSpeed(void){
    return _curr_speed;
}

float Joint::getMaxSpeed(void){
  return _max_speed;
}

float Joint::getMaxAcceleration(void){
  return _max_acceleration;
}

void Joint::stop(void){
  _motor.stop();
  _target_speed = 0.0f;
}

bool Joint::setMotionControlParadigm(motion_control_paradigm_t mode){
  _motion_control_paradigm =  mode;
  return true;
}

// ─────────────────────────────────────────────────────────────
// Private functions
// ─────────────────────────────────────────────────────────────

float Joint::clampAngleRad(float angle_rad) {
  if (angle_rad < _min_angle_rad){
    angle_rad = _min_angle_rad;
  }
  if (angle_rad > _max_angle_rad){
    angle_rad = _max_angle_rad;
  }
  return angle_rad;
}

float Joint::stepsToAngleRad(long steps) {
  return (static_cast<float>(steps) / static_cast<float>(_steps_per_joint_rev)) * 2.0f * PI;
}

long Joint::angleRadToSteps(float angle_rad) {
  return std::lround((angle_rad / (2.0f * PI)) * static_cast<float>(_steps_per_joint_rev));
}

long Joint::angleVelRadToStepsPerSec(float angle_rad_s) const {
  return std::lround((angle_rad_s / (2.0f * PI)) * static_cast<float>(_steps_per_joint_rev));
}

float Joint::stepsPerSecToAngleVelRad(float steps_per_sec) const {
  return (steps_per_sec / static_cast<float>(_steps_per_joint_rev)) * 2.0f * PI;
}

float Joint::angleAccelRadToStepsPerSec2(float angle_rad_s2) const {
  return (angle_rad_s2 / (2.0f * PI)) * static_cast<float>(_steps_per_joint_rev);
}
