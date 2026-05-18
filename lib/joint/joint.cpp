#include "joint.h"
#include "utils.h"
#include <cmath>

Joint::Joint(uint8_t step_pin, uint8_t dir_pin, uint8_t microsteps, float gear_ratio, float min_angle_rad, float max_angle_rad)
  :   _motor(step_pin, dir_pin, microsteps)
  {
      _jointConfig.gear_ratio = gear_ratio;
      _jointConfig.min_position = min_angle_rad;
      _jointConfig.max_position = max_angle_rad;
      _jointConfig.max_speed = 0.0f;
      _jointConfig.max_acceleration = 0.0f;

      _jointState.angle_rad = 0.0f;
      _jointState.angle_vel_rad_s = 0.0f;
      _jointState.angle_acc_rad_s2 = 0.0f;
      _jointState.target_angle_rad = 0.0f;
      _jointState.target_angle_vel_rad_s = 0.0f;
      _jointState.moving = false;
      _jointState.at_min_lim = false;
      _jointState.at_max_lim = false;
    }

// ─────────────────────────────────────────────────────────────
// Public functions
// ─────────────────────────────────────────────────────────────

void Joint::init() {
  _motor.init();
  _jointConfig.steps_per_joint_rev = _jointConfig.gear_ratio * _motor.getStepsPerRev();
  _jointState.joint_motion_control_paradigm = SPEED_CONTROL;
  _jointConfig.position_tolerance = 0.005f;
}

void Joint::update(void) {
  // Refresh current state

  Joint::updateState();
  Joint::checkLimits();

  float safe_vel = Joint::safeVelLimitsBased();
  float commanded_vel = safe_vel; // Later to be replaced by trapezoidal generator or similar (if even)
  
  _motor.setTargetSpeed(angleVelRadToStepsPerSec(commanded_vel));
  _motor.update();

  Joint::updateState();
}

void Joint::updateState(void){
  _jointState.angle_rad = stepsToAngleRad(_motor.getCurrPosition());
  _jointState.angle_vel_rad_s = stepsPerSecToAngleVelRad(_motor.getCurrSpeed());
  _jointState.moving = _motor.isMoving();
}

void Joint::checkLimits(void){
  _jointState.at_min_lim = (_jointState.angle_rad <= _jointConfig.min_position);
  _jointState.at_max_lim = (_jointState.angle_rad >= _jointConfig.max_position);
}

float Joint::safeVelLimitsBased(void){
  float safeVel = _jointState.target_angle_vel_rad_s;

  if (_jointState.at_min_lim && safeVel < 0.0f) {
    safeVel = 0.0f;
  }

  if (_jointState.at_max_lim && safeVel > 0.0f) {
    safeVel = 0.0f;
  } 
  return safeVel;
}

bool Joint::isMoving(void){
  return _jointState.moving;
}

void Joint::moveToAngle(float angle){
  angle = clampAngleRad(angle);
  _jointState.target_angle_rad = angle;
}

void Joint::setTargetSpeed(float angular_speed){
  angular_speed = clampAbsFloat(angular_speed, _jointConfig.max_speed);
  
  _motor.setTargetSpeed(angleVelRadToStepsPerSec(angular_speed));
  _jointState.target_angle_vel_rad_s = angular_speed;
}

void Joint::setMaxSpeed(float max_angular_speed){
  // force it positive since it's a limit not a direction/quantity
  if (max_angular_speed < 0.0f){
    max_angular_speed = -max_angular_speed;
  }
  _motor.setMaxSpeed(angleVelRadToStepsPerSec(max_angular_speed));
  _jointConfig.max_speed = max_angular_speed;
}

void Joint::setMaxAcceleration(float max_angular_acceleration){
  // force it positive since it's a limit not a direction/quantity
  if (max_angular_acceleration < 0.0f){
    max_angular_acceleration = -max_angular_acceleration;
  }
  _motor.setMaxAcceleration(angleAccelRadToStepsPerSec2(max_angular_acceleration));
  _jointConfig.max_acceleration = max_angular_acceleration;
}

JointState Joint::getState(void){
  return _jointState;
}

JointConfig Joint::getJointConfig(void){
  return _jointConfig;
}

void Joint::stop(void){
  _motor.stop();
  _jointState.target_angle_vel_rad_s = 0.0f;
}

bool Joint::setMotionControlParadigm(joint_motion_control_paradigm_t mode){
  _jointState.joint_motion_control_paradigm =  mode;
  return true;
}

// ─────────────────────────────────────────────────────────────
// Private functions
// ─────────────────────────────────────────────────────────────

float Joint::clampAngleRad(float angle_rad) {
  if (angle_rad < _jointConfig.min_position){
    angle_rad = _jointConfig.min_position;
  }
  if (angle_rad > _jointConfig.max_position){
    angle_rad = _jointConfig.max_position;
  }
  return angle_rad;
}

float Joint::stepsToAngleRad(long steps) {
  return (static_cast<float>(steps) / static_cast<float>(_jointConfig.steps_per_joint_rev)) * 2.0f * PI;
}

long Joint::angleRadToSteps(float angle_rad) {
  return std::lround((angle_rad / (2.0f * PI)) * static_cast<float>(_jointConfig.steps_per_joint_rev));
}

long Joint::angleVelRadToStepsPerSec(float angle_rad_s) const {
  return std::lround((angle_rad_s / (2.0f * PI)) * static_cast<float>(_jointConfig.steps_per_joint_rev));
}

float Joint::stepsPerSecToAngleVelRad(float steps_per_sec) const {
  return (steps_per_sec / static_cast<float>(_jointConfig.steps_per_joint_rev)) * 2.0f * PI;
}

float Joint::angleAccelRadToStepsPerSec2(float angle_rad_s2) const {
  return (angle_rad_s2 / (2.0f * PI)) * static_cast<float>(_jointConfig.steps_per_joint_rev);
}


