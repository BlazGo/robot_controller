#include "joint.h"
#include "utils.h"

Joint::Joint(uint8_t step_pin, uint8_t dir_pin, uint8_t microsteps, float gear_ratio, float min_angle_rad, float max_angle_rad, bool dir_inverted)
  :   _motor(step_pin, dir_pin, microsteps, dir_inverted)
  {
      _jointConfig.gear_ratio = gear_ratio;
      _jointConfig.min_angle_rad = min_angle_rad;
      _jointConfig.max_angle_rad = max_angle_rad;
      _jointConfig.max_angle_vel_rad_s = 0.0f;
      _jointConfig.max_angle_acc_rad_s2 = 0.0f;
      _jointConfig.dir_inverted = dir_inverted;

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
  _motor.initialize();
  _jointConfig.motor_steps_per_joint_rev  = _jointConfig.gear_ratio * _motor.getStepsPerRevolution();
  _jointState.joint_motion_control_paradigm = JOINT_SPEED_CONTROL;
  _jointConfig.angle_tolerance_rad = 0.005f;
}

void Joint::update(void) {
  Joint::updateState();   // Refresh current state
  Joint::checkLimits();   // Only checks the min and max limits and sets flags

  float commanded_angle_vel_rad_s = 0.0f; 
  
  switch (_jointState.joint_motion_control_paradigm)
  {
  case JOINT_SPEED_CONTROL:
    commanded_angle_vel_rad_s = computeSpeedControlVelRadS();
    break;

  case JOINT_POSITION_CONTROL:
    commanded_angle_vel_rad_s = computePositionControlVelRadS();
    break;
  
  default:
    commanded_angle_vel_rad_s = 0.0f;
    break;
  }

  // We preserve the target speed but we calculate safe speed from it
  commanded_angle_vel_rad_s = Joint::applyLimitSafetyToVelRadS(commanded_angle_vel_rad_s); // If we are outside of limits we set the speed to 0

   // Send the desired speed and update motor
  _motor.setTargetSpeedSteps(angleVelRadToStepsPerSec(commanded_angle_vel_rad_s));
  _motor.update();
}

float Joint::computeSpeedControlVelRadS(void){
  return clampAbsFloat(_jointState.target_angle_vel_rad_s, _jointConfig.max_angle_vel_rad_s);;
}

float Joint::computePositionControlVelRadS(void){
  float position_error_rad = _jointState.target_angle_rad - _jointState.angle_rad;

  if (fabsf(position_error_rad) <= _jointConfig.angle_tolerance_rad){
    return 0.0f;
  }

  constexpr float POSITION_Kp = 2.0f;
  float commanded_angle_vel_rad = POSITION_Kp * position_error_rad;

  commanded_angle_vel_rad = clampAbsFloat(commanded_angle_vel_rad, _jointConfig.max_angle_vel_rad_s);
  return commanded_angle_vel_rad;
}

void Joint::updateState(void){
  _jointState.angle_rad = stepsToAngleRad(_motor.getCurrentPositionSteps());
  _jointState.angle_vel_rad_s = stepsPerSecToAngleVelRad(_motor.getCurrentSpeedSteps());
  _jointState.moving = _motor.isMoving();
}

void Joint::checkLimits(void){
  _jointState.at_min_lim = (_jointState.angle_rad <= _jointConfig.min_angle_rad);
  _jointState.at_max_lim = (_jointState.angle_rad >= _jointConfig.max_angle_rad);
}

float Joint::applyLimitSafetyToVelRadS(float commanded_angle_vel_rad_s){
  if (_jointState.at_min_lim && commanded_angle_vel_rad_s < 0.0f) {
    return 0.0f;
  }

  if (_jointState.at_max_lim && commanded_angle_vel_rad_s > 0.0f) {
    return 0.0f;
  } 
  return commanded_angle_vel_rad_s;
}

bool Joint::isMoving(void){
  return _jointState.moving;
}

void Joint::moveToAngle(float angle){
  angle = clampAngleRad(angle);
  _jointState.target_angle_rad = angle;
  _jointState.joint_motion_control_paradigm = JOINT_POSITION_CONTROL;
}

void Joint::setCurrentAngle(float angle_rad){
  // Check if within limits
  float clamped_angle = clampAngleRad(angle_rad);
  // Set the stepper motors position
  _motor.setCurrentPositionSteps(angleRadToSteps(clamped_angle));
  // And update the joint state too
  _jointState.angle_rad = clamped_angle;
}

void Joint::setTargetSpeed(float angular_speed){
  angular_speed = clampAbsFloat(angular_speed, _jointConfig.max_angle_vel_rad_s);
  
  _jointState.target_angle_vel_rad_s = angular_speed;
  _jointState.joint_motion_control_paradigm = JOINT_SPEED_CONTROL;
}

void Joint::setMaxSpeed(float max_angular_speed){
  // force it positive since it's a limit not a direction/quantity
  if (max_angular_speed < 0.0f){
    max_angular_speed = -max_angular_speed;
  }
  _motor.setMaxSpeedSteps(angleVelRadToStepsPerSec(max_angular_speed));
  _jointConfig.max_angle_vel_rad_s = max_angular_speed;
}

void Joint::setMaxAcceleration(float max_angular_acceleration){
  // force it positive since it's a limit not a direction/quantity
  if (max_angular_acceleration < 0.0f){
    max_angular_acceleration = -max_angular_acceleration;
  }
  _motor.setMaxAccelerationSteps(angleAccelRadToStepsPerSec2(max_angular_acceleration));
  _jointConfig.max_angle_acc_rad_s2 = max_angular_acceleration;
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
  if (angle_rad < _jointConfig.min_angle_rad){
    angle_rad = _jointConfig.min_angle_rad;
  }
  if (angle_rad > _jointConfig.max_angle_rad){
    angle_rad = _jointConfig.max_angle_rad;
  }
  return angle_rad;
}

float Joint::stepsToAngleRad(long steps) {
  return (static_cast<float>(steps) / static_cast<float>(_jointConfig.motor_steps_per_joint_rev)) * 2.0f * PI;
}

long Joint::angleRadToSteps(float angle_rad) {
  return std::lround((angle_rad / (2.0f * PI)) * static_cast<float>(_jointConfig.motor_steps_per_joint_rev));
}

long Joint::angleVelRadToStepsPerSec(float angle_rad_s) const {
  return std::lround((angle_rad_s / (2.0f * PI)) * static_cast<float>(_jointConfig.motor_steps_per_joint_rev));
}

float Joint::stepsPerSecToAngleVelRad(float steps_per_sec) const {
  return (steps_per_sec / static_cast<float>(_jointConfig.motor_steps_per_joint_rev)) * 2.0f * PI;
}

float Joint::angleAccelRadToStepsPerSec2(float angle_rad_s2) const {
  return (angle_rad_s2 / (2.0f * PI)) * static_cast<float>(_jointConfig.motor_steps_per_joint_rev);
}
