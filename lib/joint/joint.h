#ifndef JOINT_H
#define JOINT_H

#include "motor.h"

#define ACCEL_SAFETY_MARGIN 0.8f

enum motion_control_paradigm_t {
  SPEED_CONTROL,
  POSITION_CONTROL
};

class Joint {
public:
  // Constructor angle limits are in rads
  Joint(uint8_t step_pin,
        uint8_t dir_pin,
        uint8_t microsteps,
        float gear_ratio,
        float min_angle_rad,
        float max_angle_rad);

  void init(void);
  void update(void);
  bool isMoving(void);
  void stop(void);

  void moveToAngle(float angle);

  void setTargetSpeed(float angular_speed);
  void setMaxSpeed(float max_angular_speed);
  void setMaxAcceleration(float max_angular_acceleration);
  bool setMotionControlParadigm(motion_control_paradigm_t mode);
 
  float getAngle(void);
  float getTargetAngle(void);
  float getSpeed(void);
  float getMaxSpeed(void);
  float getMaxAcceleration(void);

private:
  float _min_angle_rad;
  float _max_angle_rad;
  float _gear_ratio;

  motion_control_paradigm_t _motion_control_paradigm;
  long _steps_per_joint_rev;
  bool _is_moving;
  float _position_tolerance;

  float _start_angle;
  float _target_angle;
  long _target_steps;
  float _target_speed;
  float _cmd_speed;

  float _curr_angle;
  float _curr_speed;
  float _max_acceleration;
  float _max_speed;

  Motor _motor;

  float stepsToAngleRad(long steps);
  long angleRadToSteps(float angle_rad);
  
  long  angleVelRadToStepsPerSec(float angle_rad_s) const;
  float stepsPerSecToAngleVelRad(float steps_per_sec) const;
  
  float angleAccelRadToStepsPerSec2(float angle_rad_s2) const;

  float clampAngleRad(float angle_rad);
};

#endif