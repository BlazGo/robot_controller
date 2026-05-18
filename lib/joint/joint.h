#ifndef JOINT_H
#define JOINT_H

#include "motor.h"

#define ACCEL_SAFETY_MARGIN 0.8f

enum joint_motion_control_paradigm_t {
  SPEED_CONTROL,
  POSITION_CONTROL
};

struct JointConfig {
  // Usually static and does not change during runtime
  float min_position;
  float max_position;

  float max_speed;
  float max_acceleration;

  float gear_ratio;
  long steps_per_joint_rev;
  float position_tolerance;
};

struct JointState {
  // Usually dynamic and does change during runtime
  float angle_rad;
  float angle_vel_rad_s;
  float angle_acc_rad_s2;

  float target_angle_rad;
  float target_angle_vel_rad_s;

  bool moving;
  joint_motion_control_paradigm_t joint_motion_control_paradigm;
  bool at_min_lim;
  bool at_max_lim;
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
    bool setMotionControlParadigm(joint_motion_control_paradigm_t mode);
  
    JointState getState(void);
    JointConfig getJointConfig(void);

  private:
    JointConfig _jointConfig;
    JointState _jointState;

    Motor _motor;

    float stepsToAngleRad(long steps);
    long angleRadToSteps(float angle_rad);
    
    long  angleVelRadToStepsPerSec(float angle_rad_s) const;
    float stepsPerSecToAngleVelRad(float steps_per_sec) const;
    
    float angleAccelRadToStepsPerSec2(float angle_rad_s2) const;

    float clampAngleRad(float angle_rad);

    void checkLimits(void);
};

#endif