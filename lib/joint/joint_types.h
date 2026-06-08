#ifndef JOINT_TYPES_H
#define JOINT_TYPES_H


enum joint_motion_control_paradigm_t {
  JOINT_SPEED_CONTROL,
  JOINT_POSITION_CONTROL
};

struct JointConfig {
  // Usually static and does not change during runtime
  float min_angle_rad;
  float max_angle_rad;

  float max_angle_vel_rad_s;
  float max_angle_acc_rad_s2;

  float gear_ratio;
  long motor_steps_per_joint_rev;
  float angle_tolerance_rad;
  bool dir_inverted;
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

#endif // JOINT_TYPES_H
