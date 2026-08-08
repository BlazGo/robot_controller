#ifndef ROBOT_TYPES_H
#define ROBOT_TYPES_H

#include "utils.h"
#include "config.h"
#include "joint_types.h"
#include "robot_commands.h"

struct JointAngles {
    float values_rad[JOINT_NUM] = {};
};

enum RobotExecState {
    ROBOT_IDLE = 0,
    ROBOT_EXECUTING,
    ROBOT_DONE,
    ROBOT_ERROR,
    ROBOT_STOPPING,
    ROBOT_STOPPED
};

enum class HomingPhase : uint8_t {
  IDLE,
  SEEK_SWITCH_DIR_MIN,  // joint 0 only: drive toward MIN end switch
  BACKOFF_SWITCH_DIR_MIN,
  SEEK_SWITCH_DIR_MAX,  // joint 0 only: drive toward MAX end switch
  BACKOFF,            // joint 0 only: back off switch to repeatable zero
  READ_ENCODER,       // joints 1-5: instantaneous, no motion needed
  DONE,
  ERROR,
};

struct HomingStatus {
  bool active = false;
  uint8_t order_idx = 0;
  HomingPhase phase = HomingPhase::IDLE;
  float phase_start_q_rad = 0.0f;
  uint32_t phase_start_time_us = 0;
};

enum class robot_error_state_t{
  NO_ERROR,
  LIMIT_HIT_MIN,
  LIMIT_HIT_MAX,
  GENERAL_ERROR,
  NOT_HOMED,
};

enum class robot_motion_control_paradigm_t {
  ROBOT_CART_CONTROL,
  ROBOT_JOINT_CONTROL
};

struct RobotPlanner{
  float q_planned[JOINT_NUM];
};

struct RobotState{
  JointState joints[JOINT_NUM];   // array of individual joint states (maybe not needed?)

  float q[JOINT_NUM];       // joint angles - q
  float q_dot[JOINT_NUM];   // joint speeds - q_dot
  float q_ddot[JOINT_NUM];  // joint accelerations - q_double_dot

  float q_target[JOINT_NUM];       // joint target angles - q_target
  float q_dot_target[JOINT_NUM];   // joint target speeds - q_dot_target

  float x[6];           // cartesian EE pose
  float x_dot[6];       // cartesian EE speeds

  float x_target[6];    // cartesian target EE pose

  float q_encoders[JOINT_NUM];

  bool limits_min[JOINT_NUM];
  bool limits_max[JOINT_NUM];

  Matrix4x4 T_EE;       // cartesian EE pose as matrix
  
  bool motors_enabled;  // whether or not motors are enabled
  RobotExecState exec_state; 
  bool stop_requested;
  bool command_completed;
  bool all_homed;
  
  bool command_active;
  RobotCommandType active_cmd_type;
  uint32_t active_cmd_id;

  robot_motion_control_paradigm_t robot_motion_control_paradigm;

  robot_error_state_t robot_error_state;
  uint32_t timestamp;
};

struct RobotConfig{
  float max_joint_speeds[JOINT_NUM];
  float max_joint_accelerations[JOINT_NUM];
};

#endif
