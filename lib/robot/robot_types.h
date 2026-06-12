#ifndef ROBOT_TYPES_H
#define ROBOT_TYPES_H

#include "utils.h"
#include "config.h"
#include "joint_types.h"

enum RobotExecState {
    ROBOT_IDLE = 0,
    ROBOT_EXECUTING,
    ROBOT_DONE,
    ROBOT_ERROR,
    ROBOT_STOPPING,
    ROBOT_STOPPED
};

enum class robot_error_state_t{
  NO_ERROR,
  LIMIT_HIT_MIN,
  LIMIT_HIT_MAX,
  GENERAL_ERROR
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

  Matrix4x4 T_EE;       // cartesian EE pose as matrix
  
  bool motors_enabled;  // whether or not motors are enabled
  RobotExecState exec_state; 
  bool stop_requested;
  bool command_completed;

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
