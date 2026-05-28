#ifndef ROBOT_COMMANDS_H
#define ROBOT_COMMANDS_H

#include "config.h"
#include "joint_types.h"

enum class RobotCommandType{
  JOINT_MOVE,
  CART_MOVE,
  SET_MAX_JOINT_SPEEDS,
  SET_MAX_JOINT_ACCELERATIONS,
  CONTROL_PARADIGM_CHANGE,
};

struct RobotCommand{
  RobotCommandType type;
  // field for enumeration variables
  int id_int;
  // field for joint variables
  float q[JOINT_NUM];
  // field for cart variables
  float x[6];
};

#endif // ROBOT_COMMANDS_H
