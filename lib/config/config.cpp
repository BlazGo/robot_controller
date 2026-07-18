#include "config.h"

const float DEFAULT_JOINT_SPEEDS[JOINT_NUM] = {
  0.4f,
  0.6f,
  0.3f,
  0.25f,
  1.5f,
  2.5f
};

const float DEFAULT_JOINT_ACCELS[JOINT_NUM] = {
  0.8f,
  1.0f,
  0.4f,
  0.3f,
  1.5f,
  2.0f
};

// In degrees as encoder returns degrees by default
const float ENCODER_OFFSETS[JOINT_NUM] = {
  0.0f,
  316.4f,
  17.72f,
  168.13f,
  284.0f,
  356.0f
};

const bool MOTOR_DIR_INVERTED[JOINT_NUM] = {
  true,
  false,  
  false,
  false,
  false,
  false  
};

 const uint8_t HOMING_JOINT_ORDER[JOINT_NUM] = {1, 2, 3, 4, 5, 0};
