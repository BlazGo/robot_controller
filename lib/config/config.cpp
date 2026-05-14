#include "config.h"

const float DEFAULT_JOINT_SPEEDS[JOINT_NUM] = {
  0.5f,
  0.2f,
  0.2f,
  0.1f,
  0.2f,
  0.4f
};
const float DEFAULT_JOINT_ACCELS[JOINT_NUM] = {
  0.2f,
  0.2f,
  0.2f,
  0.1f,
  0.2f,
  0.4f
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
