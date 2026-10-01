#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

#include <Arduino.h>
#include "version.h"

#define JOINT_NUM 6

constexpr uint16_t JOINT_0_MICROSTEPS = 8U;
constexpr uint16_t JOINT_1_MICROSTEPS = 1U;
constexpr uint16_t JOINT_2_MICROSTEPS = 1U;
constexpr uint16_t JOINT_3_MICROSTEPS = 8U;
constexpr uint16_t JOINT_4_MICROSTEPS = 1U;
constexpr uint16_t JOINT_5_MICROSTEPS = 8U;

constexpr float JOINT_0_GEAR_RATIO = 12.0f;
constexpr float JOINT_1_GEAR_RATIO = 8.0f;
constexpr float JOINT_2_GEAR_RATIO = 7.0f * 14.0f;
constexpr float JOINT_3_GEAR_RATIO = 7.0f * 5.18f;
constexpr float JOINT_4_GEAR_RATIO = 3.0f;
constexpr float JOINT_5_GEAR_RATIO = 1.0f;

constexpr bool JOINT_0_DIR_INVERTED = false;
constexpr bool JOINT_1_DIR_INVERTED = true;
constexpr bool JOINT_2_DIR_INVERTED = true;
constexpr bool JOINT_3_DIR_INVERTED = true;
constexpr bool JOINT_4_DIR_INVERTED = true;
constexpr bool JOINT_5_DIR_INVERTED = true;

constexpr float JOINT_0_INTERNAL_MAX_SPEED = 1.0f;
constexpr float JOINT_1_INTERNAL_MAX_SPEED = 1.0f;
constexpr float JOINT_2_INTERNAL_MAX_SPEED = 1.0f;
constexpr float JOINT_3_INTERNAL_MAX_SPEED = 1.0f;
constexpr float JOINT_4_INTERNAL_MAX_SPEED = 1.0f;
constexpr float JOINT_5_INTERNAL_MAX_SPEED = 1.0f;

constexpr float JOINT_0_INTERNAL_MAX_ACCEL = 2.0f;
constexpr float JOINT_1_INTERNAL_MAX_ACCEL = 2.0f;
constexpr float JOINT_2_INTERNAL_MAX_ACCEL = 2.0f;
constexpr float JOINT_3_INTERNAL_MAX_ACCEL = 2.0f;
constexpr float JOINT_4_INTERNAL_MAX_ACCEL = 2.0f;
constexpr float JOINT_5_INTERNAL_MAX_ACCEL = 2.0f;

// Joint limits in radians (degrees but converted)
#define JOINT_0_MIN_ANGLE  (-175.0f * PI / 180.0f)
#define JOINT_0_MAX_ANGLE  ( 175.0f * PI / 180.0f)

#define JOINT_1_MIN_ANGLE  (-20.0f * PI / 180.0f)
#define JOINT_1_MAX_ANGLE  ( 60.0f * PI / 180.0f)

#define JOINT_2_MIN_ANGLE  (-80.0f * PI / 180.0f)
#define JOINT_2_MAX_ANGLE  ( 30.0f * PI / 180.0f)

#define JOINT_3_MIN_ANGLE  (-25.0f * PI / 180.0f)
#define JOINT_3_MAX_ANGLE  ( 25.0f * PI / 180.0f)

#define JOINT_4_MIN_ANGLE  (-175.0f * PI / 180.0f)
#define JOINT_4_MAX_ANGLE  ( 175.0f * PI / 180.0f)

#define JOINT_5_MIN_ANGLE  (-175.0f * PI / 180.0f)
#define JOINT_5_MAX_ANGLE  ( 175.0f * PI / 180.0f)

constexpr unsigned long ENCODER_MAX_AGE_MS = 500; // reject stale reads

#endif // ROBOT_CONFIG_H