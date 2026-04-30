#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// Choose which MCU you are using (Only pick one) TODO: better selection...
#define WEACT_RP2350B 1
#define WEACT_STM32H5 0
#define RP2040 0

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

#define JOINT_NUM 6

// ─────────────────────────────────────────────────────────────
// PLATFORM-SPECIFIC PIN DEFINITIONS WeAct RP2350-B variant
// ─────────────────────────────────────────────────────────────

#if (RP2040 == 1)

  #define STEP_PIN_0  17
  #define DIR_PIN_0   16

  #define STEP_PIN_1 0
  #define DIR_PIN_1  0

  #define STEP_PIN_2 0
  #define DIR_PIN_2  0
  
  #define STEP_PIN_3 0
  #define DIR_PIN_3  0

  #define STEP_PIN_4 0
  #define DIR_PIN_4  0

  #define STEP_PIN_5 0
  #define DIR_PIN_5  0

  #define ENABLE_PIN_0  18
  #define ENABLE_PIN_1  0

  #define END_SWITCH_0_N 0
  #define END_SWITCH_0_P 0

#elif (WEACT_RP2350B == 1)

  // #define LED_BUILTIN 25

  #define STEP_PIN_0  28
  #define DIR_PIN_0   27

  #define STEP_PIN_1 30
  #define DIR_PIN_1  29

  #define STEP_PIN_2 32
  #define DIR_PIN_2  31
  
  #define STEP_PIN_3 22
  #define DIR_PIN_3  21

  #define STEP_PIN_4 19
  #define DIR_PIN_4  20

  #define STEP_PIN_5 18
  #define DIR_PIN_5  17

  #define ENABLE_PIN_0 24
  #define ENABLE_PIN_1  26

  #define END_SWITCH_0_N 35
  #define END_SWITCH_0_P 36

#elif (WEACT_STM32H5 == 1)
  #error "Unsupported board: STM32 platform not yet implemented"

#else
  #error "Unsupported board: please define RP2350B or STM32H5 platform"
#endif

// ─────────────────────────────────────────────────────────────
// MOTOR CONFIGURATION: MICROSTEPS & GEAR RATIOS
// ─────────────────────────────────────────────────────────────

// Microstepping settings (e.g., 1, 2, 4, 8, 16, 32, etc.)
#define MICROSTEPS_0 1
#define MICROSTEPS_1 1
#define MICROSTEPS_2 1
#define MICROSTEPS_3 8
#define MICROSTEPS_4 1
#define MICROSTEPS_5 8

// Gear ratios (output rotations per motor shaft rotation)
#define GEAR_RATIO_0 12.0f
#define GEAR_RATIO_1 8.0f
#define GEAR_RATIO_2 7.0f*14.0f
#define GEAR_RATIO_3 7.0f*5.18f
#define GEAR_RATIO_4 3.0f
#define GEAR_RATIO_5 1.0f

// ─────────────────────────────────────────────────────────────
//  JOINT LIMITS 
// ─────────────────────────────────────────────────────────────

// Joint limits in radians (degrees but converted)
#define MIN_ANGLE_0  (-120.0f * PI / 180.0f)
#define MAX_ANGLE_0  ( 120.0f * PI / 180.0f)

#define MIN_ANGLE_1  (-20.0f * PI / 180.0f)
#define MAX_ANGLE_1  ( 60.0f * PI / 180.0f)

#define MIN_ANGLE_2  (-80.0f * PI / 180.0f)
#define MAX_ANGLE_2  ( 30.0f * PI / 180.0f)

#define MIN_ANGLE_3  (-25.0f * PI / 180.0f)
#define MAX_ANGLE_3  ( 25.0f * PI / 180.0f)

#define MIN_ANGLE_4  (-175.0f * PI / 180.0f)
#define MAX_ANGLE_4  ( 175.0f * PI / 180.0f)

#define MIN_ANGLE_5  (-175.0f * PI / 180.0f)
#define MAX_ANGLE_5  ( 175.0f * PI / 180.0f)

const float MIN_ANGLES[JOINT_NUM] = {
  MIN_ANGLE_0,
  MIN_ANGLE_1,
  MIN_ANGLE_2,
  MIN_ANGLE_3,
  MIN_ANGLE_4,
  MIN_ANGLE_5
};

const float MAX_ANGLES[JOINT_NUM] = {
  MAX_ANGLE_0,
  MAX_ANGLE_1,
  MAX_ANGLE_2,
  MAX_ANGLE_3,
  MAX_ANGLE_4,
  MAX_ANGLE_5
};

// ─────────────────────────────────────────────────────────────
//  DEFAULT MOVEMENT  
// ───────────────────────────────────────────────────────────── 

#define DEFAULT_JOINT_ACCEL 5.0f
#define DEFAULT_JOINT_SPEED 10.0f

extern const float DEFAULT_JOINT_SPEEDS[JOINT_NUM];
extern const float DEFAULT_JOINT_ACCELS[JOINT_NUM];
extern const float ENCODER_OFFSETS[JOINT_NUM];

// ─────────────────────────────────────────────────────────────
// DERIVED VALUES (optional for calculations)
// ─────────────────────────────────────────────────────────────

#define STEPS_PER_REV_0 (FULL_STEPS_PER_REV * MICROSTEPS_0 * GEAR_RATIO_0)
#define STEPS_PER_REV_1 (FULL_STEPS_PER_REV * MICROSTEPS_1 * GEAR_RATIO_1)
#define STEPS_PER_REV_2 (FULL_STEPS_PER_REV * MICROSTEPS_2 * GEAR_RATIO_2)
#define STEPS_PER_REV_3 (FULL_STEPS_PER_REV * MICROSTEPS_3 * GEAR_RATIO_3)
#define STEPS_PER_REV_4 (FULL_STEPS_PER_REV * MICROSTEPS_4 * GEAR_RATIO_4)
#define STEPS_PER_REV_5 (FULL_STEPS_PER_REV * MICROSTEPS_5 * GEAR_RATIO_5)


#endif // CONFIG_H