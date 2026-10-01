#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

#include <Arduino.h>

// Choose which MCU you are using (Only pick one) TODO: better selection...
#define WEACT_RP2350B 0
#define WEACT_RP2350A 1
#define RP2040 0

#define SERIAL_BAUDRATE 115200
#define RS485_BAUDRATE 921600

#if (RP2040 == 1 || WEACT_RP2350A == 1)
  #define BLUE_LED 25

  #define STEP_PIN_0  18
  #define DIR_PIN_0   17

  #define STEP_PIN_1 -1
  #define DIR_PIN_1  -1

  #define STEP_PIN_2 -1
  #define DIR_PIN_2  -1
  
  #define STEP_PIN_3 -1
  #define DIR_PIN_3  -1

  #define STEP_PIN_4 -1
  #define DIR_PIN_4  -1

  #define STEP_PIN_5 -1
  #define DIR_PIN_5  -1

  #define ENABLE_PIN_0  26
  #define ENABLE_PIN_1  -1

  #define END_SWITCH_0_MIN -1
  #define END_SWITCH_0_MAX -1

#elif (WEACT_RP2350B == 1)

  // #define LED_BUILTIN 25
  #define BLUE_LED 25
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

  #define END_SWITCH_0_MIN 35
  #define END_SWITCH_0_MAX 36

#else
  #error "Unsupported board: please define RP2350B or STM32H5 platform"
#endif

#endif // BOARD_CONFIG_H