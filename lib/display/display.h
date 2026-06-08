#ifndef DISPLAY_H
#define DISPLAY_H

#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include "config.h"

// ================================
// OLED configuration
// ================================
#define SCREEN_WIDTH    128
#define SCREEN_HEIGHT   64
#define OLED_RESET      -1
#define SCREEN_ADDRESS  0x3C
#define ROBOT_CONTROLLER_VERSION 0.1f

class Display{
  public:
    Display();

    void init();
    void displayInfo(float q[JOINT_NUM], float x[6]);

  private:
    Adafruit_SSD1306 _display;
};

#endif // DISPLAY_H
