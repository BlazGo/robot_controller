#ifndef DISPLAY_H
#define DISPLAY_H

#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include "config.h"



class Display{
  public:
    Display();

    void init();
    void displayInfo(float q[JOINT_NUM], float x[6], uint32_t timestamp);

  private:
    Adafruit_SSD1306 _display;
};

#endif // DISPLAY_H
