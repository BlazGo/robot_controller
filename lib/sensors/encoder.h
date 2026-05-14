#ifndef ENCODER_H
#define ENCODER_H

#include "MT6701.h"
#include <Wire.h>

class Encoder{

  public:
    Encoder(uint8_t index);

    void init(void);
    float getAngle(void);

  private:
    MT6701 _encoder;
    uint8_t  _index;
    void selectMuxChannel();
};

#endif  // End of ENCODER_H define
