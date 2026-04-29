#include "encoder.h"

Encoder::Encoder(uint8_t index)
  : _index(index),
  _encoder(){

}

// ─────────────────────────────────────────────────────────────
// Public functions
// ─────────────────────────────────────────────────────────────
void Encoder::init(){
  selectMuxChannel();
  _encoder.initializeI2C();
}

float Encoder::getAngle(){
  return _encoder.angleRead();
}

// ─────────────────────────────────────────────────────────────
// Private functions
// ─────────────────────────────────────────────────────────────
void Encoder::selectMuxChannel(){
  Wire.beginTransmission(0x70);
  Wire.write(1<<_index);
  Wire.endTransmission();
}