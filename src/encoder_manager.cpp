#include "encoder_manager.h"
#include "config.h"

EncoderManager::EncoderManager() 
  : _encoder_list { Encoder(0),
                    Encoder(1),
                    Encoder(2),
                    Encoder(3),
                    Encoder(4)}
{
  _readBuffer  = _bufferA;
  _writeBuffer = _bufferB; 

  for (int i = 0; i < JOINT_NUM; i++) {
    _bufferA[i] = {0.0f, false};
    _bufferB[i] = {0.0f, false};
  }
}

void EncoderManager::init(){
  for (int i = 0; i < 5; i++) {
    _encoder_list[i].init();
  }
}

void EncoderManager::updateAngles(){

  // --- Joint 0 (no encoder) ---
  _writeBuffer[0].angle_deg = 0.0f;   // placeholder
  _writeBuffer[0].valid = false;      // marking it

  // --- Real encoders ---
  for (int i = 0; i < 5; i++) {
    float angle = _encoder_list[i].getAngle();

    _writeBuffer[i + 1].angle_deg = angle;  // shift by 1
    _writeBuffer[i + 1].valid = true;
  }

  swapBuffers();  // pointer swap
}

void EncoderManager::swapBuffers() {
  noInterrupts();  // short critical section
  auto temp = _readBuffer;
  _readBuffer = _writeBuffer;
  _writeBuffer = temp;
  interrupts();
}

void EncoderManager::getAngles(float* out_angles) {
  JointMeasurement* measurements = (JointMeasurement*)_readBuffer;

  for (int i = 0; i < JOINT_NUM; i++) {
    if (measurements[i].valid) {
      out_angles[i] = measurements[i].angle_deg - ENCODER_OFFSETS[i];
    }
  }
}