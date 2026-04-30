#ifndef ENCODER_MANAGER_H
#define ENCODER_MANAGER_H

#include "encoder.h"
#include "config.h"

struct JointMeasurement {
  float angle_deg;
  bool  valid;
};

class EncoderManager{
  public:
    EncoderManager();

    void init(void);
    void updateAngles();     // runs on core 1

    // SAFE read (core 0) via buffers
    void getAngles(float* out_angles);

  private:
    Encoder _encoder_list[5];   // only real encoders

    // double buffer
    JointMeasurement _bufferA[JOINT_NUM];
    JointMeasurement _bufferB[JOINT_NUM];

    // Volatile -> variable might change outside of current scope -> prevents compiler optimizations
    volatile JointMeasurement* _readBuffer;
    volatile JointMeasurement* _writeBuffer;

    void swapBuffers();
};

#endif