#ifndef ENCODER_MANAGER_H
#define ENCODER_MANAGER_H

#include <Arduino.h>

#include "encoder_types.h"
#include "MT6701.h"
#include "config.h"

static constexpr uint8_t TCA_ADDR = 0x70;

class EncoderManager{
  public:
    EncoderManager();

    void init();
    void updateAngles();
    void getAngles(float* out_angles);
    EncoderFrame getLatestFrame();

  private:
    MT6701 _encoder_list[JOINT_NUM];

    // buffer intended for latest measurements (long time to fill)
    EncoderFrame _frame_latest;
    // buffer intended for reading (just copy from latest)
    EncoderFrame _frame_latest_shared;
    // Sequential number of the measurement
    uint32_t _sample_idx;

    void switchChannel(uint8_t index);
};

#endif
