#include "encoder_manager.h"
#include "config.h"

EncoderManager::EncoderManager() 
  : _encoder_list {},
    _sample_idx(0)
{
  for (int joint_idx = 0; joint_idx < JOINT_NUM; joint_idx++) {
    _frame_latest.joints[joint_idx] = {0.0f, false};
    _frame_latest_shared.joints[joint_idx] = {0.0f, false};
  }
  _frame_latest.frame_idx = 0;
  _frame_latest.timestamp_us = 0;
  _frame_latest_shared.frame_idx = 0;
  _frame_latest_shared.timestamp_us = 0;
}


void EncoderManager::init(){
  for (int i = 0; i < JOINT_NUM; i++) {
    // Skip first because no encoder
    if (i == 0){
      continue;
    }
    EncoderManager::switchChannel(i-1);
    delay(2);

    // initialize real encoders
    _encoder_list[i].initializeI2C();
    delay(20);
  }
}

EncoderFrame EncoderManager::getLatestFrame() {
  return _frame_latest_shared;
}

void EncoderManager::updateAngles() {
  for (int joint_idx = 0; joint_idx < JOINT_NUM; joint_idx++) {
    // Skip first because no encoder
    if (joint_idx == 0){
      _frame_latest.joints[joint_idx].angle_rad = 0.0f;
      _frame_latest.joints[joint_idx].valid = false;
      continue;
    }
    // Because I only have encoders on first 6 actual channels of the multiplexer
    // I have to have an offset here...
    uint8_t mux_channel = joint_idx - 1;

    EncoderManager::switchChannel(mux_channel);
    delayMicroseconds(20);
    _frame_latest.joints[joint_idx].angle_rad = DEG_TO_RAD * _encoder_list[joint_idx].angleRead();
    _frame_latest.joints[joint_idx].valid = true;
  }
  _sample_idx += 1;
  _frame_latest.frame_idx = _sample_idx;
  _frame_latest.timestamp_us = micros();
  _frame_latest_shared = _frame_latest;
}

void EncoderManager::getAngles(float* out_angles) {
  for (int joint_idx = 0; joint_idx < JOINT_NUM; joint_idx++) {
    out_angles[joint_idx] = _frame_latest_shared.joints[joint_idx].angle_rad;
  }
}

void EncoderManager::switchChannel(uint8_t mux_channel){
  if (mux_channel > 7) {
    return;
  }

  Wire.beginTransmission(TCA_ADDR);
  Wire.write(1u << mux_channel);
  Wire.endTransmission();
}
