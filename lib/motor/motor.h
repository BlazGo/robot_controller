#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>
#include <AccelStepper.h>

#define FULL_STEPS_PER_REV 200L

class Motor {
public:
  Motor(uint8_t step_pin, uint8_t dir_pin, uint8_t microsteps);

  void init(void);
  void update(void);
  bool isMoving(void) const;

  void setCurrPosition(long curr_position);
  void setTargetSpeed(float target_speed);
  void setMaxSpeed(float max_speed);
  void setMaxAcceleration(float max_acceleration);

  long getCurrPosition(void) const;
  float getCurrSpeed(void) const;
  float getMaxSpeed(void) const;
  float getMaxAcceleration(void) const;
  long getStepsPerRev(void) const;

  void stop(void);
  void emergencyStop(void);
  
private:
  uint8_t _step_pin;
  uint8_t _dir_pin;
  uint8_t _microsteps;
  long _steps_per_rev;
  
  long _curr_position;
  float _curr_speed;
  float _target_speed;
  float _max_speed;
  float _max_acceleration;

  uint32_t _last_update_us;
  bool _is_moving;

  AccelStepper _stepper;

  void setCurrSpeed(float curr_speed);
};

#endif
