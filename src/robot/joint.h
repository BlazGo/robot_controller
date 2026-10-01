#ifndef JOINT_H
#define JOINT_H

#include <Arduino.h>
#include "motor.h"
#include "shared/joint_types.h"

class Joint {
public:
    Joint(float gear_ratio, uint8_t step_pin, uint8_t direction_pin,
          uint8_t microsteps, bool dir_inverted,
          float min_angle_rad, float max_angle_rad,
          float max_speed_rad_s, float max_acceleration_rad_s2);

    // False means invalid configuration. No motion commands are then accepted.
    bool initialize(void);
    void update(void);

    // Commands select their mode. Cross-mode changes require a stopped joint.
    bool moveToRad(float q_target_rad);
    bool setSpeedRad(float q_dot_requested_rad_s);
    void stop(void);

    // Stationary reference calibration, not continuous encoder feedback.
    bool setAngleFromEncoder(float q_measured_rad);

    bool isMoving(void);
    float getPositionRad(void);
    // Current applied step velocity [rad/s], NOT the requested target velocity.
    // This is an open-loop estimate, not encoder-measured physical velocity.
    float getSpeedRad(void);
    bool getStateMaxLim(void);
    bool getStateMinLim(void);
    ControlMode getControlMode(void);

private:
    void updateState(void);
    void updateSpeedRamp(float dt_s, float elapsed_time_s);
    float getSafeSpeedRad(float elapsed_time_s);
    static float moveTowards(float current_value, float target_value,
                             float maximum_change);
    bool angleFitsSteps(float q_rad);
    long angleRadToSteps(float q_rad);

    static constexpr uint32_t MAX_RAMP_DT_US = 10000UL;

    JointConfig _config;
    JointState _state;
    Motor _motor;

    float _requested_q_dot_rad_s;
    float _max_q_dot_rad_s;
    float _max_q_ddot_rad_s2;
    uint32_t _previous_update_time_us;
    bool _initialized;
};

#endif // JOINT_H
