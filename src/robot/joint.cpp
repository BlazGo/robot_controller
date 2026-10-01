#include "joint.h"
#include <math.h>
#include <limits.h>

Joint::Joint(float gear_ratio, uint8_t step_pin, uint8_t direction_pin,
             uint8_t microsteps, bool dir_inverted,
             float min_angle_rad, float max_angle_rad,
             float max_speed_rad_s, float max_acceleration_rad_s2)
    : _config{},
      _state{},
      _motor(step_pin, direction_pin, microsteps, dir_inverted),
      _requested_q_dot_rad_s(0.0f),
      _max_q_dot_rad_s(fabsf(max_speed_rad_s)),
      _max_q_ddot_rad_s2(fabsf(max_acceleration_rad_s2)),
      _previous_update_time_us(0),
      _initialized(false)
{
    _config.gear_ratio = gear_ratio;
    _config.motor_steps_per_joint_rev = _motor.getStepsPerRev() * gear_ratio;
    _config.steps_per_rad = _config.motor_steps_per_joint_rev / (2.0f * PI);
    _config.rads_per_step = (_config.steps_per_rad > 0.0f)
        ? 1.0f / _config.steps_per_rad : 0.0f;
    _config.min_angle_rad = min_angle_rad;
    _config.max_angle_rad = max_angle_rad;
}

bool Joint::initialize()
{
    if (_initialized) {
        return true;
    }
    _motor.initialize();

    const float max_speed_steps_s =
        _max_q_dot_rad_s * _config.steps_per_rad;
    const float max_acceleration_steps_s2 =
        _max_q_ddot_rad_s2 * _config.steps_per_rad;

    if (!isfinite(_config.gear_ratio) || _config.gear_ratio <= 0.0f ||
        !isfinite(_config.steps_per_rad) || _config.steps_per_rad <= 0.0f ||
        !isfinite(_config.rads_per_step) || _config.rads_per_step <= 0.0f ||
        !angleFitsSteps(_config.min_angle_rad) ||
        !angleFitsSteps(_config.max_angle_rad) ||
        _config.min_angle_rad >= _config.max_angle_rad ||
        !isfinite(max_speed_steps_s) || max_speed_steps_s <= 0.0f ||
        !isfinite(max_acceleration_steps_s2) ||
        max_acceleration_steps_s2 <= 0.0f) {
        return false;
    }

    const double min_steps = ceil(static_cast<double>(_config.min_angle_rad)
                                  * _config.steps_per_rad);
    const double max_steps = floor(static_cast<double>(_config.max_angle_rad)
                                   * _config.steps_per_rad);
    if (min_steps > max_steps) {
        return false;
    }

    _motor.setMaxSpeedSteps(max_speed_steps_s);
    _motor.setAccelerationSteps(max_acceleration_steps_s2);
    _requested_q_dot_rad_s = 0.0f;
    _previous_update_time_us = micros();
    _initialized = true;
    updateState();
    return true;
}

void Joint::update()
{
    if (!_initialized) {
        return;
    }

    const uint32_t current_time_us = micros();
    const uint32_t elapsed_time_us = current_time_us - _previous_update_time_us;
    _previous_update_time_us = current_time_us;
    const uint32_t ramp_dt_us = (elapsed_time_us > MAX_RAMP_DT_US)
        ? MAX_RAMP_DT_US : elapsed_time_us;

    if (_motor.getControlMode() == ControlMode::SPEED) {
        updateSpeedRamp(ramp_dt_us * 0.000001f,
                        elapsed_time_us * 0.000001f);
    }

    _motor.update();
    updateState();
}

bool Joint::moveToRad(float q_target_rad)
{
    if (!_initialized || !angleFitsSteps(q_target_rad) || q_target_rad < _config.min_angle_rad || q_target_rad > _config.max_angle_rad) {
        return false;
    }

    // Keep the quantized target inside the configured angle interval.
    const long min_steps = static_cast<long>(ceil(static_cast<double>(_config.min_angle_rad) * _config.steps_per_rad));
    const long max_steps = static_cast<long>(floor(static_cast<double>(_config.max_angle_rad) * _config.steps_per_rad));
    long target_steps = angleRadToSteps(q_target_rad);
    if (target_steps < min_steps) target_steps = min_steps;
    if (target_steps > max_steps) target_steps = max_steps;

    if (!_motor.moveToSteps(target_steps)) {
        return false;
    }
    _requested_q_dot_rad_s = 0.0f;
    updateState();
    return true;
}

bool Joint::setSpeedRad(float q_dot_requested_rad_s)
{
    if (!_initialized || !isfinite(q_dot_requested_rad_s)) {
        return false;
    }

    if (_motor.getControlMode() != ControlMode::SPEED) {
        if (!_motor.setSpeedSteps(0.0f)) {
            return false;
        }
    }

    _requested_q_dot_rad_s = fmaxf(-_max_q_dot_rad_s,
        fminf(q_dot_requested_rad_s, _max_q_dot_rad_s));
    updateState();
    return true;
}

void Joint::stop()
{
    _requested_q_dot_rad_s = 0.0f;
    _motor.stopPositionMove();
    updateState();
}

bool Joint::setAngleFromEncoder(float q_measured_rad)
{
    if (!_initialized || !angleFitsSteps(q_measured_rad) ||
        !_motor.setPositionSteps(angleRadToSteps(q_measured_rad))) {
        return false;
    }
    // Calibration must not leave a pending speed request active.
    _requested_q_dot_rad_s = 0.0f;
    updateState();
    return true;
}

void Joint::updateState()
{
    _state.currentPositionRad = getPositionRad();
    _state.currentSpeedRad = getSpeedRad();
    _state.isMoving = isMoving();
    _state.min_limit = getStateMinLim();
    _state.max_limit = getStateMaxLim();
}

float Joint::getSafeSpeedRad(float elapsed_time_s)
{
    const float q_rad = getPositionRad();
    const float q_dot_rad_s = getSpeedRad();

    // Reserve one step plus the last update interval's travel.
    // This is a software braking envelope, not a hard-limit guarantee.
    const float margin_rad = _config.rads_per_step
        + fabsf(q_dot_rad_s) * elapsed_time_s;
    const float positive_distance_rad = fmaxf(0.0f,
        _config.max_angle_rad - q_rad - margin_rad);
    const float negative_distance_rad = fmaxf(0.0f,
        q_rad - _config.min_angle_rad - margin_rad);

    const float positive_speed_limit_rad_s = fminf(_max_q_dot_rad_s,
        sqrtf(2.0f * _max_q_ddot_rad_s2 * positive_distance_rad));
    const float negative_speed_limit_rad_s = fminf(_max_q_dot_rad_s,
        sqrtf(2.0f * _max_q_ddot_rad_s2 * negative_distance_rad));

    return fmaxf(-negative_speed_limit_rad_s,
        fminf(_requested_q_dot_rad_s, positive_speed_limit_rad_s));
}

void Joint::updateSpeedRamp(float dt_s, float elapsed_time_s)
{
    const float applied_q_dot_rad_s = moveTowards(
        getSpeedRad(), getSafeSpeedRad(elapsed_time_s),
        _max_q_ddot_rad_s2 * dt_s);
    _motor.setSpeedSteps(applied_q_dot_rad_s * _config.steps_per_rad);
}

float Joint::moveTowards(float current_value, float target_value,
                         float maximum_change)
{
    const float difference = target_value - current_value;
    if (fabsf(difference) <= maximum_change) {
        return target_value;
    }
    return current_value + ((difference > 0.0f)
        ? maximum_change : -maximum_change);
}

bool Joint::angleFitsSteps(float q_rad)
{
    const double steps = static_cast<double>(q_rad) * _config.steps_per_rad;
    return isfinite(q_rad) && isfinite(steps)
        && steps >= static_cast<double>(LONG_MIN) + 1.0
        && steps <= static_cast<double>(LONG_MAX) - 1.0;
}

long Joint::angleRadToSteps(float q_rad)
{
    return static_cast<long>(lround(
        static_cast<double>(q_rad) * _config.steps_per_rad));
}

bool Joint::isMoving()
{
    return _motor.isMoving();
}

float Joint::getPositionRad()
{
    return _motor.getPositionSteps() * _config.rads_per_step;
}

float Joint::getSpeedRad()
{
    return _motor.getSpeedSteps() * _config.rads_per_step;
}

bool Joint::getStateMaxLim()
{
    return getPositionRad() >= _config.max_angle_rad;
}

bool Joint::getStateMinLim()
{
    return getPositionRad() <= _config.min_angle_rad;
}

ControlMode Joint::getControlMode()
{
    return _motor.getControlMode();
}
