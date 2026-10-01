#include "robot.h"
#include "config/board_config.h"
#include "config/robot_config.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

Robot::Robot(ComHandler* com_handler_ptr, NodeProtocol* node_protocol_ptr)
    : _update_count(0), _freq_last_count(0), _freq_last_ms(0),
      _homing_started_ms(0), _homing_state(HomingState::IDLE),
      _is_initialized(false), _is_homed(false),
      _joints{
          Joint(JOINT_0_GEAR_RATIO, STEP_PIN_0, DIR_PIN_0, JOINT_0_MICROSTEPS, JOINT_0_DIR_INVERTED, JOINT_0_MIN_ANGLE, JOINT_0_MAX_ANGLE, JOINT_0_INTERNAL_MAX_SPEED, JOINT_0_INTERNAL_MAX_ACCEL),
          Joint(JOINT_1_GEAR_RATIO, STEP_PIN_1, DIR_PIN_1, JOINT_1_MICROSTEPS, JOINT_1_DIR_INVERTED, JOINT_1_MIN_ANGLE, JOINT_1_MAX_ANGLE, JOINT_1_INTERNAL_MAX_SPEED, JOINT_1_INTERNAL_MAX_ACCEL),
          Joint(JOINT_2_GEAR_RATIO, STEP_PIN_2, DIR_PIN_2, JOINT_2_MICROSTEPS, JOINT_2_DIR_INVERTED, JOINT_2_MIN_ANGLE, JOINT_2_MAX_ANGLE, JOINT_2_INTERNAL_MAX_SPEED, JOINT_2_INTERNAL_MAX_ACCEL),
          Joint(JOINT_3_GEAR_RATIO, STEP_PIN_3, DIR_PIN_3, JOINT_3_MICROSTEPS, JOINT_3_DIR_INVERTED, JOINT_3_MIN_ANGLE, JOINT_3_MAX_ANGLE, JOINT_3_INTERNAL_MAX_SPEED, JOINT_3_INTERNAL_MAX_ACCEL),
          Joint(JOINT_4_GEAR_RATIO, STEP_PIN_4, DIR_PIN_4, JOINT_4_MICROSTEPS, JOINT_4_DIR_INVERTED, JOINT_4_MIN_ANGLE, JOINT_4_MAX_ANGLE, JOINT_4_INTERNAL_MAX_SPEED, JOINT_4_INTERNAL_MAX_ACCEL),
          Joint(JOINT_5_GEAR_RATIO, STEP_PIN_5, DIR_PIN_5, JOINT_5_MICROSTEPS, JOINT_5_DIR_INVERTED, JOINT_5_MIN_ANGLE, JOINT_5_MAX_ANGLE, JOINT_5_INTERNAL_MAX_SPEED, JOINT_5_INTERNAL_MAX_ACCEL)
      },
      _config{}, _state{}, _com_handler_ptr(com_handler_ptr),
      _node_protocol_ptr(node_protocol_ptr)
{
}

void Robot::initialize()
{
    pinMode(ENABLE_PIN_0, OUTPUT);
    pinMode(ENABLE_PIN_1, OUTPUT);
    pinMode(END_SWITCH_0_MIN, INPUT_PULLUP);
    pinMode(END_SWITCH_0_MAX, INPUT_PULLUP);
    disable();

    _is_initialized = true;
    for (uint8_t i = 0; i < JOINT_NUM; ++i) {
        if (!_joints[i].initialize()) {
            _is_initialized = false;
        }
    }
    _update_count = 0;
    _freq_last_count = 0;
    _freq_last_ms = millis();
    _state.timestamp = _freq_last_ms;
    _homing_state = HomingState::IDLE;
    _is_homed = false;
    refreshRobotState();
}

void Robot::update()
{
    if (!_is_initialized) return;
    updateFrequency();
    updateHoming();

    for (uint8_t i = 0; i < JOINT_NUM; ++i) {
        _joints[i].update();
    }
    refreshRobotState();
}

void Robot::updateFrequency()
{
    ++_update_count;
    _state.timestamp = millis();
    const uint32_t elapsed_ms = _state.timestamp - _freq_last_ms;
    if (elapsed_ms >= 1000UL) {
        const uint32_t update_count_delta = _update_count - _freq_last_count;
        _state.update_frequency = update_count_delta * 1000.0f / elapsed_ms;
        _freq_last_count = _update_count;
        _freq_last_ms = _state.timestamp;
    }
}

void Robot::refreshRobotState()
{
    _state.is_moving = false;
    for (uint8_t i = 0; i < JOINT_NUM; ++i) {
        _state.q[i] = _joints[i].getPositionRad();
        _state.q_dot[i] = _joints[i].getSpeedRad();
        _state.min_limits[i] = _joints[i].getStateMinLim();
        _state.max_limits[i] = _joints[i].getStateMaxLim();
        if (_joints[i].isMoving()) _state.is_moving = true;
    }
}

void Robot::enable()
{
    digitalWrite(ENABLE_PIN_0, LOW);
    digitalWrite(ENABLE_PIN_1, LOW);
    _state.is_enabled = true;
}

void Robot::disable()
{
    digitalWrite(ENABLE_PIN_0, HIGH);
    digitalWrite(ENABLE_PIN_1, HIGH);
    _state.is_enabled = false;
}

bool Robot::moveJointToRad(const float q_target_rad[JOINT_NUM])
{
    // if (!_is_initialized || _homing_state == HomingState::SEEK_MIN || _homing_state == HomingState::SEEK_MAX) return false;
    bool accepted = true;
    for (uint8_t i = 0; i < JOINT_NUM; ++i) {
        if (!_joints[i].moveToRad(q_target_rad[i])) accepted = false;
    }
    return accepted;
}

bool Robot::setJointSpeed(const float q_dot_rad_s[JOINT_NUM])
{
    // if (!_is_initialized || _homing_state == HomingState::SEEK_MIN || _homing_state == HomingState::SEEK_MAX) return false;
    bool accepted = true;
    for (uint8_t i = 0; i < JOINT_NUM; ++i) {
        if (!_joints[i].setSpeedRad(q_dot_rad_s[i])) accepted = false;
    }
    return accepted;
}

void Robot::stop()
{
    for (uint8_t i = 0; i < JOINT_NUM; ++i) _joints[i].stop();
}

// ------------ HOMING ------------ //

bool Robot::startHoming()
{
    if (!_is_initialized || _state.is_moving || _homing_state == HomingState::SEEK_MIN || _homing_state == HomingState::SEEK_MAX) return false;
    if (!isSwitchConfigured(true) || !isSwitchConfigured(false)) return false;
    _is_homed = false;
    _homing_started_ms = millis();
    _homing_state = HomingState::SEEK_MIN;
    return beginHomingSeek(true);
}

void Robot::cancelHoming()
{
    if (_homing_state == HomingState::SEEK_MIN || _homing_state == HomingState::SEEK_MAX) _joints[HOMING_JOINT_INDEX].stop();
    _homing_state = HomingState::FAILED;
    _is_homed = false;
}

bool Robot::isHoming() const
{
    return _homing_state == HomingState::SEEK_MIN || _homing_state == HomingState::SEEK_MAX;
}

bool Robot::isHomed() const { return _is_homed; }

void Robot::updateHoming()
{
    if (!isHoming()) return;
    if (millis() - _homing_started_ms > HOMING_TIMEOUT_MS) {
        cancelHoming();
        return;
    }

    const bool seek_min = _homing_state == HomingState::SEEK_MIN;
    if (isEndSwitchActive(seek_min)) {
        _joints[HOMING_JOINT_INDEX].stop();
        if (!setJoint0HomedPosition(seek_min)) { cancelHoming(); return; }
        if (seek_min) {
            _homing_state = HomingState::SEEK_MAX;
            _homing_started_ms = millis();
            beginHomingSeek(false);
        } else {
            _homing_state = HomingState::COMPLETE;
            _is_homed = true;
        }
    }
}

bool Robot::beginHomingSeek(bool seek_min)
{
    const float speed = seek_min ? -HOMING_SPEED_RAD_S : HOMING_SPEED_RAD_S;
    return _joints[HOMING_JOINT_INDEX].setSpeedRad(speed);
}

bool Robot::setJoint0HomedPosition(bool seek_min)
{
    return _joints[HOMING_JOINT_INDEX].setAngleFromEncoder(
        seek_min ? HOMING_SWITCH_MIN_ANGLE_RAD : HOMING_SWITCH_MAX_ANGLE_RAD);
}

bool Robot::isEndSwitchActive(bool seek_min) const
{
    return digitalRead(seek_min ? END_SWITCH_0_MIN : END_SWITCH_0_MAX) == LOW;
}

bool Robot::isSwitchConfigured(bool seek_min) const
{
    return (seek_min ? END_SWITCH_0_MIN : END_SWITCH_0_MAX) >= 0;
}

// ------------ HOMING END ------------ //

// ------------ ENCODERS ------------ //

bool Robot::calibrateFromEncoders()
{
    if (!_is_initialized || _state.is_moving || isHoming()) return false;
    bool all_valid = true;
    for (uint8_t i = 1; i < JOINT_NUM; ++i) {
        const bool valid = _node_protocol_ptr && _node_protocol_ptr->isAngleValid(i) &&
            _node_protocol_ptr->getAngleAge(i) <= ENCODER_MAX_AGE_MS;
        if (valid) {
            if (!_joints[i].setAngleFromEncoder(_node_protocol_ptr->getAngle(i))) all_valid = false;
        } else {
            all_valid = false;
        }
    }
    return all_valid;
}

// ------------ ENCODERS END ------------ //

void Robot::handleCmd(const ParsedCommand& cmd)
{
    if (cmd.id == CMD_MOVE_JOINT) {
        bool result = moveJointToRad(cmd.params);
    }
    if (cmd.id == CMD_MOVE_JOINT_VEL) {
        bool result = setJointSpeed(cmd.params);
    }
    if (cmd.id == CMD_START_HOMING) {
        bool result = setJointSpeed(cmd.params);
    }   
    if (cmd.id == CMD_SET_CALIB_FROM_ENC) {
        const bool result = calibrateFromEncoders();
        if (_com_handler_ptr) _com_handler_ptr->queueResponse(result ? "CALIBRATE_DONE" : "CALIBRATE_FAILED");
    }
    if (cmd.id == CMD_GET_ENC_ANGLE) {
        char response[MAX_TX_BUFFER_SIZE];

        const float q_1_rad = _node_protocol_ptr->getAngle(1);
        const float q_2_rad = _node_protocol_ptr->getAngle(2);
        const float q_3_rad = _node_protocol_ptr->getAngle(3);
        const float q_4_rad = _node_protocol_ptr->getAngle(4);
        const float q_5_rad = _node_protocol_ptr->getAngle(5);

        snprintf(response, sizeof(response), "ENC_ANGLE,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f", q_1_rad, q_2_rad, q_3_rad, q_4_rad, q_5_rad);
        if (_com_handler_ptr) _com_handler_ptr->queueResponse(response);
    }
}

RobotState Robot::getRobotState() const
{
    return _state;
}

float Robot::getUpdateFrequency() const
{
    return _state.update_frequency;
}

bool Robot::isValidJointIndex(uint8_t i) const
{
    return i < JOINT_NUM;
}

float Robot::getJointPositionRad(uint8_t i)
{
    return isValidJointIndex(i) ? _joints[i].getPositionRad() : 0.0f;
}

float Robot::getJointSpeedRad(uint8_t i)
{
    return isValidJointIndex(i) ? _joints[i].getSpeedRad() : 0.0f;
}

bool Robot::getJointMinLimit(uint8_t i)
{
    return isValidJointIndex(i) && _joints[i].getStateMinLim();
}

bool Robot::getJointMaxLimit(uint8_t i)
{
    return isValidJointIndex(i) && _joints[i].getStateMaxLim();
}

bool Robot::getJointIsMoving(uint8_t i)
{
    return isValidJointIndex(i) && _joints[i].isMoving();
}

ControlMode Robot::getJointControlMode(uint8_t i)
{
    return isValidJointIndex(i) ? _joints[i].getControlMode() : ControlMode::POSITION;
}
