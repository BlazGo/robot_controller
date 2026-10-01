#ifndef ROBOT_H
#define ROBOT_H

#include <Arduino.h>
#include "joint.h"
#include "shared/robot_types.h"
#include "communication/com_handler.h"
#include "communication/node_protocol.h"

class Robot {
public:
    Robot(ComHandler* com_handler_ptr, NodeProtocol* node_protocol_ptr);

    void initialize();
    void update();
    void enable();
    void disable();

    bool moveJointToRad(const float q_target_rad[JOINT_NUM]);
    bool setJointSpeed(const float q_dot_rad_s[JOINT_NUM]);
    void stop();

    // Homing is non-blocking. Call update() until isHoming() is false.
    bool startHoming();
    void cancelHoming();
    bool isHoming() const;
    bool isHomed() const;

    // Applies valid, recent encoder references while all joints are stopped.
    bool calibrateFromEncoders();

    RobotState getRobotState() const;
    float getUpdateFrequency() const;
    void handleCmd(const ParsedCommand& cmd);

    float getJointPositionRad(uint8_t joint_index);
    float getJointSpeedRad(uint8_t joint_index);
    bool getJointMinLimit(uint8_t joint_index);
    bool getJointMaxLimit(uint8_t joint_index);
    bool getJointIsMoving(uint8_t joint_index);
    ControlMode getJointControlMode(uint8_t joint_index);

private:
    enum class HomingState : uint8_t { IDLE, SEEK_MIN, SEEK_MAX, COMPLETE, FAILED };

    void updateFrequency();
    void updateHoming();
    bool beginHomingSeek(bool seek_min);
    bool setJoint0HomedPosition(bool seek_min);
    bool isEndSwitchActive(bool seek_min) const;
    bool isSwitchConfigured(bool seek_min) const;
    bool isValidJointIndex(uint8_t joint_index) const;
    void refreshRobotState();

    static constexpr uint8_t HOMING_JOINT_INDEX = 0U;
    static constexpr uint32_t HOMING_TIMEOUT_MS = 30000UL;
    static constexpr float HOMING_SPEED_RAD_S = 0.20f;
    static constexpr float HOMING_SWITCH_MIN_ANGLE_RAD = JOINT_0_MIN_ANGLE;
    static constexpr float HOMING_SWITCH_MAX_ANGLE_RAD = JOINT_0_MAX_ANGLE;

    uint32_t _update_count;
    uint32_t _freq_last_count;
    uint32_t _freq_last_ms;
    uint32_t _homing_started_ms;

    HomingState _homing_state;
    bool _is_initialized;
    bool _is_homed;

    Joint _joints[JOINT_NUM];
    RobotConfig _config;
    RobotState _state;
    ComHandler* _com_handler_ptr;
    NodeProtocol* _node_protocol_ptr;
};

#endif // ROBOT_H
