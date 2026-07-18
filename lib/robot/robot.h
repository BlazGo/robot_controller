#ifndef ROBOT_H
#define ROBOT_H

#include "config.h"
#include "joint.h"
#include "robot_command_queue.h"
#include "robot_state_shared.h"
#include "robot_types.h"

struct DHParam {
    float theta;
    float alpha;
    float d;
    float a;
};

inline constexpr float kAngleRadSpeedTolerance = 0.0025f;
inline constexpr float kAngleRadPositionTolerance = 0.005f;

inline constexpr DHParam dh_table[JOINT_NUM] = {
    { 0.0f,     -PI / 2.0f, 102.0f,   0.0f   },  // joint 0
    { -PI / 2.0f, 0.0f,       0.0f,   210.0f },  // joint 1
    { 0.0f,     -PI / 2.0f,   0.0f,   0.0f   },  // joint 2
    { 0.0f,      PI / 2.0f, 202.0f,   0.0f   },  // joint 3
    { PI,        PI / 2.0f,   0.0f,   0.0f   },  // joint 4
    { 0.0f,      0.0f,       43.5f,   0.0f   }   // joint 5
};

class Robot {
public:
    Robot();

    void init();
    void update();

    void enable();
    void disable();

    bool moveJoint(const float target_joint_pose[JOINT_NUM]);
    void moveCart(const float target_cart_pose[6]);

    void setJointAngles(const float q[JOINT_NUM]);
    void setMaxJointSpeed(const float max_speed[JOINT_NUM]);
    void setMaxJointAcceleration(const float max_accel[JOINT_NUM]);
    void setMotionControlParadigm(robot_motion_control_paradigm_t motion_control_paradigm);
    void attachEncoderManager(EncoderManager* encoderManager);
    bool acceptCommand(const RobotCommand& cmd);
    bool goToZero();
    bool goToReady();

    const RobotState getState();
    const float* getMaxJointSpeed();
    const float* getMaxJointAcceleration();

    bool isBusy() const;
    bool isMoving() const;
    RobotExecState computeExecState() const;

    void computeForwardKinematics(const float (&q)[JOINT_NUM],
                                  Matrix4x4 (&T)[JOINT_NUM + 1]);
    void computeGeometricJacobian(const Matrix4x4 (&T)[JOINT_NUM + 1],
                                  Matrix6x6 (&J));
    void computeDLSMethod(float (&q_dot)[JOINT_NUM],
                          const Matrix6x6 (&J),
                          Vect6f x_dot);
    Vect6f computeCartErr(const Matrix4x4 T_curr, float (&x_goal)[6]);
    float calcTrapTrajBasic(float curr_pos,
                            float curr_vel,
                            float dt,
                            float goal,
                            float max_vel,
                            float max_accel);

private:
    float getDeltaTimeSec();
    void updateJointStates();
    void updateCartesianPlan(const Matrix4x4 (&transforms)[JOINT_NUM + 1]);
    void updateJointPlan(float dt);
    void applyPlannedJointSpeeds();
    void writePoseToState(Matrix4x4 T_EE);
    void updateFromEncoders();
    bool startHoming();
    void updateHoming();
    void advanceHomingSequence();
    bool isHoming() const;
    void updateEndSwitches();

    uint32_t last_time = 0;

    uint8_t _enable_pin_0;  // Enables joints 0, 1, 2
    uint8_t _enable_pin_1;  // Enables joints 3, 4, 5
    uint8_t _joint_end_switch_min;  // end switch in CCW dir
    uint8_t _joint_end_switch_max;  // end switch in CW dir
    
    Joint _joints[JOINT_NUM];
    RobotState _robotState{};
    RobotConfig _robotConfig{};
    RobotPlanner _robotPlanner{};
    HomingStatus _homingStatus;
};

void sharedWriteRobotState(const RobotState& rs);

#endif  // ROBOT_H