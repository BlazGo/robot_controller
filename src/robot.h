#ifndef ROBOT_H
#define ROBOT_H

#include <Arduino.h>
#include "joint.h"
#include "config.h"
#include "utils.h"

// ─────────────────────────────────────────────────────────────
// Denavit Hatenberg - Parameters
// ─────────────────────────────────────────────────────────────
struct DHParam {
  float theta;
  float alpha;
  float d;
  float a;
};

extern const DHParam dh_table[JOINT_NUM];

// all angular variables should be defined in radians
// all cartesian longitudinal variables should be defined in meters
class Robot
{
  public:
    Robot();

    void init(void);
    void update(void);
    void enableJoints(void);
    void disableJoints(void);
    bool setMotionControlParadigm(motion_control_paradigm_t motion_control_paradigm);
    const bool isMoving(void) const;
    void printInfo();

    void jointMove(float q[JOINT_NUM]);
    void cartMove(float x[6]);
    void IKMove(float q[JOINT_NUM]);

    const float* getJointAngles(void) const;
    const float* getMaxJointSpeed(void) const;
    const float* getMaxJointAcceleration(void) const;
    Vect6f getCartPose(void);
    void currCartPoseFromT(void);

    void setMaxJointSpeed(float max_joint_speeds[JOINT_NUM]);
    void setMaxJointAcceleration(float max_joint_accelerations[JOINT_NUM]);
    void computeForwardKinematics(const float (&q)[JOINT_NUM], Matrix4x4 (&T)[JOINT_NUM + 1]) const;
    void computeGeometricJacobian(const  Matrix4x4 (&T)[JOINT_NUM + 1], Matrix6x6 (&J)) const;
    void computeTransposeMethod(float (&q_dot)[JOINT_NUM], const Matrix6x6 (&J), Vect6f x_dot);
    void computeDLSMethod(float (&q_dot)[JOINT_NUM], const Matrix6x6 (&J), Vect6f x_dot);
    Vect3f computeRotErrMat(Matrix3x3 Rot_d, Matrix3x3 Rot_curr);

  private:
    bool _enabled;
    bool _moving;
    uint8_t _enable_pin0;
    uint8_t _enable_pin1;
    motion_control_paradigm_t _motion_control_paradigm;

    Matrix4x4 _curr_cart_pose_Mat;
    Matrix3x3 _curr_rot_Mat;
    Matrix6x6 _curr_jacobian;
    Vect3f _curr_eul_angles_Vect;

    float _q_dot[6];
    Vect6f _current_pose;
    Vect6f _goal_pose;
    Vect6f _task_velocity;
    Vect6f _task_err;

    float _curr_joint_angles[JOINT_NUM];
    float _max_joint_angles[JOINT_NUM];
    float _min_joint_angles[JOINT_NUM];

    float _curr_joint_speeds[JOINT_NUM];
    float _max_joint_speeds[JOINT_NUM];

    float _curr_joint_accels[JOINT_NUM];
    float _max_joint_accels[JOINT_NUM];

    Joint _joints[JOINT_NUM];

    void updateJointStates();
    Vect6f computeTaskError(const Vect6f& current, const Vect6f& goal);
    void computeKinematics(const  Matrix4x4 (&T)[JOINT_NUM + 1]) const;
    void testJacobian();

};

#endif // ROBOT_H
