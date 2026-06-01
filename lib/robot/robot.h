#ifndef ROBOT_H
#define ROBOT_H

#include "config.h"
#include "joint.h"
#include "robot_types.h"

#define ANGLE_RAD_SPEED_TOLERANCE 0.0025f
#define ANGLE_RAD_POSITION_TOLERANCE 0.005f
 
struct DHParam {
  float theta;
  float alpha;
  float d;
  float a;
};

const DHParam dh_table[JOINT_NUM] = {
  { 0.0f,     -PI / 2.0f, 102.0f,  0.0f   },  // joint 0
  { -PI/2.0f, 0.0f,       0.0f,    210.0f },  // joint 1
  { 0.0f,     -PI / 2.0f, 0.0f,    0.0f   },  // joint 2
  { 0.0f,     PI / 2.0f,  202.0f,  0.0f   },  // joint 3
  { PI,       PI / 2.0f,  0.0f,    0.0f   },  // joint 4
  { 0.0f,     0.0f,       43.5f,   0.0f   }   // joint 5
};

class Robot
{
  public:
    Robot();

    void init();
    void update();
    void enable();
    void disable();
    RobotState getState();

    void moveJoint(float target_joint_pose[JOINT_NUM]);
    void moveCart(float target_cart_pose[6]);
    void setMaxJointSpeed(float max_speed[JOINT_NUM]);
    void setMaxJointAcceleration(float max_accel[JOINT_NUM]);
    void computeForwardKinematics(const float (&q)[JOINT_NUM], Matrix4x4 (&T)[JOINT_NUM+1]);
    void computeGeometricJacobian(const Matrix4x4 (&T)[JOINT_NUM + 1], Matrix6x6 (&J));
    float calcTrapTrajBasic(float curr_pos, float curr_vel, float dt, float goal, float max_vel, float max_accel);
    float* getMaxJointSpeed();
    float* getMaxJointAcceleration();
    Vect6f computeCartError(const Matrix4x4& curr_pose, const Vect6f& goal);
    void writePoseToState(Matrix4x4 T_EE);

  private:
    uint32_t last_time;

    char _output_buffer[128];

    uint8_t _enable_pin0; // Pin to enable joints 0, 1, 2
    uint8_t _enable_pin1; // Pin to enable joints 3, 4, 5

    Joint _joints[JOINT_NUM];
    RobotState _robotState;
    RobotConfig _robotConfig;
    RobotPlanner _robotPlanner;
    void updateJointStates();
};

#endif // ROBOT_H
