#include "robot.h"
#include "utils.h"

const DHParam dh_table[JOINT_NUM] = {
  { 0.0f,     -PI / 2.0f, 102.0f,  0.0f   },  // joint 0
  { -PI/2.0f, 0.0f,       0.0f,    210.0f },  // joint 1
  { 0.0f,     -PI / 2.0f, 0.0f,    0.0f   },  // joint 2
  { 0.0f,     PI / 2.0f,  202.0f,  0.0f   },  // joint 3
  { PI,       PI / 2.0f,  0.0f,    0.0f   },  // joint 4
  { 0.0f,     0.0f,       43.5f,   0.0f   }   // joint 5
};

#define MAX_DEFAULT_SPEED 1.0f
#define MAX_DEFAULT_ACCELERATION 5.0f

Robot::Robot()
  : _moving(false),
    _enabled(false),
    _enable_pin0(ENABLE_PIN_0),
    _enable_pin1(ENABLE_PIN_1),
  _joints {Joint(STEP_PIN_0, DIR_PIN_0, MICROSTEPS_0, GEAR_RATIO_0, MIN_ANGLE_0, MAX_ANGLE_0),
           Joint(STEP_PIN_1, DIR_PIN_1, MICROSTEPS_1, GEAR_RATIO_1, MIN_ANGLE_1, MAX_ANGLE_1),
           Joint(STEP_PIN_2, DIR_PIN_2, MICROSTEPS_2, GEAR_RATIO_2, MIN_ANGLE_2, MAX_ANGLE_2),
           Joint(STEP_PIN_3, DIR_PIN_3, MICROSTEPS_3, GEAR_RATIO_3, MIN_ANGLE_3, MAX_ANGLE_3),
           Joint(STEP_PIN_4, DIR_PIN_4, MICROSTEPS_4, GEAR_RATIO_4, MIN_ANGLE_4, MAX_ANGLE_4),
           Joint(STEP_PIN_5, DIR_PIN_5, MICROSTEPS_5, GEAR_RATIO_5, MIN_ANGLE_5, MAX_ANGLE_5)
           }
{
  for (int i = 0; i < JOINT_NUM; i++) {
    _curr_joint_angles[i] = 0.0f;
    _max_joint_angles[i] = ((PI/2) - 0.1f);
    _min_joint_angles[i] = -((PI/2) - 0.1f);

    _curr_joint_speeds[i] = 0.0f;
    _max_joint_speeds[i] = MAX_DEFAULT_SPEED;

    _curr_joint_accels[i] = 0.0f;
    _max_joint_accels[i] = MAX_DEFAULT_ACCELERATION;
  }
}

// ─────────────────────────────────────────────────────────────
// Public functions
// ─────────────────────────────────────────────────────────────
void Robot::init(){
  for (int i=0; i<JOINT_NUM; i++){
    _joints[i].init();
    _joints[i].setMaxAcceleration(DEFAULT_JOINT_ACCELS[i]);
    _joints[i].setMaxSpeed(DEFAULT_JOINT_SPEEDS[i]);
    _curr_joint_angles[i] = _joints[i].getAngle();
  }
  pinMode(_enable_pin0, OUTPUT);
  pinMode(_enable_pin1, OUTPUT);
  disableJoints();  
  
  Matrix4x4 T_matrices[JOINT_NUM + 1];
  computeForwardKinematics(_curr_joint_angles, T_matrices);
  _curr_cart_pose_Mat = T_matrices[JOINT_NUM];
  currCartPoseFromT();

  for (int i=0; i<6; i++){
    _goal_pose.v[i] = _current_pose.v[i];
  }

  setMotionControlParadigm(SPEED_CONTROL);
}

void Robot::update(){
  // Executes motor moves, updates positions and checks if robot is moving
  updateJointStates();

  // Calculate EE pose from T transform matrices
  Matrix4x4 T_matrices[JOINT_NUM + 1];
  computeForwardKinematics(_curr_joint_angles, T_matrices);
  // Extract the last pose as current EE pose
  _curr_cart_pose_Mat = T_matrices[JOINT_NUM];
  // Calculates rot matrix and constructs cart and euler angles vector
  currCartPoseFromT();
  
  // Compute Jacobian matrix
  Matrix6x6 Jg;
  computeGeometricJacobian(T_matrices, Jg);
  _curr_jacobian = Jg;

  // Computes pose error vect (rotation via rotation matrix)
  Vect6f err = computeTaskError(_current_pose, _goal_pose);

  // Calculate position error
  float pos_err_norm = sqrt(err.v[0]*err.v[0] + err.v[1]*err.v[1] + err.v[2]*err.v[2]);
  
  // Construct the desired control via error
  float Kp = 1.5f;
  float Kr = 0.3f;
  float Kr_scaled = Kr * (1.0f / (1.0f + pos_err_norm));

  Vect6f x_dot; 
  for (int i = 0; i < 3; i++) {
    x_dot.v[i]     = Kp * err.v[i];
    x_dot.v[i + 3] = Kr_scaled * err.v[i + 3];
  }

  // Get the actual q_dot
  //computeTransposeMethod(_q_dot, Jg, x_dot);
  computeDLSMethod(_q_dot, Jg, x_dot);

  for (int i=0; i<JOINT_NUM; i++){
    if (fabs(_q_dot[i]) > _max_joint_speeds[i]){
      _q_dot[i] = (_q_dot[i] > 0 ? 1 : -1) * _max_joint_speeds[i];
      //_q_dot[i] = 0.8f * _q_dot_prev[i] + 0.2f * _q_dot[i];
    }
  }

  // Not really a good solution as orientation can still be wrong especially if we only move joint 5
  if (pos_err_norm < 1.0f) {   // adjust threshold
    for (int i = 0; i < JOINT_NUM; i++) {
        _joints[i].setTargetSpeed(0);
    }
    return;
  }
  else{
    for (int i=0; i<JOINT_NUM; i++){
      _joints[i].setTargetSpeed(_q_dot[i]);
    }
  }
}

void Robot::updateJointStates() {
  _moving = false;

  for (int i = 0; i < JOINT_NUM; i++) {
    _joints[i].update();
    _curr_joint_angles[i] = _joints[i].getAngle();

    if (_joints[i].isMoving()) {
      _moving = true;
    }
  }
}

void Robot::testJacobian(){

  float q[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};   // current joint angles
  float q_dot[6] = {0.0f, 0.01f, 0.01f, 0.0f, -0.01f, 0.0f}; // small random velocities
  Matrix4x4 T[JOINT_NUM+1];
  computeForwardKinematics(q, T);   // same logic as your FK loop

  Matrix6x6 J;
  computeGeometricJacobian(T, J);
  
  float x_dot_analytical[6] = {0};

  for (int i=0; i<6; i++){
    for (int j=0; j<6; j++){
        x_dot_analytical[i] += J.m[i][j] * q_dot[j];
    }
  }
  float eps = 1e-4f;
  
  float q2[6];
  for (int i=0; i<6; i++){
    q2[i] = q[i] + eps * q_dot[i];
  }

  Matrix4x4 T1_temp[JOINT_NUM + 1];
  Matrix4x4 T2_temp[JOINT_NUM + 1];
  computeForwardKinematics(q, T1_temp);
  computeForwardKinematics(q2, T2_temp);
  
  Matrix4x4 T1 = T1_temp[JOINT_NUM];
  Matrix4x4 T2 = T2_temp[JOINT_NUM];

  float x_dot_num[6];

  x_dot_num[0] = (T2.m[0][3] - T1.m[0][3]) / eps;
  x_dot_num[1] = (T2.m[1][3] - T1.m[1][3]) / eps;
  x_dot_num[2] = (T2.m[2][3] - T1.m[2][3]) / eps;
  
  Matrix3x3 R1 = getRotationMatrixFromPoseMatrix(T1);
  Matrix3x3 R2 = getRotationMatrixFromPoseMatrix(T2);

  Vect3f rot_err = computeRotErrMat(R2, R1);

  x_dot_num[3] = rot_err.v[0] / eps;
  x_dot_num[4] = rot_err.v[1] / eps;
  x_dot_num[5] = rot_err.v[2] / eps;

  Serial.println("------------ Jacobian test ------------");
  printArr6x1(x_dot_analytical);
  printArr6x1(x_dot_num);
  Serial.println("---------- Jacobian test end ----------");
}

Vect6f Robot::computeTaskError(const Vect6f& current, const Vect6f& goal){
  Vect3f err_p;
  Vect3f err_r;
  
  // Position error
  for (int i=0; i<3; i++){
    err_p.v[i] = goal.v[i] - current.v[i];
  }

  // Rotation error
  float goal_angles[3] = {goal.v[3], goal.v[4], goal.v[5]};
  Matrix3x3 R_goal = eulerAnglesToRotationMatrix(goal_angles);
  err_r = computeRotErrMat(R_goal, _curr_rot_Mat);

  Vect6f err;
  err.v[0] = err_p.v[0];
  err.v[1] = err_p.v[1];
  err.v[2] = err_p.v[2];
  err.v[3] = err_r.v[0];
  err.v[4] = err_r.v[1];
  err.v[5] = err_r.v[2];
  return err;
}

void Robot::printInfo(){
  Serial.println("--------------- Robot info -------------------------");
  Serial.println("Curre. cart pose: ");
  printVect6x1(getCartPose());
  Serial.println("Target cart pose: ");
  printVect6x1(_goal_pose);
  Serial.println("Calculated q_dot: ");
  printArr6x1(_q_dot);
  Serial.println("----------------------------------------------------");

  // testJacobian();
}

void Robot::enableJoints(){
  digitalWrite(_enable_pin0, LOW);
  digitalWrite(_enable_pin1, LOW);
  _enabled = true;
}

void Robot::disableJoints(){
  digitalWrite(_enable_pin0, HIGH);
  digitalWrite(_enable_pin1, HIGH);
  _enabled = false;
}

bool Robot::setMotionControlParadigm(motion_control_paradigm_t motion_control_paradigm){
  // if the paradigm is the same skip
  if (motion_control_paradigm == _motion_control_paradigm){
    return true;
  }

  // first check if the robot is stopped
  for (int i=0; i<JOINT_NUM; i++){
    if (_joints[i].isMoving()){
      return false;
    }
  }

  // if it is stopped we can change joint controls
  for (int i=0; i<JOINT_NUM; i++){
    if (!_joints[i].setMotionControlParadigm(motion_control_paradigm)){
      return false;
    }
  }
  _motion_control_paradigm = motion_control_paradigm;
  return true;
}

void Robot::jointMove(float q[JOINT_NUM]){
  for (int i=0; i<JOINT_NUM; i++){
    _joints[i].moveToAngle(q[i]);
  }
}

void Robot::cartMove(float x[6]){
  for (int i = 0; i<6; i++){
    _goal_pose.v[i] = x[i]; 
  }
}

const float* Robot::getMaxJointSpeed(void) const {
  return _max_joint_speeds;
}

const float* Robot::getMaxJointAcceleration(void) const {
  return _max_joint_accels;
}

void Robot::setMaxJointSpeed(float max_joint_speeds[JOINT_NUM]){
  for (int i=0; i<JOINT_NUM; i++){
    _max_joint_speeds[i] = max_joint_speeds[i]; 
    _joints[i].setMaxSpeed(max_joint_speeds[i]);
  }
}

void Robot::setMaxJointAcceleration(float max_joint_accelerations[JOINT_NUM]){
  for (int i=0; i<JOINT_NUM; i++){
    _max_joint_accels[i] = max_joint_accelerations[i];
    _joints[i].setMaxAcceleration(max_joint_accelerations[i]);
  }
}

const float* Robot::getJointAngles(void) const {
  return _curr_joint_angles;
}

Vect6f Robot::getCartPose(void) {
  return _current_pose;
}

void Robot::currCartPoseFromT(void){
  _curr_rot_Mat = getRotationMatrixFromPoseMatrix(_curr_cart_pose_Mat);
  _curr_eul_angles_Vect = rotationMatrixToEulerAngles(_curr_rot_Mat);

  _current_pose.v[0] = _curr_cart_pose_Mat.m[0][3];
  _current_pose.v[1] = _curr_cart_pose_Mat.m[1][3];
  _current_pose.v[2] = _curr_cart_pose_Mat.m[2][3];
  _current_pose.v[3] = _curr_eul_angles_Vect.v[0];
  _current_pose.v[4] = _curr_eul_angles_Vect.v[1];
  _current_pose.v[5] = _curr_eul_angles_Vect.v[2];
}

void Robot::computeForwardKinematics(const float (&q)[JOINT_NUM], Matrix4x4 (&T)[JOINT_NUM+1]) const {
  // ─────────────────────────────────────────────────────────────
  // Forward kinematics
  // ─────────────────────────────────────────────────────────────
  
  T[0] = getIdentityMatrix();

  for (int i=0; i<JOINT_NUM; i++){
    // Get the transform matrix for individual joint
    Matrix4x4 A = getAMatrix(dh_table[i].d, dh_table[i].theta, dh_table[i].a, dh_table[i].alpha, q[i]);
    // Multiply previous one with current one
    T[i+1] = multiplyMatrices(T[i], A);
  }
}

void Robot::computeGeometricJacobian(const Matrix4x4 (&T)[JOINT_NUM + 1], Matrix6x6 (&J)) const {
  // ─────────────────────────────────────────────────────────────
  // Geometric Jacobian compute
  // ─────────────────────────────────────────────────────────────
  
  // Position vector
  Vect3f o[JOINT_NUM+1];
  // Z-axis rientation vector 
  Vect3f z[JOINT_NUM+1];

  for (int i=0; i<=JOINT_NUM; i++){
    o[i].v[0] = T[i].m[0][3];
    o[i].v[1] = T[i].m[1][3];
    o[i].v[2] = T[i].m[2][3];
    
    z[i].v[0] = T[i].m[0][2];
    z[i].v[1] = T[i].m[1][2];
    z[i].v[2] = T[i].m[2][2];
  }

  Vect3f o_n = o[JOINT_NUM];

  Vect3f Jv[JOINT_NUM];
  Vect3f Jw[JOINT_NUM];
  for (int i=0; i<JOINT_NUM; i++){
    Vect3f r;
    r.v[0] = o_n.v[0] - o[i].v[0];
    r.v[1] = o_n.v[1] - o[i].v[1];
    r.v[2] = o_n.v[2] - o[i].v[2];

    Jv[i] = cross(z[i], r); 
    Jw[i] = z[i];
  }

  for (int i=0; i<JOINT_NUM; i++){
    J.m[0][i] = Jv[i].v[0];
    J.m[1][i] = Jv[i].v[1];
    J.m[2][i] = Jv[i].v[2];

    J.m[3][i] = Jw[i].v[0];
    J.m[4][i] = Jw[i].v[1];
    J.m[5][i] = Jw[i].v[2];
  }
}

void Robot::computeTransposeMethod(float (&q_dot)[JOINT_NUM], const Matrix6x6 (&J), Vect6f x_dot){
  Matrix6x6 J_T = transposeMat(J);
  for (int i=0; i<JOINT_NUM; i++){
   q_dot[i] = J_T.m[i][0] * x_dot.v[0] + J_T.m[i][1] * x_dot.v[1] + J_T.m[i][2] * x_dot.v[2] + J_T.m[i][3] * x_dot.v[3] + J_T.m[i][4] * x_dot.v[4] + J_T.m[i][5] * x_dot.v[5];
  }
}

void Robot::computeDLSMethod(float (&q_dot)[JOINT_NUM], const Matrix6x6 (&J), Vect6f x_dot){
  float lambda = 0.1f;

  Matrix6x6 J_T = transposeMat(J);
  Matrix6x6 JJ_T = multiplyMatrices(J, J_T);

  for (int i=0; i<6; i++){
    JJ_T.m[i][i] += lambda * lambda;
  }

  Matrix6x6 JJ_T_inv = invertMatrix(JJ_T);

  float temp[6] = {0};

  for (int i=0; i<6; i++){
    for (int j=0; j<6; j++){
      temp[i] += JJ_T_inv.m[i][j] * x_dot.v[j];
    }
  }

  for (int i=0; i<6; i++){
    q_dot[i] = 0;
    for (int j=0; j<6; j++){
      // should be transposed but we can just switch i and j indexes
      q_dot[i] += J.m[j][i] * temp[j];
    }
  }
}

Vect3f Robot::computeRotErrMat(Matrix3x3 Rot_d, Matrix3x3 Rot_curr){
  Matrix3x3 Rot_T = transposeMat(Rot_curr);
  Matrix3x3 Rot_res = multiplyMatrices(Rot_T, Rot_d);
  Vect3f err_res;
  err_res.v[0] = 0.5f * (Rot_res.m[2][1] - Rot_res.m[1][2]);
  err_res.v[1] = 0.5f * (Rot_res.m[0][2] - Rot_res.m[2][0]);
  err_res.v[2] = 0.5f * (Rot_res.m[1][0] - Rot_res.m[0][1]);
  return err_res;
}

const bool Robot::isMoving() const {
  return _moving;
}

// ─────────────────────────────────────────────────────────────
// Private functions
// ─────────────────────────────────────────────────────────────
