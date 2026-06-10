#include "robot.h"
#include "utils.h"


Robot::Robot()
: _enable_pin0(ENABLE_PIN_0),
  _enable_pin1(ENABLE_PIN_1),
  _encoderManager(nullptr),
  _joints {Joint(STEP_PIN_0, DIR_PIN_0, MICROSTEPS_0, GEAR_RATIO_0, MIN_ANGLE_0, MAX_ANGLE_0, MOTOR_DIR_INVERTED[0]),
           Joint(STEP_PIN_1, DIR_PIN_1, MICROSTEPS_1, GEAR_RATIO_1, MIN_ANGLE_1, MAX_ANGLE_1, MOTOR_DIR_INVERTED[1]),
           Joint(STEP_PIN_2, DIR_PIN_2, MICROSTEPS_2, GEAR_RATIO_2, MIN_ANGLE_2, MAX_ANGLE_2, MOTOR_DIR_INVERTED[2]),
           Joint(STEP_PIN_3, DIR_PIN_3, MICROSTEPS_3, GEAR_RATIO_3, MIN_ANGLE_3, MAX_ANGLE_3, MOTOR_DIR_INVERTED[3]),
           Joint(STEP_PIN_4, DIR_PIN_4, MICROSTEPS_4, GEAR_RATIO_4, MIN_ANGLE_4, MAX_ANGLE_4, MOTOR_DIR_INVERTED[4]),
           Joint(STEP_PIN_5, DIR_PIN_5, MICROSTEPS_5, GEAR_RATIO_5, MIN_ANGLE_5, MAX_ANGLE_5, MOTOR_DIR_INVERTED[5])
           }
{
  
  _robotState.robot_motion_control_paradigm = robot_motion_control_paradigm_t::ROBOT_JOINT_CONTROL;

  for (int i = 0; i < JOINT_NUM; i++) {
    _joints[i].init();
    _joints[i].setMotionControlParadigm(JOINT_SPEED_CONTROL);
    _joints[i].setTargetSpeed(0.0f);

    _robotState.q[i] = 0.0f;
    _robotState.q_dot[i] = 0.0f;
    _robotState.q_ddot[i] = 0.0f;
    _robotState.q_target[i] = 0.0f;
    _robotState.q_dot_target[i] = 0.0f;
  }


  Robot::setMaxJointSpeed((float*)DEFAULT_JOINT_SPEEDS);
  Robot::setMaxJointAcceleration((float*)DEFAULT_JOINT_ACCELS);}

void Robot::init() {
  pinMode(_enable_pin0, OUTPUT);
  pinMode(_enable_pin1, OUTPUT);

  Robot::disable();
  last_time = micros();
}

void Robot::update() {
  // Loop timing
  float dt = (micros() - last_time) / 1000000.0;
  last_time = micros();
  
  Robot::updateJointStates();

  // Calculate EE pose from T transform matrices
  Matrix4x4 T_matrices[JOINT_NUM + 1];
  computeForwardKinematics(_robotState.q, T_matrices);

  // Extract the data and update x and T_EE
  writePoseToState(T_matrices[JOINT_NUM]);

  float q_calc[JOINT_NUM] = {0.0f};

  float err_norm = 0.0f;
  float ori_err = 0.0f;
  float Kp = 0.9f;
  float Kr = 0.3f;
  Vect6f err;
  Vect6f x_dot;

  switch (_robotState.robot_motion_control_paradigm)
  {
  case robot_motion_control_paradigm_t::ROBOT_CART_CONTROL:
    Matrix6x6 Jg;
    computeGeometricJacobian(T_matrices, Jg);
    
    err = computeCartErr(_robotState.T_EE, _robotState.x_target);
    
    // Construct the desired control via error
    for (int i = 0; i < 3; i++) {
      x_dot.v[i]     = Kp * err.v[i];
      x_dot.v[i + 3] = Kr * err.v[i + 3];
    }

    // Calculate normalized? position error
    err_norm = sqrt(err.v[0]*err.v[0] +
                    err.v[1]*err.v[1] +
                    err.v[2]*err.v[2]);

    ori_err = sqrt(err.v[3]*err.v[3] +
                   err.v[4]*err.v[4] +
                   err.v[5]*err.v[5]);
                      
    // will save the calculated q_dot in the struct array
    computeDLSMethod(q_calc, Jg, x_dot);
    
    for (int i=0; i<JOINT_NUM; i++){
      _robotPlanner.q_planned[i] = q_calc[i];
    }

    break;  
  case robot_motion_control_paradigm_t::ROBOT_JOINT_CONTROL:
    for (int i=0; i<JOINT_NUM; i++){
      q_calc[i] = calcTrapTrajBasic(_robotState.q[i],
                                    _robotPlanner.q_planned[i],
                                    dt,
                                    _robotState.q_target[i],
                                    _robotConfig.max_joint_speeds[i],
                                    _robotConfig.max_joint_accelerations[i]);
      
      _robotPlanner.q_planned[i] = q_calc[i];
    }
  break;  
  
  default:
    break;
  }

  // Finally set the target speed
  for (int i=0; i<JOINT_NUM; i++){
    _joints[i].setTargetSpeed(_robotPlanner.q_planned[i]);
    _robotState.q_dot_target[i] = _robotPlanner.q_planned[i];
  }

}

void Robot::attachEncoderManager(EncoderManager *encoderManager){
  _encoderManager = encoderManager;
}

float Robot::calcTrapTrajBasic(float curr_pos,
                               float curr_vel,
                               float dt,
                               float goal,
                               float max_vel,
                               float max_accel)
{
    float error = goal - curr_pos;

    if (fabsf(error) < ANGLE_RAD_POSITION_TOLERANCE) {
        return 0.0f;
    }

    float dir_to_goal = sign(error);
    float d_stop = (curr_vel * curr_vel) / (2.0f * max_accel);
    float vel_toward_goal = curr_vel * dir_to_goal;

    float calc_accel = 0.0f;

    if (vel_toward_goal < 0.0f) {
        calc_accel = dir_to_goal * max_accel;
    } else if (fabsf(error) <= d_stop) {
        calc_accel = -sign(curr_vel) * max_accel;
    } else if (fabsf(curr_vel) < max_vel) {
        calc_accel = dir_to_goal * max_accel;
    } else {
        calc_accel = 0.0f;
    }

    float calc_vel = curr_vel + calc_accel * dt;
    return clampAbsFloat(calc_vel, max_vel);
}

void Robot::writePoseToState(Matrix4x4 T_EE){
  _robotState.T_EE = T_EE;
  _robotState.x[0] = _robotState.T_EE.m[0][3];
  _robotState.x[1] = _robotState.T_EE.m[1][3];
  _robotState.x[2] = _robotState.T_EE.m[2][3];
  Matrix3x3 _temp_rot = getRotationMatrixFromPoseMatrix(_robotState.T_EE);
  Vect3f _temp_eul = rotationMatrixToEulerAngles(_temp_rot);
  _robotState.x[3] = _temp_eul.v[0];
  _robotState.x[4] = _temp_eul.v[1];
  _robotState.x[5] = _temp_eul.v[2];
}

void Robot::enable() {
  digitalWrite(_enable_pin0, LOW);
  digitalWrite(_enable_pin1, LOW);
  _robotState.enabled = true;
}

void Robot::disable(){
  digitalWrite(_enable_pin0, HIGH);
  digitalWrite(_enable_pin1, HIGH);
  _robotState.enabled = false;
}

void Robot::updateJointStates() {
  for (int i=0; i<JOINT_NUM; i++){
    _joints[i].update();

    _robotState.joints[i] = _joints[i].getState();

    _robotState.q[i] = _robotState.joints[i].angle_rad;
    _robotState.q_dot[i] = _robotState.joints[i].angle_vel_rad_s;

    if (_robotState.joints[i].at_min_lim){
      _robotState.robot_error_state = robot_error_state_t::LIMIT_HIT_MIN;
    }
    else if (_robotState.joints[i].at_max_lim){
      _robotState.robot_error_state = robot_error_state_t::LIMIT_HIT_MAX;
    }
  }
  _robotState.moving = false;
  for (int i = 0; i < JOINT_NUM; ++i) {
    if (_robotState.joints[i].moving) {
      _robotState.moving = true;
      break;
    }
  }   
}

bool Robot::updateFromEncoders(){
  if (Robot::_encoderManager == nullptr) {
      return false;
  }

  EncoderFrame _temp = _encoderManager->getLatestFrame();
  /*
  uint32_t timestamp_us = micros();
  // if the timestamp of encoder values is older than 50 ms;
  if ((timestamp_us - _temp.timestamp_us) > 50000){
    return false;
  }
  */
  float encoder_angles[JOINT_NUM] = {0.0f};;
  for (int i=0; i<JOINT_NUM; i++){
    if (_temp.joints[i].valid){
      encoder_angles[i] = _temp.joints[i].angle_rad;
    }
  }
  return false;
}

void Robot::moveJoint(float target_joint_pose[JOINT_NUM]) {
  _robotState.robot_motion_control_paradigm = robot_motion_control_paradigm_t::ROBOT_JOINT_CONTROL;
  for (int i = 0; i<JOINT_NUM; i++){
    _robotState.q_target[i] = target_joint_pose[i];
  }
}

void Robot::moveCart(float target_cart_pose[6]) {
  _robotState.robot_motion_control_paradigm = robot_motion_control_paradigm_t::ROBOT_CART_CONTROL;
  for (int i = 0; i<6; i++){
    _robotState.x_target[i] = target_cart_pose[i];
  }
}

void Robot::computeForwardKinematics(const float (&q)[JOINT_NUM], Matrix4x4 (&T)[JOINT_NUM+1]) {
  T[0] = getIdentityMatrix();

  for (int i=0; i<JOINT_NUM; i++){
    // Get the transform matrix for individual joint
    Matrix4x4 A = getAMatrix(dh_table[i].d, dh_table[i].theta, dh_table[i].a, dh_table[i].alpha, q[i]);
    // Multiply previous one with current one
    T[i+1] = multiplyMatrices(T[i], A);
  }
}

void Robot::computeGeometricJacobian(const Matrix4x4 (&T)[JOINT_NUM + 1], Matrix6x6 (&J)) {
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

void Robot::computeDLSMethod(float (&q_dot)[JOINT_NUM], const Matrix6x6 (&J), Vect6f x_dot){
  float lambda = 0.1f;

  Matrix6x6 J_T = transposeMat(J);
  Matrix6x6 JJ_T = multiplyMatrices(J, J_T);

  for (int i=0; i<6; i++){
    JJ_T.m[i][i] += lambda*lambda;
  }

  Matrix6x6 JJ_T_lambda_inv = invertMatrix(JJ_T);

  float temp[6] = {0.0f};

  for (int i=0; i<6; i++){
    for (int j=0; j<6; j++){
      temp[i] += JJ_T_lambda_inv.m[i][j] * x_dot.v[j];
    }
  }
  for (int i=0; i<6; i++){
    for (int j=0; j<6; j++){
      q_dot[i] += J_T.m[i][j] * temp[j];
    }
  }
}

Vect6f Robot::computeCartErr(const Matrix4x4 T_curr, float (&x_goal)[6]){
  Vect3f err_p; 
  Vect3f err_r;
  
  // Position error
  for (int i=0; i<3; i++){
    err_p.v[i] = x_goal[i] - T_curr.m[i][3];
  }

  // Rotation error
  float goal_angles[3] = {x_goal[3], x_goal[4], x_goal[5]};
  Matrix3x3 R_goal = eulerAnglesToRotationMatrix(goal_angles);
  err_r = computeRotErrMat(R_goal, getRotationMatrixFromPoseMatrix(T_curr));

  Vect6f err;
  err.v[0] = err_p.v[0];
  err.v[1] = err_p.v[1];
  err.v[2] = err_p.v[2];
  err.v[3] = err_r.v[0];
  err.v[4] = err_r.v[1];
  err.v[5] = err_r.v[2];
  return err;
}


// ----------- Getters -----------
RobotState Robot::getState(){
  return _robotState;
}

float* Robot::getMaxJointSpeed() {
  return _robotConfig.max_joint_speeds;
}

float* Robot::getMaxJointAcceleration() {
  return _robotConfig.max_joint_accelerations;
}

// ----------- Setters -----------
void Robot::setJointAngles(float q[JOINT_NUM]){
  for (int i = 0; i<JOINT_NUM; i++){
    _joints[i].setCurrentAngle(q[i]);
    _robotState.q[i] = q[i];
  }
  Robot::update();
}

void Robot::setMaxJointSpeed(float max_speed[JOINT_NUM]) {
  for (int i = 0; i<JOINT_NUM; i++){
    _robotConfig.max_joint_speeds[i] = max_speed[i];
    _joints[i].setMaxSpeed(max_speed[i]);
  }
}

void Robot::setMaxJointAcceleration(float max_accel[JOINT_NUM]) {
  for (int i = 0; i<JOINT_NUM; i++){
    _robotConfig.max_joint_accelerations[i] = max_accel[i];
    _joints[i].setMaxAcceleration(max_accel[i]);
  }
}

