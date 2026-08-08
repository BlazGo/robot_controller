#include "robot.h"
#include "utils.h"

namespace {
constexpr float kPosGain = 0.9f;
constexpr float kRotGain = 0.3f;
constexpr float kDamping = 0.1f;
}

Robot::Robot()
  : _enable_pin_0(ENABLE_PIN_0),
    _enable_pin_1(ENABLE_PIN_1),
    _joint_end_switch_min(END_SWITCH_0_MIN),
    _joint_end_switch_max(END_SWITCH_0_MAX),
    
    _joints{Joint(STEP_PIN_0, DIR_PIN_0, MICROSTEPS_0, GEAR_RATIO_0, MIN_ANGLE_0, MAX_ANGLE_0, MOTOR_DIR_INVERTED[0]),
            Joint(STEP_PIN_1, DIR_PIN_1, MICROSTEPS_1, GEAR_RATIO_1, MIN_ANGLE_1, MAX_ANGLE_1, MOTOR_DIR_INVERTED[1]),
            Joint(STEP_PIN_2, DIR_PIN_2, MICROSTEPS_2, GEAR_RATIO_2, MIN_ANGLE_2, MAX_ANGLE_2, MOTOR_DIR_INVERTED[2]),
            Joint(STEP_PIN_3, DIR_PIN_3, MICROSTEPS_3, GEAR_RATIO_3, MIN_ANGLE_3, MAX_ANGLE_3, MOTOR_DIR_INVERTED[3]),
            Joint(STEP_PIN_4, DIR_PIN_4, MICROSTEPS_4, GEAR_RATIO_4, MIN_ANGLE_4, MAX_ANGLE_4, MOTOR_DIR_INVERTED[4]),
            Joint(STEP_PIN_5, DIR_PIN_5, MICROSTEPS_5, GEAR_RATIO_5, MIN_ANGLE_5, MAX_ANGLE_5, MOTOR_DIR_INVERTED[5])}
{
  _robotState.robot_motion_control_paradigm = robot_motion_control_paradigm_t::ROBOT_JOINT_CONTROL;
  _robotState.all_homed = false;
  
  for (int i = 0; i < JOINT_NUM; ++i) {
    _joints[i].init();
    _joints[i].setMotionControlParadigm(JOINT_SPEED_CONTROL);
    _joints[i].setTargetSpeed(0.0f);

    _robotState.q[i] = 0.0f;
    _robotState.q_dot[i] = 0.0f;
    _robotState.q_ddot[i] = 0.0f;
    _robotState.q_target[i] = 0.0f;
    _robotState.q_dot_target[i] = 0.0f;
    _robotPlanner.q_planned[i] = 0.0f;

    _robotState.q_encoders[i] = 0.0f;
    _robotState.limits_min[i] = false;
    _robotState.limits_max[i] = false;
  }

  setMaxJointSpeed(DEFAULT_JOINT_SPEEDS);
  setMaxJointAcceleration(DEFAULT_JOINT_ACCELS);
}

// ----------- Core functionality -----------
void Robot::init() {
  pinMode(_enable_pin_0, OUTPUT);
  pinMode(_enable_pin_1, OUTPUT);
  
  pinMode(_joint_end_switch_min, INPUT_PULLDOWN);
  pinMode(_joint_end_switch_max, INPUT_PULLDOWN);

  disable();
  last_time = micros();
}

void Robot::update() {
  const float dt = getDeltaTimeSec();
  
  updateJointStates();
  updateEndSwitches();

  if (_homingStatus.active) {
    updateHoming();
  }

  Matrix4x4 transforms[JOINT_NUM + 1];
  computeForwardKinematics(_robotState.q, transforms);
  writePoseToState(transforms[JOINT_NUM]);

  switch (_robotState.robot_motion_control_paradigm) {
    case robot_motion_control_paradigm_t::ROBOT_CART_CONTROL:
      updateCartesianPlan(transforms);
      break;

    case robot_motion_control_paradigm_t::ROBOT_JOINT_CONTROL:
      updateJointPlan(dt);
      break;

    default:
      break;
  }
  applyPlannedJointSpeeds();
}

void Robot::attachEncoderNode(NodeProtocol& encoder_node) {
    _encoder_node = &encoder_node;
}

JointAngles Robot::getLatestEncoderAngles() {
  return _encoder_node ? _encoder_node -> getAngles() : JointAngles();
}


void Robot::enable() {
  digitalWrite(_enable_pin_0, LOW);
  digitalWrite(_enable_pin_1, LOW);
  _robotState.motors_enabled = true;
}

void Robot::disable(){
  digitalWrite(_enable_pin_0, HIGH);
  digitalWrite(_enable_pin_1, HIGH);
  _robotState.motors_enabled = false;
}

void Robot::updateJointStates() {
  _robotState.robot_error_state = robot_error_state_t::NO_ERROR;

  for (int i = 0; i < JOINT_NUM; ++i) {
    _joints[i].update();
    _robotState.joints[i] = _joints[i].getState();

    _robotState.q[i] = _robotState.joints[i].angle_rad;
    _robotState.q_dot[i] = _robotState.joints[i].angle_vel_rad_s;

    if (_robotState.joints[i].at_min_lim) {
      _robotState.robot_error_state = robot_error_state_t::LIMIT_HIT_MIN;
    } else if (_robotState.joints[i].at_max_lim) {
      _robotState.robot_error_state = robot_error_state_t::LIMIT_HIT_MAX;
    }
  }

  _robotState.exec_state = computeExecState();
}

void Robot::updateEndSwitches() {
  _robotState.limits_min[0] = digitalRead(_joint_end_switch_min);
  _robotState.limits_max[0] = digitalRead(_joint_end_switch_max);
}

bool Robot::isMoving() const {
  for (int i = 0; i < JOINT_NUM; ++i) {
    if (_robotState.joints[i].moving) {
      return true;
    }
  }
  return false;
}

bool Robot::isBusy() const {
  switch (_robotState.exec_state) {
    case ROBOT_IDLE:
    case ROBOT_DONE:
      return false;

    case ROBOT_EXECUTING:
    case ROBOT_STOPPING:
    case ROBOT_ERROR:
    default:
      return true;
  }
}

bool Robot::acceptCommand(const RobotCommand& cmd) {
  // accept command only if not in error state
  if (_robotState.robot_error_state != robot_error_state_t::NO_ERROR) {
    return false;
  }
  
  switch (cmd.type){

    case RobotCommandType::SET_MAX_JOINT_SPEEDS:
      setMaxJointSpeed(cmd.q);
      _robotState.command_completed = true;
      return true;

    case RobotCommandType::SET_MAX_JOINT_ACCELERATIONS:
      setMaxJointAcceleration(cmd.q);
      _robotState.command_completed = true;
      return true;

    case RobotCommandType::CONTROL_PARADIGM_CHANGE:
      setMotionControlParadigm(static_cast<robot_motion_control_paradigm_t>(cmd.id_int));
      _robotState.command_completed = true;
      return true;

    case RobotCommandType::JOINT_MOVE:
      if (_robotState.command_active || isMoving()) {
        return false;
      }
      _robotState.command_completed = false;
      _robotState.stop_requested = false;
      moveJoint(cmd.q);
      return true;

    case RobotCommandType::CART_MOVE:
      if (_robotState.command_active || isMoving()) {
        return false;
      }
      _robotState.command_completed = false;
      _robotState.stop_requested = false;
      moveCart(cmd.x);
      return true;

    case RobotCommandType::UPDATE_FROM_ENCODERS:
    {
      if (_robotState.command_active || isMoving()) {
        return false;
      }
      float temp_q_rad[JOINT_NUM];

      for (uint8_t i=0; i<JOINT_NUM; i++){
        if (i == 0){
          temp_q_rad[i] = _robotState.q[i];
          continue;
        }
        temp_q_rad[i] = cmd.q[i];
      }

      setJointAngles(temp_q_rad);
      _robotState.command_completed = true;
      return true;
    }

    case RobotCommandType::START_HOMING:
      if (_robotState.command_active || isMoving()) {
        return false;
      }

      startHoming();

      return true;

    default:
      return false;
  }
    
}

RobotExecState Robot::computeExecState() const {
  if (_robotState.robot_error_state != robot_error_state_t::NO_ERROR) {
    return ROBOT_ERROR;
  }

  if (_robotState.stop_requested) {
    return ROBOT_STOPPING;
  }

  if (isMoving()) {
    return ROBOT_EXECUTING;
  }

  if (_robotState.command_completed) {
    return ROBOT_DONE;
  }
  return ROBOT_IDLE;
}
// ----------- Core functionality END -----------

// ----------- Move commands -----------
bool Robot::moveJoint(const float target_joint_pose[JOINT_NUM]) {
  _robotState.robot_motion_control_paradigm = robot_motion_control_paradigm_t::ROBOT_JOINT_CONTROL;
  for (int i = 0; i<JOINT_NUM; i++){
    _robotState.q_target[i] = target_joint_pose[i];
  }
  return true;
}

void Robot::moveCart(const float target_cart_pose[6]) {
  _robotState.robot_motion_control_paradigm = robot_motion_control_paradigm_t::ROBOT_CART_CONTROL;
  for (int i = 0; i<6; i++){
    _robotState.x_target[i] = target_cart_pose[i];
  }
}

bool Robot::goToZero() {
  if (!_robotState.all_homed) return false;
  return moveJoint(ZERO_POSE_RAD);
}

bool Robot::goToReady() {
  if (!_robotState.all_homed) return false;
  return moveJoint(READY_POSE_RAD);
}

float Robot::calcTrapTrajBasic(float curr_pos, float curr_vel, float dt, float goal, float max_vel, float max_accel) {
    const float error = goal - curr_pos;

    if (fabsf(error) < kAngleRadPositionTolerance) {
        return 0.0f;
    }

    const float dir_to_goal = sign(error);
    const float d_stop = (curr_vel * curr_vel) / (2.0f * max_accel);
    const float vel_toward_goal = curr_vel * dir_to_goal;

    float accel = 0.0f;

    // Selecting in which part we are
    if (vel_toward_goal < 0.0f) {
        accel = dir_to_goal * max_accel;
    } else if (fabsf(error) <= d_stop) {
        accel = -dir_to_goal * max_accel;
    } else if (fabsf(curr_vel) < max_vel) {
        accel = dir_to_goal * max_accel;
    }

    // update current speed with acceleration and timestep
    const float new_vel = curr_vel + accel * dt;
    return clampAbsFloat(new_vel, max_vel);
}

void Robot::updateCartesianPlan(const Matrix4x4 (&transforms)[JOINT_NUM + 1]) {
    Matrix6x6 jacobian;
    computeGeometricJacobian(transforms, jacobian);

    const Vect6f error = computeCartErr(_robotState.T_EE, _robotState.x_target);

    Vect6f x_dot{};
    for (int i = 0; i < 3; ++i) {
        x_dot.v[i] = kPosGain * error.v[i];
        x_dot.v[i + 3] = kRotGain * error.v[i + 3];
    }

    float q_dot[JOINT_NUM] = {0.0f};
    computeDLSMethod(q_dot, jacobian, x_dot);

    for (int i = 0; i < JOINT_NUM; ++i) {
        _robotPlanner.q_planned[i] = q_dot[i];
    }
}

void Robot::updateJointPlan(float dt) {
  // Here we can implement synchronised joint move, different acceleration curves, etc.
  for (int i = 0; i < JOINT_NUM; ++i) {
    _robotPlanner.q_planned[i] = calcTrapTrajBasic(
      _robotState.q[i],
      _robotPlanner.q_planned[i],
      dt,
      _robotState.q_target[i],
      _robotConfig.max_joint_speeds[i],
      _robotConfig.max_joint_accelerations[i]);
  }
}

void Robot::applyPlannedJointSpeeds() {
    for (int i = 0; i < JOINT_NUM; ++i) {
        _joints[i].setTargetSpeed(_robotPlanner.q_planned[i]);
        _robotState.q_dot_target[i] = _robotPlanner.q_planned[i];
    }
}
// ----------- Move commands END -----------

// ----------- Kinematics -----------
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
    q_dot[i] = 0.0f;
    for (int j=0; j<6; j++){
      q_dot[i] += J_T.m[i][j] * temp[j];
    }
  }
}

Vect6f Robot::computeCartErr(const Matrix4x4 T_curr, float (&x_goal)[6]) {
    Vect6f err{};

    for (int i = 0; i < 3; ++i) {
        err.v[i] = x_goal[i] - T_curr.m[i][3];
    }

    const float goal_angles[3] = {x_goal[3], x_goal[4], x_goal[5]};
    const Matrix3x3 R_goal = eulerAnglesToRotationMatrix(goal_angles);
    const Vect3f err_r = computeRotErrMat(R_goal, getRotationMatrixFromPoseMatrix(T_curr));

    err.v[3] = err_r.v[0];
    err.v[4] = err_r.v[1];
    err.v[5] = err_r.v[2];

    return err;
}
// ----------- Kinematics END -----------

// ----------- Homing functionality -----------
bool Robot::startHoming() {
  // If we cant home return
  if (isBusy() || _homingStatus.active) return false;
  // Otherwise proceed
  _homingStatus = HomingStatus{};
  _homingStatus.active = true;
  // Select the first joint to home
  const uint8_t first_joint = HOMING_JOINT_ORDER[0];
  // Decide if it uses encoder or endswitch (only joint 0 uses endswitches)
  _homingStatus.phase = (first_joint == 0) ? HomingPhase::SEEK_SWITCH_DIR_MIN : HomingPhase::READ_ENCODER;
  _homingStatus.phase_start_q_rad = _robotState.q[first_joint];
  // Have to set joint paradigm not cartesian
  _robotState.robot_motion_control_paradigm = robot_motion_control_paradigm_t::ROBOT_JOINT_CONTROL;
  return true;
}

bool Robot::isHoming() const {
  return _homingStatus.active;
}

void Robot::advanceHomingSequence() {
  // Increase the value immediately by 1
  _homingStatus.order_idx = _homingStatus.order_idx + 1;

  if (_homingStatus.order_idx >= JOINT_NUM) {
    _homingStatus.phase = HomingPhase::DONE;
    _homingStatus.active = false;
    return;
  }
  const uint8_t next_joint = HOMING_JOINT_ORDER[_homingStatus.order_idx];
  // if joint 0 is next we follow the end switch routes else encoder route
  _homingStatus.phase = (next_joint == 0) ? HomingPhase::SEEK_SWITCH_DIR_MIN : HomingPhase::READ_ENCODER;
  _homingStatus.phase_start_q_rad = _robotState.q[next_joint];
}

void Robot::updateHoming(){
  float angle = 0.0f;
  // Check if we're out of bounds (we're done)
  if (_homingStatus.order_idx >= JOINT_NUM) {
    _homingStatus.phase = HomingPhase::DONE;
  }

  // Select the current joint
  const uint8_t joint_idx = HOMING_JOINT_ORDER[_homingStatus.order_idx];
  Joint& joint = _joints[joint_idx];

  switch (_homingStatus.phase) {
    // Negative direction of rotation
    case HomingPhase::SEEK_SWITCH_DIR_MIN: {
      _robotState.q_target[joint_idx] = JOINT_0_HOMING_SEEK_TARGET_MIN_RAD; // e.g. beyond MIN_ANGLE_0

      const float traveled_rad = fabsf(_robotState.q[joint_idx] - _homingStatus.phase_start_q_rad);
      
      if (joint.getState().at_min_lim) {
        _robotState.q_target[joint_idx] = _robotState.q[joint_idx]; // freeze in place
        _homingStatus.phase = HomingPhase::BACKOFF;
        _homingStatus.phase_start_time_us = micros();
        break;
      }

      if (traveled_rad > JOINT_0_FULL_RANGE_RAD + HOMING_SEARCH_MARGIN_RAD) {
        _robotState.q_target[joint_idx] = _robotState.q[joint_idx]; // freeze in place
        _homingStatus.phase = HomingPhase::SEEK_SWITCH_DIR_MAX;
        _homingStatus.phase_start_q_rad = _robotState.q[joint_idx];
      }
    break;
    }

    case HomingPhase::SEEK_SWITCH_DIR_MAX: {
      _robotState.q_target[joint_idx] = JOINT_0_HOMING_SEEK_TARGET_MAX_RAD;

      const float traveled_rad = fabsf(_robotState.q[joint_idx] - _homingStatus.phase_start_q_rad);
      
      if (joint.getState().at_max_lim) {
        _robotState.q_target[joint_idx] = _robotState.q[joint_idx]; // freeze in place
        _homingStatus.phase = HomingPhase::BACKOFF;
        _homingStatus.phase_start_time_us = micros();
        break;
      }

      if (traveled_rad > JOINT_0_FULL_RANGE_RAD + HOMING_SEARCH_MARGIN_RAD) {
        _homingStatus.phase = HomingPhase::ERROR; // switch never triggered — wiring/config fault
      }
    break;
    }

    case HomingPhase::BACKOFF:
      _robotState.q_target[joint_idx] = _robotState.q[joint_idx] + HOMING_BACKOFF_TARGET_RAD; // small offset away from switch
      
      if (fabsf(_robotState.q[joint_idx] - (_robotState.q_target[joint_idx])) < kAngleRadPositionTolerance || (micros() - _homingStatus.phase_start_time_us) > HOMING_BACKOFF_TIMEOUT_US) {
        joint.setCurrentAngle(JOINT_0_HOME_ANGLE_RAD);
        _robotState.q[joint_idx] = JOINT_0_HOME_ANGLE_RAD;
        _robotState.q_target[joint_idx] = JOINT_0_HOME_ANGLE_RAD;
        advanceHomingSequence();
      }   
    break;

    case HomingPhase::READ_ENCODER:
      angle = 0.0f;
      _robotState.q_target[joint_idx] = 0.0f;
      
      if (fabsf(_robotState.q_target[joint_idx] - _robotState.q[joint_idx]) < kAngleRadPositionTolerance) {
        advanceHomingSequence();
      }
    break;
  
    case HomingPhase::DONE:
      _homingStatus.active = false;
    break;

    case HomingPhase::ERROR:
      _robotState.robot_error_state = robot_error_state_t::NOT_HOMED;
      _homingStatus.active = false;
    break;

    default:
    break;
  }
}
// ----------- Homing functionality END-----------

// ----------- Getters -----------
const RobotState Robot::getState(){
  return _robotState;
}

const float* Robot::getMaxJointSpeed() {
  return _robotConfig.max_joint_speeds;
}

const float* Robot::getMaxJointAcceleration() {
  return _robotConfig.max_joint_accelerations;
}
// ----------- Getters END -----------

// ----------- Setters -----------
void Robot::setJointAngles(const float q[JOINT_NUM]){
  for (int i = 0; i<JOINT_NUM; i++){
    _joints[i].setCurrentAngle(q[i]);
    _robotState.q[i] = q[i];
  }
  update();
}

void Robot::setMaxJointSpeed(const float max_speed[JOINT_NUM]) {
  for (int i = 0; i<JOINT_NUM; i++){
    _robotConfig.max_joint_speeds[i] = max_speed[i];
    _joints[i].setMaxSpeed(max_speed[i]);
  }
}

void Robot::setMaxJointAcceleration(const float max_accel[JOINT_NUM]) {
  for (int i = 0; i<JOINT_NUM; i++){
    _robotConfig.max_joint_accelerations[i] = max_accel[i];
    _joints[i].setMaxAcceleration(max_accel[i]);
  }
}

void Robot::setMotionControlParadigm(robot_motion_control_paradigm_t motion_control_paradigm) {
  _robotState.robot_motion_control_paradigm = motion_control_paradigm;
}
// ----------- Setters END -----------

// ----------- Helpers -----------
float Robot::getDeltaTimeSec() {
    const uint32_t now = micros();
    _robotState.timestamp = now;

    const float dt = (now - last_time) * 1e-6f;
    last_time = now;
    return dt;
}

void Robot::writePoseToState(Matrix4x4 T_EE) {
  _robotState.T_EE = T_EE;

  _robotState.x[0] = T_EE.m[0][3];
  _robotState.x[1] = T_EE.m[1][3];
  _robotState.x[2] = T_EE.m[2][3];

  const Matrix3x3 rot = getRotationMatrixFromPoseMatrix(T_EE);
  const Vect3f eul = rotationMatrixToEulerAngles(rot);

  _robotState.x[3] = eul.v[0];
  _robotState.x[4] = eul.v[1];
  _robotState.x[5] = eul.v[2];
}
// ----------- Helpers END -----------

// ----------- Non class function -----------
void sharedWriteRobotState(const RobotState &rs) {
    static uint32_t update_counter = 0;

    RobotSharedState snap{};

    for (int i = 0; i < JOINT_NUM; ++i) {
        snap.q_rad[i] = rs.q[i];
        snap.q_target_rad[i] = rs.q_target[i];
        snap.q_dot_rad[i] = rs.q_dot[i];
        snap.q_dot_target_rad[i] = rs.q_dot_target[i];
    }

    for (int i = 0; i < 3; ++i) {
        snap.x_mm[i] = rs.x[i];
        snap.x_rad[i] = rs.x[i + 3];
        snap.x_target_mm[i] = rs.x_target[i];
        snap.x_target_rad[i] = rs.x_target[i + 3];
    }

    snap.moving = rs.exec_state;
    snap.robot_error_state = rs.robot_error_state;
    snap.robot_motion_mode = rs.robot_motion_control_paradigm;
    snap.timestamp_us = micros();
    snap.update_counter = ++update_counter;

    g_robot_shared_state = snap;
}
// ----------- Non class function END -----------
