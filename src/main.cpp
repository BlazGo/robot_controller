#define __FREERTOS 1

#include <Arduino.h>
#include <FreeRTOS.h>

#include "com_interface.h"
#include "robot.h"
#include "encoder_manager.h"

void printLoopInfo(void);
void printJointState(void);
void printRobotState(void);
void printEncoderState(void);

void robotUpdateTask( void *pvParameters);
void encoderUpdateTask( void *pv_parameters);
void IOUpdateTask( void *pv_parameters);
void printTask( void *pvParameters );

volatile uint32_t g_dt_us = 0;

EncoderManager encoders;
ComHandler com(Serial);
Robot robot;

void setup() {
  robot.init();
  robot.enable();
  robot.attachEncoderManager(&encoders);

  xTaskCreate(robotUpdateTask, "robotUpdateTask", 4096, nullptr, 3, nullptr);
}

void setup1() {
  pinMode(BLUE_LED, OUTPUT);
  
  Wire.begin();
  //Wire1.begin();
  Wire.setClock(100000);
  //Wire1.setClock(100000);
  
  // Set the serial baudrate
  com.begin(115200);
  encoders.init();

  //xTaskCreate(encoderUpdateTask, "encoderUpdateTask", 4096, nullptr, 1, nullptr);
  xTaskCreate(IOUpdateTask, "IOUpdateTask", 4096, nullptr, 2, nullptr);
  xTaskCreate(printTask, "printTask", 4096, nullptr, 3, nullptr);
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}

void loop1() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}

void robotUpdateTask(void *pv_parameters){
  /*
    I dunno something with RTOS (pointer to void parameters)
  Option to reuse the same tasks for multiple purposes with
  different parameters
  */
  (void)pv_parameters;

  // Read current ticks
  TickType_t last_wake_tick = xTaskGetTickCount();
  // Set the next call to function
  const TickType_t update_period_ticks = pdMS_TO_TICKS(1);

  for (;;) {
    const uint32_t t0_us = micros();

    // robot.updateFromEncoders(); // this has old data... hmmmm is it still useful and how?
    robot.update();

    // calculate how muct time the loop took
    g_dt_us = micros() - t0_us;
    // Will run constantly at precisely 2ms period (that is including execution time)
    vTaskDelayUntil(&last_wake_tick, update_period_ticks);
  }
}

void IOUpdateTask(void *pv_parameters) {
  (void)pv_parameters;

  for (;;) {
    com.update();
/*
    if (com.cmdReady) {
      switch (com.cmd_robot.type) {
        case CMD_JOINT_MOVE:
          robot.moveJoint(com.cmd_robot.params);
          break;

        case CMD_CART_MOVE:
          robot.moveCart(com.cmd_robot.params);
          break;

        case CMD_SET_MAX_JOINT_SPEED:
          robot.setMaxJointSpeed(com.cmd_robot.params);
          break;

        case CMD_SET_MAX_JOINT_ACCELERATION:
          robot.setMaxJointAcceleration(com.cmd_robot.params);
          break;
        }
        com.cmdReady = false;
    }
*/
    // will run at 5ms + execution time
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

void encoderUpdateTask(void *pv_parameters) {
  (void)pv_parameters;

  TickType_t last_wake_tick = xTaskGetTickCount();
  const TickType_t update_period_ticks = pdMS_TO_TICKS(20);

  for (;;) {
    encoders.updateAngles();

    vTaskDelayUntil(&last_wake_tick, update_period_ticks);
  }
}

void printTask(void *pv_parameters){
  (void)pv_parameters;

  for (;;) {
    digitalWrite(LED_BUILTIN, HIGH);
        
    printLoopInfo();
    printRobotState();
    printEncoderState();

    digitalWrite(LED_BUILTIN, LOW);

    // will run at 1000ms + execution time
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void printLoopInfo(void) {
  char buffer[64];
  const int f_min = 1000;
  const float f_hz = 1000000.0f / static_cast<float>(g_dt_us);

  sprintf(buffer, "Loop time: %lu [us], Freq: %.2f [Hz] (min. %d Hz)",
          static_cast<unsigned long>(g_dt_us), f_hz, f_min);

  Serial.println(buffer);
}

void printRobotState(void){
  char buffer[256];

  RobotState rs = robot.getState();

  sprintf(buffer,
          "q:            %.2f, %.2f, %.2f, %.2f, %.2f, %.2f [rad]\n"
          "q_target:     %.2f, %.2f, %.2f, %.2f, %.2f, %.2f [rad]\n"
          "q_dot:        %.2f, %.2f, %.2f, %.2f, %.2f, %.2f [rad/s]\n"
          "q_dot_target: %.2f, %.2f, %.2f, %.2f, %.2f, %.2f [rad/s]\n"
          "x:            %.1f, %.1f, %.1f [mm] \n"
          "              %.2f, %.2f, %.2f [rad]\n"
          "x_target:     %.1f, %.1f, %.1f [mm] \n"
          "              %.2f, %.2f, %.2f [rad]\n"
          "moving:    %d\n"
          "err_state: %d\n"
          "mode:      %d (0-Cart, 1-Joint)\n",
          rs.q[0], rs.q[1], rs.q[2], rs.q[3], rs.q[4], rs.q[5],
          rs.q_target[0], rs.q_target[1], rs.q_target[2], rs.q_target[3], rs.q_target[4], rs.q_target[5],
          rs.q_dot[0], rs.q_dot[1], rs.q_dot[2], rs.q_dot[3], rs.q_dot[4], rs.q_dot[5],
          rs.q_dot_target[0], rs.q_dot_target[1], rs.q_dot_target[2], rs.q_dot_target[3], rs.q_dot_target[4], rs.q_dot_target[5],
          rs.x[0], rs.x[1], rs.x[2], rs.x[3], rs.x[4], rs.x[5],
          rs.x_target[0], rs.x_target[1], rs.x_target[2], rs.x_target[3], rs.x_target[4], rs.x_target[5],
          rs.moving,
          rs.robot_error_state,
          rs.robot_motion_control_paradigm
        );
  Serial.println(buffer);
}

void printEncoderState(void){
  char buffer[128];

  EncoderFrame _temp;
  encoders.getLatestFrame(_temp);

  sprintf(buffer,
          "q_enc:     %.2f, %.2f, %.2f, %.2f, %.2f, %.2f [rad]\n"
          "valid:     %d, %d, %d, %d, %d, %d, \n"
          "timestamp: %i",
          degToRad(_temp.joints[0].angle_deg), degToRad(_temp.joints[1].angle_deg), degToRad(_temp.joints[2].angle_deg), degToRad(_temp.joints[3].angle_deg), degToRad(_temp.joints[4].angle_deg), degToRad(_temp.joints[5].angle_deg),
          _temp.joints[0].valid, _temp.joints[1].valid, _temp.joints[2].valid, _temp.joints[3].valid, _temp.joints[4].valid, _temp.joints[5].valid,
          _temp.timestamp_us
        );
  Serial.println(buffer);
}