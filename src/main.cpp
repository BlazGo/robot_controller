#define __FREERTOS 1

#include "Arduino.h"
#include <FreeRTOS.h>

#include "com_interface.h"
#include "joint.h"
#include "robot.h"

void printLoopInfo(void);
void printJointState(void);
void printRobotState(void);

void robotUpdateTask( void *pvParameters );
void printTask( void *pvParameters );

volatile uint32_t g_dt_us = 0;

Robot robot;
ComHandler com(Serial);

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  
  // Set the serial baudrate
  com.begin(250000);

  robot.init();
  robot.enable();

  xTaskCreate(robotUpdateTask, "robotUpdateTask", 4096, nullptr, 3, nullptr);
  xTaskCreate(printTask, "printTask", 2048, nullptr, 1, nullptr);
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}

void robotUpdateTask(void *pv_parameters){
  (void)pv_parameters;

  TickType_t last_wake_tick = xTaskGetTickCount();
  const TickType_t update_period_ticks = pdMS_TO_TICKS(1);

  for (;;) {
    const uint32_t t0_us = micros();

    com.update();

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

    robot.update();

    // calculate how muct time the loop took
    g_dt_us = micros() - t0_us;

    // Will run constantly at precisely 2ms period (that is including execution time)
    vTaskDelayUntil(&last_wake_tick, update_period_ticks);
  }
}

void printTask(void *pv_parameters){
  (void)pv_parameters;

  for (;;) {
    digitalWrite(LED_BUILTIN, HIGH);
        
    printLoopInfo();
    printRobotState();

    digitalWrite(LED_BUILTIN, LOW);

    // will run at 200ms + execution time
    vTaskDelay(pdMS_TO_TICKS(200));
  }
}

void printLoopInfo() {
  char buffer[64];
  const int f_min = 1000;
  const float f_hz = 1000000.0f / static_cast<float>(g_dt_us);

  sprintf(buffer, "Loop time: %lu [us], Freq: %.2f [Hz] (min. %d Hz)",
          static_cast<unsigned long>(g_dt_us), f_hz, f_min);

  Serial.println(buffer);
}
/*
void printJointState(void) {
  char buffer[128];

  JointState js = joint.getState();

  sprintf(buffer,
          "q:\t\t%.2f [rad]\nq_dot:\t\t%.2f [rad/s]\nq_target:\t%.2f [rad]\nq_dot_target:\t%.2f [rad]\nmoving:\t\t%i\nlimit_hit:\tmin-%i max-%i",
          js.angle_rad,
          js.angle_vel_rad_s,
          js.target_angle_rad,
          js.target_angle_vel_rad_s,
          js.moving,
          js.at_min_lim,
          js.at_max_lim);

  Serial.println(buffer);
}
*/
void printRobotState(void){
  char buffer[128];

  RobotState rs = robot.getState();

  sprintf(buffer,
          "q:            %.2f, %.2f, %.2f, %.2f, %.2f, %.2f [rad]\n"
          "q_target:     %.2f, %.2f, %.2f, %.2f, %.2f, %.2f [rad]\n"
          "q_dot:        %.2f, %.2f, %.2f, %.2f, %.2f, %.2f [rad/s]\n"
          "q_dot_target: %.2f, %.2f, %.2f, %.2f, %.2f, %.2f [rad/s]\n"
          "x:            %.1f, %.1f, %.1f [mm] \n"
          "              %.2f, %.2f, %.2f [rad]\n"
          "moving:    %d\n"
          "err_state: %d\n"
          "mode:      %d (0-Cart, 1-Joint)\n",
          rs.q[0], rs.q[1], rs.q[2], rs.q[3], rs.q[4], rs.q[5],
          rs.q_target[0], rs.q_target[1], rs.q_target[2], rs.q_target[3], rs.q_target[4], rs.q_target[5],
          rs.q_dot[0], rs.q_dot[1], rs.q_dot[2], rs.q_dot[3], rs.q_dot[4], rs.q_dot[5],
          rs.q_dot_target[0], rs.q_dot_target[1], rs.q_dot_target[2], rs.q_dot_target[3], rs.q_dot_target[4], rs.q_dot_target[5],
          rs.x[0], rs.x[1], rs.x[2], rs.x[3], rs.x[4], rs.x[5],
          rs.moving,
          rs.robot_error_state,
          rs.robot_motion_control_paradigm
        );

  Serial.println(buffer);

}
