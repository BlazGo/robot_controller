#define __FREERTOS 1

#include <Arduino.h>
#include <FreeRTOS.h>

#include "com_interface.h"
#include "robot.h"
#include "robot_command_queue.h"
#include "display.h"
#include "node_protocol.h"

void printLoopInfo(void);
void printRobotState(void);

void robotUpdateTask( void *pvParameters);
void IOUpdateTask( void *pv_parameters);
void printTask( void *pvParameters );
void displayUpdateTask( void *pvParameters );

volatile uint32_t g_dt_us = 0;

ComHandler com(Serial);
Robot robot;
Display display;
NodeProtocol nodes;

// ---------------------
// Setup core_0
// ---------------------
void setup() {
  robot.init();
  robot.enable();

  xTaskCreate(robotUpdateTask, "robotUpdateTask", 4096, nullptr, 1, nullptr);
}

// ---------------------
// Setup core_1
// ---------------------
void setup1() {
  pinMode(BLUE_LED, OUTPUT);
  
  Wire.begin();
  Wire.setClock(100000);
  
  // Set the serial baudrate
  com.begin(SERIAL_BAUDRATE);
  nodes.begin(RS485_BAUDRATE);             // starts Serial1 for RS485
  display.init();

  xTaskCreate(nodeUpdateTask, "nodeUpdateTask", 4096, nullptr, 2, nullptr);
  xTaskCreate(IOUpdateTask, "IOUpdateTask", 4096, nullptr, 3, nullptr);
  xTaskCreate(printTask, "printTask", 8192, nullptr, 4, nullptr);
  xTaskCreate(displayUpdateTask, "displayUpdateTask", 8192, nullptr, 5, nullptr);
}

// ---------------------
// Loop core_0
// ---------------------
void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}

// ---------------------
// Loop core_1
// ---------------------
void loop1() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}

// ---------------------
// Tasks functions
// ---------------------

// core_0
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
    
    RobotCommand cmd;

    // if robot is avaliable for work
    if (!robot.isBusy()){
      // pop the command
      if (robotCommandPop(&cmd)) {
        // and (start) execute it
        robot.acceptCommand(cmd);
      }
    }

    robot.update();

    // calculate how muct time the loop took
    g_dt_us = micros() - t0_us;
    // Will run constantly at precisely 2ms period (that is including execution time)
    vTaskDelayUntil(&last_wake_tick, update_period_ticks);
  }
}

// core_1
void IOUpdateTask(void *pv_parameters) {
  (void)pv_parameters;

  for (;;) {
    // continuously update com port (read buffer)
    com.update();

    // if command detected parse it and put into queue
    if (com.cmdReady) {
      Serial.println(com.com_cmd_robot.type);
      RobotCommand robot_cmd;

      switch (com.com_cmd_robot.type) {
        
        case CMD_JOINT_MOVE:
          robot_cmd.type = RobotCommandType::JOINT_MOVE;
          for (uint8_t joint_idx = 0; joint_idx < JOINT_NUM; ++joint_idx) {
            robot_cmd.q[joint_idx] = com.com_cmd_robot.params[joint_idx];
          }
        break;
        
        case CMD_CART_MOVE:
          robot_cmd.type = RobotCommandType::CART_MOVE;
          for (uint8_t pose_idx = 0; pose_idx < 6; ++pose_idx) {
            robot_cmd.x[pose_idx] = com.com_cmd_robot.params[pose_idx];
          }
        break;

        case CMD_SET_MAX_JOINT_SPEED:
          robot_cmd.type = RobotCommandType::SET_MAX_JOINT_SPEEDS;
          for (uint8_t param_idx = 0; param_idx < JOINT_NUM; ++param_idx) {
            robot_cmd.x[param_idx] = com.com_cmd_robot.params[param_idx];
          }
        break;

        case CMD_SET_MAX_JOINT_ACCELERATION:
          robot_cmd.type = RobotCommandType::SET_MAX_JOINT_ACCELERATIONS;
          for (uint8_t param_idx = 0; param_idx < JOINT_NUM; ++param_idx) {
            robot_cmd.x[param_idx] = com.com_cmd_robot.params[param_idx];
          }
        break;

        case CMD_SET_CURRENT_JOINT_ANGLES_FROM_ENCODERS:
          robot_cmd.type = RobotCommandType::UPDATE_FROM_ENCODERS;

          float temp_angles[JOINT_NUM];
          nodes.getAngles(temp_angles);

          for (uint8_t param_idx = 0; param_idx < JOINT_NUM; ++param_idx) {
            robot_cmd.q[param_idx] = temp_angles[param_idx];
          }
        break;

        case CMD_START_HOMING:
          robot_cmd.type = RobotCommandType::START_HOMING;
        break;

        default:
        break;
      }
      robotCommandPush(&robot_cmd);
      com.cmdReady = false;
    }

    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

void displayUpdateTask(void *pv_parameters) {
}

void printTask(void *pv_parameters){
  (void)pv_parameters;

  for (;;) {
    digitalWrite(BLUE_LED, HIGH);
        
    printLoopInfo();
    printRobotState();

    digitalWrite(BLUE_LED, LOW);

    // will run at 1000ms + execution time
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

void nodeUpdateTask(void *pv_parameters) {
  (void)pv_parameters;

  for (;;) {
    nodes.update();             // non-blocking, call every loop, no delay() anywhere
  }
  vTaskDelay(pdMS_TO_TICKS(10));
}

// ---------------------
// Convenience functions
// ---------------------
void printLoopInfo(void) {
  char buffer[64];
  const int f_min = 1000;
  const float f_hz = 1000000.0f / static_cast<float>(g_dt_us);

  sprintf(buffer, "Loop time: %lu [us], Freq: %.2f [Hz] (min. %d Hz)",
          static_cast<unsigned long>(g_dt_us), f_hz, f_min);

  Serial.println(buffer);
}

void printRobotState(void){
  char buffer[512];

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
          "exec_state:   %d\n"
          "err_state:    %d\n"
          "mode:         %d (0-Cart, 1-Joint)\n"
          "timestamp:    %u \n",
          rs.q[0], rs.q[1], rs.q[2], rs.q[3], rs.q[4], rs.q[5],
          rs.q_target[0], rs.q_target[1], rs.q_target[2], rs.q_target[3], rs.q_target[4], rs.q_target[5],
          rs.q_dot[0], rs.q_dot[1], rs.q_dot[2], rs.q_dot[3], rs.q_dot[4], rs.q_dot[5],
          rs.q_dot_target[0], rs.q_dot_target[1], rs.q_dot_target[2], rs.q_dot_target[3], rs.q_dot_target[4], rs.q_dot_target[5],
          rs.x[0], rs.x[1], rs.x[2], rs.x[3], rs.x[4], rs.x[5],
          rs.x_target[0], rs.x_target[1], rs.x_target[2], rs.x_target[3], rs.x_target[4], rs.x_target[5],
          rs.exec_state,
          rs.robot_error_state,
          rs.robot_motion_control_paradigm,
          rs.timestamp
        );
  Serial.println(buffer);

  display.displayInfo(rs.q, rs.x, rs.timestamp);
}
