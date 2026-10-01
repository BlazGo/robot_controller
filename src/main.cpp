#include <Arduino.h>

#include "config/board_config.h"
#include "config/robot_config.h"
#include "communication/com_handler.h"
#include "communication/node_protocol.h"
#include "robot/motor.h"
#include "robot/joint.h"
#include "robot/robot.h"

void timedToggle();

ComHandler com_port;
NodeProtocol encoders;
Robot robot(&com_port, &encoders);

uint32_t blink_last;

// ---------------------
// Setup core_0
// ---------------------
void setup() {
  robot.initialize();
  robot.enable();
}

// ---------------------
// Setup core_1
// ---------------------
void setup1() {
  com_port.initialize();
  encoders.begin();
  pinMode(BLUE_LED, OUTPUT);
}

// ---------------------
// Loop core_0
// ---------------------
void loop() {
  if (com_port.isCmdReady()){
    ParsedCommand cmd = com_port.getCommand();

    robot.handleCmd(cmd);
  }

  robot.update();
}

// ---------------------
// Loop core_1 (benchmark mode)
// ---------------------
void loop1() {
  delay(1);
  com_port.update();
  encoders.update();
  
  timedToggle();

  // --- Robot state send, every iteration ---
  RobotState s = robot.getRobotState();
  com_port.sendRobotState(s);
}

void timedToggle() {
  if ((millis() - blink_last) > 250) {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    blink_last = millis();
  }
}
