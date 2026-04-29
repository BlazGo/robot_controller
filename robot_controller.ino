#include "src/communication.h"
#include "src/robot.h"

Robot test_robot;
ComHandler com(Serial);

uint32_t start = micros();
uint32_t last_print = millis();
uint32_t dt = 1;
uint32_t t0 = 1;

void setup() {
  com.begin(115200);

  pinMode(LED_BUILTIN, OUTPUT);

  test_robot.init();
  test_robot.enableJoints();
}

void loop() {
  t0 = micros();

  com.update();

  if (com.cmdReady){
    // Send to robot controller
    switch (com.cmd_robot.type){
      case CMD_JOINT_MOVE:
        test_robot.jointMove(com.cmd_robot.params);
        break;
      case CMD_CART_MOVE:
        test_robot.cartMove(com.cmd_robot.params);
        break;
      case CMD_SET_MAX_JOINT_SPEED:
        test_robot.setMaxJointSpeed(com.cmd_robot.params);
        break;
      case CMD_SET_MAX_JOINT_ACCELERATION:
        test_robot.setMaxJointAcceleration(com.cmd_robot.params);
        break;
      case CMD_SET_SPEED_CONTROL_PARADIGM:
        //test_robot.setMotionControlParadigm(SPEED_CONTROL);
        break;
      case CMD_SET_POSITION_CONTROL_PARADIGM:
        //test_robot.setMotionControlParadigm(POSITION_CONTROL);
        break;
      case CMD_ENABLE:
        test_robot.enableJoints();
        break;
      case CMD_DISABLE:
        test_robot.disableJoints();
        break;
    }
    com.cmdReady = false;
  }
  
  test_robot.update();

  if ((millis() - last_print) > 200){
    last_print = millis();
    
    digitalWrite(LED_BUILTIN, HIGH);

    Serial.print("Loop time: ");
    Serial.print(dt);
    Serial.print(" [us], Freq: ");
    Serial.print(1 / ( ((float)dt) / 1000000));
    Serial.println(" [Hz] (min. 2000)");

    test_robot.printInfo();
    digitalWrite(LED_BUILTIN, LOW);
  }
  dt = micros() - t0;

  while ((micros() - t0) < 500){}

}
