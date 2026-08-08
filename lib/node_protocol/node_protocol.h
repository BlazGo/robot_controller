#ifndef NODE_PROTOCOL_H
#define NODE_PROTOCOL_H

#include <Arduino.h>
#include "config.h"
#include "robot_types.h"
#include <atomic>

#define RS485_TX_PIN   0
#define RS485_RX_PIN   1
#define RS485_BAUD     921600

#define START_CHAR     '<'
#define END_CHAR       '>'
#define MAX_RX_CHAR    32

#define NODE_FIRST     1
#define NODE_LAST      5
#define POLL_TIMEOUT_MS 30   // how long to wait for a node reply before moving on

enum class ProtocolState {
  IDLE,            // ready to send next request
  WAITING_REPLY,   // request sent, waiting for START_CHAR..END_CHAR reply
  RECEIVING_DATA   // inside a frame, collecting chars until END_CHAR
};

class NodeProtocol {
public:
  NodeProtocol();

  void begin(uint32_t baudrate = RS485_BAUD);
  void update();                                    // call every loop(), non-blocking
  float getAngle(uint8_t nodeId) const;             // last known angle, instant
  bool  isAngleValid(uint8_t nodeId) const;         // has this node ever replied?
  unsigned long getAngleAge(uint8_t nodeId) const;  // ms since last update
  JointAngles getAngles();
private:
  void sendReadCommand(uint8_t nodeId);
  bool parseCommand(const char* line, uint8_t expectedNode, float& angle);
  void advanceToNextNode();
  void publishAngles();

  ProtocolState _protocol_state;
  uint8_t _currentNode;

  char _rx_bufer[MAX_RX_CHAR];
  uint8_t _rx_idx;

  // Atomic circular buffer for inter core communication
  JointAngles _angle_buffers[2];
  std::atomic<uint8_t> _active_buffer_idx{0};

  // Internal arrays to track the angles, age and validity
  float _angles[JOINT_NUM];
  unsigned long _angleTimestamp[JOINT_NUM];
  bool _angleValid[JOINT_NUM];
 
  unsigned long _requestSentAt;
};

#endif
