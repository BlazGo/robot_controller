#include "node_protocol.h"

NodeProtocol::NodeProtocol() {
  _currentNode = NODE_FIRST;
  _rx_bufer[0] = '\0';
  _rx_idx = 0;
  _protocol_state = ProtocolState::IDLE;
  _requestSentAt = 0;

  for (uint8_t i = 0; i < JOINT_NUM; i++) {
    _angles[i] = 0.0f;
    _angleTimestamp[i] = 0;
    _angleValid[i] = false;
  }
}

void NodeProtocol::begin(uint32_t baudrate) {
  Serial1.setTX(RS485_TX_PIN);
  Serial1.setRX(RS485_RX_PIN);
  Serial1.setFIFOSize(128);
  Serial1.begin(baudrate);
}

float NodeProtocol::getAngle(uint8_t nodeId) const {
  if (nodeId <= NODE_FIRST || nodeId >= JOINT_NUM) return 0.0f;
  return _angles[nodeId];
}

bool NodeProtocol::isAngleValid(uint8_t nodeId) const {
  if (nodeId <= NODE_FIRST || nodeId >= JOINT_NUM) return 0.0f;
  return _angleValid[nodeId];
}

unsigned long NodeProtocol::getAngleAge(uint8_t nodeId) const {
  if (nodeId <= NODE_FIRST || nodeId >= JOINT_NUM) return 0xFFFFFFFF;
  return millis() - _angleTimestamp[nodeId];
}

JointAngles NodeProtocol::getAngles(){
  uint8_t read_idx = _active_buffer_idx.load(std::memory_order_acquire);
  return _angle_buffers[read_idx];
}

void NodeProtocol::update() {

  switch (_protocol_state) {

    case ProtocolState::IDLE:
      sendReadCommand(_currentNode);
      _requestSentAt = millis();
      _rx_idx = 0;
      _protocol_state = ProtocolState::WAITING_REPLY;
      break;

    case ProtocolState::WAITING_REPLY:
      if (Serial1.available()) {
        char c = Serial1.read();
        if (c == START_CHAR) {
          _rx_idx = 0;
          _protocol_state = ProtocolState::RECEIVING_DATA;
        }
      }
      else if (millis() - _requestSentAt > POLL_TIMEOUT_MS) {
        advanceToNextNode();   // node didn't answer in time, move on
      }
      break;

    case ProtocolState::RECEIVING_DATA:
      if (Serial1.available()) {
        char c = Serial1.read();

        if (c == END_CHAR) {
          _rx_bufer[_rx_idx] = '\0';

          float angle;
          if (parseCommand(_rx_bufer, _currentNode, angle)) {
            uint8_t i = _currentNode - NODE_FIRST;
            _angles[i] = angle;
            _angleTimestamp[i] = millis();
            _angleValid[i] = true;
          }
          advanceToNextNode();
        }
        else if (_rx_idx < MAX_RX_CHAR - 1) {
          _rx_bufer[_rx_idx++] = c;
        }
        else {
          advanceToNextNode();   // frame too long, drop and resync
        }
      }
      else if (millis() - _requestSentAt > POLL_TIMEOUT_MS) {
        advanceToNextNode();     // reply started but never finished
      }
      break;
  }
}

void NodeProtocol::publishAngles(){
  uint8_t write_idx_u8 = 1 - _active_buffer_idx.load(std::memory_order_relaxed);
  for (uint8_t i=0; i<JOINT_NUM; i++){
    _angle_buffers[write_idx_u8].values_rad[i] = _angles[i];
  }
  _active_buffer_idx.store(write_idx_u8, std::memory_order_release);
}


void NodeProtocol::advanceToNextNode() {
  _currentNode++;
  if (_currentNode > NODE_LAST){
    _currentNode = NODE_FIRST;
    publishAngles();
  }
  _protocol_state = ProtocolState::IDLE;
}

void NodeProtocol::sendReadCommand(uint8_t nodeId) {
  char cmd[16];
  snprintf(cmd, sizeof(cmd), "%cR %d%c", START_CHAR, nodeId, END_CHAR);
  Serial1.print(cmd);
}

bool NodeProtocol::parseCommand(const char* line, uint8_t expectedNode, float& angle) {
  char cmd = 0;
  int id = 0;
  long angleMilliRad = 0;

  int matched = sscanf(line, " %c %d %ld", &cmd, &id, &angleMilliRad);
  if (matched != 3) return false;
  if (cmd != 'A') return false;
  if (id != expectedNode) return false;

  angle = angleMilliRad * 0.001f;
  return true;
}
