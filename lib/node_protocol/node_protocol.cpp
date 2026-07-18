#include "node_protocol.h"

NodeProtocol::NodeProtocol() {
  _currentNode = NODE_FIRST;
  _rx_bufer[0] = '\0';
  _rx_idx = 0;
  _protocol_state = ProtocolState::IDLE;
  _requestSentAt = 0;

  for (uint8_t i = 0; i < NODE_LAST - NODE_FIRST + 1; i++) {
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
  if (nodeId < NODE_FIRST || nodeId > NODE_LAST) return 0.0f;
  return _angles[nodeId - NODE_FIRST];
}

bool NodeProtocol::isAngleValid(uint8_t nodeId) const {
  if (nodeId < NODE_FIRST || nodeId > NODE_LAST) return false;
  return _angleValid[nodeId - NODE_FIRST];
}

unsigned long NodeProtocol::getAngleAge(uint8_t nodeId) const {
  if (nodeId < NODE_FIRST || nodeId > NODE_LAST) return 0xFFFFFFFF;
  return millis() - _angleTimestamp[nodeId - NODE_FIRST];
}

void NodeProtocol::getAngles(float angles[JOINT_NUM]){
  for (uint8_t i=0; i<JOINT_NUM; i++){
    if (i==0){
      angles[i] = 0.0f; 
    }
    angles[i] = _angles[i-1];
  }
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

void NodeProtocol::advanceToNextNode() {
  _currentNode++;
  if (_currentNode > NODE_LAST) _currentNode = NODE_FIRST;
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
