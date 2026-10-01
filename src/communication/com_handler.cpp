#include "com_handler.h"
#include "shared/command_table.h"
#include "config/board_config.h"

ComHandler::ComHandler()
:   
    _is_cmd_ready{false},
    _is_response_ready{false},
    _rx_state{RX_STATE_IDLE},
    _rx_idx{0},
    _rx_buffer{0},
    _tx_buffer{0},
    _pending_response{0}
{
  
} 

void ComHandler::initialize(void){
    Serial.begin(SERIAL_BAUDRATE);
    
    _rx_buffer[0] = '\0';
    _tx_buffer[0] = '\0';
}

void ComHandler::parseBuffer(void){
    ParsedCommand _cmd_out;
    
    char _rx_buffer_copy[MAX_RX_BUFFER_SIZE];
    strncpy(_rx_buffer_copy, _rx_buffer, MAX_RX_BUFFER_SIZE-1);
    _rx_buffer_copy[MAX_RX_BUFFER_SIZE-1] = '\0';

    char* save_ptr;
    char* token = strtok_r(_rx_buffer_copy, ",", &save_ptr);
    if (token == nullptr){
        // Looks like the buffer is empty - skip
        return;
    }

    strncpy(_cmd_out.cmd_name, token, MAX_CMD_NAME_LEN-1);
    _cmd_out.cmd_name[MAX_CMD_NAME_LEN-1] = '\0';

    _cmd_out.num_params = 0;
    while ((token = strtok_r(nullptr, ",", &save_ptr)) != nullptr && _cmd_out.num_params < MAX_CMD_PARAM_NUM)
    {
        _cmd_out.params[_cmd_out.num_params++] = strtof(token, nullptr);
    }

    handleParsedCommand(_cmd_out);
}

void ComHandler::handleParsedCommand(const ParsedCommand& cmd){
    const CommandDefinition* cd = findCommandDefinition(cmd.cmd_name);

    if (cd == nullptr){
        sendResponse("ERR_UNKNOWN_CMD");
        return;
    }
    if (cd->cmd_num_params != cmd.num_params){
        sendResponse("ERR_PARAM_COUNT");
        return;
    }
    if (cd->cmd_id == CMD_PING){
        sendResponse("PONG");
        return;
    }
    
    _parsed_cmd = cmd;
    _parsed_cmd.id = cd->cmd_id;
    _is_cmd_ready = true;
}

void ComHandler::update(void){
    while (Serial.available() > 0){
        char c = Serial.read();
        switch (_rx_state){
            case RX_STATE_IDLE:
                // Only look for start character to start receiving
                if (c == START_CHAR){
                    _rx_idx = 0;
                    _rx_state = RX_STATE_RECEIVING;
                }
                break;
            case RX_STATE_RECEIVING:
                // if we detect end char we end the buffer and go back to idle
                if (c == END_CHAR){
                    _rx_buffer[_rx_idx] = '\0';
                    _rx_state = RX_STATE_IDLE;

                    Serial.println(_rx_buffer);
                    parseBuffer();
                }
                // While the buffer is not full
                else if (_rx_idx < MAX_RX_BUFFER_SIZE){
                    _rx_buffer[_rx_idx++] = c;
                }
                // Upon overflow we jump out and don't pass this frame (resync on next start char)
                else{
                    _rx_state = RX_STATE_IDLE;
                }
                break;
        } 
    }

    if (_is_response_ready){
        sendResponse(_pending_response);
        _is_response_ready = false;
    }
}

void ComHandler::updateFrequency(void) {
    _update_count++;

    _timestamp = millis();
    if (_timestamp - _freq_last_ms >= 1000) {
        uint32_t d_count = _update_count - _freq_last_count;
        uint32_t dt_ms = _timestamp - _freq_last_ms;
        _update_frequency = d_count * 1000.0f / dt_ms;
        
        _freq_last_count = _update_count;
        _freq_last_ms = _timestamp;
    }
}

bool ComHandler::isCmdReady(void){
    return _is_cmd_ready;
}

void ComHandler::sendResponse(const char* response_cstr){
    size_t response_len = strlen(response_cstr);
    size_t max_payload_len = MAX_TX_BUFFER_SIZE - 3; // START,END + '\0'

    if (response_len > max_payload_len){
        response_len = max_payload_len;
    }
    _tx_buffer[0] = START_CHAR;
    memcpy(&_tx_buffer[1], response_cstr, response_len);
    _tx_buffer[1 + response_len] = END_CHAR;
    _tx_buffer[2 + response_len] = '\0';
    
    Serial.println(_tx_buffer);
}

void ComHandler::queueResponse(const char* response_cstr){
    strncpy(_pending_response, response_cstr, MAX_TX_BUFFER_SIZE-1);
    _pending_response[MAX_TX_BUFFER_SIZE-1] = '\0';
    _is_response_ready = true; 
}

void ComHandler::sendRobotState(const RobotState& state){
    updateFrequency();

    char state_cstr[MAX_TX_BUFFER_SIZE];
    snprintf(state_cstr, sizeof(state_cstr),
             "FREQ,%.3f, %.3f",
             state.update_frequency, _update_frequency);
    sendResponse(state_cstr);

    snprintf(state_cstr, sizeof(state_cstr),
             "STATE_Q,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f",
             state.q[0], state.q[1], state.q[2], state.q[3], state.q[4], state.q[5]);
    sendResponse(state_cstr);

    snprintf(state_cstr, sizeof(state_cstr),
             "STATE_Q_DOT,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f",
             state.q_dot[0], state.q_dot[1], state.q_dot[2], state.q_dot[3], state.q_dot[4], state.q_dot[5]);
    sendResponse(state_cstr);

    snprintf(state_cstr, sizeof(state_cstr),
             "STATE_LIM_MAX,%d,%d,%d,%d,%d,%d",
             state.max_limits[0], state.max_limits[1], state.max_limits[2], state.max_limits[3], state.max_limits[4], state.max_limits[5]);
    sendResponse(state_cstr);

    snprintf(state_cstr, sizeof(state_cstr),
             "STATE_LIM_MIN,%d,%d,%d,%d,%d,%d",
             state.min_limits[0], state.min_limits[1], state.min_limits[2], state.min_limits[3], state.min_limits[4], state.min_limits[5]);
    sendResponse(state_cstr);
}


ParsedCommand ComHandler::getCommand(void){
    _is_cmd_ready = false;
    return _parsed_cmd;
}

const CommandDefinition* ComHandler::findCommandDefinition(const char* cmd_name){
    for (uint8_t i = 0; i < NUM_COMMANDS; i++){
        if (strcmp(cmd_name, COMMAND_TABLE[i].cmd_name) == 0){
            return &COMMAND_TABLE[i];
        }
    }
    return nullptr;
}

float ComHandler::getUpdateFrequency(void) {
    return _update_frequency;
}
