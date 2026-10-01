#ifndef COM_HANDLER_H
#define COM_HANDLER_H

#include <Arduino.h>
#include "shared/robot_types.h"
#include "shared/command_table.h"

static constexpr uint8_t MAX_RX_BUFFER_SIZE = 128;
static constexpr uint8_t MAX_TX_BUFFER_SIZE = 128;
static constexpr char START_CHAR = '<';
static constexpr char END_CHAR = '>';

enum RxState {
    RX_STATE_IDLE,  
    RX_STATE_RECEIVING
};

class ComHandler {
    public:
        ComHandler();

        void initialize(void);
        void update(void);
        bool isCmdReady(void);

        void parseBuffer(void);
        ParsedCommand getCommand(void);
        void handleParsedCommand(const ParsedCommand& cmd);

        void sendRobotState(const RobotState& state);

        void sendResponse(const char* response_cstr);
        void queueResponse(const char* response_cstr);
        void updateFrequency(void);
        float getUpdateFrequency(void); 

    private:        
        uint32_t _update_count;
        uint32_t _freq_last_count;
        uint32_t _freq_last_ms;
        uint32_t _timestamp;
        float _update_frequency;

        // For commands comming from PC
        bool _is_cmd_ready;
        ParsedCommand _parsed_cmd;
        RxState _rx_state;
        uint8_t _rx_idx;

        // For commands that have a response from robot
        bool _is_response_ready; // the robot can use so com handler actually sends the msg while robot just fills the buffer
        char _pending_response[MAX_TX_BUFFER_SIZE];
        
        char _rx_buffer[MAX_RX_BUFFER_SIZE];
        char _tx_buffer[MAX_TX_BUFFER_SIZE];

        const CommandDefinition* findCommandDefinition(const char* cmd_name);
};

#endif // COM_HANDLER_H
