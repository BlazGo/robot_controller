#ifndef COM_INTERFACE_H
#define COM_INTERFACE_H

#include <Arduino.h>
#include "config.h"
#include "com_interface_types.h"


class ComHandler {
    public:
        ComHandler(HardwareSerial &s);
        
        void begin(unsigned long baud = 115200);  // init serial
        void update();                            // call in loop()
        void sendJointStates(const float* q);

        bool cmdReady = false;
        cmd_robot_t cmd_robot;

    private:
        HardwareSerial &stream;

        uint8_t ndx = 0;
        char receivedChars[MAX_MSG_REC_LEN];
        bool receiving = false;
        char START_CHAR = '<';
        char END_CHAR   = '>';
        const char* SEPARATOR  = ",";

        bool parseMessage(); // internal helper
};

#endif // COM_INTERFACE_H
