#include "robot_command_queue.h"

RobotCommandQueue g_robot_cmd_queue;

uint8_t _queueNextIndex(uint8_t idx) {
    // add one to idx and if its over the length it loops around
    return (uint8_t) (( idx + 1u ) % ROBOT_CMD_QUEUE_LEN);
}

void robotCommandQueueInit(void) {
    g_robot_cmd_queue.read_idx = 0;
    g_robot_cmd_queue.write_idx = 0;
}

bool robotCommandPush(const RobotCommand *cmd) {
    if (cmd == nullptr){
        return false;
    }

    uint8_t write_idx = g_robot_cmd_queue.write_idx;
    uint8_t next = _queueNextIndex(write_idx);
    uint8_t read_idx = g_robot_cmd_queue.read_idx;
    
    if (next == read_idx) {
        return false;
    }
    g_robot_cmd_queue.buffer[write_idx] = *cmd;
    g_robot_cmd_queue.write_idx = next;
    return true;
}

bool robotCommandPop(RobotCommand *cmd) {
    if (cmd == nullptr){
        return false;
    }

    uint8_t read_idx = g_robot_cmd_queue.read_idx;
    uint8_t write_idx = g_robot_cmd_queue.write_idx;

    if (read_idx == write_idx) {
        return false;
    }

    // read the command with current index and
    *cmd = g_robot_cmd_queue.buffer[read_idx];
    // move the read idx to next position
    g_robot_cmd_queue.read_idx = _queueNextIndex(read_idx);
    return true;
}


