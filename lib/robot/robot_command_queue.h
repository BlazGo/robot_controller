#ifndef ROBOT_COMMAND_H
#define ROBOT_COMMAND_H

#include "robot_commands.h"

#define ROBOT_CMD_QUEUE_LEN 8

typedef struct {
    RobotCommand buffer[ROBOT_CMD_QUEUE_LEN];
    uint8_t write_idx;
    uint8_t read_idx;
} RobotCommandQueue;
 
extern RobotCommandQueue g_robot_cmd_queue;

void robotCommandQueueInit(void);
bool robotCommandPush(const RobotCommand *cmd); // producer (core1)
bool robotCommandPop(RobotCommand *cmd);        // consumer (core0)
uint8_t _queueNextIndex(uint8_t idx);

#endif // ROBOT_COMMAND_H
