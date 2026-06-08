#ifndef MAIN_H
#define MAIN_H

#include "robot_commands.h"

#define ROBOT_CMD_QUEUE_LEN 8

typedef struct RobotCommandQueue
{
    RobotCommand buffer[ROBOT_CMD_QUEUE_LEN];
};
 
extern RobotCommandQueue g_robot_cmd_queue;

void robotCommandQueueInit(void);
bool robotCommandPush(const RobotCommand *cmd); // producer (core1)
bool robotCommandPop(RobotCommand *cmd);        // consumer (core0)

#endif // MAIN_H
