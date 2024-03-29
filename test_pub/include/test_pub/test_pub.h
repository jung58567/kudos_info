#ifndef TEST_PUB
#define TEST_PUB
/*basic library*/
#include "std_msgs/String.h"
#include "dynamixel_sdk/dynamixel_sdk.h"
#include <pthread.h>
#include <unistd.h>
#include <time.h>
#include <math.h>

#include <sys/mman.h>
#include <signal.h>
#include <iostream>
#include <stdio.h>
#include <sys/time.h>
#include <inttypes.h>
/*#xenomai libary*/
#include <alchemy/task.h>
#include <alchemy/timer.h>
#include <alchemy/mutex.h>
#include <alchemy/sem.h>
#include <boilerplate/trace.h>
#include <xenomai/init.h>

#define ADDR_PRO_TORQUE_ENABLE          64                 // Control table address is different in Dynamixel model
#define ADDR_PRO_GOAL_POSITION          116
#define ADDR_PRO_PRESENT_POSITION       132
#define LEN_PRO_GOAL_POSITION            4
#define LEN_PRO_PRESENT_POSITION         4
#define PROTOCOL_VERSION                2.0                 // See which protocol version is used in the Dynamixel
#define DXL1_ID                         19                   // Dynamixel#1 ID: 1
#define DXL2_ID                         20                   // Dynamixel#2 ID: 2
/*#define DXL1_ID                         17                  // Dynamixel#1 ID: 1
#define DXL2_ID                         11 */                  // Dynamixel#2 ID: 2
#define BAUDRATE                        2000000
#define DEVICENAME                      "/dev/ttyUSB0"
#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque
#define DXL_MOVING_STATUS_THRESHOLD     20                  // Dynamixel moving status threshold
#endif // TEST_THREAD_H