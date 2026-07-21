#ifndef REMOTE_TASK_HPP
#define REMOTE_TASK_HPP

#include "FreeRTOS.h"   // FreeRTOS 核心头文件
#include "queue.h"      // 队列相关类型/函数定义
#include "cmsis_os.h"
#include "../user/core/BSP/RemoteControl/DT7.hpp"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    float vx = 0.0f; // 前后速度，单位 m/s
    float vy = 0.0f; // 左右速度，单位 m/s
    float wz = 0.0f; // 旋转速度，单位 rad/s
    uint8_t s1 = 0; // 功能按键 S1 状态
    uint8_t s2 = 0; // 功能按键 S2 状态
} RemoteData_t;
extern RemoteData_t remoteData;     // 解析后的遥控器数据结构

typedef struct 
{
 // 加速度，单位 g（1g = 9.8 m/s²）
    float acc_x;
    float acc_y;
    float acc_z;
    // 静止时，Z 轴约等于 1g（重力）
    // 可以用来判断机器人是否在斜坡上
}IMUData_t;
extern IMUData_t imuData;        // 解析后的 IMU 数据结构

typedef struct 
{
 // 加速度，单位 g（1g = 9.8 m/s²）
    float voltage;
    float current;
    float power;
    // 静止时，Z 轴约等于 1g（重力）
    // 可以用来判断机器人是否在斜坡上
}PowerData_t;
extern PowerData_t PowerData;

void remote_task(void *argument);

extern uint8_t receivedata[18]; // 声明全局缓冲区，用于存储接收到的数据
extern uint8_t imu_rx_buffer[64]; // 声明全局缓冲区，用于存储接收到的IMU数据
extern QueueHandle_t remoteDataQueue; // 声明一个全局队列句柄，用于在任务之间传递遥控器数据
extern QueueHandle_t IMUDataQueue;  // 用于传递 IMU 数据的队列

extern BSP::REMOTE_CONTROL::RemoteController remoteController;






#ifdef __cplusplus
}
#endif

#endif // REMOTE_TASK_HPP
