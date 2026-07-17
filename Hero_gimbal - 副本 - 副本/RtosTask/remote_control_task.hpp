#ifndef REMOTE_CONTROL_TASK_HPP
#define REMOTE_CONTROL_TASK_HPP

#include "FreeRTOS.h"   // FreeRTOS 核心头文件
#include "queue.h"      // 队列相关类型/函数定义
#include "cmsis_os.h"
#include "../user/core/BSP/RemoteControl/DT7.hpp"
#include "BSP/IMU/HI12_imu.hpp"
#include "HI14.hpp" // 包含 HI14 的头文件，获取陀螺仪数据结构定义
#include "../user/core/BSP/version/vision_communication.hpp"


#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    float chassis_vx;
    float chassis_vy;
    float gimbal_yaw;
    float gimbal_pitch;
    float gimbal_roll;
    int16_t mouse_x;
    int16_t mouse_y;
    int16_t mouse_z;
    uint8_t mouse_left;
    uint8_t mouse_right;
    uint16_t keyboard;
    uint8_t s1;
    uint8_t s2;

} RemoteData_t;
extern RemoteData_t remote;



void remote_control_task(void *argument);


extern QueueHandle_t remoteDataQueue; // 声明一个全局队列句柄，用于在任务之间传递遥控器数据

extern QueueHandle_t IMUDataQueue; // 用于传递 IMU 数据的队列


extern BSP::REMOTE_CONTROL::RemoteController remoteController;
extern BSP::IMU::HI12_float imu;



#ifdef __cplusplus
}
#endif

extern BSP::Vision::VisionCommunicator vision_comm;

#endif // REMOTE_CONTROL_TASK_HPP