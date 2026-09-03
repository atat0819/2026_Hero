#ifndef CAN_SEND_TASK_HPP
#define CAN_SEND_TASK_HPP

#include "cmsis_os.h"
#include "HAL/CAN/interface/can_device.hpp"

#ifdef __cplusplus
extern "C" {
#endif

void can_send_task(void *argument);
void CAN2_RxCallback(HAL::CAN::Frame& frame);

typedef struct
{
    float yaw_offset_deg; // 云台偏移量
    float vx;
    float vy;
    float s1;
    float s2;

} Gimbal_Chassis_communicate_t;

extern float yaw_offset_deg;
extern bool yaw_offset_updated;

extern Gimbal_Chassis_communicate_t gimbalChassis_communicate;
extern uint8_t gimbalChassisSpeedUpdated;

// 云台发送的键盘位掩码，供底盘控制任务按位判断按键状态
extern volatile uint16_t gimbal_keyboard;
extern volatile uint32_t gimbal_keyboard_last_tick;
extern volatile bool gimbal_keyboard_received;


#ifdef __cplusplus
}
#endif

#endif // CAN_SEND_TASK_HPP
