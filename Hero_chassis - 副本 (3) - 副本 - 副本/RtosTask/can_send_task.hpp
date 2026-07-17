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


#ifdef __cplusplus
}
#endif

#endif // CAN_SEND_TASK_HPP
