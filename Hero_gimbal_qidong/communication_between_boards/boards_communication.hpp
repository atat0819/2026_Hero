#ifndef BOARDS_COMMUNICATION_HPP
#define BOARDS_COMMUNICATION_HPP

#include "stm32f4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

#define YAW_OFFSET_CAN2_STD_ID 0x301

void YawOffset_SetZero(void);
float YawOffset_GetDeg(void);
HAL_StatusTypeDef YawOffset_SendToCan2(void);
HAL_StatusTypeDef CAN2_SendChassisSpeed(float chassis_vx, float chassis_vy);
HAL_StatusTypeDef CAN2_Send_S1andS2_Status(uint8_t s1, uint8_t s2);
HAL_StatusTypeDef CAN2_SendGimbalIMU_Raw(float yaw, float gyro_z);
HAL_StatusTypeDef CAN2_SendKeyboard(uint16_t keyboard);


#ifdef __cplusplus
}
#endif

#endif // BOARDS_COMMUNICATION_HPP
