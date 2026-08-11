#ifndef CAN_SEND_TASK_HPP
#define CAN_SEND_TASK_HPP

#include "cmsis_os.h"
#include "remote_control_task.hpp"
#include "Alg/PID/pid.hpp"
#include "boards_communication.hpp"
#include "gimbal_task.hpp"
#include "Alg/Filter/Filter.hpp"
#ifdef __cplusplus
extern "C" {
#endif



typedef struct {
    float angle_deg;      // 实时角度 (度)
    float angle_rad;      // 实时角度 (度)
    float velocity_rpm;   // 输出轴转速 (RPM)
    float velocity_rads;   // 输出轴转速 (RADS)   
    float current_a;      // 实时电流 (A)
    float delta_angle;    // 与上次读取的角度差值 (度)，用于多圈计算
    uint8_t temperature;  // 电机温度

} MG4005_State_t;
extern  MG4005_State_t mg4005_state[2]; // 存储两个电机的状态数据

typedef struct 
{
    float yaw;
    float yaw_360;
    float total_yaw;
    float pitch;
    float pitch_180;
    float roll;
    float gyro_x;
    float gyro_y;
    float gyro_z;
} IMU_t;
extern IMU_t imuData; // 存储解析后的 IMU 数据的结构体

extern RemoteData_t RemoteData;

void can_send_task(void *argument);

extern float yaw_target_angle ;   // 来自遥控器的目标角度
extern float pitch_target_angle ; // 来自遥控器的目标角度
extern float yaw_target_speed ;   // 来自pid计算的目标速度
extern float pitch_target_speed ; // 来自pid计算的目标速度


void vofa_send(float x1, float x2, float x3, float x4, float x5, float x6);


#ifdef __cplusplus
}
#endif

#endif // CAN_SEND_TASK_HPP