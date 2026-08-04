#ifndef GIMBAL_TASK_HPP
#define GIMBAL_TASK_HPP

#include "FreeRTOS.h"   // FreeRTOS 核心头文件
#include "queue.h"      // 队列相关类型/函数定义
#include "cmsis_os.h"
#include "../user/core/BSP/Motor/Dm/DmMotor.hpp"
#include "../user/core/BSP/Motor/LK/Lk_motor.hpp"
#include "../feeder_fsm/feeder_fsm.hpp"
#include "Alg/PID/pid.hpp"
#include "remote_control_task.hpp"
#include "can_send_task.hpp"
#ifdef __cplusplus
extern "C" {
#endif


// DM4310 电机状态结构体
typedef struct
{
    float angle_deg;      // 实时角度 (度), 0~360° 输出轴
    float angle_rad;      // 实时角度 (rad), 0~2π 输出轴
    float velocity_rpm;   // 输出轴转速 (RPM)
    float velocity_rads;  // 输出轴转速 (rad/s)
    float current_a;      // 转矩 (Nm), DM电机反馈的是力矩而非电流
    float multi_angle;    // 多圈累积角度 (度), 输出轴坐标
    float multi_rad;      // 多圈累积角度 (rad)
    uint8_t temperature;  // 电机温度
} DM4310_State_t;
extern DM4310_State_t dm4310_state[1]; // 存储 DM4310 电机的状态数据

extern BSP::Motor::DM::J4310<1> dm666; // DM4310 电机控制器
extern float feeder_current_angle; // 当前角度


void gimbal_task(void *argument);
void DM4310_feedback();
float hz_to_rotor_angle_per_frame(float fire_hz);

#ifdef __cplusplus
}
#endif

#endif // GIMBAL_TASK_HPP