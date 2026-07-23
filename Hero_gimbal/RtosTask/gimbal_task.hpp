#ifndef GIMBAL_TASK_HPP
#define GIMBAL_TASK_HPP

#include "FreeRTOS.h"   // FreeRTOS 核心头文件
#include "queue.h"      // 队列相关类型/函数定义
#include "cmsis_os.h"
#include "../user/core/BSP/Motor/Dji/DjiMotor.hpp"
#include "../user/core/BSP/Motor/LK/Lk_motor.hpp"
#include "../feeder_fsm/feeder_fsm.hpp"
#include "../feeder_fsm/friction_fsm.hpp"
#include "Alg/PID/pid.hpp"
#include "remote_control_task.hpp"
#include "can_send_task.hpp"
#ifdef __cplusplus
extern "C" {
#endif


typedef struct
{
    float angle_deg;      // 实时角度 (度)
    float angle_rad;      // 实时角度 (度)
    float velocity_rpm;   // 输出轴转速 (RPM)
    float velocity_rads;   // 输出轴转速 (RADS)   
    float current_a;      // 实时电流 (A)
    float delta_angle;    // 与上次读取的角度差值 (度)，用于多圈计算
    uint8_t temperature;  // 电机温度
    float multi_angle;    // 计算得到的多圈角度 (度)，需要在 ControlTask 中更新
} DJI3508_State_t;
extern DJI3508_State_t dji3508_state[3]; // 存储三个电机的状态数据


extern BSP::Motor::Dji::GM3508<3> friction_motor; // 电机控制器，初始ID为0x200，发送ID为0x2FF
extern float feeder_current_angle; // 当前角度

// 摩擦轮当前转速 (RPM)，供弹丸速度估算
extern float friction_current_speed_left;
extern float friction_current_speed_right;



void gimbal_task(void *argument);
void DJI3508_feedback();
float hz_to_rotor_angle_per_frame(float fire_hz);

#ifdef __cplusplus
}
#endif

#endif // GIMBAL_TASK_HPP