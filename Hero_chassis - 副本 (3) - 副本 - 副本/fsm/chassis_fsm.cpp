/**
 * @file chassis_fsm.cpp
 * @brief 底盘状态机实现
 */

#include "chassis_fsm.hpp"
#include <math.h>

void Chassis_FSM::Init()
{
    Class_FSM::Init(CHASSIS_MODE_COUNT, CHASSIS_STOP);
    current_mode = CHASSIS_STOP;
    follow_pid.reset();
}

/**
 * @brief 拨杆值 -> 模式映射（逐挡位列出）
 *
 *   s1=上, s2=上 -> FOLLOW_GIMBAL（跟随）
 *   s1=上, s2=中 -> FOLLOW_GIMBAL（跟随）
 *   s1=上, s2=下 -> FOLLOW_GIMBAL（跟随）
 *   s1=中, s2=上 -> NOT_FOLLOW（不跟随）
 *   s1=中, s2=中 -> NOT_FOLLOW（不跟随）
 *   s1=中, s2=下 -> GYRO_SPIN（小陀螺）
 *   s1=下, s2=上 -> GYRO_SPIN（小陀螺）
 *   s1=下, s2=中 -> FOLLOW_GIMBAL（跟随）
 *   s1=下, s2=下 -> STOP（停止）
 *   离线         -> STOP（停止）
 */
void Chassis_FSM::StateUpdate(uint8_t s1, uint8_t s2, bool equipment_online)
{
    Enum_Chassis_Mode target_mode;

    // 离线 -> 停止
    if (!equipment_online)
    {
        target_mode = CHASSIS_STOP;
    }
    // s1上, s2上 -> 跟随
    else if (s1 == 1 && s2 == 1)
    {
        target_mode = CHASSIS_FOLLOW_GIMBAL;
    }
    // s1上, s2中 -> 跟随
    else if (s1 == 1 && s2 == 3)
    {
        target_mode = CHASSIS_FOLLOW_GIMBAL;
    }
    // s1上, s2下 -> 跟随
    else if (s1 == 1 && s2 == 2)
    {
        target_mode = CHASSIS_FOLLOW_GIMBAL;
    }
    // s1中, s2上 -> 不跟随
    else if (s1 == 3 && s2 == 1)
    {
        target_mode = CHASSIS_NOT_FOLLOW;
    }
    // s1中, s2中 -> 不跟随
    else if (s1 == 3 && s2 == 3)
    {
        target_mode = CHASSIS_NOT_FOLLOW;
    }
    // s1中, s2下 -> 小陀螺
    else if (s1 == 3 && s2 == 2)
    {
        target_mode = CHASSIS_GYRO_SPIN;
    }
    // s1下, s2上 -> 小陀螺
    else if (s1 == 2 && s2 == 1)
    {
        target_mode = CHASSIS_GYRO_SPIN;
    }
    // s1下, s2中 -> 跟随
    else if (s1 == 2 && s2 == 3)
    {
        target_mode = CHASSIS_FOLLOW_GIMBAL;
    }
    // s1下, s2下 -> 停止
    else if (s1 == 2 && s2 == 2)
    {
        target_mode = CHASSIS_STOP;
    }
    // 其他情况 -> 停止
    else
    {
        target_mode = CHASSIS_STOP;
    }

    // ---- 小陀螺延时逻辑：拨到小陀螺后必须保持2秒不动才真正切换 ----
    if (target_mode == CHASSIS_GYRO_SPIN)
    {
        if (!gyro_spin_pending)
        {
            // 第一次请求小陀螺，开始计时，保持当前模式
            gyro_spin_pending = true;
            gyro_spin_request_tick = HAL_GetTick();
            return;
        }

        // 已在等待中，检查是否到时间
        if (HAL_GetTick() - gyro_spin_request_tick < GYRO_SPIN_DELAY_MS)
        {
            return; // 没到2秒，继续等
        }
        // 到时间了，fall through 执行切换
    }
    else
    {
        // 目标不是小陀螺，清除等待状态
        // 拨杆中途切走再切回来会重新开始计时
        gyro_spin_pending = false;
    }

    if (target_mode != current_mode)
    {
        Set_Status(static_cast<uint8_t>(target_mode));
        current_mode = target_mode;
        follow_pid.reset();
    }
}

/**
 * @brief 核心输出：根据当前模式返回旋转角速度指令
 *
 * - FOLLOW_GIMBAL: 使用 ALG::PID::PID 计算跟随输出（带死区）
 * - GYRO_SPIN:    返回固定自旋速度
 * - 其他:          返回 0
 */
float Chassis_FSM::Get_wz_cmd(float yaw_offset_rad)
{
    switch (current_mode)
    {
    case CHASSIS_FOLLOW_GIMBAL:
    {

        // 归一化到 [-π, π]
        while (yaw_offset_rad >  3.14159265f) 
				{
					yaw_offset_rad -= 2 * 3.14159265f;
				}
        while (yaw_offset_rad < -3.14159265f)
				{
					yaw_offset_rad += 2 * 3.14159265f;
				}

        // 偏移超过 ±90°，说明后方更近，翻转 180° 跟随后方
        if (yaw_offset_rad >  1.5707963267f)
				{
            yaw_offset_rad -= 3.14159265f;
				}
        else if (yaw_offset_rad < -1.5707963267f)
				{yaw_offset_rad += 3.14159265f;
				}
        if (fabsf(yaw_offset_rad) < DEADZONE)
        {
            follow_pid.reset();
            return 0.0f;
        }

        // UpDate(target, feedback): 目标是0偏移，反馈是当前偏移
        float output = follow_pid.UpDate(yaw_offset_rad, 0.0f);

        // 限速：钳位到 [-max_follow_speed, max_follow_speed]
        if (output >  max_follow_speed) 
				{
				output =  max_follow_speed;
				}
        if (output < -max_follow_speed) 
				{
				output = -max_follow_speed;
				}

        return output;
    }
    case CHASSIS_GYRO_SPIN:
        return gyro_spin_speed;

    case CHASSIS_STOP:
        return 0.0f;
    case CHASSIS_NOT_FOLLOW:
        return 0.0f;
    default:
        return 0.0f;
    }
}

void Chassis_FSM::TIM_Calculate_PeriodElapsedCallback()
{
    Class_FSM::TIM_Calculate_PeriodElapsedCallback();
}
