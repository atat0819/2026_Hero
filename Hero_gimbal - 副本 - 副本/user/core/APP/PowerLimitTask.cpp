#include "../Referee/RM_RefereeSystem.h"
#include "../Motor/Dji/DjiMotor.hpp"
#include "PowerLimitTask.hpp"

using namespace STPowerControl;

// 全局功率控制对象实例化
STPowerControl::PowerTask_t PowerControl;
/**
 * @brief 等比缩放最大分配功率
 * @param pid PID控制器数组
 * @param motor 电机对象
 * @detail 根据各电机的误差比例分配最大功率限制
 */
void PowerUpData_t::UpScaleMaxPow(PID *pid)
{
    // 计算所有电机PID误差的绝对值总和
    float sumErr = 0.0f;
    for (int i = 0; i < 4; i++) {
        sumErr += fabsf(pid[i].GetErr());
    }

    // 根据误差比例分配各电机的最大功率
    for (int i = 0; i < 4; i++) {
        pMaxPower[i] = MAXPower * (fabsf(pid[i].GetErr()) / sumErr);
        if (pMaxPower[i] < 0) {
            continue;
        }
    }
}
/**
 * @brief 能量环控制
 * @detail 根据电压状态动态调整最大功率限制
 */
void PowerUpData_t::EnergyLoop()
{
    // 计算基础功率和全功率的电压误差（使用平方根线性化）
   // float base_err = sqrt(target_base_power) - sqrt(BSP::Power::pm01.cout_voltage);
  //  float full_err = sqrt(target_full_power) - sqrt(BSP::Power::pm01.cout_voltage);

    // 计算基础最大功率和全最大功率（考虑误差补偿）
   // base_Max_power = fmax(ext_game_robot_status_0x0201.chassis_power_limit - base_err * base_kp, 15.0f);
   //full_Max_power = fmax(ext_game_robot_status_0x0201.chassis_power_limit - full_err * full_kp, 15.0f);
}

/**
 * @brief 计算应分配的最大扭矩
 * @param final_Out 最终输出数组
 * @param motor 电机对象
 * @param pid PID控制器数组
 * @param toque_const 扭矩常数
 * @param rpm_to_rads RPM转rad/s系数
 * @detail 当估算功率超过限制时，通过二次方程求解限制后的最大扭矩
 */
//void PowerUpData_t::UpCalcMaxTorque(float *final_Out,PID *pid)
//{
//    // 更新能量环状态
//    EnergyLoop();

//    // 电压高于24V×0.9时使用全功率模式
//    if (BSP::Power::pm01.cout_voltage > 24 * 0.9) {
//        MAXPower = full_Max_power;
//    }

//    // 钳位最大功率在基础最大功率和全最大功率之间
//    MAXPower =Tools.clamp(MAXPower, full_Max_power, base_Max_power);

//    // 如果估算功率超过最大允许功率，进行功率限制
//    if (EstimatedPower > MAXPower) {
//        for (int i = 0; i < 4; i++) {
//            // 获取电机角速度
//            float omega = motor.GetEquipData_for(i) * rpm_to_rads;
//            // 构建二次方程系数：A×T² + B×T + C = 0
//            float A = k2;                                  // 二次项系数
//            float B = omega;                               // 一次项系数
//            float C = k1 * fabs(omega) + k3 / 4 - pMaxPower[i]; // 常数项

//            // 计算判别式
//            float delta = (B * B) - 4.0f * A * C;

//            // 判别式小于等于0时使用简化解
//            if (delta <= 0) {
//                Cmd_MaxT[i] = -B / (2.0f * A) / toque_const;
//            }

//            // 根据PID输出方向选择不同的根
//            Cmd_MaxT[i] = pid[i].GetCout() > 0.0f ? (-B + sqrtf(delta)) / (2.0f * A) / toque_const
//                                                  : (-B - sqrtf(delta)) / (2.0f * A) / toque_const;
//          
//            // 钳位扭矩指令在安全范围内
//            Cmd_MaxT[i] = Tools.clamp(Cmd_MaxT[i], 16384.0f, -16384.0f);

//            // 更新最终输出
//            final_Out[i] = Cmd_MaxT[i];
//        }
//    }
//}

//一个简易的3508功率测试
float PowerUpData_t::PowerEstimate_3508(float *final_Out,PID * pid)
{   
    using namespace BSP::Motor::Dji;
    // 获取电机角速度
   // Motor3508.getVelocityRads(1);
    //k1*T方 + 有效功率 + k2*w方 + k3*T + k4*w =power 
    float T2 =  Motor3508.getTorque(1) *  Motor3508.getTorque(1);   
    float W2 =  Motor3508.getVelocityRads(1) * Motor3508.getVelocityRads(1);
    float InitialPower =  Motor3508.getTorque(1) * Motor3508.getVelocityRads(1);

    float T = Motor3508.getTorque(1);
    float W = Motor3508.getVelocityRads(1);

    float A  = PowerControl.T3508_powerdata.k1* T2 +
                                  PowerControl.T3508_powerdata.k2* W2 +
                                                                      InitialPower;//二次项
    float B  = PowerControl.Wheel_PowerData.k3 *T +
                             PowerControl.T3508_powerdata.k4 * W +
                                                        PowerControl.T3508_powerdata.k0 ;//一次项

    float Finalpower = A + B;                                                    
   return Finalpower;
}
// PowerLimitTask.cpp
/// PowerLimitTask.cpp
void PowerUpData_t::UpCalcMaxTorque_3508(float *final_Out, PID *pid)
{
    using namespace BSP::Motor::Dji;
    
    // 首先估算各电机功率并存储
    float total_power_estimate = 0.0f;
    for(int i = 0; i < 4; i++) {
        float omega = Motor3508.getVelocityRads(i+1);
        float torque = Motor3508.getTorque(i+1);
        
        // 计算单个电机的拟合功率并存储
        Single_power[i] = k1 * torque * torque + 
                         torque * omega + 
                         k2 * omega * omega + 
                         k3 * torque + 
                         k4 * omega + 
                         k0;
        total_power_estimate += Single_power[i];
    }
    
    // 如果估算功率超过最大允许功率，进行功率重分配
    if(total_power_estimate > MAXPower) {
        // 先等比分配各电机最大功率
        UpScaleMaxPow(pid);
        
        float toque_const = 1.0f / Motor3508.params_.current_to_torque_coefficient;
        
        for(int i = 0; i < 4; i++) {
            float omega = Motor3508.getVelocityRads(i+1);
            float A = k1;
            float B = omega + k3;
            float C = k2 * omega * omega + k4 * omega + k0 - pMaxPower[i];
            float delta = (B * B) - 4.0f * A * C;
            
            float T_phys;
            if (delta <= 0) {
                T_phys = -B / (2.0f * A);
            } else {
                if (pid[i].GetCout() > 0.0f) {
                    T_phys = (-B + sqrtf(delta)) / (2.0f * A);
                } else {
                    T_phys = (-B - sqrtf(delta)) / (2.0f * A);
                }
            }
            
            float temp_cmd = T_phys * toque_const;
            temp_cmd = Tools.clamp(temp_cmd, 16384.0f, -16384.0f);
            final_Out[i] = static_cast<float>(static_cast<int16_t>(temp_cmd));
            
            // 存储最大允许扭矩指令（控制电流值）
            Cmd_MaxT[i] = static_cast<double>(temp_cmd);
        }
    } else {
        // 如果没有超过功率限制，使用PID原始输出
        for(int i = 0; i < 4; i++) {
            float pid_out = pid[i].GetCout();
            final_Out[i] = pid_out;
            
            // 也需要更新Cmd_MaxT，存储PID输出（这是最大允许的扭矩指令）
            Cmd_MaxT[i] = static_cast<double>(pid_out);
        }
    }
}

void PowerUpData_t::UpCalcMaxTorque_6020(float *final_Out, PID *pid)
{
    using namespace BSP::Motor::Dji;
    
    // 注意：6020应该使用其特有的功率模型系数
    // 这里假设使用相同的变量名，但实际值应该不同
    
    // 首先估算各电机功率并存储
    float total_power_estimate = 0.0f;
    for(int i = 0; i < 4; i++) {
        float omega = Motor6020.getVelocityRads(i+1);
        float torque = Motor6020.getTorque(i+1);
        
        // 计算单个电机的拟合功率并存储
        // 注意：这里应该使用6020特有的k1-k4系数
        Single_power[i] = k1 * torque * torque + 
                         torque * omega + 
                         k2 * omega * omega + 
                         k3 * torque + 
                         k4 * omega + 
                         k0;
        total_power_estimate += Single_power[i];
    }
    
    // 如果估算功率超过最大允许功率，进行功率重分配
    if(total_power_estimate > MAXPower) {
        // 先等比分配各电机最大功率
        UpScaleMaxPow(pid);
        
        float toque_const = 1.0f / Motor6020.params_.current_to_torque_coefficient;
        
        for(int i = 0; i < 4; i++) {
            float omega = Motor6020.getVelocityRads(i+1);
            float A = k1;
            float B = omega + k3;
            float C = k2 * omega * omega + k4 * omega + k0 - pMaxPower[i];
            float delta = (B * B) - 4.0f * A * C;
            
            float T_phys;
            if (delta <= 0) {
                T_phys = -B / (2.0f * A);
            } else {
                if (pid[i].GetCout() > 0.0f) {
                    T_phys = (-B + sqrtf(delta)) / (2.0f * A);
                } else {
                    T_phys = (-B - sqrtf(delta)) / (2.0f * A);
                }
            }
            
            float temp_cmd = T_phys * toque_const;
            temp_cmd = Tools.clamp(temp_cmd, 16384.0f, -16384.0f);
            final_Out[i] = static_cast<float>(static_cast<int16_t>(temp_cmd));
            
            // 存储最大允许扭矩指令（控制电流值）
            Cmd_MaxT[i] = static_cast<double>(temp_cmd);
        }
    } else {
        // 如果没有超过功率限制，使用PID原始输出
        for(int i = 0; i < 4; i++) {
            float pid_out = pid[i].GetCout();
            final_Out[i] = pid_out;
            
            // 也需要更新Cmd_MaxT，存储PID输出
            Cmd_MaxT[i] = static_cast<double>(pid_out);
        }
    }
}