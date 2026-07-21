#pragma once

#include "../Algorithm/alg_pid.hpp"
#include "../Motor/Dji/DjiMotor.hpp"
#include "../BSP/Tools.hpp"
#include "../BSP/Variable.hpp"
namespace STPowerControl
{ 

class PowerUpData_t
{
private:
public:
    float MAXPower;         // 最大允许功率 (W)

    // 功率模型系数（离线拟合）
    float k1;               // 电流相关功率系数
    float k2;               // 转速相关功率系数  
    float k3 ;       // 固定损耗功率系数
    float k4;               // 备用系数
    float k0;               // 备用系数

    float Energy;           // 能量累积值

    float EstimatedPower;   // 功率估算值 (W)
    float Cur_EstimatedPower; // 当前功率估算值 (W)

    float Initial_Est_power[4]; // 初始功率估算数组

    float EffectivePower;   // 有效功率值

    float pMaxPower[4];     // 各电机最大功率分配值
    double Cmd_MaxT[4];     // 各电机最大扭矩指令

    bool Init_flag;         // 初始化完成标志

    // 误差阈值参数
    float E_lower;          // 误差下限阈值
    float E_upper;          // 误差上限阈值

    // 功率控制目标值
    float target_full_power;    // 全功率目标值
    float target_base_power;    // 基础功率目标值

    float cur_power;            // 当前功率值

    // 功率控制PID参数
    float full_kp = 0.1;        // 全功率控制比例系数
    float base_kp = 0.1;        // 基础功率控制比例系数

    float base_Max_power;       // 基础最大功率
    float full_Max_power;       // 全最大功率
    float Single_power[4];  // 新增：存储单个电机的拟合功率
         /**
         * @brief 等比缩放最大分配功率
         * @param pid PID控制器对象数组
         * @param motor 电机对象
         */
        void UpScaleMaxPow(PID *pid);
        
        /**
         * @brief 计算应分配的力矩
         * @param final_Out 最终输出数组
         * @param motor 电机对象
         * @param pid PID控制器对象数组
         * @param toque_const 扭矩常数
         * @param rpm_to_rads RPM转rad/s系数
         */
        void UpCalcMaxTorque(float *final_Out,PID *pid);
        float PowerEstimate_3508(float *final_Out,PID * pid);
    /**
     * @brief 计算3508电机最大扭矩并输出控制电流
     * @param final_Out 输出电流指令数组
     * @param pid PID控制器数组
     */
    void UpCalcMaxTorque_3508(float *final_Out, PID *pid);
    
    /**
     * @brief 计算6020电机最大扭矩并输出控制电流
     * @param final_Out 输出电流指令数组
     * @param pid PID控制器数组
     */
    void UpCalcMaxTorque_6020(float *final_Out, PID *pid);
    /**
     * @brief 能量环控制
     * @detail 实现能量管理和分配
     */
    void EnergyLoop();

    /**
     * @brief 离线拟合功率估算
     * @param current 电流(A)
     * @param speed   转速(rpm)
     * @return 估算功率(W)
     */
    float EstimatePower(float current, float speed)
    {
        // 离线拟合公式：P = k1 * current + k2 * speed + k3
        return k1 * current + k2 * speed + k3;
    }
};

class PowerTask_t
{
public:
    /**
     * @brief 构造函数
     * @detail 初始化轮向和舵向功率控制参数（离线拟合参数）
     */
    PowerTask_t()
    {
        // 轮向电机功率数据初始化（离线拟合参数）
        Wheel_PowerData.MAXPower     = 60;        // 最大功率60W
        Wheel_PowerData.k1           =1.0f;       // 离线拟合电流系数
        Wheel_PowerData.k2           =1.0f;       // 离线拟合转速系数
        Wheel_PowerData.k3           =1.0f;       // 离线拟合固定损耗
        Wheel_PowerData.k4           = 1.0f;
        Wheel_PowerData.k0           = 0.0f;
        Wheel_PowerData.E_upper      = 1000;         // 误差上限
        Wheel_PowerData.E_lower      = 500;          // 误差下限

        // 舵向电机功率数据初始化（离线拟合参数）
        String_PowerData.MAXPower = 60 * 0.6f;       // 最大功率36W（60%）
        String_PowerData.k1       = 0.183f;          // 离线拟合电流系数
        String_PowerData.k2       = 8.78f;           // 离线拟合转速系数
        String_PowerData.k3       = 5.0f;            // 离线拟合固定损耗
        String_PowerData.E_upper  = 500;             // 误差上限
        String_PowerData.E_lower  = 100;             // 误差下限
			// 离线拟合功率数据结构
	//公式: P = k1*T² + k2*w² + T*w + k3*T + k4*w + k0
//R²: 0.873201
//RMSE: 13.939929
//参数: [ 2.44673055  0.01843153 -2.31935427  0.09656956  1.53806005]
           
        // 3508电机功率数据初始化（离线拟合参数）
        T3508_powerdata.k1 =2.44673055f;
        T3508_powerdata.k2 = 0.01843153f;
        T3508_powerdata.k3 =-2.31935427f;
        T3508_powerdata.k4 =0.09656956f;
        T3508_powerdata.k0 =1.53806005;
    }

    PowerUpData_t String_PowerData;  // 舵向电机功率数据
    PowerUpData_t Wheel_PowerData;   // 轮向电机功率数据
    PowerUpData_t T3508_powerdata;

    /**
     * @brief 获取轮向电机估算功率
     * @return 轮向电机功率估算值
     */
    inline float GetEstWheelPow()
    {
        return Wheel_PowerData.EstimatedPower;
    }

    /**
     * @brief 获取舵向电机估算功率
     * @return 舵向电机功率估算值
     */
    inline float GetEstStringPow()
    {
        return String_PowerData.EstimatedPower;
    }

    /**
     * @brief 设置最大功率限制
     * @param maxPower 最大功率值
     */
    inline void setMaxPower(float maxPower)
    {
        Wheel_PowerData.MAXPower  = maxPower;            // 轮向电机功率限制
        String_PowerData.MAXPower = maxPower * 0.4f;     // 舵向电机限制40%的功率上限
    }

    /**
     * @brief 获取最大功率限制
     * @return 最大功率值
     */
    inline uint16_t getMAXPower()
    {
        return Wheel_PowerData.MAXPower;
    }

    /**
     * @brief 更新功率估算
     * @param current 电流(A)
     * @param speed   转速(rpm)
     */
    void UpdateWheelPower(float current, float speed)
    {
        Wheel_PowerData.EstimatedPower = Wheel_PowerData.EstimatePower(current, speed);
    }

    void UpdateStringPower(float current, float speed)
    {
        String_PowerData.EstimatedPower = String_PowerData.EstimatePower(current, speed);
    }
};
// namespace SGPowerControl (瞻顾遗迹，如在昨日，令人长号不自禁)

/**
 * @brief 浮点数相等比较函数
 * @param a 第一个浮点数
 * @param b 第二个浮点数  
 * @return 是否相等（容差1e-5）
 */
static inline bool floatEqual(float a, float b)
{
    return fabs(a - b) < 1e-5f;
}


}//namespace SGPowerControl(瞻顾遗迹，如在昨日，令人长号不自禁)

extern STPowerControl::PowerTask_t PowerControl;