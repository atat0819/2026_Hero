#ifndef SOLENOID_VALVE_HPP
#define SOLENOID_VALVE_HPP

#include "main.h"

// =============================================================================
// 电磁阀驱动 (Solenoid Valve Driver)
// =============================================================================
//
// 功能: 通过 GPIO 高低电平控制电磁阀开/关, 用于发射弹丸 (替代摩擦轮方案)
// 电平约定: 高电平 = 开阀, 低电平 = 关阀
// 时序管理: 纯电平控制, 开/关持续时间由调用方 (FSM/任务) 负责
//
// ---- 引脚分配 ----
// solenoid_valve_1: PI0  (GPIOI, GPIO_PIN_0)
// solenoid_valve_2: PH12 (GPIOH, GPIO_PIN_12)
//
// ---- 使用示例 ----
//   solenoid_valve_1.Init();    // 上电初始化 (强制关阀)
//   solenoid_valve_1.Open();    // 开阀
//   solenoid_valve_2.Close();   // 关阀
//   solenoid_valve_1.Is_Open(); // 查询状态
// =============================================================================

class Class_Solenoid_Valve
{
public:
    Class_Solenoid_Valve(GPIO_TypeDef *port, uint16_t pin);

    void Init();          // 上电安全: 强制关阀 (低电平)
    void Open();          // 引脚置高 = 开阀
    void Close();         // 引脚置低 = 关阀
    bool Is_Open() const; // 查询当前状态

private:
    GPIO_TypeDef *port_;
    uint16_t pin_;
    bool is_open_ = false;
};

// ---- 全局实例 (定义在 solenoid_valve.cpp) ----
extern Class_Solenoid_Valve solenoid_valve_1; // PI0
extern Class_Solenoid_Valve solenoid_valve_2; // PH12

#endif
