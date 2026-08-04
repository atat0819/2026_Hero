#ifndef IR_SENSOR_HPP
#define IR_SENSOR_HPP

#include "main.h"
#include <stdint.h>

// =============================================================================
// 红外对射传感器驱动 (IR Sensor)
// =============================================================================
//
// 功能: 检测弹丸是否进入进弹口, 作为吊射送弹的触发信号
// 电平约定: 有弹(遮挡光路) = 高电平, 无弹 = 低电平
// 防抖: 连续 IR_DEBOUNCE_TICKS 次采样一致才翻转状态,
//       避免弹丸在进弹口抖动导致推杆误动作
//
// ---- 引脚 ----
// PH11 (GPIOH, GPIO_PIN_11) — 已在 CubeMX 配置: 输入 + 下拉
// (下拉: 传感器未接线/无弹时读低, 故障安全)
//
// ---- 使用示例 ----
//   ir_sensor.Init();        // 上电初始化
//   ir_sensor.Update();      // 每控制周期(5ms)调用一次, 内部防抖
//   ir_sensor.Has_Ball();    // 防抖后: 进弹口是否有弹
// =============================================================================

class Class_IR_Sensor
{
public:
    void Init();                  // 配置引脚为输入并清零防抖状态
    void Update();                // 每周期采样一次, 软件防抖滤波
    bool Has_Ball() const;        // 防抖后: 进弹口有弹
    bool Get_Raw_Level() const;   // 防抖前原始电平 (诊断用)

private:
    // 防抖参数: 连续采样一致次数 (1 tick = 5ms, 4次 = 20ms)
    static constexpr uint8_t IR_DEBOUNCE_TICKS = 4U;

    uint8_t stable_count_ = 0U;  // 连续一致的采样计数
    bool    has_ball_     = false; // 防抖后的有弹状态
    bool    raw_level_    = false; // 最近一次原始电平
};

extern Class_IR_Sensor ir_sensor;  // 全局实例, 挂在 PH11

#endif
