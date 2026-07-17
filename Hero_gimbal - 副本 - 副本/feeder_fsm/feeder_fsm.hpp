#ifndef FEEDER_FSM_HPP
#define FEEDER_FSM_HPP

#include "../user/core/Alg/FSM/alg_fsm.hpp"
#include <math.h>

#define M_PI 3.14159265358979323846

// =============================================================================
// 拨弹轮有限状态机 (Feeder FSM)
// =============================================================================
//
// 功能: 控制英雄机器人拨弹轮电机的射击行为
// 周期: Update() 每 5ms 由 gimbal_task 调用一次
//
// ---- 上游输入 (由 gimbal_task.cpp 提供) ----
// feeder_mode:     遥控器 S1/S2 拨杆组合决定 (STOP / SINGLE / CONTINUOUS)
// trigger_pressed: 扳机信号, 两种来源:
//   - 遥控器滚轮: 单发模式产生单 tick 脉冲 (边缘检测 + armed 标志)
//                 连发模式产生持续电平
//   - 视觉模块:   持续电平 (IsFireCommanded() 为 true 时一直拉高)
//
// ---- 下游输出 (供 gimbal_task.cpp PID 控制层) ----
// control_type:  STOP  → 电机刹车
//                ANGLE → 位置环 PID 追踪目标角度
//                SPEED → 速度环 PID 维持恒定转速
// control_output: 目标角度值 (°) 或 目标速度值 (RPM)
//
// ---- 状态流转图 ----
//
//                    ┌─────────────────────────────┐
//                    │         FEEDER_STOP         │ ← 待命 / 决策中枢
//                    │  连发+扳机 → CONTINUOUS     │
//                    │  反转pending → REVERSE      │
//                    │  单发边缘 → SINGLE_SHOT     │ ← 遥控器脉冲路径
//                    │  单发电平 → SINGLE_SHOT     │ ← 视觉持续路径
//                    └──────┬──────────┬───────────┘
//                           │          │
//              ┌────────────┘          └──────────────┐
//              ▼                                      ▼
//   ┌───────────────────┐               ┌──────────────────────┐
//   │ FEEDER_SINGLE_SHOT│               │FEEDER_CONTINUOUS_SHOT│
//   │  位置控制, 转固定角 │               │  速度控制, 恒速旋转    │
//   │  完成/超时 → COOLDOWN│              │  松扳机 → STOP        │
//   └────────┬──────────┘               └──────────────────────┘
//            │
//            ▼
//   ┌───────────────────────┐          ┌─────────────────────┐
//   │ FEEDER_SINGLE_COOLDOWN│          │ FEEDER_MANUAL_REVERSE│
//   │  电机停转, 计时等待     │          │  位置控制, 反向转动    │
//   │  计时到 / 扳机松 → STOP│          │  完成/超时 → STOP     │
//   └───────────┬───────────┘          └─────────────────────┘
//               │
//               ▼
//           FEEDER_STOP (循环)
//
// ---- 遥控器 vs 视觉 关键区别 ----
// 遥控器单发: trigger 是单 tick 脉冲, COOLDOWN 进入瞬间 trigger 早已
//            回到 NONE, 条件2立即命中 → 直接回 STOP, 无冷却等待
// 视觉单发:   trigger 是持续电平, COOLDOWN 期间一直为 FWD, 只能等
//            条件1 (Count_Time 涨到阈值) → 冷却结束后回 STOP 再打
//
// =============================================================================

// ---- 模式: 由遥控器拨杆决定当前射击模式 ----
enum Enum_Feeder_Mode
{
    FEEDER_MODE_STOP = 0,       // 停止
    FEEDER_MODE_SINGLE,         // 单发模式 (遥控器滚轮脉冲 / 视觉电平)
    FEEDER_MODE_CONTINUOUS,     // 连发模式 (遥控器滚轮持续)
};

// ---- 扳机: 每轮 Update 传入的瞬时扳机状态 ----
enum Enum_Feeder_Trigger
{
    FEEDER_TRIGGER_NONE = 0,    // 无触发
    FEEDER_TRIGGER_FORWARD,     // 正转 (发射)
    FEEDER_TRIGGER_REVERSE,     // 反转 (退弹)
};

// ---- FSM 内部状态 ----
enum Enum_Feeder_Status
{
    FEEDER_STOP = 0,            // 待命: 电机停转, 等待触发
    FEEDER_SINGLE_SHOT,         // 单发: 位置控制, 转动固定角度发射一发
    FEEDER_CONTINUOUS_SHOT,     // 连发: 速度控制, 恒速持续发射
    FEEDER_MANUAL_REVERSE,      // 反转: 位置控制, 反向转动 (退弹/清堵)
    FEEDER_SINGLE_COOLDOWN,     // 冷却: 单发后的间隔等待 (视觉模式核心)
    FEEDER_STATUS_COUNT         // 状态总数
};

// ---- 控制类型: 告诉 PID 层用什么策略 ----
enum Enum_Feeder_Control_Type
{
    FEEDER_CONTROL_STOP = 0,    // 刹车
    FEEDER_CONTROL_SPEED,       // 速度控制
    FEEDER_CONTROL_ANGLE,       // 位置 (角度) 控制
};

/// @brief 拨弹轮 FSM 的原始输入，FSM 内部自行判断模式与触发
typedef struct Struct_Feeder_Input
{
    uint8_t  s1, s2;            // 遥控器拨杆
    bool     friction_on;       // R 键状态 (InputDispatcher)
    bool     is_single_shot;    // T 键状态 (InputDispatcher)  true=单发, false=连发
    bool     fire_triggered;    // 左右键同时按下 (InputDispatcher)
    float    scroll_value;      // 遥控器滚轮值
    bool     vision_fire;       // 视觉开火指令（注意：不直接触发拨弹轮转动，仅滚轮/键鼠按键控制触发）
    bool     is_keymouse;       // 是否键鼠模式 (由 s1/s2 判定)
};

class Class_Feeder_FSM : public Class_FSM
{
public:
    void Init();

    /// @brief 新接口：传入原始输入，FSM 内部根据 s1/s2 自行判断模式与触发
    void Update(const Struct_Feeder_Input &input,
                float feeder_current_angle,
                float current_speed,
                float current_iq);

    // ---- 输出接口 ----
    float   Get_Control_Output();              // 目标值 (° 或 RPM)
    uint8_t Get_Control_Type();                // STOP / SPEED / ANGLE
    float   Get_Accumulated_Angle();           // 累积角度 (多圈, °)
    float   Get_Single_Shot_Target_Angle();    // 单发目标角度
    float   Get_Manual_Reverse_Target_Angle(); // 反转目标角度

private:
    void Update_Accumulated_Angle(float feeder_current_angle);  // 角度累加 + 圈数修正
    bool Is_Single_Shot_Finished()    const;                     // 单发到位判定
    bool Is_Manual_Reverse_Finished() const;                     // 反转到位判定

private:
    // ---- 输出变量 ----
    float   control_output = 0.0f;          // 控制输出值 (角度或速度目标)
    uint8_t control_type   = FEEDER_CONTROL_STOP; // 控制类型

    // ---- 角度追踪 ----
    float raw_angle                   = 0.0f;  // 上一轮原始角度 (°)
    float accumulated_angle           = 0.0f;  // 累积角度 (°), 跨圈累计
    float single_shot_target_angle    = 0.0f;  // 单发目标角度
    float manual_reverse_target_angle = 0.0f;  // 反转目标角度

    // ---- 扳机 / 模式快照 ----
    uint8_t last_trigger_pressed  = 0;         // 上一轮扳机状态 (用于边缘检测)
    uint8_t current_mode          = FEEDER_MODE_STOP;
    uint8_t current_trigger_pressed = 0;

    // ---- 事件挂起标志 (跨状态持久) ----
    uint8_t single_shot_pending   = 0;  // 正转请求: 边缘检测命中后置1, STOP消费后清零
    uint8_t manual_reverse_pending = 0; // 反转请求: 边缘检测命中后置1, STOP消费后清零

    // ---- 状态标志 ----
    uint8_t angle_initialized          = 0; // 角度初始化完成标志
    uint8_t single_shot_target_locked  = 0; // 单发目标角度已锁定 (防止进入时重复计算)
    uint8_t manual_reverse_target_locked = 0; // 反转目标角度已锁定

    // ======================================================================
    // 可调参数 (改动这些值来调整射击行为)
    // ======================================================================

    // 单发弹丸对应的电机轴转动角度 (°)
    // 计算: 拨弹轮 1/8 圈 = 45°, 减速比 2.75×19 = 52.25
    // 45° × 减速比 / 外传比 = ... 当前值: -3060
    static constexpr float SINGLE_SHOT_ANGLE = -(60.0 * 51.0f);

    // 单发到位判定阈值 (°) — 当前角度与目标角度的误差小于此值即认为完成
    // PID 调好后可收紧到 10~30
    static constexpr float SINGLE_SHOT_FINISH_THRESHOLD = 250.0f;

    // 角度跨圈修正阈值 — 相邻两次读数差值超过此值认为发生了 ±360° 跳变
    static constexpr float ANGLE_WRAP_THRESHOLD = 180.0f;

    // 完整一圈 = 360°
    static constexpr float FULL_ROTATION_ANGLE = 360.0f;

    // 连发模式恒定转速 (RPM 对应电机轴, ×10 换算)
    static constexpr float FORWARD_SPEED = (50.0f * 10.0f);

    // ---- 超时保护 (防止堵转卡死) ----
    static constexpr uint32_t SINGLE_SHOT_TIMEOUT_COUNT   = 2000; // 单发超时 (ticks)
    static constexpr uint32_t MANUAL_REVERSE_TIMEOUT_COUNT = 2000; // 反转超时 (ticks)

    // ---- 单发冷却间隔 (视觉模式核心参数) ----
    // 单位: 控制周期 ticks (1 tick = 5ms, 由 gimbal_task 的 vTaskDelay(5) 决定)
    // 换算: 1000 ticks = 5s, 200 ticks = 1s, 40 ticks = 200ms
    // 视觉持续开火时, 每发弹丸之间等待此时间
    static constexpr uint32_t SINGLE_SHOT_COOLDOWN_TICKS = 1000;
};

#endif
