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
// trigger_pressed: 扳机信号, 来源:
//   - 遥控器滚轮: 单发模式产生单 tick 脉冲 (边缘检测 + armed 标志)
//                 连发模式产生持续电平
// 注意: 视觉端 fire 指令不参与触发 (设计上丢弃, 开火只认滚轮/键鼠按键)
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
// =============================================================================

// ---- 模式: 由遥控器拨杆决定当前射击模式 ----
enum Enum_Feeder_Mode
{
    FEEDER_MODE_STOP = 0,       // 停止
    FEEDER_MODE_SINGLE,         // 单发模式 (遥控器滚轮脉冲)
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
    bool     is_single_shot;    // G 键状态 (InputDispatcher)  true=单发, false=连发（DBUS 无 T 键）
    bool     fire_triggered;    // 左右键同时按下 (InputDispatcher)
    float    scroll_value;      // 遥控器滚轮值
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
    void Update_Accumulated_Angle(float feeder_current_angle);  // 接收电机层多圈角度
    float Calculate_Single_Shot_Target() const;                 // 当前角度 + 单发角度 + 小范围相位修正
    bool Is_Single_Shot_Finished(float current_speed);   // 单发到位判定(误差+转速双条件 / 卡止停稳早退)
    bool Is_Manual_Reverse_Finished(float current_speed) const;  // 反转到位判定(误差+转速双条件)

private:
    // ---- 输出变量 ----
    float   control_output = 0.0f;          // 控制输出值 (角度或速度目标)
    uint8_t control_type   = FEEDER_CONTROL_STOP; // 控制类型

    // ---- 角度追踪 ----
    float raw_angle                   = 0.0f;  // 上一轮原始角度 (°)
    float accumulated_angle           = 0.0f;  // 累积角度 (°), 跨圈累计
    float single_shot_target_angle    = 0.0f;  // 单发目标角度
    float manual_reverse_target_angle = 0.0f;  // 反转目标角度
    float single_shot_phase_correction = 0.0f; // 上一发小误差补偿, 超限则清零

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
    uint8_t single_shot_stall_ticks    = 0; // 卡止早退: 转子低速连续周期计数
    uint8_t manual_reverse_target_locked = 0; // 反转目标角度已锁定

    // ======================================================================
    // 可调参数 (改动这些值来调整射击行为)
    // ======================================================================

    // 单发弹丸对应的电机轴转动角度 (°)
    // 计算: 拨弹轮 1/6 圈 = 60° (6 槽, 每槽 1 颗弹), 减速比 51
    // 60° × 51 = 3060 = 正好推进 1 颗弹
    static constexpr float SINGLE_SHOT_ANGLE = -(60.0 * 51.0f);
    //static constexpr float SINGLE_SHOT_ANGLE = -(36.0 * 1.66f);

    // 手动反转/退弹行程 (电机轴 °) — 方向与发射相反(+)，量远小于一发行程
    // 机械限制：拨弹轮轴只能退 4°，4° × 减速比 51 = 204°
    static constexpr float MANUAL_REVERSE_ANGLE = (4.0f * 51.0f);

    // 单发到位判定阈值 (°) — 误差小于此值 且 转速低于 FINISH_SPEED 才认为完成
    // 只判误差会在大转速下误判"到位"（250°阈值时退出转速平均-1400RPM），
    // 双条件保证转子真正停下来才进冷静期，抱角只做巩固不做补救
    static constexpr float SINGLE_SHOT_FINISH_THRESHOLD = 50.0f;

    // 单发到位判定转速阈值 (RPM) — |转速|低于此值才认为到位
    static constexpr float SINGLE_SHOT_FINISH_SPEED_RPM = 300.0f;

    // 卡止早退: 转子停死(低速持续N周期)但误差仍超窗 → 视为到位
    // 处理"转子停在误差阈值边缘(如53° > 50°)却要等满1s超时"的情况:
    // 弹丸实际已推出, 等待无意义, 直接进冷却并记录残差给相位修正
    static constexpr float SINGLE_SHOT_STALL_SPEED_RPM = 50.0f; // 停死判定转速 (RPM)
    static constexpr uint8_t SINGLE_SHOT_STALL_TICKS = 10;      // 连续低速周期数 (10×5ms=50ms)

    // ROLLBACK_MARKER_SINGLE_SHOT_PHASE_CORRECTION_BEGIN
    // 上一发最终误差绝对值不超过该转子角时, 下一发反向小修正; 超过则认为异常, 不补偿。
    static constexpr float SINGLE_SHOT_PHASE_CORRECTION_LIMIT = 100.0f;
    // ROLLBACK_MARKER_SINGLE_SHOT_PHASE_CORRECTION_END

    // 连发模式恒定转速 (RPM 对应电机轴, ×10 换算)
    static constexpr float FORWARD_SPEED = (50.0f * 10.0f);

    // ---- 超时保护 (防止堵转卡死) ----
    static constexpr uint32_t SINGLE_SHOT_TIMEOUT_COUNT   = 200; // 单发超时 (ticks)
    static constexpr uint32_t MANUAL_REVERSE_TIMEOUT_COUNT = 200; // 反转超时 (ticks)

    // ---- 单发冷却间隔 ----
    // 单位: 控制周期 ticks (1 tick = 5ms, 由 gimbal_task 的 vTaskDelay(5) 决定)
    // 换算: 1000 ticks = 5s, 200 ticks = 1s, 40 ticks = 200ms
    // 遥控器滚轮单发后, 距下一次单发的最小间隔
    static constexpr uint32_t SINGLE_SHOT_COOLDOWN_TICKS = 1000;

    // ROLLBACK_MARKER_SINGLE_COOLDOWN_HOLD_BEGIN
    // 若抱角刹车效果不好，回退时删除本常量，并把 FEEDER_SINGLE_COOLDOWN
    // 恢复为 FEEDER_CONTROL_STOP + control_output = 0.0f。
    static constexpr uint32_t SINGLE_SHOT_REMOTE_HOLD_TICKS = 30; // 遥控器单发最短抱角刹车 150ms
    static constexpr uint32_t SINGLE_SHOT_REMOTE_HOLD_TIMEOUT_TICKS = 60; // 遥控器抱角最长 300ms
    static constexpr float SINGLE_SHOT_SETTLE_THRESHOLD = 50.0f; // 转子角误差小于此值才认为锁稳
    static constexpr float SINGLE_SHOT_SETTLE_SPEED_RPM = 40.0f; // 转速低于此值才允许退出抱角
    // ROLLBACK_MARKER_SINGLE_COOLDOWN_HOLD_END
};

#endif
