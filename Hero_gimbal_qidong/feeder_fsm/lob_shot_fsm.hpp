#ifndef LOB_SHOT_FSM_HPP
#define LOB_SHOT_FSM_HPP

#include "../user/core/Alg/FSM/alg_fsm.hpp"
#include <stdint.h>

// =============================================================================
// 气动吊射有限状态机 (Lob Shot FSM)
// =============================================================================
//
// 功能: 控制吊射装置执行器 (推杆电磁阀 + 充压电磁阀 + 比例阀)
// 周期: Update() 每 5ms 由 lob_shot_task 调用一次
//
// 架构: 单一发射时序状态机 — 三个执行器属于同一条串行流水线
//   - 电磁阀2 (推杆)  = 动作输出 (PRIME 伸出 / BACK 缩回)
//   - 电磁阀1 (充压)  = 动作输出 (FIRE 关闭切断气源 / 其余打开)
//   - 比例阀 (GP8403) = 参数设定 (IDLE/BACK 目标开度, FIRE 0V)
//
// ---- 模式 (由 S1 挡位决定, 停止最高优先级) ----
// OFF    (s1=0/2): 全断 — 推杆缩 + 电磁阀1关 + 比例阀0V (安全)
// AUTO   (s1=1):   自动上膛 (红外) + s2 拨杆手动击发
// MANUAL (s1=3):   推杆 100% 摇杆控制 (状态机不干涉);
//                  击发 = s2 拨杆独立动作 (电磁阀1关+比例阀0V), 不影响推杆 (调试用)
//
// ---- 状态流转图 ----
//
//   IDLE (待发: 推杆缩, 气室满压)
//    │ 上膛: 红外有弹 / 摇杆右推
//    ▼
//   PRIME (推杆顶死密封, 等推杆到位 + 等 s2 拨杆)
//    │ 到位计时到 且 s2 拨杆
//    ▼
//   FIRE (电磁阀1关 + 比例阀0V → 快开阀放气推弹, 推杆保持顶死)
//    │ 放气计时到
//    ▼
//   BACK (推杆保持顶死 + 恢复充压)
//    │ 2s 计时到 → 推杆自动缩回 (放行下一颗弹)
//    ▼
//   IDLE (循环)
//
//   任何时刻 s1 离开激活挡 → 强制全断 (OFF)
//   击发 = 电磁阀1关 + 比例阀0V (快开阀机械换向), 无其他触发源
// =============================================================================

// ---- 模式: 由遥控器 S1 挡位决定 ----
enum Enum_Lob_Shot_Mode
{
    LOB_MODE_OFF = 0,     // 全关 (s1=0/2)
    LOB_MODE_AUTO,        // 自动挡 (s1=1): 红外上膛 + s2 击发
    LOB_MODE_MANUAL,      // 手动挡 (s1=3): 摇杆推杆 + s2 击发
};

// ---- FSM 内部状态 ----
enum Enum_Lob_Shot_Status
{
    LOB_STATE_IDLE = 0,   // 待发: 推杆缩 + 充压 + 目标开度
    LOB_STATE_PRIME,      // 上膛密封: 推杆顶死 + 充压保持
    LOB_STATE_FIRE,       // 击发: 推杆顶死 + 电磁阀1关 + 比例阀0V
    LOB_STATE_BACK,       // 复位: 推杆保持顶死 + 恢复充压, 计时到后缩回 (放行下一颗弹)
    LOB_STATUS_COUNT      // 状态总数 (4)
};

// ---- 输入: 每轮 Update 由任务层填充 ----
struct Struct_Lob_Shot_Input
{
    uint8_t s1;           // 遥控器 S1 挡位 (0=未连接 1=UP 2=DOWN 3=MIDDLE)
    uint8_t s2;           // 遥控器 S2 拨杆 (击发触发, 边缘检测)
    bool    ir_has_ball;  // 红外防抖后: 进弹口有弹
    float   left_x;       // 左摇杆 X (-1~1), 手动挡推杆
};

// ---- 输出: FSM 只算逻辑, 任务层执行 ----
struct Struct_Lob_Shot_Output
{
    bool     pusher_extended;   // 推杆是否顶死 → solenoid_valve_2
    bool     charge_valve_open; // 电磁阀1 是否通(充压) → solenoid_valve_1
    uint16_t valve_opening_mV;  // 比例阀目标电压 → gp8403.Set_Voltage_mV()
};

class Class_Lob_Shot_FSM : public Class_FSM
{
public:
    void Init();

    /// @brief 每 5ms 调用; 任务层需先调 TIM_Calculate_PeriodElapsedCallback() 递增驻留计时
    void Update(const Struct_Lob_Shot_Input &input);

    // ---- 输出接口 ----
    const Struct_Lob_Shot_Output &Get_Output() const;  // 当前执行器输出

    // ---- 诊断接口 ----
    uint8_t Get_Mode() const;         // OFF / AUTO / MANUAL

private:
    void Update_Manual(const Struct_Lob_Shot_Input &input);  // 手动挡: 推杆跟摇杆, 击发独立动作

    // ---- 可调参数 (1 tick = 5ms, 由 lob_shot_task 周期决定) ----
    // 手动挡摇杆死区: |x| < 0.5 保持当前状态, 防摇杆抖动
    static constexpr float MANUAL_DEADZONE = 0.5f;

    // 目标开度 % (决定打弹气压, 气室压力 = 比例阀设定) — 标定时从低往高试
    static constexpr float TARGET_OPENING_PERCENT = 70.0f;

    // 击发拨杆位置 (DT7: UP=1, MIDDLE=3, DOWN=2) — 边缘触发, 拨一下打一发
    static constexpr uint8_t FIRE_TRIGGER_POSITION = 1U;  // Remote::UP

    // 推杆伸出到位等待 (20 ticks = 100ms) — 标定: 观察推杆顶死时间
    static constexpr uint32_t PRIME_HOLD_TICKS = 20U;
    // 放气保持时间 (200 ticks = 1s) — 弹丸出膛即可, 太长浪费气
    static constexpr uint32_t FIRE_HOLD_TICKS = 200U;
    // 打弹后推杆保持顶死时长 (400 ticks = 2s) — 弹丸出膛/气室放完后再缩杆放行下一颗
    static constexpr uint32_t BACK_HOLD_TICKS = 400U;

    // ---- 内部变量 ----
    uint8_t current_mode_    = LOB_MODE_OFF;  // 当前模式
    uint8_t last_s2_         = 0U;            // 上一轮 s2 (边缘检测)
    bool    fire_pending_    = false;         // 拨杆击发请求 (PRIME 中消费)
    bool    pusher_extended_ = false;         // 当前推杆状态 (死区保持用)
    uint32_t fire_timer_     = 0U;            // 手动挡击发动作剩余计时 (tick)
    uint16_t opening_mV_     = 0U;            // 目标开度换算电压 (Init 时算好)
    Struct_Lob_Shot_Output output_ = {false, true, 0U};  // 执行器输出
};

#endif
