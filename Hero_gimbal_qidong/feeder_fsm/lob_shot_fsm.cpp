/**
 * @file   lob_shot_fsm.cpp
 * @brief  气动吊射有限状态机 — 充压 + 击发 (阶段二)
 *
 * 状态流转:
 *   IDLE ─(红外有弹/摇杆右推)→ PRIME ─(到位计时到 且 s2拨杆)→ FIRE
 *        ↑                                                       │
 *        └───────────(放气计时到 → BACK → 回压计时到)──────────────┘
 *
 * 模式: OFF=全断 / AUTO=红外上膛+s2击发 / MANUAL=摇杆推杆+s2击发
 * 击发 = 电磁阀1关 + 比例阀0V (快开阀机械换向放气), 无其他触发源
 */

#include "lob_shot_fsm.hpp"
#include "../user/core/BSP/RemoteControl/DT7.hpp"  // Remote::UP/MIDDLE/DOWN 枚举

using Remote = BSP::REMOTE_CONTROL::RemoteController;

// ============================================================================
// 初始化
// ============================================================================
void Class_Lob_Shot_FSM::Init()
{
    Class_FSM::Init(LOB_STATUS_COUNT, LOB_STATE_IDLE);
    current_mode_    = LOB_MODE_OFF;
    last_s2_         = FIRE_TRIGGER_POSITION;  // 假装上轮已在击发位, 防遥控器连接瞬间误触发
    fire_pending_    = false;
    pusher_extended_ = false;
    output_ = {false, true, 0U};
}

// ============================================================================
// 每 5ms 调用: 模式判定 → s2边缘检测 → 状态流转 → 输出计算
// 注意: 任务层需先调用 TIM_Calculate_PeriodElapsedCallback() 递增驻留计时
// ============================================================================
void Class_Lob_Shot_FSM::Update(const Struct_Lob_Shot_Input &input)
{
    // ---- 模式判定: 停止挡位最高优先级 (安全优先) ----
    // 明确激活挡位只有 UP=自动 / MIDDLE=手动,
    // 其余 (DOWN / 未连接=0 / 任何非法值) 一律全关
    if ((input.s1 != Remote::UP) && (input.s1 != Remote::MIDDLE))
    {
        current_mode_ = LOB_MODE_OFF;
    }
    else if (input.s1 == Remote::UP)  // UP 挡: 自动
    {
        current_mode_ = LOB_MODE_AUTO;
    }
    else                              // MIDDLE 挡: 手动
    {
        current_mode_ = LOB_MODE_MANUAL;
    }

    // ---- s2 拨杆边缘检测: 进入击发位 → 挂起击发请求 ----
    // 边缘触发防止拨杆停留在击发位导致连打
    if ((input.s2 == FIRE_TRIGGER_POSITION) && (last_s2_ != FIRE_TRIGGER_POSITION))
    {
        fire_pending_ = true;
    }
    last_s2_ = input.s2;

    // ---- OFF: 全断, 强制回 IDLE, 清挂起请求 ----
    if (current_mode_ == LOB_MODE_OFF)
    {
        if (Get_Now_Status_Serial() != LOB_STATE_IDLE)
        {
            Set_Status(LOB_STATE_IDLE);
        }
        fire_pending_    = false;
        pusher_extended_ = false;
        output_ = {false, false, 0U};   // 推杆缩 + 电磁阀1关 + 比例阀0V
        return;
    }

    // ---- 推杆目标 (AUTO=红外 / MANUAL=摇杆三区) ----
    bool target_extended;
    switch (current_mode_)
    {
    case LOB_MODE_AUTO:
        target_extended = input.ir_has_ball;
        break;

    case LOB_MODE_MANUAL:
        if (input.left_x > MANUAL_DEADZONE)
        {
            target_extended = true;
        }
        else if (input.left_x < -MANUAL_DEADZONE)
        {
            target_extended = false;
        }
        else
        {
            target_extended = pusher_extended_;  // 死区保持
        }
        break;

    default:
        target_extended = false;
        break;
    }

    // ---- 状态流转 ----
    switch (Get_Now_Status_Serial())
    {
    case LOB_STATE_IDLE:
        if (target_extended)                 // 上膛条件满足
        {
            Set_Status(LOB_STATE_PRIME);
        }
        break;

    case LOB_STATE_PRIME:
        if (!target_extended)
        {
            // 弹离开/摇杆左推 → 上膛取消, 丢弃拨杆请求 (防推杆未到位就击发)
            fire_pending_ = false;
            Set_Status(LOB_STATE_IDLE);
        }
        else if ((Status[LOB_STATE_PRIME].Count_Time >= PRIME_HOLD_TICKS) &&
                 fire_pending_)              // 推杆到位 且 拨杆 → 击发
        {
            fire_pending_ = false;
            Set_Status(LOB_STATE_FIRE);
        }
        break;

    case LOB_STATE_FIRE:
        if (Status[LOB_STATE_FIRE].Count_Time >= FIRE_HOLD_TICKS)
        {
            Set_Status(LOB_STATE_BACK);
        }
        break;

    case LOB_STATE_BACK:
        if (Status[LOB_STATE_BACK].Count_Time >= BACK_HOLD_TICKS)
        {
            Set_Status(LOB_STATE_IDLE);
        }
        break;

    default:
        Set_Status(LOB_STATE_IDLE);
        break;
    }

    // ---- 输出计算 ----
    pusher_extended_ = (Get_Now_Status_Serial() == LOB_STATE_PRIME);
    const uint16_t opening_mV =
        static_cast<uint16_t>(TARGET_OPENING_PERCENT * 50.0f + 0.5f);  // % → mV
    switch (Get_Now_Status_Serial())
    {
    case LOB_STATE_IDLE:
        output_ = {false, true, opening_mV};    // 待发: 缩杆 + 充压
        break;
    case LOB_STATE_PRIME:
        output_ = {true, true, opening_mV};     // 上膛: 顶死 + 充压保持
        break;
    case LOB_STATE_FIRE:
        output_ = {true, false, 0U};            // 击发: 顶死 + 断气源 + 0V
        break;
    case LOB_STATE_BACK:
        output_ = {false, true, opening_mV};    // 复位: 缩杆 + 恢复充压
        break;
    default:
        break;
    }
}

// ============================================================================
// 输出接口: 当前执行器输出 (推杆/电磁阀1/比例阀)
// ============================================================================
const Struct_Lob_Shot_Output &Class_Lob_Shot_FSM::Get_Output() const
{
    return output_;
}

// ============================================================================
// 诊断接口: 当前模式
// ============================================================================
uint8_t Class_Lob_Shot_FSM::Get_Mode() const
{
    return current_mode_;
}
