/**
 * @file   feeder_fsm.cpp
 * @brief  拨弹轮有限状态机 — 控制英雄机器人单发/连发/反转/冷却
 *
 * 状态流转:
 *   STOP ──┬──(连发+扳机)────→ CONTINUOUS_SHOT ──(松扳机)──→ STOP
 *          ├──(反转pending)──→ MANUAL_REVERSE ──(到位/超时)→ STOP
 *          ├──(单发边缘)────→ SINGLE_SHOT ──(到位/超时)→ COOLDOWN → STOP
 *          └──(单发电平)────→ SINGLE_SHOT (同上, 冷却后循环)
 *
 * 两类扳机信号:
 *   遥控器脉冲: 边缘触发, single_shot_pending 挂起后 STOP 消费
 *   视觉持续:   电平触发, STOP 中直接判断 current_trigger_pressed
 */

#include "feeder_fsm.hpp"
#include "../user/core/BSP/RemoteControl/DT7.hpp"

using Remote = BSP::REMOTE_CONTROL::RemoteController;

// ============================================================================
// 初始化
// ============================================================================
void Class_Feeder_FSM::Init()
{
    Class_FSM::Init(FEEDER_STATUS_COUNT, FEEDER_STOP);
    control_output               = 0.0f;
    raw_angle                    = 0.0f;
    accumulated_angle            = 0.0f;
    single_shot_target_angle     = 0.0f;
    manual_reverse_target_angle  = 0.0f;
    last_trigger_pressed         = 0;
    current_mode                 = FEEDER_MODE_STOP;
    current_trigger_pressed      = 0;
    single_shot_pending          = 0;
    manual_reverse_pending       = 0;
    control_type                 = FEEDER_CONTROL_STOP;
    angle_initialized            = 0;
    single_shot_target_locked    = 0;
    manual_reverse_target_locked = 0;
}

// ============================================================================
// 累积角度更新 — 首帧初始化 + 跨圈修正
// ============================================================================
void Class_Feeder_FSM::Update_Accumulated_Angle(float feeder_current_angle)
{
    if (angle_initialized == 0U)
    {
        raw_angle         = feeder_current_angle;
        accumulated_angle = feeder_current_angle;
        angle_initialized = 1;
        return;
    }

    float delta_angle = feeder_current_angle - raw_angle;

    if (delta_angle > ANGLE_WRAP_THRESHOLD)
    {
        delta_angle -= FULL_ROTATION_ANGLE;
    }
    else if (delta_angle < -ANGLE_WRAP_THRESHOLD)
    {
        delta_angle += FULL_ROTATION_ANGLE;
    }

    raw_angle = feeder_current_angle;
    accumulated_angle += delta_angle;
}

// ============================================================================
// 到位判定
// ============================================================================
bool Class_Feeder_FSM::Is_Single_Shot_Finished() const
{
    return fabs(single_shot_target_angle - accumulated_angle) <= SINGLE_SHOT_FINISH_THRESHOLD;
}

bool Class_Feeder_FSM::Is_Manual_Reverse_Finished() const
{
    return fabs(manual_reverse_target_angle - accumulated_angle) <= SINGLE_SHOT_FINISH_THRESHOLD;
}

// ============================================================================
// 模式与触发判定：FSM 内部根据 s1/s2 和键鼠状态自行决定
// ============================================================================
// 注意：遥控器模式下 vision_fire（视觉开火指令）不允许触发拨弹轮转动，
// 仅由遥控器滚轮 (scroll_value) 控制拨弹轮触发。
static void DetermineFeederModeAndTrigger(const Struct_Feeder_Input &input,
                                          uint8_t &feeder_mode,
                                          uint8_t &trigger_pressed)
{
    // 非法拨杆值（上电未连接时为 0）→ 强制 STOP，防止拨弹轮误动
    if (input.s1 < 1 || input.s1 > 3 || input.s2 < 1 || input.s2 > 3)
    {
        feeder_mode     = FEEDER_MODE_STOP;
        trigger_pressed = FEEDER_TRIGGER_NONE;
        return;
    }

    // ---- 键鼠模式 ----
    if (input.is_keymouse)
    {
        // 键鼠模式：忽略视觉开火指令，由键鼠按键控制触发

        if (input.friction_on)
        {
            feeder_mode = input.is_single_shot ? FEEDER_MODE_SINGLE : FEEDER_MODE_CONTINUOUS;
        }
        else
        {
            feeder_mode = FEEDER_MODE_STOP;
        }

        trigger_pressed = input.fire_triggered ? FEEDER_TRIGGER_FORWARD : FEEDER_TRIGGER_NONE;
        return;
    }

    // ---- 遥控器模式（保持原有挡位逻辑） ----
    // 视觉开火指令 (vision_fire) 不参与触发判定，仅滚轮控制拨弹轮

    if (input.s1 == Remote::DOWN && input.s2 == Remote::DOWN)
    {
        feeder_mode = FEEDER_MODE_STOP;
    }
    else if (input.s1 == Remote::DOWN && input.s2 == Remote::MIDDLE)
    {
        feeder_mode = FEEDER_MODE_SINGLE;
    }
    else if (input.s1 == Remote::DOWN && input.s2 == Remote::UP)
    {
        feeder_mode = FEEDER_MODE_SINGLE;
    }
    else if (input.s1 == Remote::MIDDLE && input.s2 == Remote::DOWN)
    {
        feeder_mode = FEEDER_MODE_STOP;
    }
    else if (input.s1 == Remote::MIDDLE && input.s2 == Remote::MIDDLE)
    {
        feeder_mode = FEEDER_MODE_SINGLE;
    }
    else if (input.s1 == Remote::MIDDLE && input.s2 == Remote::UP)
    {
        feeder_mode = FEEDER_MODE_SINGLE;
    }
    else if (input.s1 == Remote::UP && input.s2 == Remote::DOWN)
    {
        feeder_mode = FEEDER_MODE_STOP;
    }
    else if (input.s1 == Remote::UP && input.s2 == Remote::MIDDLE)
    {
        feeder_mode = FEEDER_MODE_SINGLE;
    }
    else if (input.s1 == Remote::UP && input.s2 == Remote::UP)
    {
        feeder_mode = FEEDER_MODE_CONTINUOUS;
    }
    else
    {
        feeder_mode = FEEDER_MODE_STOP;
    }

    // 遥控器触发：滚轮
    static uint8_t fwd_armed = 1;
    static uint8_t rev_armed = 1;
    trigger_pressed = FEEDER_TRIGGER_NONE;

    if (feeder_mode == FEEDER_MODE_CONTINUOUS)
    {
        trigger_pressed = (input.scroll_value > 0.95f) ? FEEDER_TRIGGER_FORWARD : FEEDER_TRIGGER_NONE;
    }
    else if (feeder_mode == FEEDER_MODE_SINGLE)
    {
        if (fwd_armed && input.scroll_value > 0.95f)
        {
            trigger_pressed = FEEDER_TRIGGER_FORWARD;
            fwd_armed = 0;
        }
        if (input.scroll_value < 0.80f)
        {
            fwd_armed = 1;
        }

        if (rev_armed && input.scroll_value < -0.95f)
        {
            trigger_pressed = FEEDER_TRIGGER_REVERSE;
            rev_armed = 0;
        }
        if (input.scroll_value > -0.80f)
        {
            rev_armed = 1;
        }
    }
}

// ============================================================================
// 核心: 状态机 Update — 每 5ms 由 gimbal_task 调用
// ============================================================================
void Class_Feeder_FSM::Update(const Struct_Feeder_Input &input,
                              float   feeder_current_angle,
                              float   current_speed,
                              float   current_iq)
{
    // ---- 第1步: 更新累积角度 ----
    Update_Accumulated_Angle(feeder_current_angle);

    // ---- 第2步: FSM 内部判断模式与触发 ----
    // 注意：视觉开火指令 (vision_fire) 不参与触发判定，仅滚轮/键鼠按键控制拨弹轮
    uint8_t feeder_mode = FEEDER_MODE_STOP;
    uint8_t trigger_pressed = FEEDER_TRIGGER_NONE;
    DetermineFeederModeAndTrigger(input, feeder_mode, trigger_pressed);

    // ---- 第3步: 快照当前输入 ----
    current_mode            = feeder_mode;
    current_trigger_pressed = trigger_pressed;

    // ---- 第4步: 边缘检测 → 挂起事件 ----
    const uint8_t forward_rising_edge =
        (trigger_pressed == FEEDER_TRIGGER_FORWARD) &&
        (last_trigger_pressed == FEEDER_TRIGGER_NONE);
    const uint8_t reverse_rising_edge =
        (trigger_pressed == FEEDER_TRIGGER_REVERSE) &&
        (last_trigger_pressed == FEEDER_TRIGGER_NONE);
    last_trigger_pressed = trigger_pressed;

    if (current_mode == FEEDER_MODE_SINGLE && forward_rising_edge != 0U)
    {
        single_shot_pending = 1;
    }

    if (current_mode != FEEDER_MODE_STOP && reverse_rising_edge != 0U)
    {
        manual_reverse_pending = 1;
    }

    // ---- 第5步: 状态机流转 ----
    switch (Get_Now_Status_Serial())
    {
    case FEEDER_STOP:
        control_type  = FEEDER_CONTROL_STOP;
        control_output = 0.0f;
        single_shot_target_locked    = 0;
        manual_reverse_target_locked = 0;

        if (current_mode == FEEDER_MODE_CONTINUOUS &&
            current_trigger_pressed == FEEDER_TRIGGER_FORWARD)
        {
            Set_Status(FEEDER_CONTINUOUS_SHOT);
        }
        else if (current_mode != FEEDER_MODE_STOP && manual_reverse_pending != 0U)
        {
            manual_reverse_pending = 0;
            manual_reverse_target_angle = accumulated_angle - SINGLE_SHOT_ANGLE;
            manual_reverse_target_locked = 1;
            Set_Status(FEEDER_MANUAL_REVERSE);
        }
        else if (single_shot_pending != 0U)
        {
            single_shot_pending = 0;
            single_shot_target_angle = accumulated_angle + SINGLE_SHOT_ANGLE;
            single_shot_target_locked = 1;
            Set_Status(FEEDER_SINGLE_SHOT);
        }
        else if (current_mode == FEEDER_MODE_SINGLE &&
                 current_trigger_pressed == FEEDER_TRIGGER_FORWARD)
        {
            single_shot_target_angle = accumulated_angle + SINGLE_SHOT_ANGLE;
            single_shot_target_locked = 1;
            Set_Status(FEEDER_SINGLE_SHOT);
        }
        break;

    case FEEDER_SINGLE_SHOT:
        if (single_shot_target_locked == 0U)
        {
            single_shot_target_angle = accumulated_angle + SINGLE_SHOT_ANGLE;
            single_shot_target_locked = 1;
        }
        control_type  = FEEDER_CONTROL_ANGLE;
        control_output = single_shot_target_angle;

        if (Is_Single_Shot_Finished() ||
            Status[FEEDER_SINGLE_SHOT].Count_Time >= SINGLE_SHOT_TIMEOUT_COUNT)
        {
            Set_Status(FEEDER_SINGLE_COOLDOWN);
        }
        break;

    case FEEDER_CONTINUOUS_SHOT:
        control_type  = FEEDER_CONTROL_SPEED;
        control_output = FORWARD_SPEED;
        single_shot_target_locked = 0;

        if (current_mode != FEEDER_MODE_CONTINUOUS ||
            current_trigger_pressed != FEEDER_TRIGGER_FORWARD)
        {
            Set_Status(FEEDER_STOP);
        }
        break;

    case FEEDER_MANUAL_REVERSE:
        if (manual_reverse_target_locked == 0U)
        {
            manual_reverse_target_angle = accumulated_angle - SINGLE_SHOT_ANGLE;
            manual_reverse_target_locked = 1;
        }
        control_type  = FEEDER_CONTROL_ANGLE;
        control_output = manual_reverse_target_angle;

        if (Is_Manual_Reverse_Finished() ||
            Status[FEEDER_MANUAL_REVERSE].Count_Time >= MANUAL_REVERSE_TIMEOUT_COUNT)
        {
            Set_Status(FEEDER_STOP);
        }
        break;

    case FEEDER_SINGLE_COOLDOWN:
        control_type  = FEEDER_CONTROL_STOP;
        control_output = 0.0f;

        if (Status[FEEDER_SINGLE_COOLDOWN].Count_Time >= SINGLE_SHOT_COOLDOWN_TICKS ||
            current_trigger_pressed == FEEDER_TRIGGER_NONE)
        {
            Set_Status(FEEDER_STOP);
        }
        break;

    default:
        control_type  = FEEDER_CONTROL_STOP;
        control_output = 0.0f;
        single_shot_target_locked    = 0;
        manual_reverse_target_locked = 0;
        Set_Status(FEEDER_STOP);
        break;
    }
}

// ============================================================================
// 输出接口
// ============================================================================
float Class_Feeder_FSM::Get_Control_Output()
{
    return control_output;
}

uint8_t Class_Feeder_FSM::Get_Control_Type()
{
    return control_type;
}

float Class_Feeder_FSM::Get_Accumulated_Angle()
{
    return accumulated_angle;
}

float Class_Feeder_FSM::Get_Single_Shot_Target_Angle()
{
    return single_shot_target_angle;
}

float Class_Feeder_FSM::Get_Manual_Reverse_Target_Angle()
{
    return manual_reverse_target_angle;
}
