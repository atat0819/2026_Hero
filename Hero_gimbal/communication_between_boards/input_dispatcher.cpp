#include "input_dispatcher.hpp"
#include "../user/core/BSP/RemoteControl/DT7.hpp"
#include "main.h"   // HAL_GetTick 消抖/视觉计时

using Remote = BSP::REMOTE_CONTROL::RemoteController;

// 全局键鼠输入状态机实例（声明见 input_dispatcher.hpp）
InputDispatcher input_dispatcher;

void InputDispatcher::Update(uint8_t s1, uint8_t s2, uint16_t keyboard,
                             bool mouse_left, bool mouse_right)
{
    // ---- 1. 判断输入源 ----
    if (s1 == Remote::DOWN && s2 == Remote::UP) {
        source_ = InputSource::KeyMouse;
    } else {
        source_ = InputSource::Remote;
        ResetKeyMouseState();  // 切出键鼠时重置，防止状态残留抖动
        return;
    }

    // ---- 2. 保存键盘位掩码 ----
    keyboard_mask_ = keyboard;

    // ---- 3. 读取原始按键/鼠标状态 ----
    bool raw_r     = (keyboard & static_cast<uint16_t>(Remote::KEY_R)) != 0;
    bool raw_g     = (keyboard & static_cast<uint16_t>(Remote::KEY_G)) != 0;
    bool raw_left  = mouse_left;
    bool raw_right = mouse_right;

    // ---- 4. 消抖（tick 差值法，消抖时间为真实毫秒，不受调用周期抖动影响） ----
    Debounce(raw_r,     prev_raw_r_,     stable_since_r_,     confirmed_r_);
    Debounce(raw_g,     prev_raw_t_,     stable_since_t_,     confirmed_t_);
    Debounce(raw_left,  prev_raw_left_,  stable_since_left_,  left_button_confirmed_);
    Debounce(raw_right, prev_raw_right_, stable_since_right_, right_button_confirmed_);

    // ---- 5. 边沿检测提取（放在外侧，确保 prev 变量每帧更新） ----
    bool r_edge = DetectToggleEdge(confirmed_r_, prev_confirmed_r_);
    bool t_edge = DetectToggleEdge(confirmed_t_, prev_confirmed_t_);

    // ---- 6. 状态机翻转 ----
    if (r_edge) {
        r_toggle_on_ = !r_toggle_on_;
        if (r_toggle_on_) {
            t_single_shot_ = true;   // 开启摩擦轮默认单发
        }
    }

    if (r_toggle_on_ && t_edge) {
        t_single_shot_ = !t_single_shot_;
    }

    // ---- 7. 视觉模式判定（tick 差值：右键确认按下后持续按住 ≥2s 判定） ----
    if (right_button_confirmed_) {
        // 首次确认按下时记录起始 tick（已按着则保持不变）
        if (right_hold_start_tick_ == 0) {
            right_hold_start_tick_ = HAL_GetTick();
        }
        vision_mode_ = (HAL_GetTick() - right_hold_start_tick_) >= VISION_HOLD_THRESHOLD;
    } else {
        right_hold_start_tick_ = 0;
        vision_mode_ = false;
    }
}

void InputDispatcher::ResetKeyMouseState()
{
    // 消抖计时基准归零（统一改为当前 tick，防止切回时残留计时导致误触发）
    const uint32_t now = HAL_GetTick();
    stable_since_r_     = now;
    stable_since_t_     = now;
    stable_since_left_  = now;
    stable_since_right_ = now;

    // 确认状态归零
    confirmed_r_     = false;
    confirmed_t_     = false;
    left_button_confirmed_  = false;
    right_button_confirmed_ = false;

    // 视觉判定归零
    right_hold_start_tick_ = 0;
    vision_mode_ = false;

    // 翻转状态重置：切出键鼠 = 摩擦轮自动关闭，重进需重新按 R 开启
    r_toggle_on_   = false;
    t_single_shot_ = true;

    // 键盘位掩码清零，防止切回遥控器后残留旧值
    keyboard_mask_ = 0;

    // prev 变量同步到当前已知状态，防止下次进入时 DetectToggleEdge 误判
    prev_raw_r_     = false;
    prev_raw_t_     = false;
    prev_raw_left_  = false;
    prev_raw_right_ = false;
    prev_confirmed_r_ = false;
    prev_confirmed_t_ = false;
}

void InputDispatcher::Debounce(bool raw, bool& prev_raw, uint32_t& stable_since_tick, bool& confirmed)
{
    const uint32_t now = HAL_GetTick();

    // 输入状态变化 → 重新计时（初始化为 0 时也在此建立基准）
    if (raw != prev_raw) {
        stable_since_tick = now;
    }
    prev_raw = raw;

    // 状态稳定超过 DEBOUNCE_THRESHOLD 后确认（tick 无符号差值，回绕安全）
    if ((now - stable_since_tick) >= DEBOUNCE_THRESHOLD) {
        confirmed = raw;
    }
}

bool InputDispatcher::DetectToggleEdge(bool confirmed, bool& prev_confirmed)
{
    bool edge = confirmed && !prev_confirmed;
    prev_confirmed = confirmed;
    return edge;
}
