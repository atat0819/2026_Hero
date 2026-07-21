#include "input_dispatcher.hpp"
#include "../user/core/BSP/RemoteControl/DT7.hpp"

using Remote = BSP::REMOTE_CONTROL::RemoteController;

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
    bool raw_t     = (keyboard & static_cast<uint16_t>(Remote::KEY_G)) != 0;
    bool raw_left  = mouse_left;
    bool raw_right = mouse_right;

    // ---- 4. 消抖 ----
    Debounce(raw_r,     prev_raw_r_,     debounce_r_,     confirmed_r_);
    Debounce(raw_t,     prev_raw_t_,     debounce_t_,     confirmed_t_);
    Debounce(raw_left,  prev_raw_left_,  debounce_left_,  left_button_confirmed_);
    Debounce(raw_right, prev_raw_right_, debounce_right_, right_button_confirmed_);

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

    // ---- 7. 视觉模式判定 ----
    if (right_button_confirmed_) {
        if (right_hold_counter_ < VISION_HOLD_THRESHOLD) {
            right_hold_counter_++;
        }
    } else {
        right_hold_counter_ = 0;
    }
    vision_mode_ = (right_hold_counter_ >= VISION_HOLD_THRESHOLD);
}

void InputDispatcher::ResetKeyMouseState()
{
    // 消抖计数器归零，防止切回时残留计数导致误触发
    debounce_r_     = 0;
    debounce_t_     = 0;
    debounce_left_  = 0;
    debounce_right_ = 0;

    // 确认状态归零
    confirmed_r_     = false;
    confirmed_t_     = false;
    left_button_confirmed_  = false;
    right_button_confirmed_ = false;

    // 视觉延时归零
    right_hold_counter_ = 0;
    vision_mode_ = false;

    // 翻转状态重置：切出键鼠 = 摩擦轮自动关闭，重进需重新按 R 开启
    r_toggle_on_   = false;
    t_single_shot_ = true;

    // prev 变量同步到当前已知状态，防止下次进入时 DetectToggleEdge 误判
    prev_raw_r_     = false;
    prev_raw_t_     = false;
    prev_raw_left_  = false;
    prev_raw_right_ = false;
    prev_confirmed_r_ = false;
    prev_confirmed_t_ = false;
}

bool InputDispatcher::Debounce(bool raw, bool& prev_raw, uint8_t& counter, bool& confirmed)
{
    if (raw == prev_raw) {
        if (counter < DEBOUNCE_THRESHOLD) {
            counter++;
        }
    } else {
        counter = 0;
    }
    prev_raw = raw;

    if (counter >= DEBOUNCE_THRESHOLD) {
        confirmed = raw;
    }
    return confirmed;
}

bool InputDispatcher::DetectToggleEdge(bool confirmed, bool& prev_confirmed)
{
    bool edge = confirmed && !prev_confirmed;
    prev_confirmed = confirmed;
    return edge;
}
