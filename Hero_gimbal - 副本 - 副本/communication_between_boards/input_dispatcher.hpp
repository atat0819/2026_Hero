#ifndef INPUT_DISPATCHER_HPP
#define INPUT_DISPATCHER_HPP

#include <cstdint>

enum class InputSource : uint8_t {
    Remote   = 0,
    KeyMouse = 1,
};

class InputDispatcher {
public:
    /// @brief 每控制周期调用一次，更新键鼠状态机
    /// @param s1       遥控器 S1 开关值 (1=UP, 3=MIDDLE, 2=DOWN)
    /// @param s2       遥控器 S2 开关值
    /// @param keyboard  16位键盘位掩码
    /// @param mouse_left   鼠标左键原始状态
    /// @param mouse_right  鼠标右键原始状态
    void Update(uint8_t s1, uint8_t s2, uint16_t keyboard,
                bool mouse_left, bool mouse_right);

    // ---- 查询接口 ----

    InputSource GetSource() const { return source_; }

    /// R 键翻转状态：摩擦轮+拨弹轮开关
    bool IsFrictionOn() const { return r_toggle_on_; }

    /// T 键翻转状态：true=单发, false=连发
    bool IsSingleShot() const { return t_single_shot_; }

    /// 右键消抖后的原始状态
    bool IsRightButtonHeld() const { return right_button_confirmed_; }

    /// 右键按住超过 2 秒 → 视觉模式
    bool IsVisionMode() const { return vision_mode_; }

    /// 左键消抖后是否按下
    bool IsLeftButtonPressed() const { return left_button_confirmed_; }

    /// 左右键同时按下 → 发射触发
    bool IsFireTriggered() const {
        return IsRightButtonHeld() && IsLeftButtonPressed();
    }

    /// 键盘位掩码（通过 CAN2 发给底盘）
    uint16_t GetKeyboardMask() const { return keyboard_mask_; }

private:
    /// @brief 通用消抖（计数器法）
    /// @return 确认后的状态
    static bool Debounce(bool raw, bool& prev_raw, uint8_t& counter, bool& confirmed);

    /// @brief 翻转边沿检测（仅 0→1 按下沿触发）
    static bool DetectToggleEdge(bool confirmed, bool& prev_confirmed);

    /// @brief 切出键鼠模式时重置所有内部状态，防止残留值抖动
    void ResetKeyMouseState();

    InputSource source_ = InputSource::Remote;

    // R 键
    bool r_toggle_on_ = false;
    bool prev_confirmed_r_ = false;
    bool prev_raw_r_ = false;
    bool confirmed_r_ = false;
    uint8_t debounce_r_ = 0;

    // T 键
    bool t_single_shot_ = true;   // 默认单发
    bool prev_confirmed_t_ = false;
    bool prev_raw_t_ = false;
    bool confirmed_t_ = false;
    uint8_t debounce_t_ = 0;

    // 鼠标右键
    bool right_button_confirmed_ = false;
    bool prev_raw_right_ = false;
    uint8_t debounce_right_ = 0;
    uint16_t right_hold_counter_ = 0;   // 按住计时 (ms)
    bool vision_mode_ = false;

    // 鼠标左键
    bool left_button_confirmed_ = false;
    bool prev_raw_left_ = false;
    uint8_t debounce_left_ = 0;

    // 键盘位掩码
    uint16_t keyboard_mask_ = 0;

    static constexpr uint8_t  DEBOUNCE_THRESHOLD    = 30;    // 消抖 30ms
    static constexpr uint16_t VISION_HOLD_THRESHOLD  = 2000;  // 视觉延时 2000ms
};

#endif // INPUT_DISPATCHER_HPP
