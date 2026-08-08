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

    /// G 键翻转状态：true=单发, false=连发（DBUS 键位掩码无 T 键）
    bool IsSingleShot() const { return t_single_shot_; }

    /// 右键消抖后的原始状态
    bool IsRightButtonHeld() const { return right_button_confirmed_; }

    /// 右键按住超过 2 秒 → 视觉模式
    bool IsVisionMode() const { return vision_mode_; }

    /// 左键消抖后是否按下
    bool IsLeftButtonPressed() const { return left_button_confirmed_; }

    /// 左右键同时按下 → 发射触发（电平判断，任何模式下均可开火，
    /// 与视觉模式解耦——不需要先进视觉就能点射）
    bool IsFireTriggered() const {
        return IsRightButtonHeld() && IsLeftButtonPressed();
    }

    /// 键盘位掩码（通过 CAN2 发给底盘）
    uint16_t GetKeyboardMask() const { return keyboard_mask_; }

private:
    /// @brief 通用消抖（tick 差值法）：输入稳定超过 DEBOUNCE_THRESHOLD 后确认状态
    /// @param stable_since_tick 输入状态最近一次变化的 tick，消抖时间基准
    static void Debounce(bool raw, bool& prev_raw, uint32_t& stable_since_tick, bool& confirmed);

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
    uint32_t stable_since_r_ = 0;   // R 键输入稳定的起始 tick

    // G 键（单发/连发切换）
    bool t_single_shot_ = true;   // 默认单发
    bool prev_confirmed_t_ = false;
    bool prev_raw_t_ = false;
    bool confirmed_t_ = false;
    uint32_t stable_since_t_ = 0;   // G 键输入稳定的起始 tick

    // 鼠标右键
    bool right_button_confirmed_ = false;
    bool prev_raw_right_ = false;
    uint32_t stable_since_right_ = 0;   // 右键输入稳定的起始 tick
    uint32_t right_hold_start_tick_ = 0;   // 右键确认按下的起始 tick（视觉判定基准）
    bool vision_mode_ = false;

    // 鼠标左键
    bool left_button_confirmed_ = false;
    bool prev_raw_left_ = false;
    uint32_t stable_since_left_ = 0;   // 左键输入稳定的起始 tick

    // 键盘位掩码
    uint16_t keyboard_mask_ = 0;

    static constexpr uint32_t DEBOUNCE_THRESHOLD    = 30;    // 消抖 30ms
    static constexpr uint32_t VISION_HOLD_THRESHOLD = 2000;  // 视觉延时 2000ms
};

// 全局键鼠输入状态机实例（定义于 input_dispatcher.cpp），
// 需要读取键鼠状态的任务 include 本头文件后直接使用
extern InputDispatcher input_dispatcher;

#endif // INPUT_DISPATCHER_HPP
