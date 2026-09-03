#ifndef CHASSIS_KEYBOARD_FSM_HPP
#define CHASSIS_KEYBOARD_FSM_HPP

#include <stdint.h>

struct KeyboardMotionCommand
{
    float vx;
    float vy;
    bool shift_pressed;
    bool gyro_enabled;
    bool follow_enabled;
    bool valid;
};

class ChassisKeyboardFSM
{
public:
    enum KeyMask : uint16_t
    {
        KEY_W     = (1U << 0),
        KEY_S     = (1U << 1),
        KEY_A     = (1U << 2),
        KEY_D     = (1U << 3),
        KEY_SHIFT = (1U << 4),
        KEY_CTRL  = (1U << 5),
        KEY_Z     = (1U << 11)
    };

    static constexpr uint32_t KEY_DEBOUNCE_MS = 20U;

    void Init();
    void Update(uint16_t raw_key_mask,
                bool keyboard_mode,
                bool keyboard_online,
                uint32_t now_tick);

    const KeyboardMotionCommand& GetCommand() const { return command_; }

private:
    void Reset();
    void UpdateCommand();

    uint16_t stable_key_mask_ = 0;
    uint16_t last_raw_key_mask_ = 0;
    uint32_t key_change_tick_[16] = {};

    uint32_t first_sample_tick_ = 0;
    bool first_sample_pending_ = true;
    bool stable_mask_initialized_ = false;

    bool last_ctrl_pressed_ = false;
    bool gyro_enabled_ = false;
    bool last_z_pressed_ = false;
    // Keyboard mode defaults to follow; Reset() clears it until mode entry.
    bool follow_enabled_ = false;

    KeyboardMotionCommand command_{};
};

#endif // CHASSIS_KEYBOARD_FSM_HPP
