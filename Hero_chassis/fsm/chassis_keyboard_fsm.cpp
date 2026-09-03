#include "chassis_keyboard_fsm.hpp"

void ChassisKeyboardFSM::Init()
{
    Reset();
}

void ChassisKeyboardFSM::Reset()
{
    stable_key_mask_ = 0;
    last_raw_key_mask_ = 0;
    first_sample_tick_ = 0;
    first_sample_pending_ = true;
    stable_mask_initialized_ = false;
    last_ctrl_pressed_ = false;
    gyro_enabled_ = false;
    last_z_pressed_ = false;
    follow_enabled_ = false;
    command_ = {};

    for (uint8_t i = 0; i < 16; ++i)
    {
        key_change_tick_[i] = 0;
    }
}

void ChassisKeyboardFSM::Update(uint16_t raw_key_mask,
                                bool keyboard_mode,
                                bool keyboard_online,
                                uint32_t now_tick)
{
    if (!keyboard_mode || !keyboard_online)
    {
        Reset();
        return;
    }

    command_.valid = true;

    // Initialize only after the first raw mask has remained stable. This
    // prevents a key held while entering keyboard mode from toggling CTRL.
    if (first_sample_pending_)
    {
        last_raw_key_mask_ = raw_key_mask;
        first_sample_tick_ = now_tick;
        first_sample_pending_ = false;
        // Entering keyboard mode starts in follow mode. A CTRL/Z key that is
        // already held during entry is synchronized below and does not toggle.
        follow_enabled_ = true;
        for (uint8_t i = 0; i < 16; ++i)
        {
            key_change_tick_[i] = now_tick;
        }
        UpdateCommand();
        return;
    }

    if (!stable_mask_initialized_)
    {
        if (raw_key_mask != last_raw_key_mask_)
        {
            last_raw_key_mask_ = raw_key_mask;
            first_sample_tick_ = now_tick;
        }
        else if (now_tick - first_sample_tick_ >= KEY_DEBOUNCE_MS)
        {
            stable_key_mask_ = last_raw_key_mask_;
            stable_mask_initialized_ = true;
            last_ctrl_pressed_ = (stable_key_mask_ & KEY_CTRL) != 0U;
            last_z_pressed_ = (stable_key_mask_ & KEY_Z) != 0U;
            UpdateCommand();
            return;
        }

        UpdateCommand();
        return;
    }

    // Debounce each key independently so simultaneous W+A or W+D input does
    // not delay or disturb another key that is already stable.
    for (uint8_t i = 0; i < 16; ++i)
    {
        const uint16_t bit = static_cast<uint16_t>(1U << i);
        const bool raw_pressed = (raw_key_mask & bit) != 0U;
        const bool last_raw_pressed = (last_raw_key_mask_ & bit) != 0U;

        if (raw_pressed != last_raw_pressed)
        {
            if (raw_pressed)
            {
                last_raw_key_mask_ |= bit;
            }
            else
            {
                last_raw_key_mask_ &= static_cast<uint16_t>(~bit);
            }
            key_change_tick_[i] = now_tick;
        }
        else if (raw_pressed != ((stable_key_mask_ & bit) != 0U) &&
                 now_tick - key_change_tick_[i] >= KEY_DEBOUNCE_MS)
        {
            if (raw_pressed)
            {
                stable_key_mask_ |= bit;
            }
            else
            {
                stable_key_mask_ &= static_cast<uint16_t>(~bit);
            }
        }
    }

    const bool ctrl_pressed = (stable_key_mask_ & KEY_CTRL) != 0U;
    if (ctrl_pressed && !last_ctrl_pressed_)
    {
        gyro_enabled_ = !gyro_enabled_;
    }
    last_ctrl_pressed_ = ctrl_pressed;

    const bool z_pressed = (stable_key_mask_ & KEY_Z) != 0U;
    if (z_pressed && !last_z_pressed_)
    {
        follow_enabled_ = !follow_enabled_;
    }
    last_z_pressed_ = z_pressed;

    UpdateCommand();
}

void ChassisKeyboardFSM::UpdateCommand()
{
    float forward = 0.0f;
    float lateral = 0.0f;

    if ((stable_key_mask_ & KEY_W) != 0U)
    {
        forward += 1.0f;
    }
    if ((stable_key_mask_ & KEY_S) != 0U)
    {
        forward -= 1.0f;
    }
    if ((stable_key_mask_ & KEY_A) != 0U)
    {
        lateral -= 1.0f;
    }
    if ((stable_key_mask_ & KEY_D) != 0U)
    {
        lateral += 1.0f;
    }

    command_.vx = forward;
    command_.vy = lateral;
    command_.shift_pressed = (stable_key_mask_ & KEY_SHIFT) != 0U;
    command_.gyro_enabled = gyro_enabled_;
    command_.follow_enabled = follow_enabled_;
}
