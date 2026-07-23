#include "friction_fsm.hpp"
#include "../user/core/BSP/RemoteControl/DT7.hpp"

using Remote = BSP::REMOTE_CONTROL::RemoteController;

void Class_Friction_FSM::Init()
{
    Class_FSM::Init(FRICTION_STATUS_COUNT, FRICTION_STOP);
    left_control_output = 0.0f;
    right_control_output = 0.0f;
    current_mode = FRICTION_MODE_STOP;
}

bool Class_Friction_FSM::Is_Speed_Ready(float left_speed, float right_speed) const
{
    return fabs(left_speed) >= READY_SPEED_THRESHOLD &&
           fabs(right_speed) >= READY_SPEED_THRESHOLD;
}

// ===== FSM 内部判断 friction_mode =====
static uint8_t DetermineFrictionMode(const struct Struct_Friction_Input &input)
{
    if (input.is_keymouse)
    {
        return input.friction_on ? FRICTION_MODE_ON : FRICTION_MODE_STOP;
    }

    // 非法拨杆值（上电未连接时为 0）→ 默认 STOP，防止摩擦轮误启动
    if (input.s1 < 1 || input.s1 > 3 || input.s2 < 1 || input.s2 > 3)
        return FRICTION_MODE_STOP;

    // 遥控器模式：除 S1↓+S2↓ / S1中+S2↓ / S1↑+S2↓ 外均为 ON
    if (input.s1 == Remote::DOWN && input.s2 == Remote::DOWN)  return FRICTION_MODE_STOP;
    if (input.s1 == Remote::MIDDLE && input.s2 == Remote::DOWN) return FRICTION_MODE_STOP;
    if (input.s1 == Remote::UP && input.s2 == Remote::DOWN)    return FRICTION_MODE_STOP;

    // S1中+S2中（视觉模式）：摩擦轮停止
    if (input.s1 == Remote::MIDDLE && input.s2 == Remote::MIDDLE) return FRICTION_MODE_STOP;

    return FRICTION_MODE_ON;
}

void Class_Friction_FSM::Update(const Struct_Friction_Input &input,
                                float left_speed, float right_speed)
{
    current_mode = DetermineFrictionMode(input);

    switch (Get_Now_Status_Serial())
    {
    case FRICTION_STOP:
        left_control_output = 0.0f;
        right_control_output = 0.0f;

        if (current_mode == FRICTION_MODE_ON)
        {
            Set_Status(FRICTION_STARTING);
        }
        break;

    case FRICTION_STARTING:
        left_control_output = TARGET_SPEED;
        right_control_output = -TARGET_SPEED;

        if (current_mode == FRICTION_MODE_STOP)
        {
            Set_Status(FRICTION_STOP);
        }
        else if (Is_Speed_Ready(left_speed, right_speed) &&
                 Status[FRICTION_STARTING].Count_Time >= STARTING_TIME_COUNT)
        {
            Set_Status(FRICTION_READY);
        }
        break;

    case FRICTION_READY:
        left_control_output = TARGET_SPEED;
        right_control_output = -TARGET_SPEED;

        if (current_mode == FRICTION_MODE_STOP)
        {
            Set_Status(FRICTION_STOP);
        }
        else if (!Is_Speed_Ready(left_speed, right_speed))
        {
            Set_Status(FRICTION_STARTING);
        }
        break;

    default:
        left_control_output = 0.0f;
        right_control_output = 0.0f;
        Set_Status(FRICTION_STOP);
        break;
    }
}

float Class_Friction_FSM::Get_Left_Control_Output()
{
    return left_control_output;
}

float Class_Friction_FSM::Get_Right_Control_Output()
{
    return right_control_output;
}

uint8_t Class_Friction_FSM::Is_Ready()
{
    return Get_Now_Status_Serial() == FRICTION_READY;
}
