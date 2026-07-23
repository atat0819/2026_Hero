#include "gimbal_fsm.hpp"
#include "../user/core/BSP/RemoteControl/DT7.hpp"

namespace
{
constexpr float INPUT_DEADBAND = 0.01f;
constexpr float FULL_CIRCLE_DEG = 360.0f;

float Absolute_Value(float value)
{
    return (value >= 0.0f) ? value : -value;
}
}

using Remote = BSP::REMOTE_CONTROL::RemoteController;

void Class_Gimbal_FSM::Init(const Struct_Gimbal_FSM_Config &__config,
                            uint8_t __initial_status)
{
    Class_FSM::Init(GIMBAL_STATUS_COUNT, __initial_status);

    config = __config;
    control_output = 0.0f;
    target_angle = 0.0f;
    target_speed = 0.0f;
    control_type = GIMBAL_CONTROL_STOP;
    angle_target_initialized = 0U;
    mode_changed_flag = 0U;

    vision_slope_.SetIncreaseValue(config.vision_slope_inc);
    vision_slope_.SetDecreaseValue(config.vision_slope_dec);

    if (__initial_status == GIMBAL_STATUS_ANGLE)
    {
        control_type = GIMBAL_CONTROL_ANGLE;
    }
    else if (__initial_status == GIMBAL_STATUS_SPEED)
    {
        control_type = GIMBAL_CONTROL_SPEED;
    }
}

// ===== 模式判定：FSM 内部根据 s1/s2 和键鼠状态自行决定 =====
uint8_t Class_Gimbal_FSM::DetermineMode(const Struct_Gimbal_Input &input) const
{
    // 非法拨杆值（上电未连接时为 0）→ 强制 STOP，防止云台误动
    if (input.s1 < 1 || input.s1 > 3 || input.s2 < 1 || input.s2 > 3)
        return GIMBAL_MODE_STOP;

    // ---- 键鼠模式：s1↓ + s2↑ ----
    if (input.s1 == Remote::DOWN && input.s2 == Remote::UP)
    {
        if (input.vision_ready && input.vision_fresh && input.mouse_right_held)
        {
            return GIMBAL_MODE_VISION;
        }
        return GIMBAL_MODE_SPEED;
    }

    // ---- 遥控器模式 ----
    if (input.s1 == Remote::DOWN && input.s2 == Remote::DOWN)
    {
        return GIMBAL_MODE_STOP;
    }
    if (input.s1 == Remote::DOWN && input.s2 == Remote::MIDDLE)
    {
        return GIMBAL_MODE_SPEED;
    }
    if (input.s1 == Remote::MIDDLE && input.s2 == Remote::MIDDLE)
    {
        if (input.vision_ready && input.vision_fresh)
        {
            return GIMBAL_MODE_VISION;
        }
        return GIMBAL_MODE_SPEED;
    }
    return GIMBAL_MODE_ANGLE;
}

void Class_Gimbal_FSM::Update(const Struct_Gimbal_Input &input, float current_angle)
{
    // 1. 确定模式
    uint8_t mode_command = DetermineMode(input);
    last_mode_command_ = mode_command;

    // 2. 计算 angle_input 和 speed_input
    float angle_input = input.joystick_speed;  // 遥控器默认用摇杆
    float speed_input = input.joystick_speed;

    if (mode_command == GIMBAL_MODE_VISION)
    {
        angle_input = input.vision_angle;
    }
    else if (mode_command == GIMBAL_MODE_SPEED)
    {
        bool is_keymouse = (input.s1 == Remote::DOWN && input.s2 == Remote::UP);
        if (is_keymouse)
        {
            speed_input = input.mouse_speed;
        }
        // 遥控器模式：speed_input 保持 joystick_speed
    }

    // 3. 状态转移（与原 Update 完全一致）
    uint8_t next_status = GIMBAL_STATUS_STOP;

    switch (mode_command)
    {
    case GIMBAL_MODE_ANGLE:
        next_status = GIMBAL_STATUS_ANGLE;
        break;
    case GIMBAL_MODE_SPEED:
        next_status = GIMBAL_STATUS_SPEED;
        break;
    case GIMBAL_MODE_VISION:
        next_status = GIMBAL_STATUS_VISION;
        break;
    case GIMBAL_MODE_STOP:
    default:
        next_status = GIMBAL_STATUS_STOP;
        break;
    }

    if (Get_Now_Status_Serial() != next_status)
    {
        Set_Status(next_status);
        mode_changed_flag = 1U;

        switch (next_status)
        {
        case GIMBAL_STATUS_ANGLE:
            Enter_Angle_State(current_angle);
            break;
        case GIMBAL_STATUS_SPEED:
            Enter_Speed_State();
            break;
        case GIMBAL_STATUS_VISION:
            Enter_Vision_State(current_angle);
            break;
        case GIMBAL_STATUS_STOP:
        default:
            Enter_Stop_State();
            break;
        }
    }

    switch (Get_Now_Status_Serial())
    {
    case GIMBAL_STATUS_ANGLE:
        control_type = GIMBAL_CONTROL_ANGLE;
        if (angle_target_initialized == 0U)
        {
            target_angle = Apply_Angle_Rule(current_angle);
            angle_target_initialized = 1U;
        }
        if (Absolute_Value(angle_input) > INPUT_DEADBAND)
        {
            target_angle += angle_input * config.angle_step;
            target_angle = Apply_Angle_Rule(target_angle);
        }
        control_output = target_angle;
        target_speed = 0.0f;
        break;

    case GIMBAL_STATUS_SPEED:
        control_type = GIMBAL_CONTROL_SPEED;
        if (Absolute_Value(speed_input) > INPUT_DEADBAND)
        {
            bool is_keymouse = (input.s1 == Remote::DOWN && input.s2 == Remote::UP);
            target_speed = speed_input * (is_keymouse ? config.mouse_speed_scale : config.speed_scale);
        }
        else
        {
            target_speed = 0.0f;
        }
        control_output = target_speed;
        break;

    case GIMBAL_STATUS_VISION:
        control_type = GIMBAL_CONTROL_ANGLE;
        if (mode_changed_flag == 0U)
        {
            float desired_angle = Apply_Angle_Rule(angle_input);
            if (config.vision_slope_inc > 0.0f || config.vision_slope_dec > 0.0f)
            {
                float now_planning = vision_slope_.GetNowPlanning();

                // yaw 轴角度环绕处理：将 target 和 feedback 展开到与 Now_Planning
                // 相同的"圈数"参考系，保证 SlopePlanning 内部数值比较正确
                if (config.normalize_angle != 0U)
                {
                    float diff_t = desired_angle - now_planning;
                    while (diff_t > 180.0f)  { desired_angle -= FULL_CIRCLE_DEG; diff_t -= FULL_CIRCLE_DEG; }
                    while (diff_t < -180.0f) { desired_angle += FULL_CIRCLE_DEG; diff_t += FULL_CIRCLE_DEG; }

                    float diff_f = current_angle - now_planning;
                    while (diff_f > 180.0f)  { current_angle -= FULL_CIRCLE_DEG; diff_f -= FULL_CIRCLE_DEG; }
                    while (diff_f < -180.0f) { current_angle += FULL_CIRCLE_DEG; diff_f += FULL_CIRCLE_DEG; }
                }

                vision_slope_.TIM_Calculate_PeriodElapsedCallback(desired_angle, current_angle);
                target_angle = Apply_Angle_Rule(vision_slope_.GetOut());

                // yaw 轴归一化后同步 Now_Planning，防止长期运行后漂移出合理范围，
                // 导致 unwrap 展开的目标角度与物理实际方向偏差过大
                if (config.normalize_angle != 0U)
                {
                    vision_slope_.SetNowPlanning(target_angle);
                }
            }
            else
            {
                target_angle = desired_angle;
            }
        }
        control_output = target_angle;
        target_speed = 0.0f;
        break;

    case GIMBAL_STATUS_STOP:
    default:
        control_type = GIMBAL_CONTROL_STOP;
        control_output = 0.0f;
        target_speed = 0.0f;
        break;
    }
}

void Class_Gimbal_FSM::Set_Target_Angle(float angle)
{
    target_angle = Apply_Angle_Rule(angle);
    angle_target_initialized = 1U;
}

void Class_Gimbal_FSM::Set_Target_Speed(float speed)
{
    target_speed = speed;
}

void Class_Gimbal_FSM::ReAnchor(float new_angle)
{
    target_angle = Apply_Angle_Rule(new_angle);
    control_output = target_angle;
    target_speed = 0.0f;
    angle_target_initialized = 1U;
    mode_changed_flag = 1U;
    vision_slope_.Reset(new_angle);
}

float Class_Gimbal_FSM::Get_Control_Output() const
{
    return control_output;
}

uint8_t Class_Gimbal_FSM::Get_Control_Type() const
{
    return control_type;
}

float Class_Gimbal_FSM::Get_Target_Angle() const
{
    return target_angle;
}

float Class_Gimbal_FSM::Get_Target_Speed() const
{
    return target_speed;
}

uint8_t Class_Gimbal_FSM::Take_Mode_Changed_Flag()
{
    uint8_t tmp = mode_changed_flag;
    mode_changed_flag = 0U;
    return tmp;
}

void Class_Gimbal_FSM::Enter_Stop_State()
{
    control_type = GIMBAL_CONTROL_STOP;
    control_output = 0.0f;
    target_speed = 0.0f;
}

void Class_Gimbal_FSM::Enter_Angle_State(float current_angle)
{
    target_angle = Apply_Angle_Rule(current_angle);
    target_speed = 0.0f;
    control_output = target_angle;
    control_type = GIMBAL_CONTROL_ANGLE;
    angle_target_initialized = 1U;
}

void Class_Gimbal_FSM::Enter_Speed_State()
{
    target_speed = 0.0f;
    control_output = 0.0f;
    control_type = GIMBAL_CONTROL_SPEED;
}

void Class_Gimbal_FSM::Enter_Vision_State(float current_angle)
{
    target_angle = Apply_Angle_Rule(current_angle);
    target_speed = 0.0f;
    control_output = target_angle;
    control_type = GIMBAL_CONTROL_ANGLE;
    angle_target_initialized = 1U;
    vision_slope_.Reset(current_angle);
}

float Class_Gimbal_FSM::Apply_Angle_Rule(float angle) const
{
    float result = angle;

    if (config.normalize_angle != 0U)
    {
        while (result >= 180.0f)
        {
            result -= FULL_CIRCLE_DEG;
        }
        while (result < -180.0f)
        {
            result += FULL_CIRCLE_DEG;
        }
    }

    if (config.limit_angle != 0U)
    {
        if (result > config.max_angle)
        {
            result = config.max_angle;
        }
        else if (result < config.min_angle)
        {
            result = config.min_angle;
        }
    }

    return result;
}
