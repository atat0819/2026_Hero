#ifndef GIMBAL_FSM_HPP
#define GIMBAL_FSM_HPP

#include "../user/core/Alg/FSM/alg_fsm.hpp"
#include <cstdint>

enum Enum_Gimbal_Mode_Command
{
    GIMBAL_MODE_STOP = 0,
    GIMBAL_MODE_ANGLE,
    GIMBAL_MODE_SPEED,
    GIMBAL_MODE_VISION,
};

enum Enum_Gimbal_Status
{
    GIMBAL_STATUS_STOP = 0,
    GIMBAL_STATUS_ANGLE,
    GIMBAL_STATUS_SPEED,
    GIMBAL_STATUS_VISION,
    GIMBAL_STATUS_COUNT,
};

enum Enum_Gimbal_Control_Type
{
    GIMBAL_CONTROL_STOP = 0,
    GIMBAL_CONTROL_ANGLE,
    GIMBAL_CONTROL_SPEED,
};

typedef struct Struct_Gimbal_FSM_Config
{
    float angle_step = 0.0f;
    float speed_scale = 0.0f;          // 遥控器摇杆速度增益
    float mouse_speed_scale = 0.2f;    // 键鼠速度增益 (deg/s per pixel)
    float min_angle = 0.0f;
    float max_angle = 0.0f;
    uint8_t limit_angle = 0U;
    uint8_t normalize_angle = 0U;
    float slew_rate_max = 0.0f;
};

/// @brief 云台 FSM 的原始输入，FSM 内部自行判断模式
typedef struct Struct_Gimbal_Input
{
    uint8_t  s1;               // 遥控器 S1 (1=UP, 3=MIDDLE, 2=DOWN)
    uint8_t  s2;               // 遥控器 S2
    float    joystick_speed;   // 遥控器摇杆值 (-1~1)
    float    mouse_speed;      // 键鼠速度：调用方预计算 (yaw: dx*GAIN, pitch: -dy*GAIN)
    int16_t  mouse_dx;         // 鼠标 X 位移 (保留)
    int16_t  mouse_dy;         // 鼠标 Y 位移 (保留)
    bool     mouse_right_held; // 右键是否按住（消抖后，供 2s 视觉延时用）
    bool     vision_ready;     // 视觉数据就绪
    bool     vision_fresh;     // 视觉数据新鲜
    float    vision_angle;     // 视觉目标角度
};

class Class_Gimbal_FSM : public Class_FSM
{
public:
    void Init(const Struct_Gimbal_FSM_Config &__config,
              uint8_t __initial_status = GIMBAL_STATUS_STOP);

    /// @brief 新接口：传入原始输入，FSM 内部根据 s1/s2 和鼠标状态自行判断模式
    /// @param input   原始输入结构体
    /// @param current_angle  当前角度（来自 IMU 或编码器）
    void Update(const Struct_Gimbal_Input &input, float current_angle);

    void Set_Target_Angle(float angle);
    void Set_Target_Speed(float speed);

    void ReAnchor(float new_angle);

    float   Get_Control_Output() const;
    uint8_t Get_Control_Type() const;
    uint8_t Get_Mode_Command() const { return last_mode_command_; }
    float   Get_Target_Angle() const;
    float   Get_Target_Speed() const;
    uint8_t Take_Mode_Changed_Flag();

private:
    void Enter_Stop_State();
    void Enter_Angle_State(float current_angle);
    void Enter_Speed_State();
    void Enter_Vision_State(float current_angle);
    float Apply_Angle_Rule(float angle) const;

    /// @brief 根据 s1/s2 和键鼠状态确定当前模式命令
    uint8_t DetermineMode(const Struct_Gimbal_Input &input) const;

private:
    Struct_Gimbal_FSM_Config config = {};

    float control_output = 0.0f;
    float target_angle = 0.0f;
    float target_speed = 0.0f;

    uint8_t control_type = GIMBAL_CONTROL_STOP;
    uint8_t angle_target_initialized = 0U;
    uint8_t mode_changed_flag = 0U;
    uint8_t last_mode_command_ = GIMBAL_MODE_STOP;
};

#endif
