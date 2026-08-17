#ifndef FRICTION_FSM_HPP
#define FRICTION_FSM_HPP

#include "../user/core/Alg/FSM/alg_fsm.hpp"
#include <math.h>

enum Enum_Friction_Mode
{
    FRICTION_MODE_STOP = 0,
    FRICTION_MODE_ON,
};

enum Enum_Friction_Status
{
    FRICTION_STOP = 0,
    FRICTION_STARTING,
    FRICTION_READY,
    FRICTION_STATUS_COUNT
};

/// @brief 三摩擦轮 FSM 的原始输入
typedef struct Struct_Friction_Input
{
    uint8_t s1, s2;        // 遥控器拨杆
    bool    friction_on;   // 键鼠模式下的摩擦轮开关
    bool    is_keymouse;   // 是否键鼠模式
} Struct_Friction_Input;

class Class_Friction_FSM : public Class_FSM
{
public:
    void Init();

    /// @brief FSM 内部根据输入判断 ON/OFF，并输出三摩擦轮目标速度
    void Update(const Struct_Friction_Input &input,
                float left_speed,
                float right_speed,
                float top_speed);

    float Get_Left_Control_Output();
    float Get_Right_Control_Output();
    float Get_Top_Control_Output();

    uint8_t Is_Ready();

private:
    bool Is_Speed_Ready(float left_speed,
                        float right_speed,
                        float top_speed) const;

private:
    float left_control_output  = 0.0f;
    float right_control_output = 0.0f;
    float top_control_output   = 0.0f;

    uint8_t current_mode = FRICTION_MODE_STOP;

    static constexpr float TARGET_SPEED = -6000.0f;
    static constexpr float READY_SPEED_THRESHOLD = -6000.0f;
    static constexpr uint32_t STARTING_TIME_COUNT = 200;
};

#endif
