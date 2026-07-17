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

// 按 README.md 的使用方式:
// 1. 外部周期调用 TIM_Calculate_PeriodElapsedCallback() 维护当前状态时间
// 2. Update() 只根据输入和 Count_Time 执行状态转移
// friction_fsm.hpp 关键修改点

/// @brief 摩擦轮 FSM 的原始输入
typedef struct Struct_Friction_Input
{
    uint8_t  s1, s2;         // 遥控器拨杆
    bool     friction_on;    // R 键状态 (InputDispatcher) 或遥控器逻辑
    bool     is_keymouse;    // 是否键鼠模式
};

class Class_Friction_FSM : public Class_FSM
{
public:
    void Init();

    /// @brief 新接口：FSM 内部根据输入判断 ON/OFF
    void Update(const Struct_Friction_Input &input,
                float left_speed,
                float right_speed,
                float above_speed); 

    float Get_Left_Control_Output();
    float Get_Right_Control_Output();
    float Get_Above_Control_Output();

    uint8_t Is_Ready();

private:
    // 修改：确保这里的声明与 cpp 文件一致（增加 above_speed）
    bool Is_Speed_Ready(float left_speed, float right_speed, float above_speed) const;

private:
    float left_control_output = 0.0f;
    float right_control_output = 0.0f;
    float above_control_output = 0.0f; // 确保这个变量已存在

    uint8_t current_mode = FRICTION_MODE_STOP;

    static constexpr float TARGET_SPEED = 3200.0f;
    static constexpr float READY_SPEED_THRESHOLD = 3200.0f; 
    static constexpr uint32_t STARTING_TIME_COUNT = 200;
};
#endif
