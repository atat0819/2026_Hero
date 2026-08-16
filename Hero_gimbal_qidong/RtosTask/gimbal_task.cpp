#include "gimbal_task.hpp"
#include "FreeRTOS.h"
#include "queue.h"
#include "cmsis_os.h"
#include "remote_control_task.hpp"
#include "../communication_between_boards/input_dispatcher.hpp"
#include "../user/core/BSP/Motor/Dm/DmMotor.hpp"


BSP::Motor::DM::J4310<1> dm666(0x00,{0x02},{0x01}); // 单个 DM4310，电机 ID 0x01，master ID 0x02（反馈帧 ID=0x00+0x02，控制帧 ID=0x01）
 DM4310_State_t dm4310_state[1]; // 存储 DM4310 电机的状态数据



using Remote = BSP::REMOTE_CONTROL::RemoteController;
extern InputDispatcher input_dispatcher;

float feeder_target_angle = 0.0f; // 来自pid计算的目标角度
float feeder_current_angle = 0.0f; // 当前角度
float feeder_speed = 0.0f; // 当前速度
float feeder_iq = 0.0f; // 当前电流
float feeder_target_speed = 0.0f; // 来自pid计算的目标速度
float feeder_velocity_cmd_rad_s = 0.0f; // DM4310 MIT 速度指令 (rad/s)
float feeder_torque_ff_nm = 0.0f; // DM4310 MIT 力矩前馈 (Nm)

// 在 gimbal_task 外部或循环上方定义
float last_gimbal_roll = 0.0f; // 用于边缘触发检测的上一次滚轮值

// feeder_mode, trigger_pressed 已迁移至 FSM 内部判定

void DM4310_feedback();
static float Limit_Feeder_DM_Velocity(float velocity_rad_s);

Class_Feeder_FSM feeder_fsm;



static constexpr float FEEDER_DEG_TO_RAD = 0.017453292519943295f;
static constexpr float FEEDER_DM_VEL_LIMIT_RADPS = 30.0f;
static constexpr float FEEDER_DM_VEL_LIMIT_DEGPS =
    FEEDER_DM_VEL_LIMIT_RADPS / FEEDER_DEG_TO_RAD;
 constexpr float SINGLE_KD = 0.55f;
 constexpr float FEEDER_STATIC_FF_NM = 0.30f;
static constexpr float FEEDER_FF_MIN_ERROR_DEG = 25.0f;
static constexpr float FEEDER_FF_MAX_SPEED_RPM = 100.0f;

ALG::PID::PID feeder_angle_pid(6.0f, 0.00f, 0.0f, FEEDER_DM_VEL_LIMIT_DEGPS, 1000.0f, 100.0f);
ALG::PID::PID feeder_angle_pid_speed(6.0f, 0.00f, 0.0f, FEEDER_DM_VEL_LIMIT_DEGPS, 1000.0f, 100.0f);
ALG::PID::PID feeder_stop_pid(0.0f, 0.00f, 0.0f, 20000.0f, 1000.0f, 100.0f);

extern "C" void gimbal_task(void *argument)
{
    feeder_fsm.Init();

    dm666.On(1,BSP::Motor::DM::Model::MIT);
    osDelay(100); // 等待电机上电稳定
    DM4310_feedback();
    feeder_target_angle = dm4310_state[0].multi_angle;


    for(;;)
    {
/************************************************************************************************ */

/************************************************************************************** */
uint8_t force_stop = 0;

 DM4310_feedback();
/******************************************************************************* */
feeder_fsm.TIM_Calculate_PeriodElapsedCallback();
/************************************************************************************ */
// FSM 内部自行判断模式：传入原始输入，FSM 根据 s1/s2/键鼠 自行决定
bool is_keymouse = (RemoteData.s1 == Remote::DOWN && RemoteData.s2 == Remote::UP);

Struct_Feeder_Input feeder_input = {};
feeder_input.s1             = RemoteData.s1;
feeder_input.s2             = RemoteData.s2;
feeder_input.feeder_on      = input_dispatcher.IsFeederOn();
feeder_input.is_single_shot = input_dispatcher.IsSingleShot();
feeder_input.fire_triggered = input_dispatcher.IsFireTriggered();
feeder_input.scroll_value   = remoteController.get_scroll_();
feeder_input.vision_fire    = vision_comm.IsFireCommanded();
feeder_input.is_keymouse    = is_keymouse;

force_stop = 0;
if (!is_keymouse)
{
    // 遥控器模式下特定挡位 = 强制停止
    if ((RemoteData.s1 == Remote::DOWN  && RemoteData.s2 == Remote::DOWN)  ||
        (RemoteData.s1 == Remote::MIDDLE && RemoteData.s2 == Remote::DOWN) ||
        (RemoteData.s1 == Remote::UP    && RemoteData.s2 == Remote::DOWN))
    {
        force_stop = 1;
    }
}
else
{
    // 键鼠模式下 R=OFF 时 = 停止
    if (!input_dispatcher.IsFeederOn())
    {
        force_stop = 1;
    }
}

last_gimbal_roll = RemoteData.gimbal_roll;
// 触发检测和视觉开火已迁移至 feeder_fsm 内部，gimbal_task 不再处理
/************************************************************************************** */
    feeder_current_angle = dm4310_state[0].multi_angle; // 从DM4310电机获取多圈累积角度（输出轴坐标, °）
    feeder_speed = dm4310_state[0].velocity_rpm;
    feeder_iq = dm4310_state[0].current_a;

feeder_fsm.Update(feeder_input, feeder_current_angle, feeder_speed, feeder_iq);
/********************************************************************************** */

    // 跟踪控制类型变化，用于检测从其他模式切换到速度模式 (SPEED) 的时刻
    static uint8_t prev_control_type = FEEDER_CONTROL_STOP;
    uint8_t current_control_type = feeder_fsm.Get_Control_Type();

    if (force_stop)
{
    // 强制停止：复位PID，跳过FSM输出，直接刹车
    feeder_angle_pid.reset();
    feeder_angle_pid_speed.reset();
    feeder_velocity_cmd_rad_s = 0.0f;
    feeder_torque_ff_nm = 0.0f;
}
else if (current_control_type == FEEDER_CONTROL_ANGLE)
{
    const float feeder_angle_error =
        feeder_fsm.Get_Control_Output() - feeder_fsm.Get_Accumulated_Angle();
    const float feeder_abs_angle_error =
        (feeder_angle_error >= 0.0f) ? feeder_angle_error : -feeder_angle_error;
    const float feeder_abs_speed =
        (feeder_speed >= 0.0f) ? feeder_speed : -feeder_speed;

    feeder_target_speed = feeder_angle_pid.UpDate(
        feeder_fsm.Get_Control_Output(),
        feeder_fsm.Get_Accumulated_Angle()); // PID 统一用度，输出目标速度 (deg/s)

    feeder_velocity_cmd_rad_s = feeder_target_speed * FEEDER_DEG_TO_RAD;
    feeder_torque_ff_nm = 0.0f;

    if (feeder_abs_angle_error > FEEDER_FF_MIN_ERROR_DEG &&
        feeder_abs_speed < FEEDER_FF_MAX_SPEED_RPM)
    {
        if (feeder_velocity_cmd_rad_s > 0.0f)
        {
            feeder_torque_ff_nm = FEEDER_STATIC_FF_NM;
        }
        else if (feeder_velocity_cmd_rad_s < 0.0f)
        {
            feeder_torque_ff_nm = -FEEDER_STATIC_FF_NM;
        }
    }
}

else if (current_control_type == FEEDER_CONTROL_SPEED)
{
    // 刚切换到速度模式时，将目标角度同步为当前累积角度，防止从单发切换连发时
    // feeder_target_angle 还停留在旧值，导致大位置误差 → 先反转再正转
    if (prev_control_type != FEEDER_CONTROL_SPEED)
    {
        feeder_target_angle = feeder_fsm.Get_Accumulated_Angle();
    }

    feeder_target_angle -= hz_to_rotor_angle_per_frame(3.0f);

    feeder_target_speed = feeder_angle_pid_speed.UpDate(
        feeder_target_angle,
        feeder_fsm.Get_Accumulated_Angle()
    ); // PID 统一用度，输出目标速度 (deg/s)

    feeder_velocity_cmd_rad_s = feeder_target_speed * FEEDER_DEG_TO_RAD;
    feeder_torque_ff_nm = 0.0f;
}
else
{
    feeder_angle_pid.reset();
    feeder_angle_pid_speed.reset();
    feeder_velocity_cmd_rad_s = 0.0f;
    feeder_torque_ff_nm = 0.0f;
}

    prev_control_type = current_control_type;

/********************************************************************************** */
// 拨弹轮通过 DM4310 MIT 模式控制：KP=0，使用 vel + KD 做速度阻尼
{
    feeder_velocity_cmd_rad_s = Limit_Feeder_DM_Velocity(feeder_velocity_cmd_rad_s);
    dm666.ctrl_Mit(1, 0.0f, feeder_velocity_cmd_rad_s, 0.0f, SINGLE_KD, feeder_torque_ff_nm);
}
/**************************************************************************** */
//vofa_send(feeder_fsm.Get_Control_Output(),feeder_fsm.Get_Accumulated_Angle(), feeder_speed, 360, 0, 0); // 发送数据到VOFA
vofa_send(
    feeder_fsm.Get_Accumulated_Angle(),                    // ch1: 当前累积角度 (°)
    feeder_fsm.Get_Single_Shot_Target_Angle(),             // ch2: 单发目标角度 (°)
    feeder_fsm.Get_Accumulated_Angle() -
        feeder_fsm.Get_Single_Shot_Target_Angle(),         // ch3: 角度误差 (°) 正=未到位
    feeder_speed,                                          // ch4: 实际转速 (RPM)
    feeder_target_speed,                                   // ch5: PID 输出目标速度 (°/s)
    feeder_velocity_cmd_rad_s,                             // ch6: MIT 速度指令 (rad/s)
    feeder_torque_ff_nm,                                   // ch7: 力矩前馈 (Nm)
    feeder_iq,                                             // ch8: 电机反馈力矩 (Nm)
    (float)feeder_fsm.Get_Now_Status_Serial(),             // ch9: FSM 状态 0=STOP 1=单发 2=连发 3=反转 4=冷却
    (float)feeder_fsm.Get_Control_Type(),                  // ch10: 控制类型 0=STOP 1=速度 2=角度
    remoteController.get_scroll_());                       // ch11: 滚轮触发输入 (-1~1)

/****************************************************************************** */
vTaskDelay(5); // 每5ms执行一次控制循环
    }
}


float hz_to_rotor_angle_per_frame(float fire_hz)
{
    const float slots_per_rotation = 8.0f;
    const float angle_per_slot = 360.0f / slots_per_rotation; // 45 deg
    const float external_reduction = 2.75f;
    const float internal_reduction = 1.0f;  // DM4310 反馈为输出轴坐标，无内部减速比换算
    const float control_period = 0.005f;    // 与 vTaskDelay(5) 一致

    return fire_hz * angle_per_slot * external_reduction * internal_reduction * control_period;
}

static float Limit_Feeder_DM_Velocity(float velocity_rad_s)
{
    if (velocity_rad_s > FEEDER_DM_VEL_LIMIT_RADPS)
    {
        return FEEDER_DM_VEL_LIMIT_RADPS;
    }
    else if (velocity_rad_s < -FEEDER_DM_VEL_LIMIT_RADPS)
    {
        return -FEEDER_DM_VEL_LIMIT_RADPS;
    }

    return velocity_rad_s;
}

void DM4310_feedback() {
    for (int i = 0; i < 1; i++) {
        uint8_t motor_id = i + 1;
        // 角度：映射到 0~360° / 0~2π（转子坐标，与FSM一致）
        dm4310_state[i].angle_deg = dm666.getAngleDeg(motor_id); // 驱动层已处理为0~360°
        dm4310_state[i].angle_rad = dm666.getAngleRad(motor_id); // 原始弧度，MIT控制用
        // 多圈角度：直接使用底座累加值
        dm4310_state[i].multi_angle = dm666.getAddAngleDeg(motor_id);
        dm4310_state[i].multi_rad = dm4310_state[i].multi_angle * 0.017453292519611f;
        // 速度：转子速度（与FSM的目标速度坐标一致）
        dm4310_state[i].velocity_rpm  = dm666.getVelocityRpm(motor_id);
        dm4310_state[i].velocity_rads = dm666.getVelocityRads(motor_id);
        // 力矩：DM电机反馈的是力矩不是电流
        dm4310_state[i].current_a    = dm666.getTorque(motor_id);
        dm4310_state[i].temperature  = dm666.getTemperature(motor_id);
    }
}
