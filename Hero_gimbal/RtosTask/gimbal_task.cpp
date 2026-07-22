#include "gimbal_task.hpp"
#include "FreeRTOS.h"
#include "queue.h"
#include "cmsis_os.h"
#include "remote_control_task.hpp"
#include "../communication_between_boards/input_dispatcher.hpp"

const uint8_t chassis_motor_idxs[4] = {1, 2, 3, 4}; // 4 个电机的接收偏移 ID
BSP::Motor::Dji::GM3508<4> friction_motor(0x200, chassis_motor_idxs, 0x200); // 底盘电机控制器示例，初始ID为0x200，发送ID为0x2FF

using Remote = BSP::REMOTE_CONTROL::RemoteController;
extern InputDispatcher input_dispatcher;

DJI3508_State_t dji3508_state[4]; // 存储四个电机的状态数据

//1 号电机是拨弹轮，2、3、4 号电机是左右和上方摩擦轮
float feeder_target_angle = 0.0f; // 来自pid计算的目标角度
float feeder_current_angle = 0.0f; // 当前角度
float feeder_speed = 0.0f; // 当前速度
float feeder_iq = 0.0f; // 当前电流
float feeder_target_speed = 0.0f; // 来自pid计算的目标速度
float feeder_out = 0.0f; // 最终控制输出

float friction_current_speed_left = 0.0f; // 当前速度
float friction_current_speed_right = 0.0f; // 当前速度
float friction_current_speed_above = 0.0f; // 当前速度

float left_out = 0.0f; // 左侧摩擦轮控制输出
float right_out = 0.0f; // 右侧摩擦轮控制输出
float above_out = 0.0f; // 上方摩擦轮控制输出

// 在 gimbal_task 外部或循环上方定义
float last_gimbal_roll = 0.0f; // 用于边缘触发检测的上一次滚轮值

// feeder_mode, friction_mode, trigger_pressed 已迁移至 FSM 内部判定

void DJI3508_feedback();

Class_Feeder_FSM feeder_fsm;
Class_Friction_FSM friction_fsm;

ALG::PID::PID feeder_angle_pid(2.0f, 0.00f, 0.0f, 5000.0f, 1000.0f, 100.0f);
ALG::PID::PID feeder_speed_pid(1.0f, 0.00f, 0.0f, 5000.0f, 1000.0f, 100.0f);
ALG::PID::PID feeder_angle_pid_speed(2.5f, 0.00f, 0.0f, 5000.0f, 1000.0f, 100.0f);
ALG::PID::PID feeder_stop_pid(0.0f, 0.00f, 0.0f, 20000.0f, 1000.0f, 100.0f);


ALG::PID::PID feeder_speed_pid_speed(1.0f, 0.00f, 0.0f, 5000.0f, 1000.0f, 100.0f);
ALG::PID::PID left_friction_pid(8.0f, 0.00f, 0.0f, 16384.0f, 1000.0f, 100.0f);
ALG::PID::PID right_friction_pid(8.0f, 0.00f, 0.0f, 16384.0f, 1000.0f, 100.0f);
ALG::PID::PID above_friction_pid(8.0f, 0.00f, 0.0f, 16384.0f, 1000.0f, 100.0f);


float leijia = 0;

extern "C" void gimbal_task(void *argument)
{
    
    feeder_fsm.Init();
    friction_fsm.Init();


    for(;;)
    {
/************************************************************************************************ */

/************************************************************************************** */
uint8_t force_stop = 0;

 DJI3508_feedback();
/******************************************************************************* */
friction_fsm.TIM_Calculate_PeriodElapsedCallback();
feeder_fsm.TIM_Calculate_PeriodElapsedCallback();
/************************************************************************************ */
// FSM 内部自行判断模式：传入原始输入，FSM 根据 s1/s2/键鼠 自行决定
bool is_keymouse = (RemoteData.s1 == Remote::DOWN && RemoteData.s2 == Remote::UP);

Struct_Feeder_Input feeder_input = {};
feeder_input.s1             = RemoteData.s1;
feeder_input.s2             = RemoteData.s2;
feeder_input.friction_on    = input_dispatcher.IsFrictionOn();
feeder_input.is_single_shot = input_dispatcher.IsSingleShot();
feeder_input.fire_triggered = input_dispatcher.IsFireTriggered();
feeder_input.scroll_value   = remoteController.get_scroll_();
feeder_input.vision_fire    = vision_comm.IsFireCommanded();
feeder_input.is_keymouse    = is_keymouse;

Struct_Friction_Input friction_input = {};
friction_input.s1          = RemoteData.s1;
friction_input.s2          = RemoteData.s2;
friction_input.friction_on = input_dispatcher.IsFrictionOn();
friction_input.is_keymouse = is_keymouse;

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
    if (!input_dispatcher.IsFrictionOn())
    {
        force_stop = 1;
    }
}

last_gimbal_roll = RemoteData.gimbal_roll;
// 触发检测和视觉开火已迁移至 feeder_fsm 内部，gimbal_task 不再处理
/************************************************************************************** */
    feeder_current_angle = dji3508_state[0].angle_deg; 
    feeder_speed = dji3508_state[0].velocity_rpm;
    feeder_iq = dji3508_state[0].current_a;
    
    friction_current_speed_left = dji3508_state[1].velocity_rpm; // 左侧摩擦轮当前速度
    friction_current_speed_right = dji3508_state[2].velocity_rpm; // 右侧摩擦轮当前速度
    friction_current_speed_above = dji3508_state[3].velocity_rpm; // 上方摩擦轮当前速度

feeder_fsm.Update(feeder_input, feeder_current_angle, feeder_speed, feeder_iq);
leijia = feeder_fsm.Get_Accumulated_Angle();
/********************************************************************************** */

    // 跟踪控制类型变化，用于检测从其他模式切换到速度模式 (SPEED) 的时刻
    static uint8_t prev_control_type = FEEDER_CONTROL_STOP;
    uint8_t current_control_type = feeder_fsm.Get_Control_Type();

    if (force_stop)
{
    // 强制停止：复位PID，跳过FSM输出，直接刹车
    feeder_angle_pid.reset();
    feeder_speed_pid.reset();
    feeder_angle_pid_speed.reset();
    feeder_speed_pid_speed.reset();
    feeder_out = 0.0f;
}
else if (current_control_type == FEEDER_CONTROL_ANGLE)
{
    feeder_target_speed = feeder_angle_pid.UpDate(
        feeder_fsm.Get_Control_Output(),
        feeder_fsm.Get_Accumulated_Angle()); // 位置环输入目标角度和当前累积角度，输出目标速度

    feeder_out = feeder_speed_pid.UpDate(feeder_target_speed, feeder_speed);
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
    );

    feeder_out = feeder_speed_pid_speed.UpDate(feeder_target_speed, feeder_speed);
}
else
{
    feeder_angle_pid.reset();
    feeder_speed_pid.reset();
    feeder_angle_pid_speed.reset();
    feeder_speed_pid_speed.reset();
    feeder_out = 0.0f;
}

    prev_control_type = current_control_type;

/********************************************************************************** */
friction_fsm.Update(friction_input, friction_current_speed_left, friction_current_speed_right, friction_current_speed_above);

 left_out = left_friction_pid.UpDate(
    friction_fsm.Get_Left_Control_Output(),
    friction_current_speed_left);

 right_out = right_friction_pid.UpDate(
    friction_fsm.Get_Right_Control_Output(),
    friction_current_speed_right);

 above_out = above_friction_pid.UpDate(
    friction_fsm.Get_Above_Control_Output(),
    friction_current_speed_above);
/********************************************************************************** */
	friction_motor.setCAN((int16_t)feeder_out, 1);
friction_motor.setCAN((int16_t)left_out, 2);
friction_motor.setCAN((int16_t)right_out, 3);
friction_motor.setCAN((int16_t)above_out, 4);

friction_motor.sendCAN();
/**************************************************************************** */
//vofa_send(feeder_fsm.Get_Control_Output(),feeder_fsm.Get_Accumulated_Angle(), feeder_speed, 360, 0, 0); // 发送数据到VOFA
/****************************************************************************** */
vTaskDelay(5); // 每5ms执行一次控制循环
    }
}


void DJI3508_feedback() {
    for (int i = 0; i < 4; i++) {
        uint8_t motor_id = i + 1; // 电机逻辑 ID 通常从 1 开始
         dji3508_state[i].multi_angle = friction_motor.getAddAngleDeg(motor_id) ; // 获取多圈角度，单位为度
        dji3508_state[i].angle_deg   = friction_motor.getAngleDeg(motor_id); // 如果拨弹轮也是19:1减速比，则同样除以19
        dji3508_state[i].angle_rad   = friction_motor.getAngleRad(motor_id);
if (motor_id == 1) {
            dji3508_state[i].velocity_rpm = friction_motor.getVelocityRpm(motor_id) ;
        } else {
            dji3508_state[i].velocity_rpm = friction_motor.getVelocityRpm(motor_id); // 摩擦轮若还是19:1则保留
        }        dji3508_state[i].velocity_rads  = friction_motor.getVelocityRads(motor_id);   //角速度，用这个控制电机
        dji3508_state[i].current_a     = friction_motor.getCurrent(motor_id);
        dji3508_state[i].temperature        = friction_motor.getTemperature(motor_id);
    }
}

float hz_to_rotor_angle_per_frame(float fire_hz)
{
    const float slots_per_rotation = 8.0f;
    const float angle_per_slot = 360.0f / slots_per_rotation; // 45 deg
    const float external_reduction = 2.75f;
    const float internal_reduction = 19.0f;
    const float control_period = 0.005f;  // 与 vTaskDelay(5) 一致

    return fire_hz * angle_per_slot * external_reduction * internal_reduction * control_period;
}
