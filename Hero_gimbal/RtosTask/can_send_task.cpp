#include "can_send_task.hpp"
#include "remote_control_task.hpp"
#include "boards_communication.hpp"
#include "FreeRTOS.h"
#include "queue.h"
#include "cmsis_os.h"
#include "gimbal_task.hpp"
#include "usart.h"
#include "usbd_cdc_if.h"
#include "../feeder_fsm/gimbal_fsm.hpp"
#include "../communication_between_boards/refree_receive.hpp"
#include "../communication_between_boards/input_dispatcher.hpp"
#include "../user/core/Alg/Feedforward/Feedforward.hpp"
#include "../user/core/HAL/UART/uart_hal.hpp"
#include <string.h>

extern "C" USBD_HandleTypeDef hUsbDeviceFS;

namespace
{
constexpr float YAW_HALF_CIRCLE_DEG = 180.0f;
constexpr float YAW_FULL_CIRCLE_DEG = 360.0f;
constexpr float YAW_MAX_UNWRAP_STEP_DEG = 45.0f;

float AbsFloat(float value)
{
    return (value >= 0.0f) ? value : -value;
}

float GetContinuousYawAngle(float raw_yaw, bool force_reset = false)
{
    static bool initialized = false;
    static float last_raw_yaw = 0.0f;
    static float continuous_yaw = 0.0f;

    if (!initialized || force_reset)
    {
        initialized = true;
        last_raw_yaw = raw_yaw;
        continuous_yaw = raw_yaw;
        return continuous_yaw;
    }

    float delta = raw_yaw - last_raw_yaw;
    while (delta > YAW_HALF_CIRCLE_DEG)
    {
        delta -= YAW_FULL_CIRCLE_DEG;
    }
    while (delta < -YAW_HALF_CIRCLE_DEG)
    {
        delta += YAW_FULL_CIRCLE_DEG;
    }

    if (AbsFloat(delta) > YAW_MAX_UNWRAP_STEP_DEG)
    {
        return continuous_yaw;
    }

    continuous_yaw += delta;
    last_raw_yaw = raw_yaw;
    return continuous_yaw;
}
}


uint8_t txDataBuffer[8], rxDataBuffer0[8], rxDataBuffer1[8];
CAN_TxHeaderTypeDef txHeader;
CAN_RxHeaderTypeDef rxHeader0, rxHeader1;

/******************************************************************************** */
// ============ 角度双环（普通角度模式：外环角度PID → 目标速度，内环速度PID → 扭矩） ============
// yaw轴角度环外环PID
ALG::PID::PID yaw_angle_pid(17.0f, 0.0f, 0.0f, 5000.0f, 1000.0f, 100.0f);
// yaw轴角度环内环PID
ALG::PID::PID yaw_angle_to_speed_pid(3.7f, 0.0f, 0.0f, 5000.0f, 1000.0f, 100.0f);
// pitch轴角度环外环PID
ALG::PID::PID pitch_angle_pid(16.5f, 0.015f, 0.0f, 5000.0f, 1000.0f, 100.0f);
// pitch轴角度环内环PID
ALG::PID::PID pitch_angle_to_speed_pid(4.7f, 0.01f, 0.0f, 5000.0f, 1000.0f, 100.0f);
// 角度模式前馈：控制周期 1 ms，输出量纲为 LK4005 扭矩命令。
Alg::Feedforward::Velocity yaw_angle_velocity_ff(
    0.0f, 0.001f);
Alg::Feedforward::Friction yaw_angle_friction_ff(
    0.0f, 0.0f);
Alg::Feedforward::GimbalFullCompensation yaw_angle_dynamics_ff(
    0.0f, 0.001f, 0.0f, 0.0f, 50.0f);
// Pitch 角度模式前馈：仅保留惯量项，静摩擦由独立前馈处理。
Alg::Feedforward::GimbalFullCompensation pitch_angle_dynamics_ff(
    0.00f, 0.001f, 0.0f, 0.0f);
Alg::Feedforward::Friction pitch_angle_friction_ff(
    0.0f, 15.0f);
// pitch 重力补偿前馈（通用无状态：角度/视觉/速度模式共用）
Alg::Feedforward::Gravity pitch_gravity_ff(
    55.0f, 0.0f);
// pitch 扰动观测补偿：保守起步，实车再调 b/gain/limit
Alg::Feedforward::UDE pitch_ude(
    0.0f, 0.0f, 0.001f, 20.0f, 100.0f, 0.0f);
/********************************************************************************** */

// ============ 速度单环（遥控器/键鼠直驱速度控制） ============
//yaw轴遥控器单速度环PID（继承原 yaw_single_speed_pid 参数，遥控器手感不变）
ALG::PID::PID yaw_remote_speed_pid(4.0f, 0.05f, 0.0f, 5000.0f, 1000.0f, 100.0f);
//pitch轴遥控器单速度环PID（继承原 pitch_single_speed_pid 参数，遥控器手感不变）
ALG::PID::PID pitch_remote_speed_pid(4.0f, 0.04f, 0.0f, 5000.0f, 1000.0f, 100.0f);
//yaw轴键鼠单速度环PID（初始参数同遥控器版，键鼠手感单独调参）
ALG::PID::PID yaw_keymouse_speed_pid(3.5f, 0.05f, 0.0f, 5000.0f, 1000.0f, 100.0f);
//pitch轴键鼠单速度环PID（初始参数同遥控器版，键鼠手感单独调参）
ALG::PID::PID pitch_keymouse_speed_pid(4.5f, 0.07f, 0.0f, 5000.0f, 1000.0f, 100.0f);
/********************************************************************************** */

// ============ 视觉模式（独立 PID 与前馈状态，防止与普通角度模式共用微分历史） ============
// yaw 视觉角度环外环PID
ALG::PID::PID yaw_version_angle_pid(10.0f, 0.0f, 0.0f, 2000.0f, 1000.0f, 100.0f);
// yaw 视觉角度环内环PID
ALG::PID::PID yaw_version_speed_pid(2.8f, 0.0f, 0.0f, 5000.0f, 1000.0f, 100.0f);
// pitch 视觉角度环外环PID（暂未启用，全零）
ALG::PID::PID pitch_version_angle_pid(16.5f, 0.015f, 0.0f, 2000.0f, 1000.0f, 100.0f);
// pitch 视觉角度环内环PID（暂未启用，全零）
ALG::PID::PID pitch_version_speed_pid(4.7f, 0.01f, 0.0f, 5000.0f, 1000.0f, 100.0f);
// 视觉模式前馈：控制周期 1 ms，输出量纲为 LK4005 扭矩命令。
Alg::Feedforward::Velocity yaw_vision_velocity_ff(
    0.0f, 0.001f);
Alg::Feedforward::Friction yaw_vision_friction_ff(
    0.0f, 0.0f);
Alg::Feedforward::GimbalFullCompensation yaw_vision_dynamics_ff(
    0.05f, 0.001f, 0.02f, 50.0f, 50.0f);
// Pitch 视觉模式使用独立前馈状态，参数与普通角度模式一致。
Alg::Feedforward::GimbalFullCompensation pitch_vision_dynamics_ff(
    0.0f, 0.001f, 0.0f, 0.0f);
Alg::Feedforward::Friction pitch_vision_friction_ff(
    0.0f, 15.0f);
// Yaw speed-mode full compensation FF, independent from angle/vision mode state.
Alg::Feedforward::GimbalFullCompensation yaw_speed_dynamics_ff(
    0.0f, 0.001f, 0.0f, 0.0f, 20.0f);

/*********************************************************************** */

using Remote = BSP::REMOTE_CONTROL::RemoteController;


Struct_Gimbal_FSM_Config yaw_gimbal_fsm_config;   // yaw 轴的 FSM 配置结构体(限位、归一化等参数)
Struct_Gimbal_FSM_Config pitch_gimbal_fsm_config;  // pitch 轴的 FSM 配置结构体
Class_Gimbal_FSM yaw_gimbal_fsm;   // 定义 yaw 轴的 FSM 实例
Class_Gimbal_FSM pitch_gimbal_fsm;   // 定义 pitch 轴的 FSM 实例

MG4005_State_t mg4005_state[2]; // 存储两个电机的状态数据
RemoteData_t RemoteData;
IMU_t ImuData_user; // 存储解析后的 IMU 数据的结构体


float yaw_target_angle = 0.0f;   // 来自遥控器的目标角度
float pitch_target_angle = 300.0f; // 来自遥控器的目标角度
float yaw_current_angle = 0.0f;   // 来自IMU的当前角度
float pitch_current_angle = 0.0f; // 来自IMU的当前角度

float yaw_target_speed = 0.0f;   // 来自pid计算的目标速度
float pitch_target_speed = 0.0f; // 来自pid计算的目标速度
float yaw_current_speed = 0.0f;   // 来自IMU的当前速度
float pitch_current_speed = 0.0f; // 来自IMU的当前速度

float filted_yaw = 0.0f; // 滤波后的角度
float filted_gyroz = 0.0f; // 滤波后的陀螺仪 z 轴角速度



uint8_t yaw_mode = GIMBAL_MODE_SPEED;
uint8_t pitch_mode = GIMBAL_MODE_SPEED;


uint8_t gimbal_recv_idxs[2] = {1, 2};     // 接收偏移 ID
uint32_t gimbal_send_idxs[2] = {1, 2};    // 发送偏移 ID (相对于 0x140)

// 2 个云台电机
// GM6020 反馈ID = 0x204 + 拨码值，拨码1→0x205，拨码2→0x206
//BSP::Motor::Dji::GM6020<2> gimbal_motor(0x204, gimbal_motor_idxs, 0x1FF);
BSP::Motor::LK::LK4005<2> gimbal_motor(0x140, gimbal_recv_idxs, gimbal_send_idxs); // 底盘电机控制器示例，初始ID为0x200，发送ID为0x2FF

void CAN2_RxCallback(HAL::CAN::Frame& frame);
void ControlTask();
static bool IMU_Fault_Protection(float &yaw_angle, float &yaw_speed,float &pitch_angle, float &pitch_speed);
                                  




extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  	HAL::CAN::Frame frame;   

    if (hcan->Instance == CAN1)
    {
        HAL::CAN::get_can_bus_instance().get_can1().receive(frame); // 从 CAN1 接收数据到 frame 结构体
    }

}

extern "C" void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    HAL::CAN::Frame frame;

    if (hcan->Instance == CAN2)
    {
        while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO1) > 0)
        {
            if (!HAL::CAN::get_can_bus_instance().get_can2().receive(frame))
            {
                break;
            }
        }
    }
}

extern "C" void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan) {
    if (hcan->Instance == CAN2) {
        // 如果发生溢出 (ErrorCode & HAL_CAN_ERROR_RX_FOV1)
        // 必须重启 CAN 接收或清除错误位
        HAL_CAN_ResetError(hcan);
    }
    else if (hcan->Instance == CAN1)
    {
        // CAN1 上挂着拨弹轮 C620 / 摩擦轮 / yaw。总线错误(尤其 bus-off)后若不复位,
        // CAN1 永久停摆, 这些电机将永远保持最后一条指令一直转 (疯转根因之一)。
        // 使能 ABOM: bxCAN 在 bus-off 后自动恢复; 再清错误码让 HAL 保持可用。
        SET_BIT(hcan->Instance->MCR, CAN_MCR_ABOM);
        HAL_CAN_ResetError(hcan);
    }
}

float yaw_error = 0.0f;
float pitch_error = 0.0f;
float yaw_control_output = 0.0f;
float yaw_speed_pid_output = 0.0f;
float yaw_speed_ff_output = 0.0f;
float pitch_control_output = 0.0f;
float pitch_ude_output = 0.0f;
float pitch_last_control_output = 0.0f;
volatile uint8_t gimbal_remote_offline = 0;

void vofa_send(float x1, float x2, float x3, float x4, float x5, float x6);
void vofa_sendN(const float *data, uint8_t count);


extern "C" void can_send_task(void *argument)
{
        // 等待裁判系统就位（约需5秒），确保电机上电前裁判系统已就绪
    osDelay(1000);

/*********************************************************************************** */
yaw_gimbal_fsm_config.angle_step = 0.20f;
yaw_gimbal_fsm_config.speed_scale = 110.0f;
yaw_gimbal_fsm_config.mouse_speed_scale = 0.2f;   // 键鼠 yaw 手感 (°/s per pixel)
yaw_gimbal_fsm_config.min_angle = 0.0f;
yaw_gimbal_fsm_config.max_angle = 0.0f;
yaw_gimbal_fsm_config.limit_angle = 0U;
yaw_gimbal_fsm_config.normalize_angle = 1U;
yaw_gimbal_fsm_config.continuous_angle = 1U;
yaw_gimbal_fsm.Init(yaw_gimbal_fsm_config, GIMBAL_STATUS_STOP);

pitch_gimbal_fsm_config.angle_step = 0.14f;
pitch_gimbal_fsm_config.speed_scale = 95.0f;
pitch_gimbal_fsm_config.mouse_speed_scale = 0.2f;   // 键鼠 pitch 手感 (°/s per pixel)
pitch_gimbal_fsm_config.min_angle = -11.75f;   // IMU pitch 最低点
pitch_gimbal_fsm_config.max_angle = 22.8f;    // IMU pitch 最高点
pitch_gimbal_fsm_config.limit_angle = 1U;
pitch_gimbal_fsm_config.normalize_angle = 0U;
pitch_gimbal_fsm_config.continuous_angle = 0U;
pitch_gimbal_fsm.Init(pitch_gimbal_fsm_config, GIMBAL_STATUS_STOP);
/************************************************************************************* */

	static uint32_t can2_tick = 0;
static uint8_t last_s1 = 0xFF;
static uint8_t last_s2 = 0xFF;
static uint16_t startup_protect = 0; // 上电保护计数器

	// --- 在进入循环前初始化 ---
    // 这行代码执行时会触发 HAL_CAN_Start 和开启接收中断     
	HAL::CAN::get_can_bus_instance(); // 触发 CAN bus 初始化：HAL_CAN_Start + 激活中断通知
	    auto &can1 = HAL::CAN::get_can_bus_instance().get_device(HAL::CAN::CanDeviceId::HAL_Can1);
        auto &can2 = HAL::CAN::get_can_bus_instance().get_device(HAL::CAN::CanDeviceId::HAL_Can2);
    can1.register_rx_callback([](const HAL::CAN::Frame &frame) {
        if (frame.id == 0x142)
    {
        // 这是底盘电机的数据，交给 chassis_motor 解析
        gimbal_motor.Parse(frame);
    }
    if (frame.id >= 0x201 && frame.id <= 0x204)
    {
        friction_motor.Parse(frame);
    }
    });

    can2.register_rx_callback([](const HAL::CAN::Frame &frame) {
        if (frame.id == 0x141)
        {
            // 这是云台电机的数据，交给 gimbal_motor 解析
            gimbal_motor.Parse(frame);
        }
        if (frame.id == 0x520)
        {
            // 底盘发来的裁判系统热量数据
            Communication::GimbalRefree::instance().parse(frame);
        }
    });


// --- 改成这个顺序 ---
gimbal_motor.setAllowAccumulate(2, true);

// 等遥控器数据就绪（最多等200ms）
for (uint32_t wait = 0; wait < 200; wait++)
{
    vTaskDelay(1);
    if (!(remoteController.get_left_y() == -1
        && remoteController.get_left_x() == -1
        && remoteController.get_right_x() == -1
        && remoteController.get_right_y() == -1))
    {
        break;  // 遥控器数据有效了，提前退出
    }
}

// 读电机状态（此时CAN已跑了一段时间，电机数据可靠）
ControlTask();

// 设置初始目标（此时IMU也收敛了）
ImuData_user.yaw = GetContinuousYawAngle(imu.GetAngle(2), true);
ImuData_user.pitch = imu.GetAngle(1);
yaw_target_angle = ImuData_user.yaw;   // 目标=当前，偏差为0
pitch_target_angle = ImuData_user.pitch; // pitch 用 IMU 闭环

// 最后才使能电机
gimbal_motor.On(1, 1); // pitch → CAN2
gimbal_motor.On(2, 2); // yaw  → CAN1
vTaskDelay(500); // 等电机上电完成

// 等待 IMU 数据就绪并稳定，防止上电初期 IMU 未收敛导致猛转
// 第一步：等 IMU 板开始发送有效数据（最多等 1000ms）
for (uint32_t i = 0; i < 1000; i++)
{
    if (imu.isConnected()) break;
    vTaskDelay(1);
}

// 第二步：等角度收敛（连续10次变化<1°，最多等 200ms）
{
    float prev_yaw = imu.GetAngle(2);
    uint8_t stable_count = 0;
    for (uint32_t i = 0; i < 200; i++)
    {
        vTaskDelay(1);
        float cur_yaw = imu.GetAngle(2);
        float diff = cur_yaw - prev_yaw;
        if (diff > 180.0f)  diff -= 360.0f;
        if (diff < -180.0f) diff += 360.0f;
        if (diff < 1.0f && diff > -1.0f)
        {
            stable_count++;
            if (stable_count >= 10) break;
        }
        else
        {
            stable_count = 0;
        }
        prev_yaw = cur_yaw;
    }
}

// 用收敛后的 IMU 重新校准初始目标角度
ImuData_user.yaw = GetContinuousYawAngle(imu.GetAngle(2), true);
ImuData_user.pitch = imu.GetAngle(1);
yaw_target_angle = ImuData_user.yaw;
pitch_target_angle = ImuData_user.pitch;

    for(;;)
    {
         if(!remoteController.isConnected() ||   // 遥控器失联
            (remoteController.get_left_y() == -1 // 或摇杆数据全-1（解析异常，数据不可信）
         && remoteController.get_left_x() == -1
         && remoteController.get_right_x() == -1
         && remoteController.get_right_y() == -1)
        )
        {
            gimbal_remote_offline = 1;
            yaw_control_output = 0.0f;
            pitch_control_output = 0.0f;
            RemoteData = {};
            CAN2_SendChassisSpeed(0.0f, 0.0f);

            yaw_angle_pid.reset();
            yaw_angle_to_speed_pid.reset();
            pitch_angle_pid.reset();
            pitch_angle_to_speed_pid.reset();
            yaw_version_angle_pid.reset();
            yaw_version_speed_pid.reset();
            pitch_version_angle_pid.reset();
            pitch_version_speed_pid.reset();
            pitch_ude.ResetState(pitch_current_speed);
            yaw_speed_dynamics_ff.ResetState(0.0f);
            pitch_ude_output = 0.0f;
            pitch_last_control_output = 0.0f;
            gimbal_motor.ctrl_Torque(2, 2, 0); // yaw   → CAN1
            gimbal_motor.ctrl_Torque(1, 1, 0); // pitch → CAN2

            vTaskDelay(1);
            continue;
        }
         else {
        gimbal_remote_offline = 0;
                 RemoteData.chassis_vx = remoteController.get_left_y();
        RemoteData.chassis_vy = remoteController.get_left_x();
        RemoteData.gimbal_yaw = -remoteController.get_right_x();
        RemoteData.gimbal_pitch = remoteController.get_right_y();
        RemoteData.s1 = remoteController.get_s1();
        RemoteData.s2 = remoteController.get_s2();

        // [新增] 填充键鼠数据字段
        RemoteData.mouse_x    = remoteController.get_mouseX();
        RemoteData.mouse_y    = remoteController.get_mouseY();
        RemoteData.mouse_z    = remoteController.get_mouseZ();
        RemoteData.mouse_left = remoteController.get_mouseLeft() ? 1 : 0;
        RemoteData.mouse_right = remoteController.get_mouseRight() ? 1 : 0;
        RemoteData.keyboard   = remoteController.get_keyboard();

        // [新增] 更新 InputDispatcher 状态机
        input_dispatcher.Update(
            RemoteData.s1,
            RemoteData.s2,
            RemoteData.keyboard,
            RemoteData.mouse_left != 0,
            RemoteData.mouse_right != 0
        );

ImuData_user.yaw = GetContinuousYawAngle(imu.GetAngle(2)); // continuous yaw feedback
ImuData_user.pitch = imu.GetAngle(1); // pitch 角度，视觉模式闭环用
ImuData_user.gyro_y = imu.GetGyro(0); // pitch 角速度，视觉模式闭环用
ImuData_user.gyro_z = imu.GetGyro(2); // 使用原始陀螺仪数据进行滤波，保持控制响应的及时性


        
/******************************************************************** */
// 1. 核心控制指令：直接脱离 switch-case，每 1ms 循环必发！
// 这样底盘能接收到最高频、最实时的坐标方向
CAN2_SendChassisSpeed(RemoteData.chassis_vx, RemoteData.chassis_vy);
YawOffset_SendToCan2(); 

// 2. 其他辅助数据：使用 switch-case 分频发送，避免 CAN 总线拥堵
switch (can2_tick % 10) 
{
    case 0:
        // 云台 IMU 数据改成 10ms 发送一次（对底盘来说足够了）
        CAN2_SendGimbalIMU_Raw(ImuData_user.yaw, ImuData_user.gyro_z);
        break;

    case 5:
        // 遥控器开关状态发生突变时立马发送
        if (RemoteData.s1 != last_s1 || RemoteData.s2 != last_s2)
        {
            CAN2_Send_S1andS2_Status(RemoteData.s1, RemoteData.s2);
            last_s1 = RemoteData.s1;
            last_s2 = RemoteData.s2;
        }
        // [新增] 键鼠模式下每 10ms 发送键盘位掩码给底盘
        if (input_dispatcher.GetSource() == InputSource::KeyMouse)
        {
            CAN2_SendKeyboard(input_dispatcher.GetKeyboardMask());
        }
        break;
}

// 3. 兜底保护：每 20ms 强行同步一次开关状态，防止漏帧
if ((can2_tick % 20) == 19)
{
    CAN2_Send_S1andS2_Status(RemoteData.s1, RemoteData.s2);
    last_s1 = RemoteData.s1;
    last_s2 = RemoteData.s2;
}

can2_tick++;/************************************************************************************** */
/************************************************************************************** */
       ControlTask(); // 读取2个电机的数据

bool imu_fault = IMU_Fault_Protection(yaw_current_angle, yaw_current_speed,
                     pitch_current_angle, pitch_current_speed);

/************************************************************************************** */
//判断为何种模式

if (startup_protect < 500)
{
    // 上电保护：前 500ms 强制 STOP，覆盖拨杆值让 FSM 判定为停止
    startup_protect++;
    Struct_Gimbal_Input stop_input = {};
    stop_input.s1 = Remote::DOWN;
    stop_input.s2 = Remote::DOWN;
    yaw_gimbal_fsm.Update(stop_input, yaw_current_angle);
    pitch_gimbal_fsm.Update(stop_input, pitch_current_angle);
}
else
{
    /************************************************************************************* */
    // FSM 内部自行判断模式：传入原始输入，FSM 根据 s1/s2/键鼠 自行决定
    Struct_Gimbal_Input yaw_input = {};
    yaw_input.s1       = RemoteData.s1;
    yaw_input.s2       = RemoteData.s2;
    yaw_input.joystick_speed = RemoteData.gimbal_yaw;
    yaw_input.mouse_speed = static_cast<float>(RemoteData.mouse_x);
    yaw_input.mouse_dx = RemoteData.mouse_x;
    yaw_input.mouse_dy = RemoteData.mouse_y;
    yaw_input.mouse_right_held = input_dispatcher.IsVisionMode();
    yaw_input.vision_ready = vision_comm.IsVisionReady();
    yaw_input.vision_fresh = vision_comm.IsDataFresh();
    yaw_input.vision_angle  = vision_comm.GetYawAngle();
    yaw_gimbal_fsm.Update(yaw_input, yaw_current_angle);

    Struct_Gimbal_Input pitch_input = {};
    pitch_input.s1       = RemoteData.s1;
    pitch_input.s2       = RemoteData.s2;
    pitch_input.joystick_speed = RemoteData.gimbal_pitch;
    pitch_input.mouse_speed = -static_cast<float>(RemoteData.mouse_y);
    pitch_input.mouse_dx = RemoteData.mouse_x;
    pitch_input.mouse_dy = RemoteData.mouse_y;
    pitch_input.mouse_right_held = input_dispatcher.IsVisionMode();
    pitch_input.vision_ready = vision_comm.IsVisionReady();
    pitch_input.vision_fresh = vision_comm.IsDataFresh();
    pitch_input.vision_angle  = vision_comm.GetPitchAngle();
    pitch_gimbal_fsm.Update(pitch_input, pitch_current_angle);
}
yaw_mode   = yaw_gimbal_fsm.Get_Mode_Command();
pitch_mode = pitch_gimbal_fsm.Get_Mode_Command();

//        if (yaw_target_angle > 60.0f) yaw_target_angle = 60.0f; // 限制最大角度
//        if (yaw_target_angle < -60.0f) yaw_target_angle = -60.0f; // 限制最小角度

if (yaw_gimbal_fsm.Take_Mode_Changed_Flag() != 0U)
{
    yaw_angle_pid.reset();
    yaw_angle_to_speed_pid.reset();
    yaw_keymouse_speed_pid.reset();
    yaw_remote_speed_pid.reset();
    yaw_version_angle_pid.reset();
    yaw_version_speed_pid.reset();
    const float yaw_mode_target_angle = yaw_gimbal_fsm.Get_Target_Angle();
    yaw_angle_velocity_ff.VelocityFeedforward(yaw_mode_target_angle);
    yaw_vision_velocity_ff.VelocityFeedforward(yaw_mode_target_angle);
    yaw_angle_dynamics_ff.ResetState(0.0f);
    yaw_vision_dynamics_ff.ResetState(0.0f);
    yaw_speed_dynamics_ff.ResetState(0.0f);
}

if (pitch_gimbal_fsm.Take_Mode_Changed_Flag() != 0U)
{
    pitch_angle_pid.reset();
    pitch_angle_to_speed_pid.reset();
    pitch_keymouse_speed_pid.reset();
    pitch_remote_speed_pid.reset();
    pitch_version_angle_pid.reset();
    pitch_version_speed_pid.reset();
    pitch_angle_dynamics_ff.ResetState(0.0f);
    pitch_vision_dynamics_ff.ResetState(0.0f);
    pitch_ude.ResetState(pitch_current_speed);
    pitch_ude_output = 0.0f;
    pitch_last_control_output = pitch_control_output;
}

yaw_target_angle = yaw_gimbal_fsm.Get_Target_Angle();
pitch_target_angle = pitch_gimbal_fsm.Get_Target_Angle();

yaw_error = yaw_target_angle - yaw_current_angle;

pitch_error = pitch_target_angle - pitch_current_angle;
while (pitch_error > 180.0f)  pitch_error -= 360.0f;
while (pitch_error < -180.0f) pitch_error += 360.0f;
yaw_speed_pid_output = 0.0f;
yaw_speed_ff_output = 0.0f;

/********************************************************************************* */
if (yaw_gimbal_fsm.Get_Control_Type() == GIMBAL_CONTROL_STOP )
{
    yaw_target_speed = 0.0f;
    yaw_control_output = 0.0f;
    yaw_angle_pid.reset();
    yaw_angle_to_speed_pid.reset();
    yaw_keymouse_speed_pid.reset();
    yaw_remote_speed_pid.reset();
    yaw_version_angle_pid.reset();
    yaw_version_speed_pid.reset();
    yaw_speed_dynamics_ff.ResetState(0.0f);
}
else if (yaw_gimbal_fsm.Get_Control_Type() == GIMBAL_CONTROL_ANGLE && !imu_fault)
{
    if (yaw_mode == GIMBAL_MODE_VISION)
    {
        // 视觉模式：使用视觉专用PID
        yaw_target_speed = yaw_version_angle_pid.UpDate(yaw_error, 0.0f);
        yaw_vision_dynamics_ff.MomentOfInertiaTuning(yaw_current_speed, yaw_target_speed);
        yaw_vision_friction_ff.FrictionFeedforward(yaw_target_speed);
        yaw_vision_velocity_ff.VelocityFeedforward(yaw_target_angle);
        yaw_control_output = yaw_version_speed_pid.UpDate(
            yaw_target_speed,
            yaw_current_speed
        ) + yaw_vision_dynamics_ff.getTorque()
          + yaw_vision_friction_ff.getFeedforward()
          + yaw_vision_velocity_ff.getFeedforward();
    }
    else
    {
        yaw_target_speed = yaw_angle_pid.UpDate(yaw_error, 0.0f);
        yaw_angle_dynamics_ff.MomentOfInertiaTuning(yaw_current_speed, yaw_target_speed);
        yaw_angle_friction_ff.FrictionFeedforward(yaw_target_speed);
        yaw_angle_velocity_ff.VelocityFeedforward(yaw_target_angle);
        yaw_control_output = yaw_angle_to_speed_pid.UpDate(
            yaw_target_speed,
            yaw_current_speed
        ) + yaw_angle_dynamics_ff.getTorque()
          + yaw_angle_friction_ff.getFeedforward()
          + yaw_angle_velocity_ff.getFeedforward();
    }
}
else if (yaw_gimbal_fsm.Get_Control_Type() == GIMBAL_CONTROL_SPEED || imu_fault)
{
    // 正常速度模式 或 IMU故障降级：直驱单速度环
    // 键鼠/遥控器单速度环分开调参（is_keymouse 判定统一走 InputDispatcher）
    const bool is_keymouse = (input_dispatcher.GetSource() == InputSource::KeyMouse);

    if (imu_fault && yaw_gimbal_fsm.Get_Control_Type() != GIMBAL_CONTROL_SPEED)
    {
        float scale = is_keymouse ? yaw_gimbal_fsm_config.mouse_speed_scale : yaw_gimbal_fsm_config.speed_scale;
        yaw_target_speed = RemoteData.gimbal_yaw * scale;
    }
    else
    {
        yaw_target_speed = yaw_gimbal_fsm.Get_Control_Output();
    }

    yaw_speed_dynamics_ff.MomentOfInertiaTuning(yaw_current_speed, yaw_target_speed);
    if (is_keymouse)
    {
        yaw_speed_pid_output = yaw_keymouse_speed_pid.UpDate(yaw_target_speed, yaw_current_speed);
    }
    else
    {
        yaw_speed_pid_output = yaw_remote_speed_pid.UpDate(yaw_target_speed, yaw_current_speed);
    }
    yaw_speed_ff_output = yaw_speed_dynamics_ff.getTorque();
    yaw_control_output = yaw_speed_pid_output + yaw_speed_ff_output;

    // IMU 故障时复位角度环PID，防止恢复时积分突变
    if (imu_fault)
    {
        yaw_angle_pid.reset();
        yaw_angle_to_speed_pid.reset();
        yaw_version_angle_pid.reset();
        yaw_version_speed_pid.reset();
    }
}
/****************************************************************************************** */
if (pitch_gimbal_fsm.Get_Control_Type() == GIMBAL_CONTROL_STOP)
{
    pitch_target_speed = 0.0f;
    pitch_control_output = 0.0f;
    pitch_angle_pid.reset();
    pitch_angle_to_speed_pid.reset();
    pitch_keymouse_speed_pid.reset();
    pitch_remote_speed_pid.reset();
    pitch_version_angle_pid.reset();
    pitch_version_speed_pid.reset();
    pitch_ude.ResetState(pitch_current_speed);
    pitch_ude_output = 0.0f;
}
else if (pitch_gimbal_fsm.Get_Control_Type() == GIMBAL_CONTROL_ANGLE && !imu_fault)
{
    if (pitch_mode == GIMBAL_MODE_VISION)
    {
        // 视觉模式：使用视觉专用PID
        pitch_target_speed = pitch_version_angle_pid.UpDate(pitch_error, 0.0f);
        pitch_vision_dynamics_ff.MomentOfInertiaTuning(pitch_current_speed, pitch_target_speed);
        pitch_vision_friction_ff.FrictionFeedforward(pitch_target_speed);
        pitch_gravity_ff.GravityFeedforward(pitch_current_angle);
        pitch_control_output = pitch_version_speed_pid.UpDate(
            pitch_target_speed,
            pitch_current_speed
        ) + pitch_vision_dynamics_ff.getTorque()
          + pitch_vision_friction_ff.getFeedforward()
          + pitch_gravity_ff.getFeedforward();
    }
    else
    {
        pitch_target_speed = pitch_angle_pid.UpDate(pitch_error, 0.0f);
        pitch_angle_dynamics_ff.MomentOfInertiaTuning(pitch_current_speed, pitch_target_speed);
        pitch_angle_friction_ff.FrictionFeedforward(pitch_target_speed);
        pitch_gravity_ff.GravityFeedforward(pitch_current_angle);
        pitch_control_output = pitch_angle_to_speed_pid.UpDate(
            pitch_target_speed,
            pitch_current_speed
        ) + pitch_angle_dynamics_ff.getTorque()
          + pitch_angle_friction_ff.getFeedforward()
          + pitch_gravity_ff.getFeedforward();
    }
}
else if (pitch_gimbal_fsm.Get_Control_Type() == GIMBAL_CONTROL_SPEED || imu_fault)
{
    // 正常速度模式 或 IMU故障降级：直驱单速度环
    // 键鼠/遥控器单速度环分开调参（is_keymouse 判定统一走 InputDispatcher）
    const bool is_keymouse = (input_dispatcher.GetSource() == InputSource::KeyMouse);

    if (imu_fault && pitch_gimbal_fsm.Get_Control_Type() != GIMBAL_CONTROL_SPEED)
    {
        float scale = is_keymouse ? pitch_gimbal_fsm_config.mouse_speed_scale : pitch_gimbal_fsm_config.speed_scale;
        pitch_target_speed = RemoteData.gimbal_pitch * scale;
    }
    else
    {
        pitch_target_speed = pitch_gimbal_fsm.Get_Control_Output();
    }

    // 速度模式同样叠加重力前馈，防止松杆后 pitch 被重力拽下垂
    pitch_gravity_ff.GravityFeedforward(pitch_current_angle);
    if (is_keymouse)
    {
        pitch_control_output = pitch_keymouse_speed_pid.UpDate(pitch_target_speed, pitch_current_speed)
                             + pitch_gravity_ff.getFeedforward();
    }
    else
    {
        pitch_control_output = pitch_remote_speed_pid.UpDate(pitch_target_speed, pitch_current_speed)
                             + pitch_gravity_ff.getFeedforward();
    }

    // IMU 故障时复位角度环PID，防止恢复时积分突变
    if (imu_fault)
    {
        pitch_angle_pid.reset();
        pitch_angle_to_speed_pid.reset();
        pitch_version_angle_pid.reset();
        pitch_version_speed_pid.reset();
    }
}

if (pitch_gimbal_fsm.Get_Control_Type() == GIMBAL_CONTROL_ANGLE && !imu_fault)
{
    pitch_ude.UDE_Update(pitch_last_control_output, pitch_current_speed);
    pitch_ude_output = pitch_ude.getOutput();
    pitch_control_output += pitch_ude_output;
}
else
{
    pitch_ude.ResetState(pitch_current_speed);
    pitch_ude_output = 0.0f;
}
/********************************************************************************** */
        // 扭矩输出限幅（匹配 LK4005 ctrl_Torque 范围 +/-2048）
        if (yaw_control_output >  2048.0f) yaw_control_output =  2048.0f;
        if (yaw_control_output < -2048.0f) yaw_control_output = -2048.0f;
        if (pitch_control_output >  2048.0f) pitch_control_output =  2048.0f;
        if (pitch_control_output < -2048.0f) pitch_control_output = -2048.0f;

        pitch_last_control_output = pitch_control_output;

           gimbal_motor.ctrl_Torque(2, 2, (int16_t)yaw_control_output);   // yaw   → CAN1
            gimbal_motor.ctrl_Torque(1, 1, (int16_t)pitch_control_output); // pitch → CAN2
        // vofa 发送: 1ms 循环里 28 字节帧 @115200 需 2.43ms, 每 10 拍(10ms)发一帧 → 100Hz
        
        
            if ((can2_tick % 10) == 0)
            {
                // VOFA channels for yaw tuning.
                float vofa_data[] = {
                    yaw_target_speed,                              // ch1: yaw speed command
                    yaw_current_speed,                             // ch2: yaw speed feedback
                    yaw_target_speed - yaw_current_speed,          // ch3: yaw speed error
                    yaw_speed_pid_output,                          // ch4: yaw speed PID output only
                    yaw_speed_ff_output,                           // ch5: yaw speed full-compensation FF
                    yaw_control_output,                            // ch6: yaw final torque command
                    yaw_target_angle,                              // ch7: yaw angle command
                    yaw_current_angle,                             // ch8: yaw angle feedback
                    mg4005_state[1].velocity_rpm,                  // ch9: yaw motor velocity rpm
                    mg4005_state[1].current_a                      // ch10: yaw motor current
                };
                vofa_sendN(vofa_data, static_cast<uint8_t>(sizeof(vofa_data) / sizeof(vofa_data[0]))); // send data to VOFA
            }
        

         }



        vTaskDelay(1); // 每5ms执行一次控制循环
    }
}







void SafetyCheck()
{
    // 遍历所有电机
    for (int i = 1; i <= 2; i++)
    {
        // isConnected(电机ID, CAN ID)
        if (!gimbal_motor.isConnected(i, 0x140 + i))
        {
            // 电机 i 掉线了！
            // 蜂鸣器会自动报警

            // 你可以在这里做一些安全处理
            // 比如停止所有电机
			remote.chassis_vx = 0.0f;
			remote.chassis_vy = 0.0f;
			remote.gimbal_yaw = 0.0f;
			remote.gimbal_pitch = 0.0f;
			remote.s1 = 0;
			remote.s2 = 0;
        }
    }

    // 或者直接获取掉线的电机编号
    uint8_t offline_motor = gimbal_motor.getOfflineStatus();
	
    // 返回 0 表示都在线
    // 返回 1-4 表示对应电机掉线

}

//开vofa软件的justfloat模式
uint8_t send_str2[sizeof(float) * 11]; // 分配11个float空间（44字节，10数据+1帧尾）
void vofa_sendN(const float *data, uint8_t count)
{
    if (data == nullptr || count == 0)
    {
        return;
    }

    if (count > 10)
    {
        count = 10;
    }

    USBD_CDC_HandleTypeDef *hcdc = static_cast<USBD_CDC_HandleTypeDef*>(hUsbDeviceFS.pClassData);
    if (hcdc == nullptr || hcdc->TxState != 0)
    {
        return;
    }

    memcpy(send_str2, data, sizeof(float) * count);
    *((uint32_t*)&send_str2[sizeof(float) * count]) = 0x7F800000;

    CDC_Transmit_FS(send_str2, static_cast<uint16_t>(sizeof(float) * (count + 1)));
}


// uint8_t send_str2[sizeof(float) * 7]; // 6个float数据 + 4字节帧尾（28字节）

// 开vofa软件的justfloat模式
void vofa_send(float x1, float x2, float x3, float x4, float x5, float x6)
{
    const uint8_t sendSize = sizeof(float); // 单浮点数占4字节

    // 将6个浮点数据写入缓冲区（小端模式）
    *((float*)&send_str2[sendSize * 0]) = x1;
    *((float*)&send_str2[sendSize * 1]) = x2;
    *((float*)&send_str2[sendSize * 2]) = x3;
    *((float*)&send_str2[sendSize * 3]) = x4;
    *((float*)&send_str2[sendSize * 4]) = x5;
    *((float*)&send_str2[sendSize * 5]) = x6;

    // 写入帧尾（协议要求 0x00 0x00 0x80 0x7F）
    *((uint32_t*)&send_str2[sizeof(float) * 6]) = 0x7F800000; // 小端存储为 00 00 80 7F

    HAL::UART::Data vofa_tx_data{send_str2, sizeof(float) * 7};
    HAL::UART::get_uart_bus_instance().get_uart6().transmit_dma(vofa_tx_data);
}

 void ControlTask() {
    for (int i = 0; i < 2; i++) {
        uint8_t motor_id = i + 1; // 电机逻辑 ID 通常从 1 开始
        
        mg4005_state[i].angle_deg   = gimbal_motor.getAngleDeg(motor_id);
        mg4005_state[i].angle_rad   = gimbal_motor.getAngleRad(motor_id);
        mg4005_state[i].velocity_rpm   = gimbal_motor.getVelocityRpm(motor_id);
        mg4005_state[i].velocity_rads  = gimbal_motor.getVelocityRads(motor_id);   //角速度，用这个控制电机
        mg4005_state[i].current_a     = gimbal_motor.getCurrent(motor_id);
        mg4005_state[i].temperature        = gimbal_motor.getTemperature(motor_id);
        mg4005_state[i].delta_angle       = gimbal_motor.getMultiAngle(motor_id); // 计算多圈角度时需要用到
    }
}

// ==================== IMU 故障检测与编码器降级 ====================
// 检测条件：数据全零 或 IMU 从未就绪 → 自动切到电机编码器反馈
// 故障翻转时调用 FSM::ReAnchor + PID 复位，防止疯车
// 返回 true 表示 IMU 故障，调用方应强制切到单速度环控制
static bool IMU_Fault_Protection(float &yaw_angle, float &yaw_speed,
                                  float &pitch_angle, float &pitch_speed)
{
    static uint16_t imu_fault_counter = 0;
    static uint8_t  imu_fault = 0;
    static uint8_t  last_imu_fault = 0;
    static float    yaw_enc_offset = 0.0f;
    static float    pitch_enc_offset = 0.0f;
    static constexpr float RPM_TO_DEGPS = 6.0f;           // RPM × 360 / 60
    static constexpr float RADS_TO_DEGPS = 57.29578f;     // rad/s → °/s  (180 / PI)

    bool imu_all_zero = (imu.GetAngle(2) == 0.0f && imu.GetAngle(1) == 0.0f &&
                         imu.GetGyro(2) == 0.0f && imu.GetGyro(0) == 0.0f);
    bool imu_lost = (!imu.isConnected());

    if (imu_all_zero || imu_lost)
        imu_fault_counter++;
    else
        imu_fault_counter = 0;

    imu_fault = (imu_fault_counter > 500) ? 1 : 0; // 500ms 防抖

    if (imu_fault != last_imu_fault)
    {
        if (imu_fault)
        {
            yaw_enc_offset   = mg4005_state[1].delta_angle - ImuData_user.yaw;
            pitch_enc_offset = mg4005_state[0].delta_angle - ImuData_user.pitch;
        }
        float new_yaw   = imu_fault ? (mg4005_state[1].delta_angle - yaw_enc_offset) : ImuData_user.yaw;
        float new_pitch = imu_fault ? (mg4005_state[0].delta_angle - pitch_enc_offset) : ImuData_user.pitch;
        yaw_gimbal_fsm.ReAnchor(new_yaw);
        pitch_gimbal_fsm.ReAnchor(new_pitch);

        yaw_angle_pid.reset();
        yaw_angle_to_speed_pid.reset();
        yaw_keymouse_speed_pid.reset();
        yaw_remote_speed_pid.reset();
        yaw_version_angle_pid.reset();
        yaw_version_speed_pid.reset();
        pitch_angle_pid.reset();
        pitch_angle_to_speed_pid.reset();
        pitch_keymouse_speed_pid.reset();
        pitch_remote_speed_pid.reset();
        pitch_version_angle_pid.reset();
        pitch_version_speed_pid.reset();

        last_imu_fault = imu_fault;
    }

    if (imu_fault)
    {
        yaw_angle   = mg4005_state[1].delta_angle - yaw_enc_offset;
        pitch_angle = mg4005_state[0].delta_angle - pitch_enc_offset;
        yaw_speed   = mg4005_state[1].velocity_rads * RADS_TO_DEGPS;
        pitch_speed = mg4005_state[0].velocity_rads * RADS_TO_DEGPS;  // TODO: 实测确认符号
    }
    else
    {
        yaw_angle   = ImuData_user.yaw;
        pitch_angle = ImuData_user.pitch;
        yaw_speed   = ImuData_user.gyro_z;
        pitch_speed = ImuData_user.gyro_y;
    }

    return imu_fault != 0;
}



