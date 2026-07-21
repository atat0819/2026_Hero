#include "can_send_task.hpp"
#include "remote_task.hpp"
#include "FreeRTOS.h"
#include "queue.h"
#include "cmsis_os.h"
#include "chassis_task.hpp"
#include "Alg/PID/pid.hpp"
#include "usart.h"
#include "Alg/Filter/Filter.hpp"
#include <math.h>
#include <string.h>
// 必须包含实现类的头文件，否则编译器不知道 CanDevice 是什么
#include "HAL/CAN/impl/can_device_impl.hpp" 
// 确保包含这个头文件以获取 hcan1 的定义
#include "can.h"
#include "../user/core/Alg/ChassisCalculation/OmniCalculation.hpp"
#include "../user/core/BSP/Motor/Dji/DjiMotor.hpp"
#include "../fsm/chassis_fsm.hpp"
#include "../user/core/Alg/PowerControl-TestVersion/PowerControlTestVersion.hpp"
#include "../user/core/Alg/PowerControl/PowerControl.hpp"
#include "../user/core/HAL/UART/uart_hal.hpp"
#include "../communication/super_cupacitor.hpp"
#include "../communication/gimbal_refree.hpp"
#include "../user/core/APP/Referee/RM_RefereeSystem.h"

#define Gain 4.7

QueueHandle_t motorspeedtargetQueue; // 声明一个全局队列句柄，用于在任务之间传递电机转速数据
QueueHandle_t motorCurrentDataQueue; // 声明一个全局队列句柄，用于在任务之间传递电机当前数据
QueueHandle_t chassisCurrentDataQueue; // 声明一个全局队列句柄，用于在任务之间传递底盘当前数据


MotorSpeedTarget_t motorSpeedTarget; // 定义一个全局变量来存储电机速度数据
MotorCurrentData_t motorCurrentData[4]; // 定义一个结构体来存储电机当前数据
chassisCurrentData_t chassisCurrentData; // 定义一个结构体来存储底盘当前数据

float yaw_offset_deg = 0.0f;    //云台偏移量
bool yaw_offset_updated = false; //标志位，表示是否接收到新的云台偏移量数据，接收到了才允许底盘控制任务使用这个数据进行计算
static uint32_t yaw_offset_timeout_cnt = 0; // 超时计数器

// 定义全局变量来存储云台底盘速度数据
Gimbal_Chassis_communicate_t gimbalChassis_communicate;

uint8_t gimbalChassisSpeedUpdated = 0;

// 超级电容通信实例
Communication::SuperCapacitor supercap(500); // 500ms 超时阈值

// 裁判系统→云台转发实例
Communication::GimbalRefree gimbal_refree;

	// 1. 定义物理常数（根据你的麦轮实际参数修改）
	const float wheel_azimuth[4] = {-M_PI/4, M_PI/4, -M_PI/4, M_PI/4}; // 麦克纳姆轮上那些**小辊子（Roller）**相对于轮轴的偏转角度
    //参数解读：{{0.2f, 0.2f}, ...} 表示轮子安装在前方 $20cm$，左侧 $20cm$ 的位置。
	const float wheel_coords[4][2] = {{0.22f, 0.19545f}, {-0.22f, 0.19545f}, {-0.22f, -0.19545f}, {0.22f, -0.19545f}}; // 轮子位置
	float azimuth_for_fk[4] = {M_PI/4.0f,7*M_PI/4.0f, 5*M_PI/4.0f,3*M_PI/4.0f   }; // 正运动学使用的轮子方位角（与安装方向相关）


// 2. 实例化三大核心模块
//第一个数字：底盘中心到轮心的距离，第二个数字：轮子半径，第三个数字：电机数  
//第4个参数：麦克纳姆轮上那些**小辊子（Roller）**相对于轮轴的偏转角度 第四个参数：轮子安装位置坐标
	Alg::CalculationBase::Omni_IK ik(0.2943f, 0.076f, wheel_azimuth, wheel_coords); // 逆运动学
	Alg::CalculationBase::Omni_FK fk(0.2943f, 0.076f, 4.0f, wheel_azimuth, azimuth_for_fk); // 正运动学
	Alg::CalculationBase::Omni_ID id(0.2943f, 0.076f, 4.0f, wheel_azimuth, wheel_coords); // 逆动力学


// 模板参数 <N> 表示电机数量
// 4 个底盘电机
const uint8_t chassis_motor_idxs[4] = {1, 2, 3, 4};
BSP::Motor::Dji::GM3508<4> chassis_motor(0x200, chassis_motor_idxs, 0x200);

void ControlTask();
void CAN1_RxCallback(HAL::CAN::Frame& frame);


RemoteData_t ChassisData;



// 创建滤波器
// 参数：滤波系数 α（0-1），越小滤波越强
LPFFilter acc_filter_x(0.2f);
LPFFilter acc_filter_y(0.2f);
LPFFilter acc_filter_z(0.2f);


// 4 个电机，4 个 PID
ALG::PID::PID motor_pid[4] = {
    {130.0f, 0.0f, 0.0f, 16384.0f, 5000.0f, 500.0f},   //电机1 (扫频PID)
	{130.0f, 0.0f, 0.0f, 16384.0f, 5000.0f, 500.0f},    //电机2
	{130.0f, 0.0f, 0.0f, 16384.0f, 5000.0f, 500.0f},    //电机3
	{130.0f, 0.0f, 0.0f, 16384.0f, 5000.0f, 500.0f}     //电机4	
};

ALG::PID::PID test_pid = {0.0f, 0.0f, 0.0f, 10000.0f, 5000.0f, 500.0f};

// 底盘状态机实例
Chassis_FSM chassis_fsm(5.0f, 0.0f, 0.0f, 100.0f, 25.0f, 2.5f); // PID 参数可以根据需要调整

float output = 0.0f; // PID 输出变量
float target = 10.0f; // 目标速度（示例值）
// ALG::PID::PID chassis_pid[3] = {
// 	{0.0f, 0.0f, 0.0f, 10000.0f, 5000.0f, 500.0f},    //底盘X轴速度PID
// 	{0.0f, 0.0f, 0.0f, 10000.0f, 5000.0f, 500.0f},    //底盘Y轴速度PID
// 	{0.0f, 0.0f, 0.0f, 10000.0f, 5000.0f, 500.0f}		 //底盘旋转速度PID
// };


void vofa_send(float x1, float x2, float x3, float x4, float x5, float x6);
float motor_target_speed[4];
float motor_output[4];
float motor_output_pre[4];  // 功率控制前的电流，用于VOFA对比
float current_speed_rads[4];
float c = 2.0f;
float phase_comp = 0.0f;   // 这个变量用于补偿系统的相位滞后，具体值需要通过实验调整
    float yaw_offset_rad = 0.0f;

// ========== 功率校准扫频测试 ==========
Alg::PowerControlTestVersion::PowerControlTestVersion sweep_tester; // 扫频信号生成器
// 功率多项式系数: P_in = K0 + K1*I + K2*w + K3*I*w + K4*I*I + K5*w*w
float poly_coeffs[6] = {
	    3.641922f,   // K0
	   -0.017876f,   // K1
	   -0.000042f,   // K2
	    0.017527f,   // K3
	    0.136120f,   // K4
	    0.000026f    // K5
	};
// ========== 功率校准扫频测试结束 ==========

// ========== 底盘功率控制 (衰减电流法) ==========
// 超电配置 (根据实际硬件修改总容量)
const float SUPERCAP_TOTAL_CAPACITY = 2000.0f;                        // 超电总容量 (J)
const float ABUNDANCE_LINE          = SUPERCAP_TOTAL_CAPACITY * 0.8f; // 富足线 = 80%总容量
const float POVERTY_LINE            = 250.0f;                         // 贫困线 (生死线, <250J强制80%功率)

// 功率控制器 (衰减电流法执行层)
ALG::PowerControl::PowerControl<4> chassis_power_ctrl;

// 能量环状态机                                        //富足线，贫困线，最小功率限制(防止功率太小跑不动)
ALG::PowerControl::EnergyRing energy_ring(ABUNDANCE_LINE, POVERTY_LINE, 56.0f);

// 策略层 (数据源仲裁: 裁判系统/超电 在线/离线判断)
ALG::PowerControl::PowerControlStrategy power_strategy(ABUNDANCE_LINE);

// 富足环 PID — 目标: 1600J(80%总容量), 仅energy≥1600J时被EnergyRing使用
//   能量越高 → AbundanceOut越负 → P_max越大 → 释放过剩能量, 防过充
//   能量<1600J → EnergyRing走中间分支, 直接P_ref, 此PID不参与
//   Kp=0.05 → 能量2000J时AbundanceOut≈-20W, P_max≈100W
ALG::PID::PID abundance_energy_pid(0.05f, 0.0f, 0.0f, 80.0f, 80.0f, 0.0f);

// 贫困环 PID — 目标: 250J (POVERTY_LINE), Shift爆发模式生效
//   能量>>250J → PovertyOut为负 → P_max = P_ref - (负数) = P_ref + 增量 → 超功率！
//   Kp=0.05 → 能量1000J时PovertyOut≈-37.5W, P_max≈117.5W
ALG::PID::PID poverty_energy_pid(0.05f, 0.0f, 0.0f, 80.0f, 80.0f, 0.0f);
// ========== 功率控制初始化结束 ==========

extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    HAL::CAN::Frame frame;

    if (hcan->Instance == CAN1)
    {
        if (HAL::CAN::get_can_bus_instance().get_can1().receive(frame))
        {
            // ControlTask();
        }
    }
   
}

extern "C" void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    HAL::CAN::Frame frame;

    if (hcan->Instance == CAN2)
    {
        if (HAL::CAN::get_can_bus_instance().get_can2().receive(frame))
        {
            // ControlTask();
        }
    }

}
/******************************************************* */
    float wz_cmd = 0.0f;

extern "C" void can_send_task(void *argument)
{

    // 等待裁判系统就位（约需5秒），确保电机上电前裁判系统已就绪
    osDelay(1000);

    // 触发 CAN bus 初始化：HAL_CAN_Start + 激活中断通知
    HAL::CAN::get_can_bus_instance();

    auto &can1 = HAL::CAN::get_can_bus_instance().get_device(HAL::CAN::CanDeviceId::HAL_Can1);
    auto &can2 = HAL::CAN::get_can_bus_instance().get_device(HAL::CAN::CanDeviceId::HAL_Can2);
/************************************************************************************** */
/************************************************************************************** */
    can1.register_rx_callback([](const HAL::CAN::Frame &frame) {
        if (frame.id >= 0x201 && frame.id <= 0x204)
    {
        // 这是底盘电机的数据，交给 chassis_motor 解析
        chassis_motor.Parse(frame);
    }

    });
/************************************************************************************** */
   can2.register_rx_callback([](const HAL::CAN::Frame &frame) {
   // 直接把逻辑写在这里
    if (frame.id == 0x301 ) {
       memcpy(&gimbalChassis_communicate.yaw_offset_deg, frame.data, sizeof(float));
       yaw_offset_updated = true;
       yaw_offset_timeout_cnt = 0; // 收到数据，清零计数器
   }
   else if (frame.id == 0x302) {
       memcpy(&gimbalChassis_communicate.vx, &frame.data[0], sizeof(float));
       memcpy(&gimbalChassis_communicate.vy, &frame.data[4], sizeof(float));
       gimbalChassisSpeedUpdated = 1;
   }
   else if (frame.id == 0x303 ) {
       gimbalChassis_communicate.s1 = frame.data[0];
       gimbalChassis_communicate.s2 = frame.data[1];
   }
    else if (frame.id == 0x777) {
       supercap.parse(frame); // 超级电容数据
   }

});
/************************************************************************************** */
    MotorCurrentData_t MotorCurrentData[4];
    
    
    float vx_gimbal = 0.0f;
    float vy_gimbal = 0.0f;
    float vx_body = 0.0f;
    float vy_body = 0.0f;

    // 初始化底盘状态机
    chassis_fsm.Init();
memset(&gimbalChassis_communicate, 0, sizeof(gimbalChassis_communicate));

for (int i = 0; i < 4; i++)
{
    chassis_motor.setCAN((int16_t)0, i + 1);
}
chassis_motor.sendCAN();

// 在循环前声明
Enum_Chassis_Mode last_mode = CHASSIS_STOP; //为了不疯车

    //IMU的变量
    IMUData_t IMUData;           // IMU 数据结构体
osDelay(500);
    for (;;)
    {
         //获取底盘旋转速度
        //  ChassisData.vx = remoteController.get_left_y()*Gain;
        //  ChassisData.vy = remoteController.get_left_x()*Gain;
        //  ChassisData.wz = remoteController.get_right_x() * c;
        //  ChassisData.s1 = remoteController.get_s1();
        //  ChassisData.s2 = remoteController.get_s2();

                 //获取底盘旋转速度
         //ChassisData.wz = remoteController.get_right_x() * c;

         // 测试模式：遥控器直接控制底盘，绕过云台CAN通信
         gimbalChassis_communicate.vx = remoteController.get_left_y();
         gimbalChassis_communicate.vy = remoteController.get_left_x();
         gimbalChassis_communicate.s1 = remoteController.get_s1();
         gimbalChassis_communicate.s2 = remoteController.get_s2();
         yaw_offset_updated = true;  // 模拟云台在线，否则FSM强制STOP


         // 超级电容在线状态更新
         supercap.updateOnlineStatus();
         if(remoteController.get_left_y() == -1 
         && remoteController.get_left_x() == -1 
         && remoteController.get_right_x() == -1)
         {
        for (int i = 0; i < 4; i++) 
        {
        motor_output[i] = 0;
        motor_pid[i].reset();
        chassis_motor.setCAN((int16_t)0, i + 1);
        }
         chassis_fsm.Get_Follow_PID().reset();
  chassis_motor.sendCAN();
  continue; // 跳过本次循环，直接进入下一次循环
    }
    else if (gimbalChassis_communicate.vx == -1 && gimbalChassis_communicate.vy == -1)
    {
        for (int i = 0; i < 4; i++) 
        {
        motor_output[i] = 0;
        motor_pid[i].reset();
        chassis_motor.setCAN((int16_t)0, i + 1);
        }
         chassis_fsm.Get_Follow_PID().reset();
 chassis_motor.sendCAN();
  continue; // 跳过本次循环，直接进入下一次循环
    }
    else {
             ControlTask(); // 获取电机当前数据并更新全局 变量

            // ==================== 功率校准扫频模式 ====================
            // 遥控器 S1 拨到最上方 → 进入校准模式，电机4自动跑扫频
            // S1 拨到其他位置 → 恢复正常遥控模式
    // if (remoteController.get_s1() == 1)
    // {
    //     // 1. 生成扫频目标转速 (rad/s, 电机转子端)
    //     //    覆盖: 幅值 0~800 rad/s, 频率 1~4 Hz
    //     float sweep_target = sweep_tester.SinExpected(0.001f, 20.0f, 800.0f, 4.0f);

    //     // 2. 电机4: PID 速度闭环跟踪扫频信号
    //     //    getVelocityRads = 转子转速 (rad/s), sweep_target 也是转子转速, 直接对齐
    //     motor_output[3] = motor_pid[3].UpDate(sweep_target,
    //                                           chassis_motor.getVelocityRads(4));
    //     chassis_motor.setCAN((int16_t)motor_output[3], 4);

    //     // 3. 电机1/2/3: 停转
    //     for (int i = 0; i < 3; i++)
    //     {
    //         motor_pid[i].reset();
    //         chassis_motor.setCAN((int16_t)0, i + 1);
    //     }
    //     chassis_motor.sendCAN();

    //     // 4. 采集数据 → VOFA+ (JustFloat 模式)
    //     //    PowerData 来自 UART8 功率计, 在 remote_task.cpp 中断中更新
    //     float I     = chassis_motor.getCurrent(4);           // 电流 (A)
    //     float omega = chassis_motor.getVelocityRads(4);      // 转子转速 (rad/s)
    //     float P_in  = PowerData.power;                       // 功率计功率 (W)
    //     // 多项式模型估算功率: P = K0 + K1*I + K2*w + K3*I*w + K4*I² + K5*w²
    //     float P_est = poly_coeffs[0]
    //                 + poly_coeffs[1] * I
    //                 + poly_coeffs[2] * omega
    //                 + poly_coeffs[3] * I * omega
    //                 + poly_coeffs[4] * I * I
    //                 + poly_coeffs[5] * omega * omega;
    //     // VOFA列: [P_in(功率计), omega, I, sweep_target, P_est(模型), 0]
    //     vofa_send(P_in, omega, I, sweep_target, P_est, 0.0f);

    //     osDelay(1);
    //     continue; // 跳过正常遥控逻辑
    //                 }
            // ==================== 校准模式结束 ====================

           yaw_offset_rad = gimbalChassis_communicate.yaw_offset_deg * M_PI / 180.0f;//将云台偏移角从度转换为弧度

           // 超时检测：500 次循环（约 500ms）未收到新数据，视为离线
           if (yaw_offset_updated)
           {
               yaw_offset_timeout_cnt++;
               if (yaw_offset_timeout_cnt > 500)
               {
                   yaw_offset_updated = false;
                   yaw_offset_timeout_cnt = 0;
               }
           }

           // 底盘状态机：更新模式并获取旋转角速度指令
           // yaw_offset_updated 为 1 表示 CAN 通信已建立，否则强制保持 STOP
           chassis_fsm.StateUpdate(
               (uint8_t)gimbalChassis_communicate.s1,
               (uint8_t)gimbalChassis_communicate.s2,
               yaw_offset_updated);
           wz_cmd = chassis_fsm.Get_wz_cmd(yaw_offset_rad);

        // 2. 获取电机当前反馈 (当前轮速)
        phase_comp = 0.0f; // 这里暂时不使用相位补偿，后续可以根据实际情况调整
if (chassis_fsm.Get_Mode() == CHASSIS_GYRO_SPIN)
{
    phase_comp = (-0.007f * wz_cmd);  // 逆时针转为正，顺时针转为负
}
            // --- A. 运动学逆解算：底盘速度 -> 4个轮子的目标转速 ---
            // 直接解算遥控器给出的目标值
            vx_gimbal = gimbalChassis_communicate.vx * Gain;
            vy_gimbal = gimbalChassis_communicate.vy * Gain;
float compensated_angle = yaw_offset_rad + phase_comp;
vx_body = vx_gimbal * cosf(compensated_angle) + vy_gimbal * sinf(compensated_angle);
vy_body = -vx_gimbal * sinf(compensated_angle) + vy_gimbal * cosf(compensated_angle);
/**************************************************************** */
if (chassis_fsm.Get_Mode() != last_mode)
{
    // 模式切换：重置所有电机 PID，清零积分
    for (int i = 0; i < 4; i++)
    {
        motor_pid[i].reset();
    }
    last_mode = chassis_fsm.Get_Mode();
}
/**************************************************************** */
           // STOP 模式：直接清零，跳过后续控制，防疯车
if (chassis_fsm.Get_Mode() == CHASSIS_STOP)
{
   for (int i = 0; i < 4; i++)
   {
       motor_pid[i].reset();
       chassis_motor.setCAN((int16_t)0, i + 1);
   }
   chassis_motor.sendCAN();
osDelay(1);
   continue;
}
/**************************************************************** */
        ik.OmniInvKinematics(vx_body, vy_body, wz_cmd, 0.0f, 1.0f, 1.0f);
        //ik.OmniInvKinematics(ChassisData.vx, ChassisData.vy, -ChassisData.wz, 0.0f, 1.0f, 1.0f);
            
          current_speed_rads[0] = chassis_motor.getVelocityRads(1) / 19.0f; // 转换为电机轴转速
          current_speed_rads[1] = chassis_motor.getVelocityRads(2) / 19.0f;
          current_speed_rads[2] = chassis_motor.getVelocityRads(3) / 19.0f;
          current_speed_rads[3] = chassis_motor.getVelocityRads(4) / 19.0f;

          motor_target_speed[0] = ik.GetMotor(0);
            motor_target_speed[1] = ik.GetMotor(1);
            motor_target_speed[2] = -ik.GetMotor(2);
            motor_target_speed[3] = -ik.GetMotor(3);
					 
		 motor_output[0] = motor_pid[0].UpDate(motor_target_speed[0], current_speed_rads[0]);
     motor_output[1] = motor_pid[1].UpDate(motor_target_speed[1], current_speed_rads[1]);
     motor_output[2] = motor_pid[2].UpDate(motor_target_speed[2], current_speed_rads[2]);
     motor_output[3] = motor_pid[3].UpDate(motor_target_speed[3], current_speed_rads[3]);
	// 保存功率控制前的电流，用于VOFA对比衰减前后
		memcpy(motor_output_pre, motor_output, sizeof(motor_output_pre));

	// ========== 底盘功率控制 (衰减电流法) ==========
	{
	    // 1. 策略层: 数据源仲裁 (裁判系统 / 超电 在线判断)
	    power_strategy.Update(
	        supercap.isOnline(),                              // 超电在线状态
	        !RM_RefereeSystemDirFlag,                         // 裁判系统在线
	        (float)ext_power_heat_data_0x0201.chassis_power_limit,  // 裁判功率上限 (W)
	        (float)ext_power_heat_data_0x0202.chassis_power_buffer, // 缓冲能量 (J)
	        supercap.getEnergy()                              // 超电剩余能量 (J)
	    );

	    float current_energy = power_strategy.GetInputEnergy();  //超电剩余能量
	    float ref_limit      = power_strategy.GetInputLimit();   //

	    // 2. 富足环 PID — 目标=ABUNDANCE_LINE(1600J), 仅energy≥1600J时被EnergyRing使用
	    //    能量>1600J → AbundanceOut<0 → P_max放大(防过充)
	    //    能量<1600J → EnergyRing走中间分支, 直接P_ref, 此PID不参与
	    // 对能量开根号：E = ½CU²，√E ∝ U，保证全电压区间PID响应一致
	    float abundance_out = abundance_energy_pid.UpDate(sqrtf(ABUNDANCE_LINE), sqrtf(current_energy));

	    // 3. 贫困环 PID — 目标=POVERTY_LINE(250J), Shift爆发模式用
	    //    能量>>250J → PovertyOut为负 → P_max = P_ref - (负数) = 超功率!
	    float poverty_out = poverty_energy_pid.UpDate(sqrtf(POVERTY_LINE), sqrtf(current_energy));

	    // 4. 能量环状态机: 根据能量状态动态计算 PowerMax
	    energy_ring.energyring(
	        abundance_out,      // 富足环输出
	        poverty_out,        // 贫困环输出
	        ref_limit,          // 裁判系统功率上限
	        current_energy,     // 当前能量 (策略层可能伪造)
	        false,              // isShift (暂未映射)
	        false               // isPower (充电模式暂未实现)
	    );
	    float PowerMax = energy_ring.GetPowerMax();

	    // 5. 转为物理电流 → 调用电流衰减法
	    //    PID 输出 [-16384, 16384] 映射到 [-20A, 20A] (GM3508)
	    float I[4], V[4], I_other[4];
	    for (int i = 0; i < 4; i++) {
	        I[i]       = motor_output[i] * (20.0f / 16384.0f);     // raw → 物理电流 (A)
	        V[i]       = chassis_motor.getVelocityRads(i + 1);       // 转子转速 (rad/s)
	        I_other[i] = 0.0f;                                       // 无前馈
	    }

	    // CorrectionConstant = 3*K0, 补偿模型与真实功率偏差
	    chassis_power_ctrl.DecayingCurrent(
	        I, V, poly_coeffs, I_other,
	        -3.0f * poly_coeffs[0],   // CorrectionConstant ≈ 10.7W
	        PowerMax
	    );

	    // 6. 物理电流 → raw, 覆写 motor_output
	     for (int i = 0; i < 4; i++) {
	         motor_output[i] = chassis_power_ctrl.getCurrentCalculate(i) * (16384.0f / 20.0f);
	     }
	}
	// ========== 功率控制结束 ==========
		// 用衰减后的电流重算功率，用于VOFA对比预测功率 vs 衰减后实际功率
		float post_power = 0.0f;
		for (int i = 0; i < 4; i++) {
		    float I_post = motor_output[i] * (20.0f / 16384.0f);
		    float w = chassis_motor.getVelocityRads(i + 1);
		    post_power += poly_coeffs[0] + poly_coeffs[1]*I_post + poly_coeffs[2]*w
		                + poly_coeffs[3]*I_post*w + poly_coeffs[4]*I_post*I_post + poly_coeffs[5]*w*w;
		}
		post_power += -3.0f * poly_coeffs[0]; // CorrectionConstant

            for (int i = 0; i < 4; i++) {
    // motor_target_speed[i] = ik.GetMotor(i);
     //motor_output[i] = motor_pid[i].UpDate(motor_target_speed[i], current_speed_rads[i]);
     chassis_motor.setCAN((int16_t)motor_output[i], i + 1);
 }
//		chassis_motor.setCAN((int16_t)motor_output[0],  1);
// 		chassis_motor.setCAN((int16_t)motor_output[1],  2);
//		chassis_motor.setCAN((int16_t)motor_output[2],  3);
//		chassis_motor.setCAN((int16_t)motor_output[3],  4);

 

 chassis_motor.sendCAN();

           
       //4. VOFA: pre_I, post_I, PowerTotal(预测), post_power(衰减后), PowerMax, eta*100
	       vofa_send(motor_output_pre[0], motor_output[0],
	                 chassis_power_ctrl.getPowerTotal(), post_power,
	                 energy_ring.GetPowerMax(), chassis_power_ctrl.getEta() * 100.0f);
// 修复后：加上了取地址符 &
//HAL_UART_Transmit_DMA(&huart6, (const uint8_t*)&yaw_offset_rad, sizeof(yaw_offset_rad));

    }
        



       
        // 转发裁判系统枪管热量数据给云台 (英雄机器人: 仅42mm)
        gimbal_refree.send(
            ext_power_heat_data_0x0201.shooter_barrel_cooling_value,   // 枪管冷却值
            ext_power_heat_data_0x0201.shooter_barrel_heat_limit,      // 枪管热量上限
            ext_power_heat_data_0x0202.shooter_id1_42mm_cooling_heat   // 42mm枪管当前热量
        );

        // 向超级电容发送控制数据
        supercap.sendToSuperCap(
            (float)ext_power_heat_data_0x0201.chassis_power_limit,   // 等级功率 (W)
            0,                                                        // 超电指令: 0=开启
            (float)ext_power_heat_data_0x0202.chassis_power_buffer,   // 缓冲能量 (J)
            supercap.isOnline() ? 1 : 0,                              // 超电在线标志
            RM_RefereeSystemDirFlag ? 0 : 1                           // 裁判系统在线标志
        );

osDelay(1); 
    }
}

//开vofa软件的justfloat模式
uint8_t send_str2[sizeof(float) * 8]; // 分配8个float空间（32字节）
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

    // 通过 UART 库发送（使用 UART3，UART6 留给裁判系统）
    HAL::UART::Data tx_data{send_str2, sizeof(float) * 7};
    HAL::UART::get_uart_bus_instance().get_uart3().transmit_dma(tx_data);
}


 
//从云台数据发送到can2.让底盘接收，用于控制底盘的vx，vy和旋转速度
void CAN2_RxCallback(HAL::CAN::Frame& frame)
{
    if (frame.id == 0x301 && frame.dlc == 4)
    {
        memcpy(&gimbalChassis_communicate.yaw_offset_deg, frame.data, sizeof(float));
        yaw_offset_updated = true;
        yaw_offset_timeout_cnt = 0; // 收到数据，清零计数器
    }
    else if (frame.id == 0x302 && frame.dlc == 8)
    {
        memcpy(&gimbalChassis_communicate.vx, &frame.data[0], sizeof(float));
        memcpy(&gimbalChassis_communicate.vy, &frame.data[4], sizeof(float));
        gimbalChassisSpeedUpdated = 1;
    }
    else if (frame.id == 0x303 )
    {
        gimbalChassis_communicate.s1 = frame.data[0];
        gimbalChassis_communicate.s2 = frame.data[1];
    }
}





 void ControlTask() {
    for (int i = 0; i < 4; i++) {
        uint8_t motor_id = i + 1; // 电机逻辑 ID 通常从 1 开始
        
        motorCurrentData[i].angle_deg   = chassis_motor.getAngleDeg(motor_id);
        motorCurrentData[i].angle_rad   = chassis_motor.getAngleRad(motor_id);
        motorCurrentData[i].last_angle  = chassis_motor.getLastAngleDeg(motor_id);
        motorCurrentData[i].delta_angle = chassis_motor.getAddAngleDeg(motor_id);
        motorCurrentData[i].speed_rpm   = chassis_motor.getVelocityRpm(motor_id);
        motorCurrentData[i].speed_rads  = chassis_motor.getVelocityRads(motor_id);   //角速度，用这个控制电机
        motorCurrentData[i].current     = chassis_motor.getCurrent(motor_id);
        motorCurrentData[i].temp        = chassis_motor.getTemperature(motor_id);
        motorCurrentData[i].torque      = chassis_motor.getTorque(motor_id);
    }
}






