#include "control.hpp"
#include "../BSP/Bsp_Can.hpp"
#include "../Motor/MotorBase.hpp"
#include "../BSP/Tools.hpp"
#include "../BSP/PM01.hpp"
#include "cmsis_os2.h"
#define PI 3.14159265358979f

void ChassisTask(void *argument)
{
	for(;;)
	{
	ControlTask.select_power();	
	ControlTask.Control_3508_MotorSpeed();
	ControlTask.powertest();
		osDelay(1);
	}
}

// Chassis_Data.tar_speed[0]
void ControlTask_t::select_power() {
  // 控制逻辑已移到Control_3508_MotorSpeed函数中
}

void ControlTask_t::Control_3508_MotorSpeed()
{
    using namespace BSP::Motor::Dji;
    using namespace powerMeter;
    
//    static int counter = 0;
//    counter++;
//    
//    // 默认使用PID控制
//    bool use_open_loop = false;
//    float open_loop_value = 0.0f;
//    
//    // 基于时间的测试序列
//    if(counter <= 2000) {
//        Chassis_Data.tar_speed[0] = -100.0f;
//    }
//    else if(counter <= 6000) {
//        Chassis_Data.tar_speed[0] = -50.0f;
//    }
//    else if(counter <= 8000) {
//        Chassis_Data.tar_speed[0] = -20.0f;
//    }
//    else if(counter <= 12000) {
//        Chassis_Data.tar_speed[0] = 30.0f;
//    }
//    else if(counter <= 16000) {
//        Chassis_Data.tar_speed[0] = 50.0f;
//    }
//    else if(counter <= 18000) {
//        Chassis_Data.tar_speed[0] = 80.0f;
//    }
//    else if(counter <= 22000) {
//        Chassis_Data.tar_speed[0] = 100.0f;
//    }
//    else if(counter <= 26000) {
//        // 第一阶段：振幅1.0，输出到[-16384, 16384]
//        use_open_loop = true;
//        open_loop_value = 1.0f * sinf(10 * PI * (counter - 22000) / 1000);
//        Chassis_Data.final_3508_Out[0] = open_loop_value * 16384.0f; // 缩放到合适的控制范围
//    }
//    else if(counter <= 30000) {
//        // 第二阶段：振幅2.0，但输出缩放到[-16384, 16384]
//        use_open_loop = true;
//        open_loop_value = 2.0f * sinf(10 * PI * (counter - 22000) / 1000);
//        // 将振幅为2的信号缩放到[-1,1]范围，再乘以16384
//        Chassis_Data.final_3508_Out[0] = (open_loop_value / 2.0f) * 16384.0f; // 缩放到合适的控制范围
//    }
//    else if(counter <= 34000) {
//        // 第三阶段：振幅2.0，但输出缩放到[-16384, 16384]
//        use_open_loop = true;
//        open_loop_value = 2.0f * sinf(10 * PI * (counter - 22000) / 1000);
//        // 将振幅为2的信号缩放到[-1,1]范围，再乘以16384
//        Chassis_Data.final_3508_Out[0] = (open_loop_value / 2.0f) * 16384.0f; // 缩放到合适的控制范围
//    }
//    else {
//        counter = 0;
//    }
    
    // 如果不使用开环控制，则使用PID控制
//    if (!use_open_loop) 
        // 获取当前速度和扭矩
        float omega = BSP::Motor::Dji::Motor3508.getVelocityRads(1);
        float current_torque = BSP::Motor::Dji::Motor3508.getTorque(1);
        
        // PID控制计算
        pid_vel_Wheel[0].GetPidPos(Kpid_3508_vel, Chassis_Data.tar_speed[0], 
                                  omega, 16384.0f);
        
        // 获取PID输出作为电机控制信号
        float motor_output = pid_vel_Wheel[0].GetCout();
        Chassis_Data.final_3508_Out[0] = motor_output;
    
    
    // 设置并发送CAN消息
    BSP::Motor::Dji::Motor3508.setCAN(Chassis_Data.final_3508_Out[0], 1);
    BSP::Motor::Dji::Motor3508.sendCAN(&hcan1, 0);
}

void ControlTask_t::powertest()
{ 
    using namespace STPowerControl;
    float Torque = BSP::Motor::Dji::Motor3508.getTorque(1);
    float W = BSP::Motor::Dji::Motor3508.getVelocityRads(1);
    float test_power = PowerControl.T3508_powerdata.PowerEstimate_3508(Chassis_Data.final_3508_Out, pid_vel_Wheel);
    float Reality_power = powerMeter::rx_message_t.pm_power;
    Tools.vofaSend(Reality_power, Torque, W, test_power, 0, 0);
}