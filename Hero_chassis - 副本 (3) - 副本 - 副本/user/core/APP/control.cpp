/* control.cpp — 废弃代码，control.hpp 不存在，已整体禁用。
   如需恢复，请在 Keil 工程中移除此文件，或补全 control.hpp */
#if 0
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

void ControlTask_t::select_power() {
}

void ControlTask_t::Control_3508_MotorSpeed()
{
    using namespace BSP::Motor::Dji;
    using namespace powerMeter;

        float omega = BSP::Motor::Dji::Motor3508.getVelocityRads(1);
        float current_torque = BSP::Motor::Dji::Motor3508.getTorque(1);

        pid_vel_Wheel[0].GetPidPos(Kpid_3508_vel, Chassis_Data.tar_speed[0],
                                  omega, 16384.0f);

        float motor_output = pid_vel_Wheel[0].GetCout();
        Chassis_Data.final_3508_Out[0] = motor_output;

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
#endif
