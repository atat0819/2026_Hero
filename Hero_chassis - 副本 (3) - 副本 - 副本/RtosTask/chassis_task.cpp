#include "chassis_task.hpp"
#include "FreeRTOS.h"
#include "queue.h"
#include "cmsis_os.h"
#include "remote_task.hpp"
//#include "../user/core/Alg/ChassisCalculation/CalculationBase.hpp"


extern "C" void chassis_task(void *argument)
{
	RemoteData_t ChassisData;     // 解析后的遥控器数据结构体
	

	

	// migrated from Core/Src/freertos.c
	for (;;)
	{
		

			//发送电机can反馈数据
			//SafetyCheck(); // 安全检查，确保电机在线

        //     //运动学正解算，得到底盘速度
        // fk.OmniForKinematics(motorCurrentData[0].angle_rad, motorCurrentData[1].angle_rad, motorCurrentData[2].angle_rad, motorCurrentData[3].angle_rad);
        // chassisCurrentData.vx = fk.GetChassisVx();  //得到底盘X轴方向的速度
        // chassisCurrentData.vy = fk.GetChassisVy();   //得到底盘X轴方向的速度
        // chassisCurrentData.wz = fk.GetChassisVw();    //得到底盘X轴方向的速度

        // xQueueSend(chassisCurrentDataQueue, &chassisCurrentData, pdMS_TO_TICKS(10)); // 将底盘当前数据发送到队列

		osDelay(20);
	}
}




//extern "C" void SafetyCheck()
//{
//    // 遍历所有电机
//    for (int i = 1; i <= 4; i++)
//    {
//        // isConnected(电机ID, CAN ID)
//        if (!chassis_motor.isConnected(i, 0x200 + i))
//        {
//            // 电机 i 掉线了！
//            // 蜂鸣器会自动报警

//            // 你可以在这里做一些安全处理
//            // 比如停止所有电机
//			remoteData.vx = 0.0f;
//			remoteData.vy = 0.0f;
//			remoteData.wz = 0.0f;
//			remoteData.s1 = 0;
//			remoteData.s2 = 0;
//        }
//    }

//    // 或者直接获取掉线的电机编号
//    uint8_t offline_motor = chassis_motor.getOfflineStatus();
//	
//    // 返回 0 表示都在线
//    // 返回 1-4 表示对应电机掉线

//}
