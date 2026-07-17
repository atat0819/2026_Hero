#ifndef CHASSIS_TASK_HPP
#define CHASSIS_TASK_HPP

#include "FreeRTOS.h"   // FreeRTOS 核心头文件
#include "queue.h"      // 队列相关类型/函数定义
#include "cmsis_os.h"
#include "../user/core/BSP/Motor/Dji/DjiMotor.hpp"
#include "../user/core/Alg/ChassisCalculation/OmniCalculation.hpp"


#ifdef __cplusplus
extern "C" {
#endif

#define M_PI 3.14159265358979323846f

typedef struct
{
    float motor_speeds_1;
    float motor_speeds_2;
    float motor_speeds_3;
    float motor_speeds_4;
} MotorSpeedTarget_t;
extern MotorSpeedTarget_t motorSpeedTarget; // 声明一个全局变量来存储电机速度数据

typedef struct 
{
    float vx;
    float vy;
    float wz;
} chassisCurrentData_t;
extern chassisCurrentData_t chassisCurrentData; // 定义一个结构体来存储底盘当前数据

typedef struct 
{
    float angle_deg;
    float angle_rad;
    float last_angle;
    float delta_angle;
    float speed_rpm;
    float speed_rads;
    float current;
    float torque;
    float temp;
} MotorCurrentData_t;
extern MotorCurrentData_t motorCurrentData[4];



extern QueueHandle_t motorspeedtargetQueue; // 声明一个全局队列句柄，用于在任务之间传递电机转速数据
extern QueueHandle_t motorCurrentDataQueue; // 声明一个全局队列句柄，用于在任务之间传递电机当前数据
extern QueueHandle_t chassisCurrentDataQueue; // 声明一个全局队列句柄，用于在任务之间传递底盘当前数据




	// 1. 定义物理常数（根据你的麦轮实际参数修改）
	extern const float wheel_azimuth[4]; // 辊子角度
	extern const float wheel_coords[4][2]; // 轮子位置
	extern float azimuth_for_fk[4]; // 正运动学使用的轮子方位角（与安装方向相关）

    // 2. 实例化三大核心模块
	extern Alg::CalculationBase::Omni_IK ik; // 逆运动学
	extern Alg::CalculationBase::Omni_FK fk; // 正运动学
	extern Alg::CalculationBase::Omni_ID id; // 逆动力学


// 模板参数 <N> 表示电机数量
// 4 个底盘电机
 extern const uint8_t chassis_motor_idxs[4];
extern BSP::Motor::Dji::GM3508<4> chassis_motor;



void chassis_task(void *argument);
void CAN1_RxCallback(HAL::CAN::Frame& frame);
void SafetyCheck();


#ifdef __cplusplus
}
#endif

#endif // CHASSIS_TASK_HPP
