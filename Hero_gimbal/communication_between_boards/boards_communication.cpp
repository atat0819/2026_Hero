#include "can.h"
#include "can_send_task.hpp"
#include "gimbal_task.hpp"
#include "boards_communication.hpp"
#include "../user/core/HAL/CAN/can_hal.hpp"
#include <string.h>

static bool yaw_zero_inited = false;
static float yaw_zero_add_angle_deg = 0.0f;

    float yaw_offset_deg = 0.0f;


void YawOffset_SetZero(void)
{
    yaw_zero_add_angle_deg = mg4005_state[1].delta_angle;
    yaw_zero_inited = true;
}

float YawOffset_GetDeg(void)
{
    

    if (!yaw_zero_inited)
    {
        yaw_zero_add_angle_deg = mg4005_state[1].delta_angle;
        yaw_zero_inited = true;
    }

    yaw_offset_deg = mg4005_state[1].delta_angle - yaw_zero_add_angle_deg;

    while (yaw_offset_deg > 180.0f)  yaw_offset_deg -= 360.0f;
    while (yaw_offset_deg < -180.0f) yaw_offset_deg += 360.0f;

    return yaw_offset_deg - 18;   //18是电机上电时yaw角的偏值，目的是为了底盘对准云台正方向
}


/**
 * @brief 通过 CAN2 将 Yaw 偏移量发送给另一块板子
 */
HAL_StatusTypeDef YawOffset_SendToCan2(void)
{
    HAL::CAN::Frame frame;
    const float yaw_offset_deg = YawOffset_GetDeg();

    // 填充 CAN 帧
    frame.id = 0x301;
    frame.dlc = 8;
    frame.is_extended_id = false;
    frame.is_remote_frame = false;
    
    // 清空并拷贝数据（float 占 4 字节）
    memset(frame.data, 0, sizeof(frame.data));
    memcpy(frame.data, &yaw_offset_deg, sizeof(yaw_offset_deg));

    // 使用你库中的单例模式获取 CAN2 实例并发送
    // 假设返回 true 代表发送成功
    // 通过 instance() 获取单例，通过 get_device 获取 CAN2 设备
    auto& can_bus = HAL::CAN::get_can_bus_instance();
    bool success = can_bus.get_device(HAL::CAN::CanDeviceId::HAL_Can2).send(frame);
    
    return success ? HAL_OK : HAL_ERROR;
}


/**
 * @brief 通过 CAN2 发送底盘当前移动速度 (Vx, Vy)
 * @param chassis_vx 底盘 X 轴速度 (float)
 * @param chassis_vy 底盘 Y 轴速度 (float)
 * @return HAL_StatusTypeDef 返回 HAL_OK 表示发送成功
 */
HAL_StatusTypeDef CAN2_SendChassisSpeed(float chassis_vx, float chassis_vy)
{
    // 1. 使用项目统一的 Frame 结构体
    HAL::CAN::Frame frame = {};

    // 2. 配置帧信息
    frame.id = 0x302;           // 底盘速度反馈 ID
    frame.dlc = 8;              // 两个 float 共 8 字节
    frame.is_extended_id = false;
    frame.is_remote_frame = false;

    // 3. 使用 memcpy 填充数据 (确保浮点数精度不丢失)
    // frame.data 已经在初始化时清零
    memcpy(&frame.data[0], &chassis_vx, sizeof(float));
    memcpy(&frame.data[4], &chassis_vy, sizeof(float));

    // 4. 调用库中的单例发送接口
    // 通过 instance() 获取单例，通过 get_device 获取 CAN2 设备
    auto& can_bus = HAL::CAN::get_can_bus_instance();
    bool success = can_bus.get_device(HAL::CAN::CanDeviceId::HAL_Can2).send(frame);
    
    return success ? HAL_OK : HAL_ERROR;
}


/**
 * @brief 通过 CAN2 发送 S1 和 S2 状态（使用 memcpy 确保内存操作规范）
 * @param s1 状态1
 * @param s2 状态2
 * @return HAL_StatusTypeDef 返回 HAL_OK 表示发送成功
 */
HAL_StatusTypeDef CAN2_Send_S1andS2_Status(uint8_t s1, uint8_t s2)
{
    // 1. 使用项目统一的 Frame 结构体
    HAL::CAN::Frame frame = {};

    // 2. 配置帧信息
    frame.id = 0x303;           // 状态位 ID
    frame.dlc = 2;              // 数据长度为 2 字节
    frame.is_extended_id = false;
    frame.is_remote_frame = false;

    // 3. 准备临时缓冲区并使用 memcpy 填充数据
    // 这样做可以确保数据在内存中是连续且安全的
    uint8_t status_buffer[2] = {s1, s2};
    
    // 初始化 frame.data 后进行拷贝
    memset(frame.data, 0, sizeof(frame.data));
    memcpy(&frame.data[0], status_buffer, sizeof(status_buffer));

    // 4. 调用库中的单例发送接口
  // 通过 instance() 获取单例，通过 get_device 获取 CAN2 设备
    auto& can_bus = HAL::CAN::get_can_bus_instance();
    bool success = can_bus.get_device(HAL::CAN::CanDeviceId::HAL_Can2).send(frame);
    
    return success ? HAL_OK : HAL_ERROR;
}

/**
 * @brief 通过 CAN2 发送云台 IMU 原始浮点数据
 * @param yaw 角度 (float)
 * @param gyro_z 角速度 (float)
 */
HAL_StatusTypeDef CAN2_SendGimbalIMU_Raw(float yaw, float gyro_z)
{
    HAL::CAN::Frame frame = {};

    frame.id = 0x304;           // 云台 IMU 反馈 ID
    frame.dlc = 8;              // 两个 float 共 8 字节
    frame.is_extended_id = false;
    frame.is_remote_frame = false;

    // IMU yaw 现在是 [-180, 180)，底盘如仍用 [0, 360) 则在此转换
    float yaw_0_360 = yaw;
    while (yaw_0_360 < 0.0f) yaw_0_360 += 360.0f;
    memcpy(&frame.data[0], &yaw_0_360, sizeof(float));
    memcpy(&frame.data[4], &gyro_z, sizeof(float));

    auto& can_bus = HAL::CAN::get_can_bus_instance();
    bool success = can_bus.get_device(HAL::CAN::CanDeviceId::HAL_Can2).send(frame);

    return success ? HAL_OK : HAL_ERROR;
}

/**
 * @brief 通过 CAN2 发送键盘位掩码给底盘开发板
 * @param keyboard 16 位键盘位掩码（小端）
 * @return HAL_StatusTypeDef 返回 HAL_OK 表示发送成功
 */
HAL_StatusTypeDef CAN2_SendKeyboard(uint16_t keyboard)
{
    HAL::CAN::Frame frame = {};

    frame.id = 0x305;           // 键盘数据 ID
    frame.dlc = 2;              // uint16_t = 2 字节
    frame.is_extended_id = false;
    frame.is_remote_frame = false;

    memcpy(&frame.data[0], &keyboard, sizeof(keyboard));

    auto& can_bus = HAL::CAN::get_can_bus_instance();
    bool success = can_bus.get_device(HAL::CAN::CanDeviceId::HAL_Can2).send(frame);

    return success ? HAL_OK : HAL_ERROR;
}
