/**
 * @file gimbal_refree.cpp
 * @brief 裁判系统数据 → 云台转发类实现 (英雄机器人)
 * @version 0.0.1
 * @date 2025-07-16
 */

#include "gimbal_refree.hpp"
#include "HAL/CAN/interface/can_bus.hpp"

namespace Communication
{

HAL_StatusTypeDef GimbalRefree::send(uint16_t cooling_value,
                                      uint16_t heat_limit,
                                      uint16_t heat_42mm)
{
    HAL::CAN::Frame frame = {};

    frame.id              = TX_ID;
    frame.dlc             = 8;
    frame.is_extended_id  = false;
    frame.is_remote_frame = false;

    // Byte 0-1: 枪管每秒冷却值 (uint16_t 大端)
    frame.data[0] = (cooling_value >> 8) & 0xFF;
    frame.data[1] = cooling_value & 0xFF;

    // Byte 2-3: 枪管热量上限 (uint16_t 大端)
    frame.data[2] = (heat_limit >> 8) & 0xFF;
    frame.data[3] = heat_limit & 0xFF;

    // Byte 4-5: 42mm枪管当前热量 (uint16_t 大端)
    frame.data[4] = (heat_42mm >> 8) & 0xFF;
    frame.data[5] = heat_42mm & 0xFF;

    // Byte 6-7: 保留
    frame.data[6] = 0;
    frame.data[7] = 0;

    auto &can_bus = HAL::CAN::get_can_bus_instance();
    bool success  = can_bus.get_device(HAL::CAN::CanDeviceId::HAL_Can2).send(frame);

    return success ? HAL_OK : HAL_ERROR;
}

} // namespace Communication
