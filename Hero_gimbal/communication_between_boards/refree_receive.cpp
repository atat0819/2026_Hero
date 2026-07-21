/**
 * @file refree_receive.cpp
 * @brief 裁判系统热量数据接收类实现 (云台端)
 * @version 0.0.1
 * @date   2025-07-16
 */

#include "refree_receive.hpp"
#include "../user/core/HAL/CAN/can_hal.hpp"

namespace Communication
{

GimbalRefree &GimbalRefree::instance()
{
    static GimbalRefree inst;
    return inst;
}

/**
 * @brief 解析 CAN 帧 (大端字节序 → 主机字节序)
 *
 * 与底盘 Communication::GimbalRefree::send() 的封包顺序严格对应:
 *   data[0:1] → cooling_value
 *   data[2:3] → heat_limit
 *   data[4:5] → heat_42mm
 */
void GimbalRefree::parse(const HAL::CAN::Frame &frame)
{
    // DLC 校验：底盘发送固定 8 字节
    if (frame.dlc != 8)
    {
        return;
    }

    // 大端解码：高字节在前
    cooling_value_ = (static_cast<uint16_t>(frame.data[0]) << 8) | frame.data[1];
    heat_limit_    = (static_cast<uint16_t>(frame.data[2]) << 8) | frame.data[3];
    heat_42mm_     = (static_cast<uint16_t>(frame.data[4]) << 8) | frame.data[5];

    data_received_ = true;
}

} // namespace Communication
