/**
 * @file gimbal_refree.hpp
 * @brief 裁判系统数据 → 云台转发类 (英雄机器人)
 * @version 0.0.1
 * @date 2025-07-16
 *
 * @details
 * 通过 CAN2 将裁判系统枪管热量相关数据转发给云台主控板:
 *   0x520: [0-1] 枪管冷却值        (uint16_t, 大端)
 *           [2-3] 枪管热量上限      (uint16_t, 大端)
 *           [4-5] 42mm枪管当前热量  (uint16_t, 大端)
 *           [6-7] 保留
 */

#pragma once

#include "HAL/CAN/interface/can_device.hpp"
#include <cstdint>

namespace Communication
{

class GimbalRefree
{
public:
    GimbalRefree()  = default;
    ~GimbalRefree() = default;

    /**
     * @brief 发送枪管热量数据给云台 (CAN ID: 0x520)
     * @param cooling_value 枪管每秒冷却值
     * @param heat_limit    枪管热量上限
     * @param heat_42mm     42mm枪管当前热量
     * @return HAL_OK / HAL_ERROR
     */
    HAL_StatusTypeDef send(uint16_t cooling_value,
                           uint16_t heat_limit,
                           uint16_t heat_42mm);

private:
    static constexpr uint32_t TX_ID = 0x520;
};

} // namespace Communication
