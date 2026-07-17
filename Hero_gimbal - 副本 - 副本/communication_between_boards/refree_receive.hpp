/**
 * @file refree_receive.hpp
 * @brief 裁判系统热量数据接收类 (云台端)
 * @version 0.0.1
 * @date   2025-07-16
 *
 * @details
 * 接收底盘通过 CAN2 (ID=0x520) 发来的裁判系统枪管热量数据:
 *   Byte 0-1: 枪管每秒冷却值   (uint16_t, 大端)
 *   Byte 2-3: 枪管热量上限     (uint16_t, 大端)
 *   Byte 4-5: 42mm枪管当前热量 (uint16_t, 大端)
 *   Byte 6-7: 保留
 *
 * 使用方式：
 *   // can_send_task.cpp 中注册回调:
 *   can2.register_rx_callback([](const HAL::CAN::Frame &frame) {
 *       if (frame.id == 0x520) {
 *           Communication::GimbalRefree::instance().parse(frame);
 *       }
 *   });
 *
 *   // 任意位置读取:
 *   uint16_t cooling = Communication::GimbalRefree::instance().getCoolingValue();
 */

#pragma once

#include <stdint.h>

// 前向声明 CAN 帧类型
namespace HAL {
namespace CAN {
struct Frame;
}
}

namespace Communication
{

class GimbalRefree
{
  public:
    /// @brief 获取单例实例
    static GimbalRefree &instance();

    /**
     * @brief 解析一帧 CAN 数据并存储 (由 CAN 回调调用)
     * @param frame CAN2 收到的帧 (ID=0x520)
     */
    void parse(const HAL::CAN::Frame &frame);

    // ---- get 接口 ----

    /// @brief 获取枪管每秒冷却值
    uint16_t getCoolingValue() const { return cooling_value_; }

    /// @brief 获取枪管热量上限
    uint16_t getHeatLimit() const { return heat_limit_; }

    /// @brief 获取 42mm 枪管当前热量
    uint16_t getHeat42mm() const { return heat_42mm_; }

    /// @brief 是否已收到过数据 (用于判断通信是否建立)
    bool isDataReceived() const { return data_received_; }

  private:
    GimbalRefree() = default;

    uint16_t cooling_value_ = 0;
    uint16_t heat_limit_    = 0;
    uint16_t heat_42mm_     = 0;
    bool     data_received_  = false;
};

} // namespace Communication
