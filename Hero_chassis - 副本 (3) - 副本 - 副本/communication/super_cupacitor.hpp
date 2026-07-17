/**
 * @file super_cupacitor.hpp
 * @brief 超级电容通信协议解析类
 * @version 0.0.1
 * @date 2025-07-16
 *
 * @details
 * 协议格式 (8字节，CAN ID: 0x777):
 *   [0-1] Power_In * 10.0f  (int16_t, 大端)
 *   [2-3] 超电剩余能量 J     (int16_t, 大端)
 *   [4-5] 底盘功率 W         (int16_t, 大端)
 *   [6]   超电状态           (0=Normal, 1=Error, 2=Warning/放电中)
 *   [7]   当前指令           (0=启动, 1=停止)
 */

#pragma once

#include "HAL/CAN/interface/can_device.hpp"
#include <cstdint>

namespace Communication
{

class SuperCapacitor
{
public:
    // 超电状态枚举
    enum class State : uint8_t
    {
        NORMAL  = 0,  // 正常
        ERROR   = 1,  // 错误
        WARNING = 2   // 警告 / 放电中
    };

    // 超电指令枚举
    enum class Command : uint8_t
    {
        START = 0,  // 启动（默认）
        STOP  = 1   // 停止
    };

    // 构造函数
    // time_threshold_ms: 超时阈值(ms)，超过此时间未收到数据视为离线
    explicit SuperCapacitor(uint32_t time_threshold_ms = 500);

    ~SuperCapacitor() = default;

    // ======================================================
    // 核心函数：数据解析入口
    // ======================================================

    /**
     * @brief 解析超级电容发来的CAN帧数据
     * @param frame CAN帧引用
     */
    void parse(const HAL::CAN::Frame &frame);

    /**
     * @brief 更新在线状态（需要在主循环中周期性调用，如每 1ms 调用一次）
     * @note 通过检查距离上次收到数据的时间来判断超电是否在线
     */
    void updateOnlineStatus();

    // ======================================================
    // 发送函数：底盘 → 超级电容 (CAN ID: 0x666)
    // ======================================================

    /**
     * @brief 通过 CAN2 向超级电容发送控制数据 (CAN ID: 0x666)
     * @param level_power 等级功率 (W)
     * @param super_cap_cmd 超电指令 (0=开启, 1=关闭)
     * @param communication_energy 缓冲能量 (J), 建议范围 0~60
     * @param is_supercap_online 超电在线标志 (0=离线, 非0=在线)
     * @param is_referee_online 裁判系统在线标志 (0=离线, 非0=在线)
     * @return HAL_StatusTypeDef 返回 HAL_OK 表示发送成功
     */
    HAL_StatusTypeDef sendToSuperCap(float level_power,
                                     uint8_t super_cap_cmd,
                                     float communication_energy,
                                     uint8_t is_supercap_online,
                                     uint8_t is_referee_online);

    // ======================================================
    // 数据访问接口
    // ======================================================

    /// 获取输入功率 (W)
    inline float getPowerIn() const { return power_in_; }

    /// 获取超电剩余能量 (J)
    inline float getEnergy() const { return energy_; }

    /// 获取底盘功率 (W)
    inline float getChassisPower() const { return chassis_power_; }

    /// 获取超电状态
    inline State getState() const { return state_; }

    /// 获取当前指令
    inline Command getCommand() const { return cmd_; }

    /// 获取超电在线状态
    inline bool isOnline() const { return online_; }

private:
    static constexpr uint32_t RX_CAN_ID = 0x777;  // 接收：超电 → 底盘
    static constexpr uint32_t TX_CAN_ID = 0x666;  // 发送：底盘 → 超电

    /// 重置超时计数器（收到数据时自动调用）
    inline void resetTimeout() { timeout_cnt_ = 0; }

    // 解析后的数据
    float   power_in_;       // 输入功率 (W)
    float   energy_;         // 剩余能量 (J)
    float   chassis_power_;  // 底盘功率 (W)
    State   state_;          // 超电状态
    Command cmd_;            // 当前指令

    // 在线检测
    bool     online_;           // 在线标志
    uint32_t timeout_cnt_;      // 超时计数器 (ms)
    uint32_t time_threshold_ms_; // 超时阈值 (ms)
};

} // namespace Communication
