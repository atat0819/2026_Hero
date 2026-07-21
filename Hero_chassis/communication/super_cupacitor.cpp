/**
 * @file super_cupacitor.cpp
 * @brief 超级电容通信协议解析类实现
 * @version 0.0.1
 * @date 2025-07-16
 */

#include "super_cupacitor.hpp"
#include "HAL/CAN/interface/can_bus.hpp"

namespace Communication
{

SuperCapacitor::SuperCapacitor(uint32_t time_threshold_ms)
    : power_in_(0.0f)
    , energy_(0.0f)
    , chassis_power_(0.0f)
    , state_(State::NORMAL)
    , cmd_(Command::START)
    , online_(false)
    , timeout_cnt_(time_threshold_ms + 1)  // 初始化为离线，收到数据后 resetTimeout 置零
    , time_threshold_ms_(time_threshold_ms)
{
}

void SuperCapacitor::parse(const HAL::CAN::Frame &frame)
{
    // 仅处理超级电容的CAN ID，且数据长度必须为8字节
    if (frame.id != RX_CAN_ID || frame.dlc < 8)
        return;

    // Byte 0-1: Power_In * 10.0f → int16_t 大端
    int16_t power_in_raw = (int16_t)((frame.data[0] << 8) | frame.data[1]);
    power_in_ = (float)power_in_raw / 10.0f;

    // Byte 2-3: 超电剩余能量 (J) → int16_t 大端
    int16_t energy_raw = (int16_t)((frame.data[2] << 8) | frame.data[3]);
    energy_ = (float)energy_raw;

    // Byte 4-5: 底盘功率 (W) → int16_t 大端
    int16_t chassis_power_raw = (int16_t)((frame.data[4] << 8) | frame.data[5]);
    chassis_power_ = (float)chassis_power_raw;

    // Byte 6: 超电状态
    state_ = static_cast<State>(frame.data[6]);

    // Byte 7: 当前指令
    cmd_ = static_cast<Command>(frame.data[7]);

    // 收到数据，清零超时计数器
    resetTimeout();
}

void SuperCapacitor::updateOnlineStatus()
{
    // 防止计数器无限增长（溢出前封顶）
    if (timeout_cnt_ <= time_threshold_ms_)
    {
        timeout_cnt_++;
    }

    online_ = (timeout_cnt_ <= time_threshold_ms_);
}

HAL_StatusTypeDef SuperCapacitor::sendToSuperCap(float level_power,
                                                uint8_t super_cap_cmd,
                                                float communication_energy,
                                                uint8_t is_supercap_online,
                                                uint8_t is_referee_online)
{
    // 1. 使用项目统一的 Frame 结构体（零初始化）
    HAL::CAN::Frame frame = {};

    // 2. 配置帧信息
    frame.id              = TX_CAN_ID;
    frame.dlc             = 8;
    frame.is_extended_id  = false;
    frame.is_remote_frame = false;

    // 3. 按协议填充数据（超电协议使用 int16_t 大端，非 raw float）
    // Byte 0-1: 等级功率 (int16_t 大端)
    int16_t power_val = (int16_t)level_power;
    frame.data[0] = (power_val >> 8) & 0xFF;
    frame.data[1] = power_val & 0xFF;

    // Byte 2: 超电指令
    frame.data[2] = super_cap_cmd;

    // Byte 3-4: 缓冲能量 (int16_t 大端)
    int16_t energy_val = (int16_t)communication_energy;
    frame.data[3] = (energy_val >> 8) & 0xFF;
    frame.data[4] = energy_val & 0xFF;

    // Byte 5: 超电在线标志
    frame.data[5] = is_supercap_online;

    // Byte 6: 裁判系统在线标志
    frame.data[6] = is_referee_online;

    // Byte 7: 保留
    frame.data[7] = 0;

    // 4. 通过 CAN1 发送
    auto &can_bus = HAL::CAN::get_can_bus_instance();
    bool success = can_bus.get_device(HAL::CAN::CanDeviceId::HAL_Can1).send(frame);

    return success ? HAL_OK : HAL_ERROR;
}

} // namespace Communication
