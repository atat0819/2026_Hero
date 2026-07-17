#include "vision_communication.hpp"
#include "../../HAL/UART/uart_hal.hpp"
#include <cstring>
#include "usbd_cdc_if.h"

namespace BSP::Vision
{

VisionCommunicator::VisionCommunicator()
    : pitch_angle_(0.0f), yaw_angle_(0.0f),
      vision_ready_(false), fire_command_(false), timestamp_(0),
      aim_x_(0), aim_y_(0), last_rx_tick_(0)
{
    memset(tx_buffer_, 0, TX_FRAME_SIZE);
    tx_buffer_[0]  = HEADER;
    tx_buffer_[1]  = HEADER;
    tx_buffer_[24] = TAIL;

    memset(rx_buffer_, 0, RX_BUFFER_SIZE);
}

// ==================== 电控 → 视觉 ====================

void VisionCommunicator::writeFloatBE(uint8_t offset, float value)
{
    uint32_t raw;
    memcpy(&raw, &value, sizeof(float));
    raw = __REV(raw);
    memcpy(&tx_buffer_[offset], &raw, sizeof(float));
}

void VisionCommunicator::SendToVision(const float quaternion[4], float bullet_rate,
                                       uint8_t enemy_color, uint8_t vision_mode)
{
    // 四元数 w, x, y, z
    writeFloatBE(Q_OFFSET,      quaternion[0]);
    writeFloatBE(Q_OFFSET + 4,  quaternion[1]);
    writeFloatBE(Q_OFFSET + 8,  quaternion[2]);
    writeFloatBE(Q_OFFSET + 12, quaternion[3]);

    // 弹速
    writeFloatBE(BR_OFFSET, bullet_rate);

    // 敌方颜色 / 视觉模式
    tx_buffer_[EC_OFFSET] = enemy_color;
    tx_buffer_[VM_OFFSET] = vision_mode;

    // 时间戳 (大端 uint32)
    uint32_t tick = HAL_GetTick();
    tx_buffer_[TIME_OFFSET]     = (tick >> 24) & 0xFF;
    tx_buffer_[TIME_OFFSET + 1] = (tick >> 16) & 0xFF;
    tx_buffer_[TIME_OFFSET + 2] = (tick >> 8)  & 0xFF;
    tx_buffer_[TIME_OFFSET + 3] =  tick        & 0xFF;

    CDC_Transmit_FS(tx_buffer_, TX_FRAME_SIZE);
}

// ==================== 视觉 → 电控 ====================

void VisionCommunicator::ParseRxData(const uint8_t* data, uint16_t size)
{
    if (size < RX_BUFFER_SIZE) 
		{return;}
    if (data[0] != HEADER || data[1] != HEADER) 
		{return;}
    if (data[12] != TAIL) 
		{return;}

    // Pitch 角度：大端 int32，单位 0.01 度
    int32_t pitch_raw = (data[2] << 24) | (data[3] << 16)
                      | (data[4] << 8)  |  data[5];
    float pitch = pitch_raw * 0.01f;

    // Yaw 角度：大端 int32，单位 0.01 度
    int32_t yaw_raw = (data[6] << 24) | (data[7] << 16)
                    | (data[8] << 8)  |  data[9];
    float yaw = yaw_raw * 0.01f;

    // --- 内容校验：拒绝 UART 干扰导致的野值，防止疯车 ---
    // Pitch 物理极限 ±90°（远超实际机械限位 -19°~40°，拦截任何 bit 翻转）
    if (pitch > 90.0f || pitch < -90.0f) return;
    // Yaw 合理范围 ±360°（1 圈余量，覆盖视觉 [0,360) 或 [-180,180] 两种
    // 约定，同时拦截中间字节 bit 翻转产生的异常值）
    if (yaw > 360.0f || yaw < -360.0f) return;

    // 校验通过，写入成员变量
    pitch_angle_ = pitch;
    yaw_angle_   = yaw;

    // 标志位
    vision_ready_ = (data[10] != 0);
    fire_command_ = (data[11] != 0);

    // 时间戳：大端 uint32
    timestamp_ = (data[13] << 24) | (data[14] << 16)
               | (data[15] << 8) |  data[16];

    // 目标坐标
    aim_x_ = data[17];
    aim_y_ = data[18];

    // 记录最后一次有效帧的时间戳
    last_rx_tick_ = HAL_GetTick();
}

bool VisionCommunicator::IsDataFresh() const
{
    return (HAL_GetTick() - last_rx_tick_) < DATA_TIMEOUT_MS;
}

} // namespace BSP::Vision
