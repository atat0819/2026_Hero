#ifndef VISION_COMMUNICATION_HPP
#define VISION_COMMUNICATION_HPP

#include <cstdint>

namespace HAL::UART { struct Data; }

namespace BSP::Vision
{

class VisionCommunicator
{
public:
    VisionCommunicator();
    ~VisionCommunicator() = default;

    // 敌方颜色常量
    static constexpr uint8_t ENEMY_RED   = 0x42;
    static constexpr uint8_t ENEMY_BLUE  = 0x52;

    // ==================== 电控 → 视觉 ====================
    /**
     * @brief 发送数据给视觉模块
     * @param quaternion 四元数数组 [w, x, y, z]
     * @param bullet_rate 弹速 (m/s)
     * @param enemy_color 敌方颜色 (ENEMY_RED / ENEMY_BLUE)
     * @param vision_mode 视觉模式
     */
    void SendToVision(const float quaternion[4], float bullet_rate,
                      uint8_t enemy_color, uint8_t vision_mode);

    // ==================== 视觉 → 电控 ====================

    void ParseRxData(const uint8_t* data, uint16_t size);

    float    GetPitchAngle()      const { return pitch_angle_; }
    float    GetYawAngle()        const { return yaw_angle_; }
    bool     IsVisionReady()      const { return vision_ready_; }
    bool     IsFireCommanded()    const { return fire_command_; }
    uint32_t GetTimestamp()       const { return timestamp_; }
    uint8_t  GetAimX()            const { return aim_x_; }
    uint8_t  GetAimY()            const { return aim_y_; }

    /// @brief 视觉断联超时阈值（毫秒），超过此时间未收到有效帧视为断联
    static constexpr uint32_t DATA_TIMEOUT_MS = 1000;

    /// @brief 检查最近一次有效帧是否在超时时间内
    bool     IsDataFresh() const;

    uint8_t* GetRxBuffer() { return rx_buffer_; }
    static constexpr uint16_t RX_BUFFER_SIZE = 19;

private:
    // TX
    static constexpr uint8_t TX_FRAME_SIZE = 29;
    static constexpr uint8_t HEADER        = 0x39;
    static constexpr uint8_t TAIL          = 0xFF;
    static constexpr uint8_t Q_OFFSET      = 2;
    static constexpr uint8_t BR_OFFSET     = 18;   // bullet_rate
    static constexpr uint8_t EC_OFFSET     = 22;   // enemy_color
    static constexpr uint8_t VM_OFFSET     = 23;   // vision_mode
    static constexpr uint8_t TIME_OFFSET   = 25;   // timestamp (after tail!)

    uint8_t tx_buffer_[TX_FRAME_SIZE];
    void writeFloatBE(uint8_t offset, float value);

    // RX
    uint8_t rx_buffer_[RX_BUFFER_SIZE];

    float    pitch_angle_;
    float    yaw_angle_;
    bool     vision_ready_;
    bool     fire_command_;
    uint32_t timestamp_;
    uint8_t  aim_x_;
    uint8_t  aim_y_;
    uint32_t last_rx_tick_;
};

} // namespace BSP::Vision

#endif // VISION_COMMUNICATION_HPP
