/**
 * @file can_device_impl.hpp
 * @author 竹节虫 (k.yixiang@qq.com)
 * @brief CAN设备实现
 * @version 0.0.1
 * @date 2025-06-03
 *
 * @copyright SZPU-RCIA (c) 2025
 *
 */

#pragma once
#include "../interface/can_device.hpp"
#include <vector>

namespace HAL::CAN
{

struct CanTxDebug
{
    volatile uint32_t ok_count;
    volatile uint32_t fail_count;
    volatile uint32_t no_mailbox_count;
    volatile uint32_t add_tx_fail_count;
    volatile uint32_t last_hal_status;
    volatile uint32_t last_hal_error;
    volatile uint32_t last_esr;
    volatile uint32_t last_tsr;
    volatile uint32_t last_frame_id;
    volatile uint32_t recover_count;
    volatile uint8_t last_free_mailboxes;
};

extern CanTxDebug can1_tx_debug;
extern CanTxDebug can2_tx_debug;

// CAN硬件设备实现类
class CanDevice : public ICanDevice
{
  public:
    // 构造函数，初始化CAN设备
    explicit CanDevice(CAN_HandleTypeDef *handle, uint32_t filter_bank, uint32_t fifo);

    // 析构函数
    ~CanDevice() override = default;

    // 禁止拷贝构造和赋值操作
    CanDevice(const CanDevice &) = delete;
    CanDevice &operator=(const CanDevice &) = delete;

    // 实现ICanDevice接口
    void init() override;
    void start() override;
    bool send(const Frame &frame) override;
    bool receive(Frame &frame) override;
    CAN_HandleTypeDef *get_handle() const override;

    // 实现回调机制
    void register_rx_callback(RxCallback callback) override;
    void trigger_rx_callbacks(const Frame &frame) override;

  private:
    CAN_HandleTypeDef *handle_;
    uint32_t filter_bank_;
    uint32_t fifo_;
    uint32_t mailbox_;

    // 存储注册的回调函数
    std::vector<RxCallback> rx_callbacks_;

    // 配置过滤器
    void configure_filter();
};

} // namespace HAL::CAN
