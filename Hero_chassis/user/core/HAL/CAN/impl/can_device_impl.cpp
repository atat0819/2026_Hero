#include "can_device_impl.hpp"

extern "C" {
typedef struct
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
} CanTxWatch;

volatile CanTxWatch can1_tx_watch = {};
volatile CanTxWatch can2_tx_watch = {};
}

namespace HAL::CAN
{
CanTxDebug can1_tx_debug = {};
CanTxDebug can2_tx_debug = {};


static CanTxDebug &get_tx_debug(CAN_HandleTypeDef *handle)
{
    return (handle->Instance == CAN2) ? can2_tx_debug : can1_tx_debug;
}

static void capture_tx_error(CanTxDebug &debug, CAN_HandleTypeDef *handle)
{
    debug.last_hal_error = HAL_CAN_GetError(handle);
    debug.last_esr = handle->Instance->ESR;
    debug.last_tsr = handle->Instance->TSR;
}

static void reactivate_rx_notification(CAN_HandleTypeDef *handle, uint32_t fifo)
{
    if (fifo == CAN_FILTER_FIFO0)
    {
        HAL_CAN_ActivateNotification(handle, CAN_IT_RX_FIFO0_MSG_PENDING);
    }
    else if (fifo == CAN_FILTER_FIFO1)
    {
        HAL_CAN_ActivateNotification(handle, CAN_IT_RX_FIFO1_MSG_PENDING);
    }
}

static void recover_stuck_tx(CanTxDebug &debug, CAN_HandleTypeDef *handle, uint32_t fifo)
{
    debug.recover_count++;

    HAL_CAN_AbortTxRequest(handle, CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2);

    if ((debug.last_esr & CAN_ESR_BOFF) != 0U)
    {
        HAL_CAN_Stop(handle);
        HAL_CAN_Start(handle);
        reactivate_rx_notification(handle, fifo);
    }

    debug.last_hal_error = HAL_CAN_GetError(handle);
    debug.last_esr = handle->Instance->ESR;
    debug.last_tsr = handle->Instance->TSR;
    debug.last_free_mailboxes = HAL_CAN_GetTxMailboxesFreeLevel(handle);
}

static void mirror_tx_debug_to_global(CAN_HandleTypeDef *handle, const CanTxDebug &debug)
{
    if (handle->Instance == CAN2)
    {
        can2_tx_watch.ok_count = debug.ok_count;
        can2_tx_watch.fail_count = debug.fail_count;
        can2_tx_watch.no_mailbox_count = debug.no_mailbox_count;
        can2_tx_watch.add_tx_fail_count = debug.add_tx_fail_count;
        can2_tx_watch.last_hal_status = debug.last_hal_status;
        can2_tx_watch.last_hal_error = debug.last_hal_error;
        can2_tx_watch.last_esr = debug.last_esr;
        can2_tx_watch.last_tsr = debug.last_tsr;
        can2_tx_watch.last_frame_id = debug.last_frame_id;
        can2_tx_watch.recover_count = debug.recover_count;
        can2_tx_watch.last_free_mailboxes = debug.last_free_mailboxes;
    }
    else
    {
        can1_tx_watch.ok_count = debug.ok_count;
        can1_tx_watch.fail_count = debug.fail_count;
        can1_tx_watch.no_mailbox_count = debug.no_mailbox_count;
        can1_tx_watch.add_tx_fail_count = debug.add_tx_fail_count;
        can1_tx_watch.last_hal_status = debug.last_hal_status;
        can1_tx_watch.last_hal_error = debug.last_hal_error;
        can1_tx_watch.last_esr = debug.last_esr;
        can1_tx_watch.last_tsr = debug.last_tsr;
        can1_tx_watch.last_frame_id = debug.last_frame_id;
        can1_tx_watch.recover_count = debug.recover_count;
        can1_tx_watch.last_free_mailboxes = debug.last_free_mailboxes;
    }
}

// CanDevice实现
CanDevice::CanDevice(CAN_HandleTypeDef *handle, uint32_t filter_bank, uint32_t fifo)
    : handle_(handle), filter_bank_(filter_bank), fifo_(fifo), mailbox_(0)
{
}

void CanDevice::init()
{
    configure_filter();
}

void CanDevice::start()
{
    HAL_CAN_Start(handle_);

    // 设置中断
    if (fifo_ == CAN_FILTER_FIFO0)
    {
        HAL_CAN_ActivateNotification(handle_, CAN_IT_RX_FIFO0_MSG_PENDING);
    }
    else if (fifo_ == CAN_FILTER_FIFO1)
    {
        HAL_CAN_ActivateNotification(handle_, CAN_IT_RX_FIFO1_MSG_PENDING);
    }
}

bool CanDevice::send(const Frame &frame)
{
    CanTxDebug &debug = get_tx_debug(handle_);
    debug.last_frame_id = frame.id;
    debug.last_free_mailboxes = HAL_CAN_GetTxMailboxesFreeLevel(handle_);

    if (debug.last_free_mailboxes == 0)
    {
        debug.fail_count++;
        debug.no_mailbox_count++;
        debug.last_hal_status = HAL_BUSY;
        capture_tx_error(debug, handle_);
        recover_stuck_tx(debug, handle_, fifo_);
        mirror_tx_debug_to_global(handle_, debug);
        return false;
    }

    CAN_TxHeaderTypeDef tx_header;
    tx_header.DLC = frame.dlc;
    tx_header.IDE = frame.is_extended_id ? CAN_ID_EXT : CAN_ID_STD;
    tx_header.RTR = frame.is_remote_frame ? CAN_RTR_REMOTE : CAN_RTR_DATA;
    uint32_t temp_mailbox = frame.mailbox;

    if (frame.is_extended_id)
    {
        tx_header.ExtId = frame.id;
        tx_header.StdId = 0;
    }
    else
    {
        tx_header.StdId = frame.id;
        tx_header.ExtId = 0;
    }

    tx_header.TransmitGlobalTime = DISABLE;

    HAL_StatusTypeDef status = HAL_CAN_AddTxMessage(handle_, &tx_header, const_cast<uint8_t *>(frame.data), &temp_mailbox);
    if (status != HAL_OK)
    {
        debug.fail_count++;
        debug.add_tx_fail_count++;
        debug.last_hal_status = status;
        capture_tx_error(debug, handle_);
        mirror_tx_debug_to_global(handle_, debug);
        return false;
    }

    debug.ok_count++;
    debug.last_hal_status = HAL_OK;
    debug.last_hal_error = 0;
    debug.last_tsr = handle_->Instance->TSR;
    mirror_tx_debug_to_global(handle_, debug);

    return true;
}

bool CanDevice::receive(Frame &frame)
{
    CAN_RxHeaderTypeDef rx_header;

    if (HAL_CAN_GetRxFifoFillLevel(handle_, fifo_) == 0)
    {
        return false;
    }

    if (HAL_CAN_GetRxMessage(handle_, fifo_, &rx_header, frame.data) != HAL_OK)
    {
        return false;
    }

    // 填充Frame结构体
    frame.id = rx_header.IDE == CAN_ID_STD ? rx_header.StdId : rx_header.ExtId;
    frame.dlc = rx_header.DLC;
    frame.is_extended_id = (rx_header.IDE == CAN_ID_EXT);
    frame.is_remote_frame = (rx_header.RTR == CAN_RTR_REMOTE);

    // 自动触发所有注册的回调函数
    trigger_rx_callbacks(frame);

    return true;
}

CAN_HandleTypeDef *CanDevice::get_handle() const
{
    return handle_;
}

void CanDevice::configure_filter()
{
    CAN_FilterTypeDef filter;
    filter.FilterActivation = CAN_FILTER_ENABLE; // 使能过滤器
    filter.FilterBank = filter_bank_;            // 通道
    filter.FilterFIFOAssignment = fifo_;         // 缓冲器
    filter.FilterIdHigh = 0x0;                   // 高16
    filter.FilterIdLow = 0x0;                    // 低16
    filter.FilterMaskIdHigh = 0x0;               // 高16
    filter.FilterMaskIdLow = 0x0;                // 低16
    filter.FilterMode = CAN_FILTERMODE_IDMASK;   // 掩码
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.SlaveStartFilterBank = 14;

    HAL_CAN_ConfigFilter(handle_, &filter);
}

void CanDevice::register_rx_callback(RxCallback callback)
{
    if (callback)
    {
        rx_callbacks_.push_back(callback);
    }
}

void CanDevice::trigger_rx_callbacks(const Frame &frame)
{
    for (auto &callback : rx_callbacks_)
    {
        if (callback)
        {
            callback(frame);
        }
    }
}

} // namespace HAL::CAN
