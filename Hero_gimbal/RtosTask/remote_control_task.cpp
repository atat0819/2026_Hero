#include "remote_control_task.hpp"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "usart.h"
#include "../user/core/HAL/UART/uart_hal.hpp"
#include "../user/core/BSP/version/vision_communication.hpp"
#include "gimbal_task.hpp"
#include <cstring>

QueueHandle_t remoteDataQueue;
QueueHandle_t IMUDataQueue;

uint8_t receivedata[18];
RemoteData_t remote;
BSP::REMOTE_CONTROL::RemoteController remoteController(100);
BSP::IMU::HI12_float imu;

extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_uart8_rx;

#define IMU_DMA_BUF_SIZE 82   // HI12 数据帧长度
static uint8_t imu_dma_buffer[IMU_DMA_BUF_SIZE];

// 视觉通信实例（文件作用域，HAL 回调需要访问其 RX 缓冲区）
BSP::Vision::VisionCommunicator vision_comm;

// ==================== HAL UART 回调 ====================

// UART8 改用 DMA+Idle，不再使用逐字节 IT，此回调不再处理 UART8
extern "C" void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    (void)huart;
}

extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    auto &uart1 = HAL::UART::get_uart_bus_instance().get_uart1();

    if (huart == uart1.get_handle())
    {
        HAL::UART::Data rx{receivedata, Size};
        uart1.trigger_rx_callbacks(rx);

        HAL::UART::Data next_rx{receivedata, sizeof(receivedata)};
        uart1.receive_dma_idle(next_rx);
        __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
    }

    // USART3 — 视觉通信接收
    auto &uart3 = HAL::UART::get_uart_bus_instance().get_uart3();
    if (huart == uart3.get_handle())
    {
        HAL::UART::Data rx{vision_comm.GetRxBuffer(), Size};
        uart3.trigger_rx_callbacks(rx);

        HAL::UART::Data next_rx{vision_comm.GetRxBuffer(), BSP::Vision::VisionCommunicator::RX_BUFFER_SIZE};
        uart3.receive_dma_idle(next_rx);
        __HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT);
    }

    // UART8 — IMU DMA+Idle 接收
    auto &uart8 = HAL::UART::get_uart_bus_instance().get_uart8();
    if (huart == uart8.get_handle())
    {
        HAL::UART::Data rx{imu_dma_buffer, Size};
        uart8.trigger_rx_callbacks(rx);

        HAL::UART::Data next_rx{imu_dma_buffer, IMU_DMA_BUF_SIZE};
        uart8.receive_dma_idle(next_rx);
        __HAL_DMA_DISABLE_IT(&hdma_uart8_rx, DMA_IT_HT);
    }
}

extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    auto &uart1 = HAL::UART::get_uart_bus_instance().get_uart1();
    auto &uart3 = HAL::UART::get_uart_bus_instance().get_uart3();
    auto &uart8 = HAL::UART::get_uart_bus_instance().get_uart8();

    if (huart == uart1.get_handle())
    {
        volatile uint32_t sr = huart->Instance->SR;
        volatile uint32_t dr = huart->Instance->DR;
        (void)sr;
        (void)dr;

        __HAL_UART_DISABLE_IT(huart, UART_IT_RXNE);
        __HAL_UART_CLEAR_OREFLAG(huart);
        __HAL_UART_CLEAR_NEFLAG(huart);
        __HAL_UART_CLEAR_FEFLAG(huart);
        __HAL_UART_CLEAR_PEFLAG(huart);

        huart->RxState = HAL_UART_STATE_READY;
        huart->Lock = HAL_UNLOCKED;

        HAL::UART::Data rx{receivedata, sizeof(receivedata)};
        uart1.receive_dma_idle(rx);
        __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
    }
    else if (huart == uart3.get_handle())
    {
        volatile uint32_t sr = huart->Instance->SR;
        volatile uint32_t dr = huart->Instance->DR;
        (void)sr;
        (void)dr;

        __HAL_UART_DISABLE_IT(huart, UART_IT_RXNE);
        __HAL_UART_CLEAR_OREFLAG(huart);
        __HAL_UART_CLEAR_NEFLAG(huart);
        __HAL_UART_CLEAR_FEFLAG(huart);
        __HAL_UART_CLEAR_PEFLAG(huart);

        huart->RxState = HAL_UART_STATE_READY;
        huart->Lock = HAL_UNLOCKED;

        HAL::UART::Data rx{vision_comm.GetRxBuffer(), BSP::Vision::VisionCommunicator::RX_BUFFER_SIZE};
        uart3.receive_dma_idle(rx);
        __HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT);
    }
    else if (huart == uart8.get_handle())
    {
        volatile uint32_t sr = huart->Instance->SR;
        volatile uint32_t dr = huart->Instance->DR;
        (void)sr;
        (void)dr;

        __HAL_UART_DISABLE_IT(huart, UART_IT_RXNE);
        __HAL_UART_CLEAR_OREFLAG(huart);
        __HAL_UART_CLEAR_NEFLAG(huart);
        __HAL_UART_CLEAR_FEFLAG(huart);
        __HAL_UART_CLEAR_PEFLAG(huart);

        huart->RxState = HAL_UART_STATE_READY;
        huart->Lock = HAL_UNLOCKED;

        HAL::UART::Data rx{imu_dma_buffer, IMU_DMA_BUF_SIZE};
        uart8.receive_dma_idle(rx);
        __HAL_DMA_DISABLE_IT(&hdma_uart8_rx, DMA_IT_HT);
    }
}

//虚拟串口假回调 (见USB_DEVICE\App\usbd_cdc_if.c)
extern "C" void USB_Receive_Callback(uint8_t *Buf, uint32_t Len)
{
    // 帧长固定 19 字节，不对就丢掉
    if (Len != BSP::Vision::VisionCommunicator::RX_BUFFER_SIZE)
        {return;}

    memset(vision_comm.GetRxBuffer(), 0, BSP::Vision::VisionCommunicator::RX_BUFFER_SIZE);
    memcpy(vision_comm.GetRxBuffer(), Buf, BSP::Vision::VisionCommunicator::RX_BUFFER_SIZE);
    vision_comm.ParseRxData(vision_comm.GetRxBuffer(), BSP::Vision::VisionCommunicator::RX_BUFFER_SIZE);
}

// ==================== 任务函数 ====================

extern "C" void remote_control_task(void *argument)
{
    remoteDataQueue = xQueueCreate(1, sizeof(RemoteData_t));

    // 初始化 UART 总线（懒加载单例，触发所有设备 init + start）
    HAL::UART::get_uart_bus_instance();

    auto &uart1 = HAL::UART::get_uart_bus_instance().get_uart1();
    auto &uart3 = HAL::UART::get_uart_bus_instance().get_uart3();
    auto &uart8 = HAL::UART::get_uart_bus_instance().get_uart8();

    // USART1 回调：DMA+空闲 → 遥控器解析
    uart1.register_rx_callback([](const HAL::UART::Data &data) {
        if (data.size == 18) {
            remoteController.parseData(data.buffer);
            remoteController.updateTimestamp();
        }
    });

    // USART3 回调：DMA+空闲 → 视觉数据解析（保留兼容）
    uart3.register_rx_callback([](const HAL::UART::Data &data) {
        vision_comm.ParseRxData(data.buffer, data.size);
    });

    // UART8 回调：DMA+空闲 → 整帧喂给 HI12 IMU
    uart8.register_rx_callback([](const HAL::UART::Data &data) {
        imu.DataUpdate(data.buffer);
    });

    // 启动接收
    HAL::UART::Data uart1_rx{receivedata, sizeof(receivedata)};
    uart1.receive_dma_idle(uart1_rx);
    __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);

    HAL::UART::Data uart3_rx{vision_comm.GetRxBuffer(), BSP::Vision::VisionCommunicator::RX_BUFFER_SIZE};
    uart3.receive_dma_idle(uart3_rx);
    __HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT);

    HAL::UART::Data uart8_rx{imu_dma_buffer, IMU_DMA_BUF_SIZE};
    uart8.receive_dma_idle(uart8_rx);
    __HAL_DMA_DISABLE_IT(&hdma_uart8_rx, DMA_IT_HT);

    remoteController.SetDeadzone(0.1f);

    static uint32_t vision_tick = 0;

    for (;;)
    {
        if (!remoteController.isConnected())
        {
            remote.chassis_vx = 0.0f;
            remote.chassis_vy = 0.0f;
            remote.gimbal_yaw = 0.0f;
            remote.gimbal_pitch = 0.0f;
            remote.s1 = remote.s2 = 0;
            xQueueOverwrite(remoteDataQueue, &remote);
        }

        if (imu.isConnected())
        {
            // 视觉模式：S1 中 + S2 中
            uint8_t vision_mode = (remoteController.get_s1() == BSP::REMOTE_CONTROL::RemoteController::MIDDLE &&
                                   remoteController.get_s2() == BSP::REMOTE_CONTROL::RemoteController::MIDDLE) ? 1 : 0;

            float quaternion[4] = {imu.GetQuaternion(0), imu.GetQuaternion(1), imu.GetQuaternion(2), imu.GetQuaternion(3)};

            // 弹丸预估初速度：取2个摩擦轮转速的绝对值平均值，换算为线速度
            // v (m/s) = RPM × π × d / 60,  d = 64.1mm
            // 右侧摩擦轮反向旋转，取绝对值后参与平均
            float abs_left  = friction_current_speed_left  < 0.0f ? -friction_current_speed_left  : friction_current_speed_left;
            float abs_right = friction_current_speed_right < 0.0f ? -friction_current_speed_right : friction_current_speed_right;
            float avg_friction_rpm = (abs_left + abs_right) / 2.0f;
            float bullet_speed = avg_friction_rpm * 3.14159265358979323846f * 0.0641f / 60.0f;

            vision_comm.SendToVision(quaternion, bullet_speed, BSP::Vision::VisionCommunicator::ENEMY_RED, vision_mode);
        }
       

        vTaskDelay(2);
    }
}