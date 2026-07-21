#include "remote_task.hpp"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "cmsis_os.h"
#include "usart.h"
#include <string.h>
#include "BSP/IMU/HI12_imu.hpp"
#include "../user/core/HAL/UART/uart_hal.hpp"
#include "../user/core/APP/Referee/RM_RefereeSystem.h"

// ================= 全局变量定义 =================
// 1. IMU 相关
BSP::IMU::HI12_float imu;    // 创建 IMU 对象
uint8_t power_rx_buffer[12];   // 功率计接收缓冲区
QueueHandle_t IMUDataQueue;  // 用于传递 IMU 数据的队列
IMUData_t imuData;           // IMU 数据结构体
PowerData_t PowerData;

// 2. 遥控器相关
uint8_t receivedata[18];     // 遥控器接收缓冲区
QueueHandle_t remoteDataQueue; // 用于传递遥控器数据的队列
RemoteData_t remoteData;     // 遥控器解析后的数据结构体
float a = 0;

BSP::REMOTE_CONTROL::RemoteController remoteController(100);
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_uart8_rx;
extern DMA_HandleTypeDef hdma_usart6_rx;

// 3. 裁判系统相关
uint8_t referee_buffer[512];    // 裁判系统 DMA 接收缓冲区

// ================= UART 库静态缓冲区描述符 =================
// 对标 CAN 的 Frame 静态分配模式
static HAL::UART::Data uart1_rx_data;   // 遥控器 DMA+空闲接收
static HAL::UART::Data uart8_rx_data;   // 功率计 DMA+空闲接收
HAL::UART::Data uart6_rx_data;          // 裁判系统 DMA+空闲接收（非static，RM_RefereeSystemInit 需引用）

// ================= 中断回调函数（只做路由，不写业务逻辑） =================

/**
 * @brief UART DMA 接收完成/空闲中断回调函数
 * @note  1. 先重启 DMA（减少死区）再处理数据
 *        2. huart == get_handle() 防止设备查找失败时操作到错误的 UART
 *        3. 局部 Data 避免跨中断静态变量污染
 */
extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART1)
    {
        HAL::UART::Data rx_data{receivedata, sizeof(receivedata)};
        auto &uart1 = HAL::UART::get_uart_bus_instance().get_uart1();
        if (huart == uart1.get_handle())
        {
            uart1.receive_dma_idle(rx_data);
            __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
            rx_data.size = Size;  // 实际接收字节数
            uart1.trigger_rx_callbacks(rx_data);
        }
    }
    else if (huart->Instance == UART8)
    {
        HAL::UART::Data rx_data{power_rx_buffer, sizeof(power_rx_buffer)};
        auto &uart8 = HAL::UART::get_uart_bus_instance().get_uart8();
        if (huart == uart8.get_handle())
        {
            uart8.receive_dma_idle(rx_data);
            __HAL_DMA_DISABLE_IT(&hdma_uart8_rx, DMA_IT_HT);
            rx_data.size = Size;
            uart8.trigger_rx_callbacks(rx_data);
        }
    }
    else if (huart->Instance == USART6)
    {
        HAL::UART::Data rx_data{referee_buffer, sizeof(referee_buffer)};
        auto &uart6 = HAL::UART::get_uart_bus_instance().get_uart6();
        if (huart == uart6.get_handle())
        {
            uart6.receive_dma_idle(rx_data);
            __HAL_DMA_DISABLE_IT(&hdma_usart6_rx, DMA_IT_HT);
            rx_data.size = Size;
            uart6.trigger_rx_callbacks(rx_data);
        }
    }
}

/**
 * @brief UART 错误回调
 * @note  完整错误恢复流程：清标志 → 解锁 HAL 状态机 → 重启 DMA 接收
 */
extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1) // 遥控器
    {
        // 1. 读 SR/DR 清空残留字节状态
        volatile uint32_t sr = huart->Instance->SR;
        volatile uint32_t dr = huart->Instance->DR;
        (void)sr;
        (void)dr;

        // 2. 关闭 RXNE 中断，清除全部错误标志
        __HAL_UART_DISABLE_IT(huart, UART_IT_RXNE);
        __HAL_UART_CLEAR_OREFLAG(huart);
        __HAL_UART_CLEAR_NEFLAG(huart);
        __HAL_UART_CLEAR_FEFLAG(huart);
        __HAL_UART_CLEAR_PEFLAG(huart);

        // 3. 强制解锁 HAL 状态机（解决 HAL_BUSY 导致 DMA 无法重启的关键）
        huart->RxState = HAL_UART_STATE_READY;
        huart->Lock = HAL_UNLOCKED;

        // 4. 重启 DMA+空闲接收
        HAL::UART::get_uart_bus_instance().get_uart1().receive_dma_idle(uart1_rx_data);
    }
    else if (huart->Instance == UART8) // 功率计
    {
        // 1. 读 SR/DR 清空残留字节状态
        volatile uint32_t sr = huart->Instance->SR;
        volatile uint32_t dr = huart->Instance->DR;
        (void)sr;
        (void)dr;

        // 2. 关闭 RXNE 中断，清除全部错误标志
        __HAL_UART_DISABLE_IT(huart, UART_IT_RXNE);
        __HAL_UART_CLEAR_OREFLAG(huart);
        __HAL_UART_CLEAR_NEFLAG(huart);
        __HAL_UART_CLEAR_FEFLAG(huart);
        __HAL_UART_CLEAR_PEFLAG(huart);

        // 3. 强制解锁 HAL 状态机
        huart->RxState = HAL_UART_STATE_READY;
        huart->Lock = HAL_UNLOCKED;

        // 4. 重启 DMA+空闲接收
        HAL::UART::get_uart_bus_instance().get_uart8().receive_dma_idle(uart8_rx_data);
    }
    else if (huart->Instance == USART6) // 裁判系统
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

        HAL::UART::get_uart_bus_instance().get_uart6().receive_dma_idle(uart6_rx_data);
    }
}

// ================= FreeRTOS 任务 =================

/**
 * @brief 遥控器与传感器监控任务
 */
extern "C" void remote_task(void *argument)
{
    // ---- 1. 初始化 UART 总线（懒加载单例） ----
    HAL::UART::get_uart_bus_instance();

    // ---- 2. 获取设备引用（对标 CAN 的 get_can1() / get_can2()） ----
    auto &uart1 = HAL::UART::get_uart_bus_instance().get_uart1();
    auto &uart8 = HAL::UART::get_uart_bus_instance().get_uart8();

    // ---- 3. 初始化参数 ----
    remoteController.SetDeadzone(0.0f);

    // ---- 4. UART1 遥控器：注册回调 + 启动 DMA 空闲接收 ----
    uart1_rx_data.buffer = receivedata;
    uart1_rx_data.size   = sizeof(receivedata);
    uart1.register_rx_callback([](const HAL::UART::Data &data) {
        if (data.size == 18)
        {
            remoteController.parseData(data.buffer);
            remoteController.updateTimestamp();
            remoteData.vx = remoteController.DeadzoneCompensation(remoteController.get_left_x());
            remoteData.vy = remoteController.DeadzoneCompensation(remoteController.get_left_y());
            remoteData.wz = remoteController.DeadzoneCompensation(remoteController.get_right_x());
            remoteData.s1 = remoteController.get_s1();
            remoteData.s2 = remoteController.get_s2();
        }
    });
    uart1.receive_dma_idle(uart1_rx_data);
    __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);

    // ---- 5. UART8 功率计：注册回调 + 启动 DMA 空闲接收 ----
    uart8_rx_data.buffer = power_rx_buffer;
    uart8_rx_data.size   = sizeof(power_rx_buffer);
    uart8.register_rx_callback([](const HAL::UART::Data &data) {
        if (data.size == 12)
        {
            memcpy(&PowerData, data.buffer, 12);
        }
    });
    uart8.receive_dma_idle(uart8_rx_data);
    __HAL_DMA_DISABLE_IT(&hdma_uart8_rx, DMA_IT_HT);

    // ---- 6. UART6 裁判系统：注册回调 + 启动 DMA 空闲接收 ----
    auto &uart6 = HAL::UART::get_uart_bus_instance().get_uart6();
    uart6_rx_data.buffer = referee_buffer;
    uart6_rx_data.size   = sizeof(referee_buffer);
    uart6.register_rx_callback([](const HAL::UART::Data &data) {
        if (data.size > 0 && data.buffer != nullptr)
        {
            RM_RefereeSystem::RM_RefereeSystemParse(data.buffer, data.size);
        }
    });
    uart6.receive_dma_idle(uart6_rx_data);
    __HAL_DMA_DISABLE_IT(&hdma_usart6_rx, DMA_IT_HT);

    // ---- 7. 主循环 ----
    for (;;)
    {
        // 遥控器断连检测（可在此处扩展）
        osDelay(1);
    }
}
