# UART 中断无法触发 - 深度诊断

## 🔴 发现的关键问题

### 问题 1：队列在 main.c 中创建，但在 FreeRTOS 启动之前

**代码位置：** `Core/Src/main.c`

```c
/* USER CODE BEGIN 2 */
// 1. 创建长度为 1 的队列（用于 Overwrite 机制）
remoteDataQueue = xQueueCreate(1, sizeof(RemoteData_t));  // ❌ 在 FreeRTOS 启动前创建
IMUDataQueue = xQueueCreate(1, sizeof(IMUData_t));

// ...

// 3. 启动 UART DMA (此时队列已存在，中断安全了)
HAL_UARTEx_ReceiveToIdle_DMA(&huart1, receivedata, 18);  // ❌ 在 FreeRTOS 启动前启动
HAL_UARTEx_ReceiveToIdle_DMA(&huart8, imu_rx_buffer, 64);

/* USER CODE END 2 */

/* Init scheduler */
osKernelInitialize();  // FreeRTOS 初始化
MX_FREERTOS_Init();    // 创建任务

/* Start scheduler */
osKernelStart();       // ⚠️ FreeRTOS 启动
```

**问题分析：**
1. 队列在 `osKernelStart()` 之前创建 - 这是**不安全**的
2. UART DMA 在 FreeRTOS 启动前就开始接收
3. 如果此时触发中断，调用 `xQueueOverwriteFromISR()` 会失败（FreeRTOS 还没运行）

### 问题 2：remote_task 中重复启动 DMA

**代码位置：** `RtosTask/remote_task.cpp`

```cpp
extern "C" void remote_task(void *argument)
{
    // 1. 创建长度为 1 的队列（用于 Overwrite 机制）
    remoteDataQueue = xQueueCreate(1, sizeof(RemoteData_t));  // ❌ 重复创建！
    IMUDataQueue = xQueueCreate(1, sizeof(IMUData_t));
    
    // ...

    // 3. 启动 DMA 接收
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, receivedata, 18);  // ❌ 重复启动！
    HAL_UARTEx_ReceiveToIdle_DMA(&huart8, imu_rx_buffer, 64);
```

**问题分析：**
- 队列在 main.c 和 remote_task 中都创建了
- DMA 在 main.c 和 remote_task 中都启动了
- 这会导致冲突和内存泄漏

### 问题 3：中断优先级已修复，但可能还有其他问题

已修复的内容：
- ✅ 所有中断优先级从 5 改为 6
- ✅ CAN 过滤器已配置
- ✅ CAN 中断已开启

---

## ✅ 修复方案

### 方案 1：移除 main.c 中的队列创建和 DMA 启动（推荐）

**修改 `Core/Src/main.c`：**

```c
/* USER CODE BEGIN 2 */
// ❌ 删除这些代码
// remoteDataQueue = xQueueCreate(1, sizeof(RemoteData_t));
// IMUDataQueue = xQueueCreate(1, sizeof(IMUData_t));
// motorspeedtargetQueue = xQueueCreate(10,sizeof(MotorSpeedTarget_t));
// motorCurrentDataQueue = xQueueCreate(1,sizeof(MotorCurrentData_t) * 4);
// chassisCurrentDataQueue = xQueueCreate(10,sizeof(chassisCurrentData_t));

User_CAN_Start(); // ✅ 保留 CAN 启动

// ❌ 删除这些代码
// HAL_UARTEx_ReceiveToIdle_DMA(&huart1, receivedata, 18);
// HAL_UARTEx_ReceiveToIdle_DMA(&huart8, imu_rx_buffer, 64);

/* USER CODE END 2 */
```

**保留 `RtosTask/remote_task.cpp` 中的代码：**

```cpp
extern "C" void remote_task(void *argument)
{
    // ✅ 在任务中创建队列（FreeRTOS 已启动）
    remoteDataQueue = xQueueCreate(1, sizeof(RemoteData_t));
    IMUDataQueue = xQueueCreate(1, sizeof(IMUData_t));
    
    // ✅ 在任务中启动 DMA
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, receivedata, 18);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart8, imu_rx_buffer, 64);
    
    // ...
}
```

### 方案 2：在 main.c 中创建队列，但在任务中启动 DMA

**修改 `Core/Src/main.c`：**

```c
/* USER CODE BEGIN 2 */
// ✅ 在 FreeRTOS 启动前创建队列（可以接受）
remoteDataQueue = xQueueCreate(1, sizeof(RemoteData_t));
IMUDataQueue = xQueueCreate(1, sizeof(IMUData_t));
motorspeedtargetQueue = xQueueCreate(10,sizeof(MotorSpeedTarget_t));
motorCurrentDataQueue = xQueueCreate(1,sizeof(MotorCurrentData_t) * 4);
chassisCurrentDataQueue = xQueueCreate(10,sizeof(chassisCurrentData_t));

User_CAN_Start();

// ❌ 删除 DMA 启动代码
// HAL_UARTEx_ReceiveToIdle_DMA(&huart1, receivedata, 18);
// HAL_UARTEx_ReceiveToIdle_DMA(&huart8, imu_rx_buffer, 64);

/* USER CODE END 2 */
```

**修改 `RtosTask/remote_task.cpp`：**

```cpp
extern "C" void remote_task(void *argument)
{
    // ❌ 删除队列创建（已在 main.c 中创建）
    // remoteDataQueue = xQueueCreate(1, sizeof(RemoteData_t));
    // IMUDataQueue = xQueueCreate(1, sizeof(IMUData_t));
    
    // 2. 初始化参数
    remoteController.SetDeadzone(50.0f);

    // 3. ✅ 启动 DMA 接收（在 FreeRTOS 任务中）
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, receivedata, 18);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart8, imu_rx_buffer, 64);
    
    // ...
}
```

---

## 🔍 诊断步骤

### 步骤 1：验证中断是否触发

在 `Core/Src/stm32f4xx_it.c` 中添加调试代码：

```c
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */
  static uint32_t irq_count = 0;
  irq_count++;  // 在调试器中观察这个变量
  /* USER CODE END USART1_IRQn 0 */
  
  HAL_UART_IRQHandler(&huart1);
  
  /* USER CODE BEGIN USART1_IRQn 1 */
  /* USER CODE END USART1_IRQn 1 */
}
```

### 步骤 2：验证 DMA 是否工作

在 `remote_task.cpp` 中添加：

```cpp
extern "C" void remote_task(void *argument)
{
    // ...
    
    // 启动 DMA 后检查返回值
    HAL_StatusTypeDef status1 = HAL_UARTEx_ReceiveToIdle_DMA(&huart1, receivedata, 18);
    HAL_StatusTypeDef status2 = HAL_UARTEx_ReceiveToIdle_DMA(&huart8, imu_rx_buffer, 64);
    
    // 在调试器中查看 status1 和 status2 的值
    // HAL_OK = 0x00
    // HAL_ERROR = 0x01
    // HAL_BUSY = 0x02
    
    for (;;)
    {
        // ...
    }
}
```

### 步骤 3：验证回调函数是否被调用

在 `remote_task.cpp` 中添加：

```cpp
extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    static uint32_t callback_count = 0;
    callback_count++;  // 在调试器中观察
    
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (huart->Instance == USART1) 
    {
        static uint32_t uart1_count = 0;
        uart1_count++;  // 在调试器中观察
        
        // ...
    }
    // ...
}
```

### 步骤 4：检查 UART 配置

在调试器中检查：
- `huart1.Init.Mode` 应该包含 `UART_MODE_RX`
- `huart1.hdmarx` 不应该是 NULL
- `huart1.State` 应该是 `HAL_UART_STATE_READY` 或 `HAL_UART_STATE_BUSY_RX`

---

## 📝 检查清单

- [ ] 移除 main.c 中的队列创建或 remote_task 中的队列创建（二选一）
- [ ] 移除 main.c 中的 DMA 启动代码
- [ ] 确保 DMA 只在 FreeRTOS 任务中启动
- [ ] 验证中断优先级为 6
- [ ] 检查硬件连接（遥控器接收器连接到 PB6/PB7）
- [ ] 检查遥控器是否开机并发送数据
- [ ] 使用逻辑分析仪或示波器检查 RX 引脚是否有信号

---

## 🎯 预期结果

修复后：
- ✅ 中断计数器 `irq_count` 应该持续增加
- ✅ 回调计数器 `callback_count` 应该持续增加
- ✅ UART1 计数器 `uart1_count` 应该持续增加
- ✅ 队列中应该能收到遥控器数据

---

## 💡 额外建议

### 使用 LED 指示中断状态

```cpp
extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART1) 
    {
        // 每次接收到数据，翻转 LED
        HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
        
        // ...
    }
}
```

### 添加超时保护

```cpp
extern "C" void remote_task(void *argument)
{
    // ...
    
    uint32_t last_receive_time = 0;
    
    for (;;)
    {
        if (remoteController.isConnected())
        {
            last_receive_time = HAL_GetTick();
        }
        else
        {
            // 超过 500ms 没收到数据
            if (HAL_GetTick() - last_receive_time > 500)
            {
                // 清零数据，防止失控
                remoteData.vx = 0.0f;
                remoteData.vy = 0.0f;
                remoteData.wz = 0.0f;
            }
        }
        
        osDelay(10);
    }
}
```
