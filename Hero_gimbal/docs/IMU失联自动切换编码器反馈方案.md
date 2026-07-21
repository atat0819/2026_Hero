# IMU 失联自动切换编码器反馈方案

## 防护目标

IMU 断线或故障时（数据全零 / 从未就绪），云台 yaw/pitch 两轴自动从 IMU 反馈切换到电机编码器反馈，切换过程不疯车。

---

## 一、FSM 加 ReAnchor 方法

**文件**：`gimbal_fsm.hpp` / `gimbal_fsm.cpp`

### 声明

```cpp
// gimbal_fsm.hpp - Class_Gimbal_FSM 类中添加
/// @brief 反馈源切换时调用，锚定目标角度防止疯车
/// @param new_angle 新的当前角度（编码器角度或 IMU 恢复后的值）
void ReAnchor(float new_angle);
```

### 实现

```cpp
// gimbal_fsm.cpp
void Class_Gimbal_FSM::ReAnchor(float new_angle)
{
    target_angle = Apply_Angle_Rule(new_angle);  // 归一化 + 限幅
    control_output = target_angle;
    target_speed = 0.0f;
    angle_target_initialized = 1U;
    mode_changed_flag = 1U;  // 触发 Take_Mode_Changed_Flag() → 外层复位 PID
}
```

**作用**：将 FSM 的目标角度强制锚定到当前实际角度，告诉 PID "你就在这里，不需要动"。`mode_changed_flag = 1U` 令下一帧 `Take_Mode_Changed_Flag()` 返回 1，外层代码即可执行 `pid.reset()`。

---

## 二、IMU 故障检测 + 编码器降级函数

**文件**：`can_send_task.cpp`

放在 `can_send_task` 函数定义之前。

### 函数代码

```cpp
// ==================== IMU 故障检测与编码器降级 ====================
// 检测条件：数据全零 或 IMU 从未就绪 → 自动切到电机编码器反馈
// 故障翻转时调用 FSM::ReAnchor + PID 复位，防止疯车
static void IMU_Fault_Protection(float &yaw_angle, float &yaw_speed,
                                  float &pitch_angle, float &pitch_speed)
{
    static uint16_t imu_fault_counter = 0;
    static uint8_t  imu_fault = 0;
    static uint8_t  last_imu_fault = 0;
    static float    yaw_enc_offset = 0.0f;
    static float    pitch_enc_offset = 0.0f;
    static constexpr float RPM_TO_DEGPS = 6.0f; // RPM × 360 / 60

    // ---- 检测条件 ----
    bool imu_all_zero = (ImuFloat.yaw == 0.0f && ImuFloat.pitch == 0.0f &&
                         ImuFloat.gyr_yaw == 0.0f && ImuFloat.gyr_pitch == 0.0f);
    bool imu_lost = (ImuDataReady == 0);

    if (imu_all_zero || imu_lost)
        imu_fault_counter++;
    else
        imu_fault_counter = 0;

    imu_fault = (imu_fault_counter > 500) ? 1 : 0; // 500ms 防抖

    // ---- 故障翻转：算偏移 → 重锚 → 复位 PID ----
    if (imu_fault != last_imu_fault)
    {
        if (imu_fault)
        {
            // IMU → 编码器：记录偏移，使编码器与最后 IMU 值连续
            yaw_enc_offset   = mg4005_state[1].delta_angle - ImuData_user.yaw;
            pitch_enc_offset = mg4005_state[0].delta_angle - ImuData_user.pitch;
        }
        // 故障进入和恢复都走重锚
        float new_yaw   = imu_fault ? (mg4005_state[1].delta_angle - yaw_enc_offset) : ImuData_user.yaw;
        float new_pitch = imu_fault ? (mg4005_state[0].delta_angle - pitch_enc_offset) : ImuData_user.pitch;
        yaw_gimbal_fsm.ReAnchor(new_yaw);
        pitch_gimbal_fsm.ReAnchor(new_pitch);

        // 复位全部 PID（清空基于旧反馈源的历史误差）
        yaw_angle_pid.reset();
        yaw_angle_to_speed_pid.reset();
        yaw_single_speed_pid.reset();
        yaw_version_angle_pid.reset();
        yaw_version_speed_pid.reset();
        pitch_angle_pid.reset();
        pitch_angle_to_speed_pid.reset();
        pitch_single_speed_pid.reset();
        pitch_version_angle_pid.reset();
        pitch_version_speed_pid.reset();

        last_imu_fault = imu_fault;
    }

    // ---- 每帧选择反馈源 ----
    if (imu_fault)
    {
        yaw_angle   = mg4005_state[1].delta_angle - yaw_enc_offset;
        pitch_angle = mg4005_state[0].delta_angle - pitch_enc_offset;
        yaw_speed   = mg4005_state[1].velocity_rpm * RPM_TO_DEGPS;
        pitch_speed = mg4005_state[0].velocity_rpm * RPM_TO_DEGPS;
    }
    else
    {
        yaw_angle   = ImuData_user.yaw;
        pitch_angle = ImuData_user.pitch;
        yaw_speed   = ImuData_user.gyro_z;
        pitch_speed = ImuData_user.gyro_y;
    }
}
```

---

## 三、主循环调用

**文件**：`can_send_task.cpp` — `can_send_task` 函数主循环内

```cpp
for (;;)
{
    // ... 遥控器数据处理 ...

    ControlTask();  // 读取电机编码器

    IMU_Fault_Protection(yaw_current_angle, yaw_current_speed,
                         pitch_current_angle, pitch_current_speed);

    // ---- 之后照常：模式判断 → FSM Update → PID 计算 → 电机输出 ----

    // 判断模式
    if (视觉模式条件) {
        yaw_mode   = GIMBAL_MODE_VISION;
        pitch_mode = GIMBAL_MODE_VISION;
    } else if (...) { ... }

    yaw_gimbal_fsm.Update(yaw_mode, yaw_current_angle, ...);
    pitch_gimbal_fsm.Update(pitch_mode, pitch_current_angle, ...);

    if (yaw_gimbal_fsm.Take_Mode_Changed_Flag() != 0U) {
        // ReAnchor 触发的 mode_changed_flag 也会走到这里，复位 PID
        yaw_angle_pid.reset();
        // ...
    }

    // ... PID 计算，电机扭矩输出 ...

    vTaskDelay(1);
}
```

---

## 四、反馈源映射

| 信号 | IMU 正常 | IMU 故障（编码器降级） |
|---|---|---|
| Yaw 角度 (°) | `ImuData_user.yaw` | `mg4005_state[1].delta_angle - yaw_enc_offset` |
| Pitch 角度 (°) | `ImuData_user.pitch` | `mg4005_state[0].delta_angle - pitch_enc_offset` |
| Yaw 角速度 (°/s) | `ImuData_user.gyro_z` | `mg4005_state[1].velocity_rpm × 6.0` |
| Pitch 角速度 (°/s) | `ImuData_user.gyro_y` | `mg4005_state[0].velocity_rpm × 6.0` |

电机编号与轴对应：
- Motor 1 → Pitch（`mg4005_state[0]`）
- Motor 2 → Yaw（`mg4005_state[1]`）

---

## 五、核心原理

### 5.1 为什么需要偏移（offset）

编码器和 IMU 的零点不同。切换瞬间如果直接拿编码器值，角度会跳变。

```
示例：
  切换前 IMU yaw  = 45°
  编码器多圈角度   = 720°（电机已转 2 圈）
  不做偏移就切：    feedback 从 45 → 720，误差 675° → 疯车

  算偏移：         offset = 720 - 45 = 675
  切换后 feedback  = 720 - 675 = 45°  ← 和 IMU 最后一帧一模一样，连续
```

### 5.2 时序

```
Tick 1..500:   IMU 全零 → counter++ → 满 500 → imu_fault = 1
Tick 501:      fault ≠ last → 算 offset → ReAnchor → 复位 PID → 切编码器
Tick 502..N:   每帧走编码器分支，offset 固定不变
... IMU 恢复 ...
Tick M..M+500: IMU 正常 → counter 归零 → imu_fault = 0
Tick M+501:    fault ≠ last → ReAnchor(ImuData_user.yaw) → 复位 PID → 切回 IMU
Tick M+502..:  每帧走 IMU 分支
```

### 5.3 防抖

连续 500 周期（500ms @1kHz）异常才判定故障。UART 偶发丢一两个字节不会误触发切换。

---

## 六、其他工程接入清单

| 步骤 | 文件 | 内容 |
|---|---|---|
| 1 | `gimbal_fsm.hpp` | 加 `ReAnchor(float new_angle)` 声明 |
| 2 | `gimbal_fsm.cpp` | 实现 `ReAnchor`（锚定 target_angle + 置 mode_changed_flag） |
| 3 | `can_send_task.cpp` | 加 `IMU_Fault_Protection` 函数（检测 + 偏移 + 选源） |
| 4 | `can_send_task.cpp` | 主循环 `ControlTask()` 后调用 `IMU_Fault_Protection` |

### 需替换的变量名

| 本工程变量 | 替换为你的工程变量 |
|---|---|
| `ImuFloat` | 原始 IMU 数据结构（yaw/pitch/gyr_yaw/gyr_pitch 字段） |
| `ImuDataReady` | IMU 有效数据帧就绪标志 |
| `ImuData_user` | 转换后的 IMU 角度/角速度 (°/s) |
| `mg4005_state[i].delta_angle` | 电机多圈累积角度 (°) |
| `mg4005_state[i].velocity_rpm` | 电机输出轴转速 (RPM) |
| `yaw_gimbal_fsm` / `pitch_gimbal_fsm` | 云台 FSM 实例 |
| `yaw_*_pid` / `pitch_*_pid` | PID 实例列表（全部 reset） |

### 关键参数

| 参数 | 值 | 说明 |
|---|---|---|
| 防抖时间 | 500ms | 连续异常才切换 |
| RPM → °/s | `× 6.0` | RPM × 360° ÷ 60s |
