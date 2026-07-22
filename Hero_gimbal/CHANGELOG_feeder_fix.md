# 拨弹轮 FSM 连发模式修复记录

**日期**: 2026-07-22
**工程**: Hero_gimbal
**分支**: main

---

## 修复 1：单发切连发时先反转再正转

### 现象

遥控器单发模式下打几发弹丸后，切换拨杆到连发模式（S1=UP, S2=UP），连发模式会先反转再正转。

### 根因

**文件**: `RtosTask/gimbal_task.cpp`

全局变量 `feeder_target_angle`（初始值 0.0f）在连发模式的速度控制路径中用作位置环目标值，每帧递减以追踪发射进度。但该变量在进入连发模式时**未与 FSM 的累积角度同步**。

- 单发模式每发 `accumulated_angle` 向负方向累积约 3060°（`SINGLE_SHOT_ANGLE = -3060`）
- 打 4 发后 `accumulated_angle ≈ -12240°`
- 切换到连发模式时，`feeder_target_angle ≈ 0`，`accumulated_angle ≈ -12240`
- 位置环误差 = 0 - (-12240) = **+12240°** → PID 输出正向速度 → 电机反转
- 随着 `feeder_target_angle` 持续递减追上 `accumulated_angle`，电机才转为正转

### 修改

**文件**: `RtosTask/gimbal_task.cpp`

在 `else if (Get_Control_Type() == FEEDER_CONTROL_SPEED)` 分支中，添加进入连发模式时的角度同步逻辑。

修改前（关键代码）：
```cpp
else if (feeder_fsm.Get_Control_Type() == FEEDER_CONTROL_SPEED)
{
    feeder_target_angle -= hz_to_rotor_angle_per_frame(3.0f);
    // ...
}
```

修改后：
```cpp
// 在 if-else 链之前添加控制类型追踪（约第 129-131 行）
// 跟踪控制类型变化，用于检测从其他模式切换到速度模式 (SPEED) 的时刻
static uint8_t prev_control_type = FEEDER_CONTROL_STOP;
uint8_t current_control_type = feeder_fsm.Get_Control_Type();

// SPEED 路径中添加同步（约第 151-168 行）
else if (current_control_type == FEEDER_CONTROL_SPEED)
{
    // 刚切换到速度模式时，将目标角度同步为当前累积角度
    if (prev_control_type != FEEDER_CONTROL_SPEED)
    {
        feeder_target_angle = feeder_fsm.Get_Accumulated_Angle();
    }

    feeder_target_angle -= hz_to_rotor_angle_per_frame(3.0f);
    // ...
}

// 在 if-else 链之后更新追踪变量（约第 178 行）
prev_control_type = current_control_type;
```

**要点**：
1. `prev_control_type` 必须在 if-else 链**之外**声明和更新，确保离开 SPEED 模式后能正确记录变化
2. 使用 `static` 保证变量跨帧持久化
3. 所有 `feeder_fsm.Get_Control_Type()` 调用替换为一次性捕获的 `current_control_type`

---

## 修复 2：连发射速计算的控制周期错误

### 根因

**文件**: `RtosTask/gimbal_task.cpp`，函数 `hz_to_rotor_angle_per_frame`

`control_period` 写死为 `0.002f`（2ms），但实际控制循环周期是 `vTaskDelay(5)` = **5ms**。导致每帧角度递减量只有正确值的 40%，实际射速远低于设定值。

### 修改

```cpp
// 修改前
const float control_period = 0.002f;

// 修改后
const float control_period = 0.005f;  // 与 vTaskDelay(5) 一致
```

### 注意

修正后连发模式每帧目标角度递减量增大约 2.5 倍，如果另一个工程的 PID 参数是针对旧速率调试的，切换到连发模式后可能需要重新调整 `feeder_angle_pid_speed` 和 `feeder_speed_pid_speed` 的 P/I/D 参数。

---

## 涉及文件清单

| 文件 | 修改内容 |
|------|---------|
| `RtosTask/gimbal_task.cpp` 第 129-178 行 | 添加控制类型切换检测 + 进入 SPEED 模式时同步 `feeder_target_angle` |
| `RtosTask/gimbal_task.cpp` 第 231 行 | `control_period` 从 `0.002f` 改为 `0.005f` |

其他文件（`feeder_fsm/feeder_fsm.cpp`、`feeder_fsm/feeder_fsm.hpp`）**未修改**，无需同步。
