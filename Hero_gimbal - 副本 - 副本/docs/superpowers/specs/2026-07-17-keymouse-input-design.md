# 键鼠操作功能设计文档

> 日期: 2026-07-17 | 状态: 待审核

## 1. 概述

在现有 DT7 遥控器基础上，增加键鼠操作模式。DT7 遥控器的 18 字节协议已同时包含摇杆/开关/键盘/鼠标数据，无需额外硬件。两种输入设备（遥控器、键鼠）在不同 S1/S2 挡位下工作。

## 2. 挡位映射

| S1 | S2 | 输入源 | 云台模式 | 发射机构 |
|----|----|--------|----------|----------|
| ↓ | ↓ | 遥控器 | STOP | 全部停止 |
| ↓ | 中 | 遥控器 | SPEED | 单发 + 摩擦轮 |
| 中 | 中 | 遥控器 | VISION | 单发 + 视觉开火 |
| 中 | 下 | 遥控器 | STOP | 全部停止 |
| **↓** | **↑** | **键鼠** | **速度/视觉** (右键切换) | **R/T 键控制** |
| 其他 | 任意 | 遥控器 | ANGLE | 单发 + 摩擦轮 |

> 唯一新增挡位：**S1↓ + S2↑ = 键鼠模式**。其余挡位行为不变。

## 3. 键鼠模式内部逻辑

### 3.1 云台控制

```
鼠标移动 → 速度环 PID（鼠标 X→yaw 转速, 鼠标 Y→pitch 转速）
鼠标右键按住超过 2 秒 → 视觉模式（自动跟踪视觉目标）
鼠标右键松开 → 立即回到速度环 PID
```

- 视觉模式复用现有 `vision_comm` 数据通道
- 鼠标灵敏度通过 `MOUSE_YAW_GAIN` / `MOUSE_PITCH_GAIN` 可调参数控制
- 2 秒延时确保左右键同时按下打弹时不会误触发视觉模式

### 3.2 发射机构

```
R 键（翻转开关，带消抖）：
  初始: OFF（摩擦轮关闭，拨弹轮关闭）
  按1次: ON（摩擦轮开启 + 拨弹轮默认单发）
  按2次: OFF
  按3次: ON ...
  以此类推

T 键（翻转开关，仅 R=ON 时有效）：
  按1次: 单发 → 连发
  按2次: 连发 → 单发
  R=OFF 时 T 无效

鼠标左右键同时按下（发射触发）：
  单发模式: 左右键同时点下发射一发（边沿触发）
  连发模式: 左右键同时按住连续发射（电平触发）

重要：键鼠模式下忽略视觉开火指令，仅由操作手鼠标左右键同时按下才允许打弹
```

### 3.3 底盘

- 键盘位掩码通过 CAN2 发送给底盘开发板
- 底盘自行解析 WASD/Shift 等按键控制底盘运动
- CAN ID 使用 `0x305`，协议：2 字节键盘位掩码（小端）

## 4. 新增模块：InputDispatcher

### 4.1 文件位置

- `communication_between_boards/input_dispatcher.hpp`
- `communication_between_boards/input_dispatcher.cpp`

### 4.2 职责

- 根据 S1/S2 判断输入源（Remote / KeyMouse）
- 追踪 KeyMouse 下的 R/T 翻转状态（消抖 + 边沿检测）
- 追踪鼠标左右键状态
- 提供统一查询接口

### 4.3 核心 API

```cpp
enum class InputSource { Remote, KeyMouse };

class InputDispatcher {
public:
    void Update(const RemoteData_t& data);

    InputSource GetSource() const;
    bool IsFrictionOn() const;        // R键ON/OFF
    bool IsSingleShot() const;        // true=单发, false=连发
    bool IsRightButtonHeld() const;   // 右键消抖后原始状态
    bool IsVisionMode() const;         // 右键按住超过2秒→视觉模式，松开→立即退出
    bool IsLeftButtonPressed() const; // 左键消抖后按下
    bool IsFireTriggered() const;      // 左右键同时按下=发射触发
    uint16_t GetKeyboardMask() const; // 键盘位掩码（发给底盘）

private:
    // R/T 消抖 + 边沿检测
    // ...
};
```

### 4.4 消抖机制

计数器消抖，阈值 `DEBOUNCE_THRESHOLD = 30`（30ms @ 1kHz 控制周期）：

```
每个周期读取原始按键状态:
  if (当前 == 上一周期): stable_counter++
  else:                  stable_counter = 0
  if (stable_counter >= THRESHOLD): 确认状态 = 当前
```

确认状态后再做边沿翻转检测（仅按下沿触发翻转），避免按住时连续触发。

R、T 键和鼠标左右键均通过同一消抖机制处理，阈值统一使用 `DEBOUNCE_THRESHOLD`。

## 5. 现有文件改动

### 5.1 新增文件

| 文件 | 说明 |
|------|------|
| `communication_between_boards/input_dispatcher.hpp` | 接口定义 |
| `communication_between_boards/input_dispatcher.cpp` | 状态机 + 消抖实现 |

### 5.2 修改文件

| 文件 | 改动 |
|------|------|
| `RtosTask/can_send_task.cpp` | 引入 InputDispatcher；KeyMouse 分支：右键按住2s→视觉、松开→速度环、鼠标位移→速度环；调用发送键盘位掩码给底盘 |
| `RtosTask/gimbal_task.cpp` | 引入 InputDispatcher；S1↓S2↑→KeyMouse 分支；R 控制开关、T 控制单连发、左右键同时按下触发发射；键鼠模式下忽略视觉开火指令 |
| `communication_between_boards/boards_communication.hpp` | 新增 `CAN2_SendKeyboard(uint16_t keyboard)` 声明 |
| `communication_between_boards/boards_communication.cpp` | 新增 `CAN2_SendKeyboard()` 实现（CAN ID 0x305） |

### 5.3 废弃文件

| 文件 | 说明 |
|------|------|
| `communication_between_boards/keyboard_task.hpp` | 被 input_dispatcher 替代 |
| `communication_between_boards/keyboard_task.cpp` | 被 input_dispatcher 替代 |

## 6. 数据流

```
DT7 (USART1, 18 bytes)
  │
  ▼
remoteController.parseData()
  ├── 摇杆/开关 → get_left_x/y, get_right_x/y, get_s1/s2
  ├── 鼠标     → get_mouseX/Y/Z, get_mouseLeft/Right
  └── 键盘     → get_key(KEY_*)
  │
  ▼
InputDispatcher::Update()
  ├── 判断输入源 (Remote / KeyMouse)
  ├── R/T 消抖 + 边沿翻转
  └── 更新内部状态
  │
  ├──────────────────┬──────────────────┐
  ▼                  ▼                  ▼
can_send_task    gimbal_task        CAN2→底盘
(云台控制)       (发射机构)         (键盘位掩码)
```

## 7. 可调参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `DEBOUNCE_THRESHOLD` | 30 | 按键/鼠标消抖阈值 (ms) |
| `VISION_HOLD_THRESHOLD` | 2000 | 右键按住进入视觉模式延时 (ms) |
| `MOUSE_YAW_GAIN` | 0.01f | 鼠标 X → yaw 转速增益 |
| `MOUSE_PITCH_GAIN` | 0.01f | 鼠标 Y → pitch 转速增益 |

## 8. 边界情况

- **挡位切换**：从键鼠切回遥控器时，R/T/右键状态保持（由 Dispatcher 内部持续追踪）；发射机构挡位映射回遥控器逻辑
- **遥控器断联**：`remoteController.isConnected()==false` 时，强制回到 STOP，键鼠模式不可用
- **视觉断联**：键鼠视觉模式下 `vision_comm.IsDataFresh()==false`，自动回退到速度环
- **T 键在 R=OFF 时**：忽略，保持 R=OFF 状态（摩擦轮不开、不发射）
- **视觉开火在键鼠模式下**：忽略视觉的开火指令，仅由操作手鼠标左右键同时按下才允许发射
- **挡位切出键鼠模式时**：忽略所有键鼠专属按键（R/T/鼠标左右键），发射机构行为由遥控器挡位决定
