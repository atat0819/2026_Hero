# 键鼠操作功能 — 实现计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在现有 DT7 遥控器基础上增加键鼠操作模式，S1↓+S2↑ 进入键鼠模式，通过 InputDispatcher 统一管理键鼠状态（消抖/翻转/视觉延时）。

**Architecture:** 新增 `InputDispatcher` 模块管理键鼠状态机和消抖逻辑；修改 `can_send_task.cpp` 增加键鼠云台控制分支；修改 `gimbal_task.cpp` 增加键鼠发射机构控制分支；新增 CAN2 键盘位掩码发送给底盘。

**Tech Stack:** C++17, FreeRTOS, STM32F4 HAL, CAN bus

---

## 文件结构

| 文件 | 操作 | 职责 |
|------|------|------|
| `user/core/BSP/RemoteControl/DT7.hpp` | 修改 | 新增 `get_keyboard()` 公开 getter |
| `communication_between_boards/input_dispatcher.hpp` | 新建 | InputDispatcher 接口 |
| `communication_between_boards/input_dispatcher.cpp` | 新建 | 消抖 + 翻转 + 视觉延时状态机 |
| `communication_between_boards/boards_communication.hpp` | 修改 | 声明 `CAN2_SendKeyboard()` |
| `communication_between_boards/boards_communication.cpp` | 修改 | 实现 `CAN2_SendKeyboard()` |
| `RtosTask/can_send_task.cpp` | 修改 | KeyMouse 云台控制分支 |
| `RtosTask/gimbal_task.cpp` | 修改 | KeyMouse 发射机构控制分支 |
| `communication_between_boards/keyboard_task.hpp` | 删除 | 被 input_dispatcher 替代 |
| `communication_between_boards/keyboard_task.cpp` | 删除 | 被 input_dispatcher 替代 |

---

### Task 1: 在 DT7.hpp 增加 `get_keyboard()` 公开 getter

**Files:**
- Modify: `user/core/BSP/RemoteControl/DT7.hpp`

- [ ] **Step 1: 添加公开 getter**

在 `get_key(Keyboard key)` 方法下方添加一行：

```cpp
// 键盘数据
inline bool get_key(Keyboard key) const { return (keyboard_ & static_cast<uint16_t>(key)) != 0; }
inline uint16_t get_keyboard() const { return keyboard_; }  // [新增] 获取原始键盘位掩码
```

位置：`DT7.hpp` 第 141 行后。

- [ ] **Step 2: 确认编译通过**

此改动为纯头文件 inline getter，无需修改 .cpp。单独编译验证。

---

### Task 2: 创建 InputDispatcher 头文件

**Files:**
- Create: `communication_between_boards/input_dispatcher.hpp`

- [ ] **Step 1: 写入完整头文件**

```cpp
#ifndef INPUT_DISPATCHER_HPP
#define INPUT_DISPATCHER_HPP

#include <cstdint>

enum class InputSource : uint8_t {
    Remote   = 0,
    KeyMouse = 1,
};

class InputDispatcher {
public:
    /// @brief 每控制周期调用一次，更新键鼠状态机
    /// @param s1       遥控器 S1 开关值 (1=UP, 3=MIDDLE, 2=DOWN)
    /// @param s2       遥控器 S2 开关值
    /// @param keyboard  16位键盘位掩码
    /// @param mouse_left   鼠标左键原始状态
    /// @param mouse_right  鼠标右键原始状态
    void Update(uint8_t s1, uint8_t s2, uint16_t keyboard,
                bool mouse_left, bool mouse_right);

    // ---- 查询接口 ----

    InputSource GetSource() const { return source_; }

    /// R 键翻转状态：摩擦轮+拨弹轮开关
    bool IsFrictionOn() const { return r_toggle_on_; }

    /// T 键翻转状态：true=单发, false=连发
    bool IsSingleShot() const { return t_single_shot_; }

    /// 右键消抖后的原始状态
    bool IsRightButtonHeld() const { return right_button_confirmed_; }

    /// 右键按住超过 2 秒 → 视觉模式
    bool IsVisionMode() const { return vision_mode_; }

    /// 左键消抖后是否按下
    bool IsLeftButtonPressed() const { return left_button_confirmed_; }

    /// 左右键同时按下 → 发射触发
    bool IsFireTriggered() const {
        return IsRightButtonHeld() && IsLeftButtonPressed();
    }

    /// 键盘位掩码（通过 CAN2 发给底盘）
    uint16_t GetKeyboardMask() const { return keyboard_mask_; }

private:
    /// @brief 通用消抖（计数器法）
    /// @return 确认后的状态
    static bool Debounce(bool raw, bool& prev_raw, uint8_t& counter, bool& confirmed);

    /// @brief 翻转边沿检测（仅 0→1 按下沿触发）
    static bool DetectToggleEdge(bool confirmed, bool& prev_confirmed);

    InputSource source_ = InputSource::Remote;

    // R 键
    bool r_toggle_on_ = false;
    bool prev_confirmed_r_ = false;
    bool prev_raw_r_ = false;
    bool confirmed_r_ = false;
    uint8_t debounce_r_ = 0;

    // T 键
    bool t_single_shot_ = true;   // 默认单发
    bool prev_confirmed_t_ = false;
    bool prev_raw_t_ = false;
    bool confirmed_t_ = false;
    uint8_t debounce_t_ = 0;

    // 鼠标右键
    bool right_button_confirmed_ = false;
    bool prev_raw_right_ = false;
    uint8_t debounce_right_ = 0;
    uint16_t right_hold_counter_ = 0;   // 按住计时 (ms)
    bool vision_mode_ = false;

    // 鼠标左键
    bool left_button_confirmed_ = false;
    bool prev_raw_left_ = false;
    uint8_t debounce_left_ = 0;

    // 键盘位掩码
    uint16_t keyboard_mask_ = 0;

    static constexpr uint8_t  DEBOUNCE_THRESHOLD    = 30;    // 消抖 30ms
    static constexpr uint16_t VISION_HOLD_THRESHOLD  = 2000;  // 视觉延时 2000ms
};

#endif // INPUT_DISPATCHER_HPP
```

- [ ] **Step 2: 确认头文件语法正确**

---

### Task 3: 创建 InputDispatcher 实现文件

**Files:**
- Create: `communication_between_boards/input_dispatcher.cpp`

- [ ] **Step 1: 写入完整实现**

```cpp
#include "input_dispatcher.hpp"
#include "../user/core/BSP/RemoteControl/DT7.hpp"

using Remote = BSP::REMOTE_CONTROL::RemoteController;

void InputDispatcher::Update(uint8_t s1, uint8_t s2, uint16_t keyboard,
                             bool mouse_left, bool mouse_right)
{
    // ---- 1. 判断输入源 ----
    if (s1 == Remote::DOWN && s2 == Remote::UP) {
        source_ = InputSource::KeyMouse;
    } else {
        source_ = InputSource::Remote;
        return;  // 非键鼠模式，不更新键鼠内部状态
    }

    // ---- 2. 保存键盘位掩码 ----
    keyboard_mask_ = keyboard;

    // ---- 3. 读取原始按键/鼠标状态 ----
    bool raw_r     = (keyboard & static_cast<uint16_t>(Remote::KEY_R)) != 0;
    bool raw_t     = (keyboard & static_cast<uint16_t>(Remote::KEY_T)) != 0;
    bool raw_left  = mouse_left;
    bool raw_right = mouse_right;

    // ---- 4. 消抖 ----
    Debounce(raw_r,     prev_raw_r_,     debounce_r_,     confirmed_r_);
    Debounce(raw_t,     prev_raw_t_,     debounce_t_,     confirmed_t_);
    Debounce(raw_left,  prev_raw_left_,  debounce_left_,  left_button_confirmed_);
    Debounce(raw_right, prev_raw_right_, debounce_right_, right_button_confirmed_);

    // ---- 5. R 键翻转：按下沿触发 ON/OFF 切换 ----
    if (DetectToggleEdge(confirmed_r_, prev_confirmed_r_)) {
        r_toggle_on_ = !r_toggle_on_;
        if (r_toggle_on_) {
            t_single_shot_ = true;   // 开启时默认单发
        }
    }

    // ---- 6. T 键翻转：仅 R=ON 时有效 ----
    if (r_toggle_on_ && DetectToggleEdge(confirmed_t_, prev_confirmed_t_)) {
        t_single_shot_ = !t_single_shot_;
    }

    // ---- 7. 视觉模式判定：右键按住超过 VISION_HOLD_THRESHOLD ms ----
    if (right_button_confirmed_) {
        if (right_hold_counter_ < VISION_HOLD_THRESHOLD) {
            right_hold_counter_++;
        }
    } else {
        right_hold_counter_ = 0;
    }
    vision_mode_ = (right_hold_counter_ >= VISION_HOLD_THRESHOLD);
}

bool InputDispatcher::Debounce(bool raw, bool& prev_raw, uint8_t& counter, bool& confirmed)
{
    if (raw == prev_raw) {
        if (counter < DEBOUNCE_THRESHOLD) {
            counter++;
        }
    } else {
        counter = 0;
    }
    prev_raw = raw;

    if (counter >= DEBOUNCE_THRESHOLD) {
        confirmed = raw;
    }
    return confirmed;
}

bool InputDispatcher::DetectToggleEdge(bool confirmed, bool& prev_confirmed)
{
    bool edge = confirmed && !prev_confirmed;
    prev_confirmed = confirmed;
    return edge;
}
```

- [ ] **Step 2: 确认实现与头文件一致**

---

### Task 4: 新增 CAN2_SendKeyboard 发送键盘位掩码给底盘

**Files:**
- Modify: `communication_between_boards/boards_communication.hpp`
- Modify: `communication_between_boards/boards_communication.cpp`

- [ ] **Step 1: 在 .hpp 中声明**

在 `CAN2_SendGimbalIMU_Raw` 声明下方（第 17 行后）添加：

```cpp
HAL_StatusTypeDef CAN2_SendKeyboard(uint16_t keyboard);
```

- [ ] **Step 2: 在 .cpp 中实现**

在文件末尾（`CAN2_SendGimbalIMU_Raw` 实现之后）添加：

```cpp
/**
 * @brief 通过 CAN2 发送键盘位掩码给底盘开发板
 * @param keyboard 16 位键盘位掩码（小端）
 * @return HAL_StatusTypeDef 返回 HAL_OK 表示发送成功
 */
HAL_StatusTypeDef CAN2_SendKeyboard(uint16_t keyboard)
{
    HAL::CAN::Frame frame = {};

    frame.id = 0x305;           // 键盘数据 ID
    frame.dlc = 2;              // uint16_t = 2 字节
    frame.is_extended_id = false;
    frame.is_remote_frame = false;

    memcpy(&frame.data[0], &keyboard, sizeof(keyboard));

    auto& can_bus = HAL::CAN::get_can_bus_instance();
    bool success = can_bus.get_device(HAL::CAN::CanDeviceId::HAL_Can2).send(frame);

    return success ? HAL_OK : HAL_ERROR;
}
```

---

### Task 5: 修改 can_send_task.cpp — 键鼠云台控制

**Files:**
- Modify: `RtosTask/can_send_task.cpp`

改动点：引入 InputDispatcher、填充鼠标键盘字段、模式判断新增 KeyMouse 分支、覆盖云台输入值、发送键盘位掩码。

- [ ] **Step 1: 添加头文件引用**

在第 13 行 `#include "../communication_between_boards/refree_receive.hpp"` 后添加：

```cpp
#include "../communication_between_boards/input_dispatcher.hpp"
```

- [ ] **Step 2: 创建 InputDispatcher 全局实例**

在第 67 行 `RemoteData_t RemoteData;` 后添加：

```cpp
InputDispatcher input_dispatcher;
```

- [ ] **Step 3: 填充 RemoteData 的鼠标/键盘字段 + 调用 InputDispatcher::Update**

在原有 `RemoteData` 填充代码块（约第 307-311 行）后添加。当前代码为：

```cpp
RemoteData.chassis_vx = remoteController.get_left_y();
RemoteData.chassis_vy = remoteController.get_left_x();
RemoteData.gimbal_yaw = -remoteController.get_right_x();
RemoteData.gimbal_pitch = remoteController.get_right_y();
RemoteData.s1 = remoteController.get_s1();
RemoteData.s2 = remoteController.get_s2();
```

在其后追加：

```cpp
// [新增] 填充键鼠数据字段
RemoteData.mouse_x    = remoteController.get_mouseX();
RemoteData.mouse_y    = remoteController.get_mouseY();
RemoteData.mouse_z    = remoteController.get_mouseZ();
RemoteData.mouse_left = remoteController.get_mouseLeft() ? 1 : 0;
RemoteData.mouse_right = remoteController.get_mouseRight() ? 1 : 0;
RemoteData.keyboard   = remoteController.get_keyboard();

// [新增] 更新 InputDispatcher 状态机
input_dispatcher.Update(
    RemoteData.s1,
    RemoteData.s2,
    RemoteData.keyboard,
    RemoteData.mouse_left != 0,
    RemoteData.mouse_right != 0
);
```

- [ ] **Step 4: 修改模式判断逻辑，新增 KeyMouse 分支**

原有代码（约第 364-402 行）的模式判断 if-else 链：

```cpp
if (startup_protect < 500)
{
    yaw_mode = GIMBAL_MODE_STOP;
    pitch_mode = GIMBAL_MODE_STOP;
    startup_protect++;
}
else if (RemoteData.s1 == Remote::DOWN && RemoteData.s2 == Remote::DOWN)
{
    yaw_mode = GIMBAL_MODE_STOP;
    pitch_mode = GIMBAL_MODE_STOP;
}
else if (RemoteData.s1 == Remote::DOWN && RemoteData.s2 == Remote::MIDDLE)
{
    yaw_mode = GIMBAL_MODE_SPEED;
    pitch_mode = GIMBAL_MODE_SPEED;
}
else if (RemoteData.s1 == Remote::MIDDLE && RemoteData.s2 == Remote::MIDDLE)
{
    if (vision_comm.IsVisionReady() && vision_comm.IsDataFresh())
    {
        yaw_mode = GIMBAL_MODE_VISION;
        pitch_mode = GIMBAL_MODE_VISION;
    }
    else
    {
        yaw_mode = GIMBAL_MODE_SPEED;
        pitch_mode = GIMBAL_MODE_SPEED;
    }
}
else
{
    yaw_mode = GIMBAL_MODE_ANGLE;
    pitch_mode = GIMBAL_MODE_ANGLE;
}
```

替换为：

```cpp
if (startup_protect < 500)
{
    yaw_mode = GIMBAL_MODE_STOP;
    pitch_mode = GIMBAL_MODE_STOP;
    startup_protect++;
}
else if (input_dispatcher.GetSource() == InputSource::KeyMouse)
{
    // ---- 键鼠模式 ----
    if (input_dispatcher.IsVisionMode())
    {
        yaw_mode = GIMBAL_MODE_VISION;
        pitch_mode = GIMBAL_MODE_VISION;
    }
    else
    {
        yaw_mode = GIMBAL_MODE_SPEED;
        pitch_mode = GIMBAL_MODE_SPEED;
    }
}
else if (RemoteData.s1 == Remote::DOWN && RemoteData.s2 == Remote::DOWN)
{
    yaw_mode = GIMBAL_MODE_STOP;
    pitch_mode = GIMBAL_MODE_STOP;
}
else if (RemoteData.s1 == Remote::DOWN && RemoteData.s2 == Remote::MIDDLE)
{
    yaw_mode = GIMBAL_MODE_SPEED;
    pitch_mode = GIMBAL_MODE_SPEED;
}
else if (RemoteData.s1 == Remote::MIDDLE && RemoteData.s2 == Remote::MIDDLE)
{
    if (vision_comm.IsVisionReady() && vision_comm.IsDataFresh())
    {
        yaw_mode = GIMBAL_MODE_VISION;
        pitch_mode = GIMBAL_MODE_VISION;
    }
    else
    {
        yaw_mode = GIMBAL_MODE_SPEED;
        pitch_mode = GIMBAL_MODE_SPEED;
    }
}
else
{
    yaw_mode = GIMBAL_MODE_ANGLE;
    pitch_mode = GIMBAL_MODE_ANGLE;
}
```

- [ ] **Step 5: 键鼠速度模式下用鼠标位移覆盖 gimbal_yaw/gimbal_pitch**

在 Step 4 的模式判断之后、FSM Update 之前（约第 404 行），添加鼠标→速度映射：

```cpp
// [新增] 键鼠速度模式：用鼠标位移作为速度输入
if (input_dispatcher.GetSource() == InputSource::KeyMouse && yaw_mode == GIMBAL_MODE_SPEED)
{
    constexpr float MOUSE_YAW_GAIN   = 0.01f;
    constexpr float MOUSE_PITCH_GAIN = 0.01f;

    RemoteData.gimbal_yaw   = static_cast<float>(RemoteData.mouse_x) * MOUSE_YAW_GAIN;
    RemoteData.gimbal_pitch = -static_cast<float>(RemoteData.mouse_y) * MOUSE_PITCH_GAIN;
}
```

> 注意：这个映射放在 `yaw_gimbal_fsm.Update()` 和 `pitch_gimbal_fsm.Update()` 调用之前，确保 FSM 收到的是鼠标速度而非摇杆值。

- [ ] **Step 6: 发送键盘位掩码给底盘**

在 CAN2 发送段（约第 327 行），`CAN2_SendChassisSpeed` 调用后，添加键盘发送调用。在 switch-case 或合适的发送时机添加：

```cpp
// [新增] 键鼠模式下发送键盘位掩码给底盘（每 10ms 发送一次）
if (input_dispatcher.GetSource() == InputSource::KeyMouse)
{
    if ((can2_tick % 10) == 5)
    {
        CAN2_SendKeyboard(input_dispatcher.GetKeyboardMask());
    }
}
```

插入位置：`switch (can2_tick % 10)` 块内的 `case 5:` 分支中，`CAN2_Send_S1andS2_Status` 调用之前或之后。

---

### Task 6: 修改 gimbal_task.cpp — 键鼠发射机构控制

**Files:**
- Modify: `RtosTask/gimbal_task.cpp`

改动点：引入 InputDispatcher（extern 声明）、挡位分支新增 S1↓+S2↑=KeyMouse、触发源改为左右键同时按下、忽略视觉开火。

- [ ] **Step 1: 添加头文件 + extern 声明**

在文件头部 `#include "remote_control_task.hpp"` 后添加：

```cpp
#include "../communication_between_boards/input_dispatcher.hpp"
```

在 extern 声明区域（约第 33 行 `using Remote = ...` 前）添加：

```cpp
extern InputDispatcher input_dispatcher;
```

- [ ] **Step 2: 挡位分支新增 S1↓+S2↑ = KeyMouse**

在挡位控制 if-else 链中，于 `S1下+S2下` 分支前插入 KeyMouse 分支。完整替换后的挡位逻辑如下。

**定位**：第 78-140 行，在 `if (RemoteData.s1 == Remote::DOWN && RemoteData.s2 == Remote::DOWN)` 之前插入：

```cpp
// S1↓+S2↑: 键鼠模式
if (RemoteData.s1 == Remote::DOWN && RemoteData.s2 == Remote::UP)
{
    if (input_dispatcher.IsFrictionOn())
    {
        friction_mode = FRICTION_MODE_ON;
        feeder_mode   = input_dispatcher.IsSingleShot()
                        ? FEEDER_MODE_SINGLE
                        : FEEDER_MODE_CONTINUOUS;
    }
    else
    {
        friction_mode = FRICTION_MODE_STOP;
        feeder_mode   = FEEDER_MODE_STOP;
    }
    force_stop = 0;
}
// S1下+S2下: 全部停止
else if (RemoteData.s1 == Remote::DOWN && RemoteData.s2 == Remote::DOWN)
{
    feeder_mode = FEEDER_MODE_STOP;
    friction_mode = FRICTION_MODE_STOP;
    force_stop = 1;
}
// ... 后续原有分支保持不变 ...
```

> **实现方式**：在原来的第一个 `if (RemoteData.s1 == Remote::DOWN && RemoteData.s2 == Remote::DOWN)` 之前插入 KeyMouse 的 `if` 块，并把原有的 `if` 改为 `else if`。

- [ ] **Step 3: 触发检测 — 键鼠模式下使用左右键同时按下**

定位原触发检测代码（约第 149-181 行）。在 `trigger_pressed = FEEDER_TRIGGER_NONE;` 后、`if (feeder_mode == FEEDER_MODE_CONTINUOUS)` 前插入：

```cpp
// [新增] 键鼠模式：鼠标左右键同时按下 = 发射触发
if (input_dispatcher.GetSource() == InputSource::KeyMouse)
{
    if (input_dispatcher.IsFireTriggered())
    {
        if (feeder_mode == FEEDER_MODE_CONTINUOUS)
        {
            trigger_pressed = FEEDER_TRIGGER_FORWARD;  // 连发：电平触发
        }
        else if (feeder_mode == FEEDER_MODE_SINGLE)
        {
            // 单发：边沿触发（由 feeder_fsm 内部处理，此处持续给信号）
            trigger_pressed = FEEDER_TRIGGER_FORWARD;
        }
    }
    // 跳过遥控器滚轮相关触发逻辑
}
else
{
// 原有遥控器滚轮触发逻辑包裹在 else 中
```

然后需要将原有的滚轮触发逻辑（第 154-180 行）用 `else` 包裹。具体做法：在原有的滚轮触发代码段外添加 `else {` ... `}`。

**实际操作**：将约第 149-181 行：

```cpp
static uint8_t fwd_armed = 1;
static uint8_t rev_armed = 1;
trigger_pressed = FEEDER_TRIGGER_NONE;

// 正转: 连发模式用电平检测，单发模式用边缘触发
if (feeder_mode == FEEDER_MODE_CONTINUOUS)
{
    trigger_pressed = (RemoteData.gimbal_roll > 0.95f) ? FEEDER_TRIGGER_FORWARD : FEEDER_TRIGGER_NONE;
}
else if (feeder_mode == FEEDER_MODE_SINGLE)
{
    // ... fwd_armed / rev_armed 逻辑 ...
}
// ...
last_gimbal_roll = RemoteData.gimbal_roll;
```

替换为（包裹在 if-else 中）：

```cpp
static uint8_t fwd_armed = 1;
static uint8_t rev_armed = 1;
trigger_pressed = FEEDER_TRIGGER_NONE;

if (input_dispatcher.GetSource() == InputSource::KeyMouse)
{
    // 键鼠模式：鼠标左右键同时按下 → 发射
    if (input_dispatcher.IsFireTriggered())
    {
        trigger_pressed = FEEDER_TRIGGER_FORWARD;
    }
}
else
{
    // 正转: 连发模式用电平检测，单发模式用边缘触发
    if (feeder_mode == FEEDER_MODE_CONTINUOUS)
    {
        trigger_pressed = (RemoteData.gimbal_roll > 0.95f) ? FEEDER_TRIGGER_FORWARD : FEEDER_TRIGGER_NONE;
    }
    else if (feeder_mode == FEEDER_MODE_SINGLE)
    {
        if (fwd_armed && RemoteData.gimbal_roll > 0.95f)
        {
            trigger_pressed = FEEDER_TRIGGER_FORWARD;
            fwd_armed = 0;
        }
        if (RemoteData.gimbal_roll < 0.80f)
        {
            fwd_armed = 1;
        }

        if (rev_armed && RemoteData.gimbal_roll < -0.95f)
        {
            trigger_pressed = FEEDER_TRIGGER_REVERSE;
            rev_armed = 0;
        }
        if (RemoteData.gimbal_roll > -0.80f)
        {
            rev_armed = 1;
        }
    }
}
last_gimbal_roll = RemoteData.gimbal_roll;
```

- [ ] **Step 4: 视觉开火指令仅对遥控器模式生效**

将原有视觉开火代码（约第 185-195 行）：

```cpp
if (RemoteData.s1 == Remote::MIDDLE && RemoteData.s2 == Remote::MIDDLE)
{
    if (trigger_pressed == FEEDER_TRIGGER_REVERSE)
    {
    }
    else if (vision_comm.IsFireCommanded())
    {
        trigger_pressed = FEEDER_TRIGGER_FORWARD;
    }
}
```

替换为（增加 InputSource 判断）：

```cpp
// 视觉开火：仅遥控器视觉模式生效，键鼠模式下忽略
if (input_dispatcher.GetSource() != InputSource::KeyMouse)
{
    if (RemoteData.s1 == Remote::MIDDLE && RemoteData.s2 == Remote::MIDDLE)
    {
        if (trigger_pressed == FEEDER_TRIGGER_REVERSE)
        {
        }
        else if (vision_comm.IsFireCommanded())
        {
            trigger_pressed = FEEDER_TRIGGER_FORWARD;
        }
    }
}
```

---

### Task 7: 清理旧 keyboard_task 文件

**Files:**
- Delete: `communication_between_boards/keyboard_task.hpp`
- Delete: `communication_between_boards/keyboard_task.cpp`

- [ ] **Step 1: 删除旧文件**

```bash
git rm communication_between_boards/keyboard_task.hpp
git rm communication_between_boards/keyboard_task.cpp
```

- [ ] **Step 2: 检查其他文件是否引用了 keyboard_task.hpp**

搜索引用并确认已无依赖：

```bash
grep -r "keyboard_task" --include="*.cpp" --include="*.hpp" --include="*.h" .
```

预期：无引用（如果有，则需要移除对应 #include）。

---

### Task 8: 构建验证

- [ ] **Step 1: 确认所有文件语法正确**

检查新增/修改文件的 include 路径是否存在，确保无循环引用。

- [ ] **Step 2: 在 MDK-ARM 中编译**

打开 `MDK-ARM/Hero_gimbal.uvprojx`，添加新文件到工程：
- `communication_between_boards/input_dispatcher.cpp`

编译确认 0 errors。

- [ ] **Step 3: 代码走查检查清单**

| 检查项 | 说明 |
|--------|------|
| `extern InputDispatcher input_dispatcher` | gimbal_task.cpp 与 can_send_task.cpp 引用同一实例 |
| 消抖阈值 30ms | 1kHz 控制周期下 30 个 tick |
| 视觉延时 2000ms | R键开关不影响视觉延时计数 |
| R=OFF 时 T 无效 | `DetectToggleEdge` 前检查 `r_toggle_on_` |
| 挡位切出键鼠 | Update() 内 `source_=Remote` 且 `return`，不更新键鼠状态 |
| 视觉开火仅遥控器 | gimbal_task.cpp 中 `GetSource() != KeyMouse` 守卫 |
| CAN ID 0x305 不冲突 | 现有 ID: 0x301-0x304, 0x305 未使用 |
