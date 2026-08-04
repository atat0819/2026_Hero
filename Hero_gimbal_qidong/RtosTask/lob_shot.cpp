/**
 * @file   lob_shot.cpp
 * @brief  吊射任务 — 红外检测 + 推杆 + 充压击发 (替代摩擦轮方案)
 *
 * 执行器: solenoid_valve_1 (PI0)  充压/击发 — 常开充压, 击发瞬间关闭切断气源
 *         solenoid_valve_2 (PH12) 推杆     — 通电伸出顶死密封气室, 断电缩回
 *         gp8403 (比例阀)          — 平时保持目标开度(气室满压), 击发时 0V
 * 红外:   ir_sensor (PH11) 对射式, 有弹=高电平, 驱动内已软件防抖
 *
 * 模式: s1=0/2 全断 | s1=1 自动(红外上膛+s2击发) | s1=3 手动(摇杆推杆+s2击发)
 * 逻辑: 状态机在 feeder_fsm/lob_shot_fsm, 本任务只做采集输入 + 执行输出
 */

#include "lob_shot.hpp"
#include "can_send_task.hpp"              // RemoteData (遥控器解析结果统一出口)
#include "../feeder_fsm/gp8403.hpp"       // GP8403 DAC, 比例阀电压控制
#include "../feeder_fsm/solenoid_valve.hpp"
#include "../feeder_fsm/ir_sensor.hpp"
#include "../feeder_fsm/lob_shot_fsm.hpp"

Class_Lob_Shot_FSM lob_shot_fsm;

void lob_shot_task(void *argument)
{
    // 上电安全: 强制关阀
    solenoid_valve_1.Init();
    solenoid_valve_2.Init();
    // 上电安全: 探测比例阀DAC芯片并强制输出0V（比例阀关闭）
    gp8403.Init();
    // 红外传感器: 清零防抖状态 (引脚已在 CubeMX 配置)
    ir_sensor.Init();
    // 吊射状态机初始化
    lob_shot_fsm.Init();

    Struct_Lob_Shot_Input input = {};
    uint16_t last_opening_mV = 0U;   // 上次比例阀指令电压 (变化才写 I2C)

    for (;;)
    {
        // ---- 采集输入 (统一从 RemoteData, 由 can_send_task 每帧填充) ----
        ir_sensor.Update();                       // 红外采样 + 防抖
        input.s1          = RemoteData.s1;        // 挡位切模式
        input.s2          = RemoteData.s2;        // 击发拨杆
        input.ir_has_ball = ir_sensor.Has_Ball(); // 防抖后: 进弹口有弹
        input.left_x      = RemoteData.chassis_vy; // 左摇杆 X (填充自 get_left_x(), can_send_task.cpp:365)

        // ---- 状态机: 先更新驻留计时, 再算逻辑 ----
        lob_shot_fsm.TIM_Calculate_PeriodElapsedCallback();
        lob_shot_fsm.Update(input);

        // ---- 执行输出 ----
        const Struct_Lob_Shot_Output &out = lob_shot_fsm.Get_Output();

        // 推杆: 顶死 / 缩回
        if (out.pusher_extended)
        {
            solenoid_valve_2.Open();
        }
        else
        {
            solenoid_valve_2.Close();
        }

        // 电磁阀1: 充压通断 (击发瞬间关闭切断气源)
        if (out.charge_valve_open)
        {
            solenoid_valve_1.Open();
        }
        else
        {
            solenoid_valve_1.Close();
        }

        // 比例阀: 开度变化才写 (DAC 锁存, 避免每周期刷 I2C)
        if (out.valve_opening_mV != last_opening_mV)
        {
            gp8403.Set_Voltage_mV(out.valve_opening_mV);
            last_opening_mV = out.valve_opening_mV;
        }

        vTaskDelay(5); // 每 5ms 一次, 与 gimbal_task 控制周期一致
    }
}
