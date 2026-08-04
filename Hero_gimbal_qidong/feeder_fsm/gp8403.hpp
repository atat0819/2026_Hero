#ifndef GP8403_HPP
#define GP8403_HPP

#include "i2c.h"
#include <stdint.h>

// GP8403 双通道 0-5V DAC 驱动，用 VOUT0 控制一路 0-5V 比例阀。
// 所有写操作均为阻塞式，必须在同一个任务中调用。
class Class_GP8403
{
public:
    explicit Class_GP8403(I2C_HandleTypeDef *i2c);  // 构造函数，传入I2C句柄

    bool Init();                                    // 探测芯片并强制输出0V（上电安全）
    bool Probe();                                   // 检查芯片是否在线（默认地址0x58）
    bool Set_Voltage_mV(uint16_t voltage_mV);       // 设置输出电压(mV)，超过5000mV自动截断
    bool Set_Valve_Opening(float percent);          // 0~100% 开度映射到 0~5V；NaN/±inf/负值一律钳到0%
    bool Stop();                                    // 输出0V，即关闭阀门

    bool Is_Ready() const;                          // 芯片是否就绪
    uint16_t Get_Commanded_Voltage_mV() const;      // 当前指令电压(mV)
    float Get_Commanded_Opening() const;            // 当前指令开度(%)
    HAL_StatusTypeDef Get_Last_HAL_Status() const;  // 最近一次HAL调用结果
    uint32_t Get_Error_Count() const;               // 累计通信错误次数

private:
    bool Write_Channel0(uint16_t dac_code);         // 写12位DAC码到通道0寄存器
    bool Recover_I2C_Bus();                         // GPIO位带恢复卡死的I2C总线

private:
    I2C_HandleTypeDef *i2c_;            // I2C句柄
    HAL_StatusTypeDef last_hal_status_; // 最近一次HAL调用状态
    uint16_t commanded_voltage_mV_;     // 当前指令电压(mV)
    uint32_t error_count_;              // 错误计数
    bool ready_;                        // 就绪标志
};

extern Class_GP8403 gp8403;  // 全局实例，挂在 I2C2 (PF0=SDA, PF1=SCL)

#endif
