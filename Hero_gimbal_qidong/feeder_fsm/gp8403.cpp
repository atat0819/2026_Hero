#include "gp8403.hpp"
#include <cmath>  // std::isfinite：判断 NaN/±inf 等非有限浮点值
#include "cmsis_os.h"  // osDelay：任务上下文挂起等待，不忙等

namespace
{
constexpr uint8_t GP8403_ADDRESS_7BIT = 0x58U;    // 芯片7位I2C地址
constexpr uint16_t GP8403_ADDRESS_HAL =           // 8位地址 = 7位地址左移1位
    static_cast<uint16_t>(GP8403_ADDRESS_7BIT << 1U);
constexpr uint16_t GP8403_FULL_SCALE_MV = 5000U;  // 输出电压满量程 5V
constexpr uint16_t GP8403_MAX_DAC_CODE = 4095U;   // 12位DAC满量程
constexpr uint16_t GP8403_CHANNEL0_REGISTER = 0x02U;  // 通道0输出寄存器地址
constexpr uint32_t GP8403_I2C_TIMEOUT_MS = 20U;   // 单次I2C操作超时
constexpr uint32_t GP8403_RETRY_COUNT = 3U;       // 通信失败重试次数

void Bus_Delay()
{
    // 任务上下文调用：挂起1个tick（≥1ms），避免忙等占用CPU
    osDelay(1U);
}
}

// 构造函数：保存I2C句柄并清零状态
Class_GP8403::Class_GP8403(I2C_HandleTypeDef *i2c)
    : i2c_(i2c),
      last_hal_status_(HAL_ERROR),
      commanded_voltage_mV_(0U),
      error_count_(0U),
      ready_(false)
{
}

// 初始化：先探测芯片，再强制输出0V，保证上电后阀门处于关闭状态
bool Class_GP8403::Init()
{
    commanded_voltage_mV_ = 0U;
    error_count_ = 0U;
    ready_ = false;

    if (!Probe())
    {
        return false;
    }

    // 上电后始终先建立一个安全指令（0V）。
    return Stop();
}

// 探测：轮询芯片应答，确认其在总线上
bool Class_GP8403::Probe()
{
    if (i2c_ == nullptr)
    {
        last_hal_status_ = HAL_ERROR;
        ready_ = false;
        return false;
    }

    for (uint32_t attempt = 0U; attempt < GP8403_RETRY_COUNT; ++attempt)
    {
        last_hal_status_ = HAL_I2C_IsDeviceReady(
            i2c_, GP8403_ADDRESS_HAL, 1U, GP8403_I2C_TIMEOUT_MS);

        if (last_hal_status_ == HAL_OK)
        {
            ready_ = true;
            return true;
        }

        ++error_count_;
        if ((attempt + 1U) < GP8403_RETRY_COUNT)
        {
            Recover_I2C_Bus();
        }
    }

    ready_ = false;
    return false;
}

// 设置输出电压(mV)：超量程截断，按 mV*4095/5000 四舍五入换算成12位DAC码
bool Class_GP8403::Set_Voltage_mV(uint16_t voltage_mV)
{
    if (voltage_mV > GP8403_FULL_SCALE_MV)
    {
        voltage_mV = GP8403_FULL_SCALE_MV;
    }

    const uint32_t rounded_code =
        (static_cast<uint32_t>(voltage_mV) * GP8403_MAX_DAC_CODE +
         (GP8403_FULL_SCALE_MV / 2U)) /
        GP8403_FULL_SCALE_MV;

    if (!Write_Channel0(static_cast<uint16_t>(rounded_code)))
    {
        return false;
    }

    commanded_voltage_mV_ = voltage_mV;
    return true;
}

// 设置阀门开度：0~100% 线性映射到 0~5V（mV = 百分比×50）
// 非法值(NaN/±inf)与负值统一钳到 0%，阀门保持关闭（故障安全）
bool Class_GP8403::Set_Valve_Opening(float percent)
{
    if (!std::isfinite(percent) || (percent < 0.0f))
    {
        percent = 0.0f;
    }
    else if (percent > 100.0f)
    {
        percent = 100.0f;
    }

    const uint16_t voltage_mV =
        static_cast<uint16_t>(percent * 50.0f + 0.5f);
    return Set_Voltage_mV(voltage_mV);
}

// 停止：输出0V，阀门完全关闭
bool Class_GP8403::Stop()
{
    return Set_Voltage_mV(0U);
}

// 写通道0：DAC码左移4位组成16位数据帧，写寄存器0x02，失败重试并恢复总线
bool Class_GP8403::Write_Channel0(uint16_t dac_code)
{
    if (i2c_ == nullptr)
    {
        last_hal_status_ = HAL_ERROR;
        ready_ = false;
        return false;
    }

    if (dac_code > GP8403_MAX_DAC_CODE)
    {
        dac_code = GP8403_MAX_DAC_CODE;
    }

    const uint16_t frame = static_cast<uint16_t>(dac_code << 4U);
    uint8_t data[2] = {
        static_cast<uint8_t>(frame & 0xFFU),
        static_cast<uint8_t>((frame >> 8U) & 0xFFU),
    };

    for (uint32_t attempt = 0U; attempt < GP8403_RETRY_COUNT; ++attempt)
    {
        last_hal_status_ = HAL_I2C_Mem_Write(
            i2c_,
            GP8403_ADDRESS_HAL,
            GP8403_CHANNEL0_REGISTER,
            I2C_MEMADD_SIZE_8BIT,
            data,
            sizeof(data),
            GP8403_I2C_TIMEOUT_MS);

        if (last_hal_status_ == HAL_OK)
        {
            ready_ = true;
            return true;
        }

        ++error_count_;
        if ((attempt + 1U) < GP8403_RETRY_COUNT)
        {
            Recover_I2C_Bus();
        }
    }

    ready_ = false;
    return false;
}

// 总线恢复：I2C2被从设备拉死时，DeInit后用GPIO开漏位带法
// 发9个SCL脉冲把从设备顶出故障态，再发STOP条件，最后重新初始化I2C
bool Class_GP8403::Recover_I2C_Bus()
{
    if ((i2c_ == nullptr) || (i2c_->Instance != I2C2))
    {
        last_hal_status_ = HAL_ERROR;
        return false;
    }

    HAL_I2C_DeInit(i2c_);

    __HAL_RCC_GPIOF_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {};
    gpio.Pin = GPIO_PIN_0 | GPIO_PIN_1;
    gpio.Mode = GPIO_MODE_OUTPUT_OD;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_MEDIUM;
    HAL_GPIO_Init(GPIOF, &gpio);

    // PF0是SDA、PF1是SCL，先释放两条线再开始时钟恢复。
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0 | GPIO_PIN_1, GPIO_PIN_SET);
    Bus_Delay();

    for (uint32_t pulse = 0U; pulse < 9U; ++pulse)
    {
        HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1, GPIO_PIN_RESET);
        Bus_Delay();
        HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1, GPIO_PIN_SET);
        Bus_Delay();
    }

    // 生成STOP条件：SDA拉低、SCL拉高，再把SDA释放。
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0, GPIO_PIN_RESET);
    Bus_Delay();
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1, GPIO_PIN_SET);
    Bus_Delay();
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0, GPIO_PIN_SET);
    Bus_Delay();

    last_hal_status_ = HAL_I2C_Init(i2c_);
    if (last_hal_status_ != HAL_OK)
    {
        ready_ = false;
        return false;
    }

    last_hal_status_ = HAL_I2CEx_ConfigAnalogFilter(
        i2c_, I2C_ANALOGFILTER_ENABLE);
    if (last_hal_status_ != HAL_OK)
    {
        ready_ = false;
        return false;
    }

    last_hal_status_ = HAL_I2CEx_ConfigDigitalFilter(i2c_, 0U);
    if (last_hal_status_ != HAL_OK)
    {
        ready_ = false;
        return false;
    }

    return true;
}

bool Class_GP8403::Is_Ready() const
{
    return ready_;  // 芯片是否就绪
}

uint16_t Class_GP8403::Get_Commanded_Voltage_mV() const
{
    return commanded_voltage_mV_;  // 当前指令电压
}

float Class_GP8403::Get_Commanded_Opening() const
{
    return static_cast<float>(commanded_voltage_mV_) / 50.0f;  // 电压换算回开度
}

HAL_StatusTypeDef Class_GP8403::Get_Last_HAL_Status() const
{
    return last_hal_status_;  // 最近一次HAL调用结果
}

uint32_t Class_GP8403::Get_Error_Count() const
{
    return error_count_;  // 累计通信错误次数
}

Class_GP8403 gp8403(&hi2c2);  // 全局实例，挂在 I2C2 (PF0=SDA, PF1=SCL)
