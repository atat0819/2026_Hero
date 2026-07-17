#include "HI14.hpp"
#include <string.h>
#include "FreeRTOS.h"
#include "queue.h"

// 外部队列（在 remote_control_task.cpp 中定义）
extern QueueHandle_t IMUDataQueue;

// 全局变量定义
IMU_Handle_t hImu;
IMU_Data_t   ImuData;
IMU_Float_t  ImuFloat;
volatile uint8_t ImuDataReady = 0;  // 收到第一帧有效 IMU 数据后置 1

void IMU_Init(void)
{
    hImu.Step = 0;
    hImu.Index = 0;
    hImu.ErrCount = 0;
}

void IMU_UART_RxCallBack(uint8_t rxdata)
{
    switch (hImu.Step)
    {
        case 0:
            if (rxdata == 0xA5) {
                hImu.Step = 1;
                hImu.Index = 0;
            }
            break;
        case 1:
            if (hImu.Index < IMU_DATA_LEN) {
                hImu.RxBuffer[hImu.Index++] = rxdata;
            }
            if (hImu.Index >= IMU_DATA_LEN) {
                // 校验
                if (hImu.RxBuffer[IMU_CMD_POS] == IMU_EXPECTED_CMD &&
                    hImu.RxBuffer[IMU_TAIL1_POS] == 0x5A &&
                    hImu.RxBuffer[IMU_TAIL2_POS] == 0xA5) {
                    // 解析原始数据
                    memcpy(&ImuData.Roll,  &hImu.RxBuffer[32], 4);
                    memcpy(&ImuData.Pitch, &hImu.RxBuffer[36], 4);
                    memcpy(&ImuData.Yaw,   &hImu.RxBuffer[40], 4);
                    memcpy(&ImuData.GyrPitch, &hImu.RxBuffer[14], 2);
                    memcpy(&ImuData.GyrRoll,  &hImu.RxBuffer[16], 2);
                    memcpy(&ImuData.GyrYaw,   &hImu.RxBuffer[18], 2);
                    memcpy(&ImuData.Qw, &hImu.RxBuffer[44], 2);
                    memcpy(&ImuData.Qx, &hImu.RxBuffer[46], 2);
                    memcpy(&ImuData.Qy, &hImu.RxBuffer[48], 2);
                    memcpy(&ImuData.Qz, &hImu.RxBuffer[50], 2);

                    // 转换成浮点
                    ImuFloat.roll  = (float)ImuData.Roll / 87.890625f;
                    ImuFloat.pitch = (float)ImuData.Pitch / 87.890625f;
                    ImuFloat.yaw = (float)ImuData.Yaw / 87.890625f;
                    ImuFloat.gyr_pitch = (float)ImuData.GyrPitch * 0.0275f;
                    ImuFloat.gyr_roll  = (float)ImuData.GyrRoll  * 0.0275f;
                    ImuFloat.gyr_yaw   = (float)ImuData.GyrYaw   * 0.0275f;
                    ImuFloat.qw = (float)ImuData.Qw * 0.0001f;
                    ImuFloat.qx = (float)ImuData.Qx * 0.0001f;
                    ImuFloat.qy = (float)ImuData.Qy * 0.0001f;
                    ImuFloat.qz = (float)ImuData.Qz * 0.0001f;

                    ImuDataReady = 1;  // 标记 IMU 数据已就绪

                } else {
                    hImu.ErrCount++;
                    if (hImu.ErrCount >= 20) {
                        hImu.Step = 0;
                        hImu.ErrCount = 0;
                    }
                }
                hImu.Step = 0;   // 复位，准备下一帧
            }
            break;
        default:
            hImu.Step = 0;
            break;
    }
}