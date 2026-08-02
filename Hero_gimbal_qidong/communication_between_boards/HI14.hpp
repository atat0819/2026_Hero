#ifndef HI14_HPP
#define HI14_HPP

#include <stdint.h>

#define IMU_DATA_LEN         54
#define IMU_CMD_POS          4
#define IMU_EXPECTED_CMD     0x92
#define IMU_TAIL1_POS        (IMU_DATA_LEN - 2)
#define IMU_TAIL2_POS        (IMU_DATA_LEN - 1)

typedef struct {
    int32_t Roll;
    int32_t Pitch;
    int32_t Yaw;
    int16_t GyrPitch;
    int16_t GyrRoll;
    int16_t GyrYaw;
    int16_t Qw, Qx, Qy, Qz;
} IMU_Data_t;

typedef struct {
    float roll;
    float pitch;
    float yaw;
    float gyr_roll;
    float gyr_pitch;
    float gyr_yaw;
    float qw, qx, qy, qz;
} IMU_Float_t;

typedef struct {
    uint8_t  Step;
    uint8_t  RxBuffer[IMU_DATA_LEN];
    uint8_t  Index;
    uint32_t ErrCount;
} IMU_Handle_t;

// 全局变量（在 HI14.cpp 中定义）
extern IMU_Handle_t hImu;
extern IMU_Data_t   ImuData;
extern IMU_Float_t  ImuFloat;
extern volatile uint8_t ImuDataReady;  // IMU 是否已收到过有效数据帧

// 函数声明
void IMU_UART_RxCallBack(uint8_t rxdata);
void IMU_Init(void);   // 初始化状态机

#endif