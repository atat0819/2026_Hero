#ifndef GIMBAL_TASK_HPP
#define GIMBAL_TASK_HPP

#include "cmsis_os.h"
#include <stdint.h>

#ifdef __cplusplus
#include "../user/core/BSP/Motor/Dm/DmMotor.hpp"
#include "../user/core/BSP/Motor/LK/Lk_motor.hpp"
#include "../feeder_fsm/feeder_fsm.hpp"
#include "Alg/PID/pid.hpp"
#include "../user/core/BSP/Motor/Dji/DjiMotor.hpp"


extern BSP::Motor::DM::J4310<1> dm666;
#endif

typedef struct
{
    float angle_deg;
    float angle_rad;
    float velocity_rpm;
    float velocity_rads;
    float current_a;
    float multi_angle;
    float multi_rad;
    uint8_t temperature;
} DM4310_State_t;

typedef struct
{
    float angle_deg;
    float angle_rad;
    float velocity_rpm;
    float velocity_rads;
    float current_a;
    float delta_angle;
    float multi_angle;
    uint8_t temperature;
} DJI3508_State_t;

extern DM4310_State_t dm4310_state[1];
extern DJI3508_State_t dji3508_state[3];
extern float feeder_current_angle;

#ifdef __cplusplus
extern "C" {
#endif

void gimbal_task(void *argument);
void DM4310_feedback(void);
void DJI3508_feedback(void);
float hz_to_rotor_angle_per_frame(float fire_hz);

extern BSP::Motor::Dji::GM3508<3> friction_motor; // 电机控制器，初始ID为0x200，发送ID为0x2FF


#ifdef __cplusplus
}
#endif

#endif // GIMBAL_TASK_HPP
