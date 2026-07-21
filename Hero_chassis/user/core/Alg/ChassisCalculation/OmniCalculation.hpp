#ifndef OmniCalculation_HPP
#define OmniCalculation_HPP

#include "../user/core/Alg/ChassisCalculation/CalculationBase.hpp"
#include <math.h>

namespace Alg::CalculationBase
{
    /**
     * @class Omni_FK
     * @brief 全向轮底盘正向运动学计算类
     */
    class Omni_FK : public ForwardKinematicsBase
    {
        public:
            // 修复：去掉默认参数，避免编译错误
            Omni_FK(float r, float s, float n, const float wheel_azimuth[4], const float wheel_direction[4]) 
                : R(r), S(s), N(n), ChassisVx(0.0f), ChassisVy(0.0f), ChassisVw(0.0f) 
            {
                for(int i = 0; i < 4; i++)
                {
                    Wheel_Azimuth[i] = wheel_azimuth[i];
                    Wheel_Direction[i] = wheel_direction[i];
                }
            }

            void ForKinematics()
            {
                ChassisVx = 0.0f; ChassisVy = 0.0f; ChassisVw = 0.0f; // 每次计算前清零
                for(int i = 0; i < 4; i++)
                {
                    float wheel_vx = S * Get_w(i) * cosf(Wheel_Azimuth[i]);
                    float wheel_vy = S * Get_w(i) * sinf(Wheel_Azimuth[i]);

                    ChassisVx += wheel_vx;
                    ChassisVy += wheel_vy;

                    float delta_angle = Wheel_Azimuth[i] - Wheel_Direction[i];
                    ChassisVw += (Get_w(i) * S * sinf(delta_angle)) / R;
                }
                ChassisVx /= N;
                ChassisVy /= N;
                ChassisVw /= N;
            }

            void OmniForKinematics(float w0, float w1, float w2, float w3)  //传入4个轮子的角速度
            {
                Set_w0w1w2w3(w0, w1, w2, w3);
                ForKinematics();
            }

            float GetChassisVx() const { return ChassisVx; }
            float GetChassisVy() const { return ChassisVy; }
            float GetChassisVw() const { return ChassisVw; }

        private:
            float R, S, N;
            float ChassisVx, ChassisVy, ChassisVw;
            float Wheel_Azimuth[4];   // 修复：去掉 const，以便在构造函数中赋值
            float Wheel_Direction[4]; 
    };

    /**
     * @class Omni_ID
     * @brief 全向轮底盘逆向动力学计算类
     */
    class Omni_ID : public InverseDynamicsBase
    {
        public:
            Omni_ID(float r, float s, float n, const float wheel_azimuth[4], const float wheel_coordinate[4][2]) 
                : R(r), S(s), N(n)
            {
                for(int i = 0; i < 4; i++)
                {
                    MotorTorque[i] = 0.0f;
                    Wheel_Azimuth[i] = wheel_azimuth[i]; // 修复：统一拼写为 Wheel_Azimuth
                    Wheel_Coordinates[i][0] = wheel_coordinate[i][0];
                    Wheel_Coordinates[i][1] = wheel_coordinate[i][1];
                }
            }

            void InverseDynamics()
            {
                for(int i = 0; i < 4; i++)
                {
                    MotorTorque[i] = (cosf(Wheel_Azimuth[i]) / N * GetFx() + 
                                     sinf(Wheel_Azimuth[i]) / N * GetFy() + 
                                     (-Wheel_Coordinates[i][1] * cosf(Wheel_Azimuth[i]) + 
                                      Wheel_Coordinates[i][0] * sinf(Wheel_Azimuth[i])) / N * GetTorque()) * S;
                }
            }

            void OmniInvDynamics(float fx, float fy, float torque)
            {
                Set_FxFyTor(fx, fy, torque);
                InverseDynamics();
            }

            float GetMotorTorque(int index) const { return (index >= 0 && index < 4) ? MotorTorque[index] : 0.0f; }

        private:
            float R, S, N;
            float MotorTorque[4];
            float Wheel_Azimuth[4];      // 修复：去掉 const
            float Wheel_Coordinates[4][2]; 
    };

    /**
     * @class Omni_IK
     * @brief 全向轮底盘逆向运动学计算类
     */
    class Omni_IK : public InverseKinematicsBase
    {
        public:
            Omni_IK(float r, float s, const float wheel_azimuth[4], const float wheel_coordinate[4][2]) 
                : R(r), S(s) 
            {
                for(int i = 0; i < 4; i++)
                {
                    Motor[i] = 0.0f;
                    Wheel_Azimuth[i] = wheel_azimuth[i]; // 修复：统一拼写
                    Wheel_Coordinates[i][0] = wheel_coordinate[i][0];
                    Wheel_Coordinates[i][1] = wheel_coordinate[i][1];
                }
            }

            void CalculateVelocities()
            {
                Vx = GetSpeedGain() * (GetSignal_x() * cosf(GetPhase()) + GetSignal_y() * sinf(GetPhase()));
                Vy = GetSpeedGain() * (GetSignal_x() * -sinf(GetPhase()) + GetSignal_y() * cosf(GetPhase()));
                Vw = GetRotationalGain() * GetSignal_w();
            }

            void InvKinematics()
            {   
                for(int i = 0; i < 4; i++)
                {
                    // 修复：WheelAzimuth -> Wheel_Azimuth
                    Motor[i] = (cosf(Wheel_Azimuth[i]) * Vx -   //手动改了这个-号，后面要改回来
                                sinf(Wheel_Azimuth[i]) * Vy + 
                                Vw * (-Wheel_Coordinates[i][1] * cosf(Wheel_Azimuth[i]) +   
                                      Wheel_Coordinates[i][0] * sinf(Wheel_Azimuth[i]))) / S;
                }
            }

            void OmniInvKinematics(float vx, float vy, float vw, float phase, float speed_gain, float rotate_gain)
            {
                SetPhase(phase);
                SetSpeedGain(speed_gain);
                SetRotationalGain(rotate_gain);
                SetSignal_xyw(vx, vy, vw);
                CalculateVelocities(); 
                InvKinematics();
            }

            float GetMotor(int index) const { return (index >= 0 && index < 4) ? Motor[index] : 0.0f; }

        private:
            float Vx{0.0f}, Vy{0.0f}, Vw{0.0f};
            float R, S;
            float Motor[4];
            float Wheel_Azimuth[4];      // 修复：去掉 const
            float Wheel_Coordinates[4][2]; 
    };
}

#endif