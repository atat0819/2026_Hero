#ifndef FEEDFORWARD_HPP
#define FEEDFORWARD_HPP 

#include <math.h>
#define MY_PI 3.141592653589
#define g 9.80665

namespace Alg::Feedforward 
{
    /**
     * @class Uphill
     * @brief 上坡前馈计算类
     * 
     * 该类用于计算机器人在斜坡上行驶时所需的前馈力和扭矩，
     * 根据坡度和机器人的质量计算每个轮子需要施加的力和扭矩。
     */
    class Uphill
    {
        public:
            /**
             * @brief 构造函数
             * @param m 机器人质量(kg)
             * @param coeffs 系数矩阵，用于计算各轮子分配比例，在MatLab中生成
             * @param s 轮子半径，用于力到扭矩的转换计算
             */
            Uphill(double m, const double coeffs[4][4], float s) :  m_(m), S(s)
            {
                // 初始化系数矩阵
                for(int i = 0; i < 4; i++) 
                {
                    for(int j = 0; j < 4; j++) 
                    {
                        p[i][j] = coeffs[i][j];
                    }
                }
                slope_ = 0.0;
                slope_rad_ = 0.0;
                total_force = 0.0;
                // 初始化轮子力和增益数组
                for(int i = 0; i < 4; i++)
                {
                    Gain[i] = 0.0;
                    force[i] = 0.0;
                }
            }

            /**
             * @brief 计算给定坡度下的增益系数
             * @param slope_deg 坡度(角度)
             * @param coeffs 系数数组
             * @return 计算得到的增益值
             * 
             * 使用三次多项式计算增益系数:
             * gain = coeffs[0]*slope³ + coeffs[1]*slope² + coeffs[2]*slope + coeffs[3]
             */
            double calculate_gain(double slope_deg, double* coeffs)
            {
                return coeffs[0] * slope_deg * slope_deg * slope_deg +
                coeffs[1] * slope_deg * slope_deg +
                coeffs[2] * slope_deg +
                coeffs[3];
            }

            /**
             * @brief 执行上坡前馈力计算
             * @param slope 当前坡度(角度)
             * 
             * 根据当前坡度计算每个轮子需要的前馈力
             */
            void Uphill_FeedForward(double slope)
            {
                SetSlope(slope);
                for(int i = 0; i < 4; i++)
                {
                    this->Gain[i] = calculate_gain(slope_, p[i]);
                    this->force[i] = Gain[i] * total_force;
                }                
            }

            /**
             * @brief 设置坡度并计算总力
             * @param slope 坡度值(角度)
             * 
             * 更新坡度值并重新计算总的前馈力
             */
            void SetSlope(double slope)
            {
                slope_ = slope;
                slope_rad_ = slope_ * MY_PI / 180.0;                
                total_force = m_ * g * sin(slope_rad_);
            }

            /**
             * @brief 获取指定轮子的前馈力
             * @param index 轮子索引(0-3)
             * @return 对应轮子的前馈力(N)
             */
            double GetForce(int index) const
            {
                return force[index];
            }

            /**
             * @brief 获取指定轮子的增益系数
             * @param index 轮子索引(0-3)
             * @return 对应轮子的增益系数
             */
            double GetGain(int index) const
            {
                return Gain[index];
            }

            /**
             * @brief 获取总前馈力
             * @return 总前馈力(N)
             * 
             * 总前馈力 = 质量 × 重力加速度 × sin(坡度)
             */
            double GetTotalForce() const
            {
                return total_force;
            }

        private:
            double m_;               // 机器人质量(kg)
            double slope_;           // 坡度(角度)
            double slope_rad_;       // 坡度(弧度)
            double Gain[4];          // 各轮子增益系数
            double p[4][4];          // 系数矩阵
            double force[4];         // 各轮子前馈力(N)
            double total_force;      // 总前馈力(N)

        public:
            /**
             * @brief 全向轮力到扭矩转换 在cpp中实现 
             * 
             * 将计算出的前馈力转换为电机扭矩，适用于全向轮底盘
             */
            void Omni_ForceToTorque();
            /**
             * @brief 麦克纳姆轮力到扭矩转换 在cpp中实现 
             * 
             * 将计算出的前馈力转换为电机扭矩，适用于麦克纳姆轮底盘
             */
            void Mecanum_ForceToTorque();
            /**
             * @brief 舵轮底盘力到扭矩转换 在cpp中实现 
             * 
             * 将计算出的前馈力转换为电机扭矩，适用于转向型底盘
             */
            void steering_ForceToTorque();

            /**
             * @brief 获取指定轮子的扭矩
             * @param index 轮子索引(0-3)
             * @return 对应轮子的扭矩(N·m)
             */
            float GetTorque(int index) const 
            { 
                if(index >= 0 && index < 4) 
                {
                    return torque[index];
                }
                return 0.0f; // 错误情况返回0
            }

        private:
            float torque[4];            // 各轮子扭矩(N·m)
            float sqrt2 = sqrtf(2.0f);  // 预计算值: √2
            float S;                    // 轮子半径，用于力到扭矩的转换计算

    };


    /**
     * @class Acceleration
     * @brief 加速度前馈控制器
     * 
     * 根据目标速度的变化率（即加速度）计算相应的前馈补偿量
     */
    class Acceleration
    {
        public:
            /**
             * @brief 构造函数
             * @param k_acc_ 加速度前馈系数
             * @param control_cycle_ 控制周期(s)
             */
            Acceleration(float k_acc_, float control_cycle_)
            {
                last_target_speed = 0.0f;
                acc_ff = 0.0f;
                k_acc = k_acc_;
                control_cycle = control_cycle_;
            }

            /**
             * @brief 计算加速度前馈值
             * @param target_speed 目标速度 单位与控制器期望速度一致
             * 
             * 根据目标速度变化率计算加速度前馈值
             */
            void AccelerationFeedforward(float target_speed)
            {
                // 计算速度变化率（即加速度）
                acc_ff = (target_speed - last_target_speed) / control_cycle;
                // 计算前馈输出
                feedforward = k_acc * acc_ff;
                // 更新上次目标速度
                last_target_speed = target_speed;
            }

            /**
             * @brief 获取前馈输出值
             * @return 前馈输出值 单位与速度控制器具有相同的量纲维度
             */
            float getFeedforward()
            {
                return feedforward;
            }

        private:
            float last_target_speed;    // 上一次的目标速度
            float control_cycle;        // 控制周期
            float acc_ff;               // 加速度
            float k_acc;                // 前馈系数
            float feedforward;          // 前馈输出
    };

    /**
     * @class Velocity
     * @brief 速度前馈控制器
     * 
     * 根据目标速度的变化率计算速度前馈补偿量
     */
    class Velocity
    {
        public:
            /**
             * @brief 构造函数
             * @param k_vel_ 速度前馈系数
             * @param control_cycle_ 控制周期(s)
             */
            Velocity(float k_vel_, float control_cycle_)
            {
                last_target_angle = 0.0f;
                ff_velocity = 0.0f;
                k_vel = k_vel_;
                control_cycle = control_cycle_;
            }

            /**
             * @brief 计算速度前馈值
             * @param target_angle 目标角度 单位与角度控制器期望（反馈）同单位
             * 
             * 根据目标角度变化率计算速度前馈值
             */
            void VelocityFeedforward(float target_angle)
            {
                // 计算速度变化率
                ff_velocity = (target_angle - last_target_angle) / control_cycle;
                // 计算前馈输出
                feedforward = k_vel * ff_velocity;
                // 更新上次目标速度
                last_target_angle = target_angle;
            }

            /**
             * @brief 获取前馈输出值
             * @return 前馈输出值 单位与角度控制器具有相同的量纲维度
             */
            float getFeedforward()
            {
                return feedforward;
            }

        private:
            float last_target_angle;      // 上一次的目标角度
            float control_cycle;          // 控制周期
            float k_vel;                  // 前馈系数
            float ff_velocity;            // 速度
            float feedforward;            // 前馈输出
    };

    /**
     * @class Gravity
     * @brief 重力前馈控制器
     * 
     * 用于补偿由于重力引起的系统静态误差
     */
    class Gravity
    {
        public:
            /**
             * @brief 构造函数
             * @param k_gravity_ 重力补偿系数
             * @param phi_ 相位补偿（单位 度）补偿到水平，水平为0点
             */
            Gravity(float k_gravity_, float phi_)
            {
                k_gravity = k_gravity_;
                phi = phi_;
                feedforward = 0.0f;
            }

            /**
             * @brief 计算重力前馈值
             * @param theta 当前角度（单位 度）
             * 
             * 根据当前角度和相位补偿计算重力前馈值
             */
            void GravityFeedforward(float theta)
            {
                feedforward = k_gravity * cosf((theta + phi) * 3.1415926f / 180.0f);
            }

            /**
             * @brief 获取前馈输出值
             * @return 前馈输出值
             */
            float getFeedforward()
            {
                return feedforward;
            }

        private:
            float k_gravity;    // 重力系数
            float phi;          // 相位补偿（单位 度） 补偿到水平，水平为0点
            float feedforward;  // 前馈输出
    };

    /**
     * @class Friction
     * @brief 摩擦力前馈控制器
     * 
     * 根据目标值的方向提供恒定的摩擦力补偿
     */
    class Friction
    {
        public:
            /**
             * @brief 构造函数
             * @param friction_value_ 摩擦力补偿值 与控制器输出同量纲
             */
            Friction(float friction_value_)
            {
                friction_value = friction_value_;
                feedforward = 0.0f;
            }

            /**
             * @brief 计算摩擦力前馈值
             * @param target 目标值 无单位限制
             * 
             * 根据目标值正负方向施加摩擦力补偿
             */
            void FrictionFeedforward(float target)
            {
                if (target > 0.0f) {
                    feedforward = friction_value;
                } else if (target < 0.0f) {
                    feedforward = -friction_value;
                } else {
                    feedforward = 0.0f;
                }
            }

            /**
             * @brief 获取前馈输出值
             * @return 前馈输出值 
             */
            float getFeedforward()
            {
                return feedforward;
            }

        private:
            float friction_value;   // 摩擦力补偿值
            float feedforward;      // 前馈输出
    };

    /**
     * @class GimbalFullCompensation
     * @brief 云台全补偿前馈控制器
     * 
     * 结合了基于速度的摩擦力前馈（带零速死区线性化处理）与基于模型差分的加速度前馈，
     * 用于克服云台转动时的摩擦阻力和惯性迟滞，提供极高的动态跟踪性能。
     */
    class GimbalFullCompensation
    {
        public:
            /**
             * @brief 构造函数
             * @param kJ 转动惯量前馈系数，需要调整到与实际J相等
             * @param dt 控制周期(s)
             * @param viscousfriction 粘性摩擦系数
             * @param coulombfriction 库伦摩擦系数
             */
            GimbalFullCompensation(float kJ = 0.0f, float dt = 0.001f, float viscousfriction = 0.0f, float coulombfriction = 0.0f)
                : k_J(kJ), control_dt(dt), torque(0.0f), friction(0.0f),
                  acc_feedforward(0.0f), last_ref_velocity(0.0f), filtered_acc(0.0f), 
                  ViscousFriction(viscousfriction), CoulombFriction(coulombfriction){}

            /**
             * @brief 摩擦力 + 惯量前馈
             * @param feedback_velocity 反馈速度(RPM)，用于摩擦力补偿 k*θ̇ + sgn(θ̇)*fc
             * @param ref_velocity 期望速度(RPM)，用于差分求角加速度 θ̈_r
             * 
             * 输出: friction = k_J * θ̈_r + k * θ̇ + sgn(θ̇) * fc
             */
            void MomentOfInertiaTuning(float feedback_velocity, float ref_velocity)
            {
                // 摩擦力补偿: k * θ̇ + sgn(θ̇) * fc
                // 使用平滑过渡代替sgn，消除零速附近抖颤，也就是零点线性化。
                float deadzone = 3.0f;  // 死区阈值(RPM)，在此范围内线性过渡
                float sgn;
                if (feedback_velocity > deadzone)
                    sgn = 1.0f;
                else if (feedback_velocity < -deadzone)
                    sgn = -1.0f;
                else
                    sgn = feedback_velocity / deadzone;  // 线性过渡
                friction = ViscousFriction * feedback_velocity + sgn * CoulombFriction;

                // 角加速度前馈: k_J * θ̈_r (对期望速度差分 + 低通滤波)
                float ref_acc_raw = (ref_velocity - last_ref_velocity) / control_dt;
                last_ref_velocity = ref_velocity;
                // 一阶低通滤波（0.1~0.3之间调整）
                float alpha = 0.15f;
                filtered_acc = filtered_acc * (1.0f - alpha) + ref_acc_raw * alpha;
                acc_feedforward = k_J * filtered_acc;

                // 总前馈 = 摩擦力 + 惯量前馈
                torque = friction + acc_feedforward;
            }

            /**
             * @brief 获取总扭矩输出
             * @return 扭矩值
             */
            float getTorque()
            {
                return torque;
            }

            /**
             * @brief 获取摩擦力前馈输出
             * @return 摩擦力前馈量
             */
            float getFriction()
            {
                return friction;
            }

            /**
             * @brief 获取角加速度前馈输出
             * @return 角加速度前馈量
             */
            float getAccFeedforward()
            {
                return acc_feedforward;
            }

            /**
             * @brief 设置转动惯量前馈系数
             * @param kJ 新的k_J值
             */
            void setKJ(float kJ) { k_J = kJ; }

        private:
            float k_J;                // 转动惯量前馈系数
            float control_dt;         // 控制周期(s)
            float torque;             // 总扭矩输出
            float friction;           // 摩擦力前馈输出
            float acc_feedforward;    // 角加速度前馈量
            float last_ref_velocity;  // 上一次期望速度
            float filtered_acc;       // 低通滤波后的角加速度
            float ViscousFriction;    // 粘性摩擦力系数
            float CoulombFriction;    // 库伦摩擦力系数
    };

    /**
     * @class UDE
     * @brief 不确定性与扰动估计器 (Uncertainty and Disturbance Estimator)
     * 
     * 用于实时观测并补偿系统所受到的外界扰动（如底盘甩尾、碰撞干扰），
     * 通过比较理论模型产生的加速度与实际加速度，反推干扰量并前馈补偿。
     */
    class UDE
    {
        public:
            /**
             * @brief 构造函数
             * @param a_ 系统特征系数a
             * @param b_ 系统特征系数b（控制增益）
             */
            UDE(float a_, float b_)
            {
                a = a_;
                b = b_;
                Feedback = 0.0f;
                Output = 0.0f;
                X_dot = 0.0f;
                Input = 0.0f;
            }

            /**
             * @brief 重置观测器状态
             * 
             * 在切换状态或重新启动时调用，防止历史过时的观测数据产生错误补偿
             */
            void ResetState()
            {
                Output = 0.0f;
                X_dot = 0.0f;
                Input = 0.0f;
            }

            /**
             * @brief UDE 观测器状态更新
             * @param Input_ 上一控制周期的系统输入（遵守物理因果律的历史输出量）
             * @param X_dot_ 当前系统状态的导数观测值（如当前实际的角加速度）
             */
            void UDE_Update(float Input_, float X_dot_)
            {
                Input = Input_;
                X_dot = X_dot_;
                
                Output = 1/b * (X_dot - a * Feedback - b * Input);
            }

            /**
             * @brief 获取 UDE 扰动补偿输出
             * @return 扰动补偿前馈量
             */
            float getOutput()
            {
                return Output;
            }

        private:
            float Output;   // UDE 扰动补偿输出
            float X_dot;    // 当前系统状态的导数观测值
            float a, b;     // 系统特征系数
            float Input;    // 上一控制周期的系统输入
            float Feedback; // 上一控制周期的系统状态
    };
}

#endif
