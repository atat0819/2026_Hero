#ifndef POWERCONTROL
#define POWERCONTROL

#include "../User/core/Alg/PowerControl/PowerControlBase.hpp"

namespace ALG::PowerControl
{
    template <uint8_t N> class PowerControl : public PowerControlBase<N>
    {
        public:
            /**
             * @brief 构造函数，初始化所有成员变量为0，防止未定义行为
             */
            PowerControl()
            {
                GenerateElectricity = 0.0f;
                PowerConsumption = 0.0f;
                PowerTotal = 0.0f;
                PowerMax = 0.0f;
                rho = 0.0f;
                eta = 0.0f;
                A = 0.0f;
                B = 0.0f;
                C = 0.0f;
                Delta = 0.0f;
                for(int i = 0; i < N; i++)
                {
                    Current[i] = 0.0f;
                    a[i] = 0.0f;
                    b[i] = 0.0f;
                    c[i] = 0.0f;
                    delta[i] = 0.0f;
                }
            }

            /**
             * @brief 功率衰减法 (方案A: 局部衰减)
             * @note 通过计算功率衰减系数 rho，对每个耗电电机分别解一元二次方程来限制功率。
             *       该方法可能改变不同电机间的受力比例。
             * 
             * @param I 原始电流数组
             * @param V 当前转速数组
             * @param K 功率模型系数数组 (k0-k5)
             * @param CorrectionConstant 功率补偿常数 (e.g. 3*k0)
             * @param PowerMax 最大功率限制
             */
            void AttenuatedPower(float *I, float *V, float *K, float CorrectionConstant, float PowerMax)
            {
                GenerateElectricity = 0.0f;
                PowerConsumption = 0.0f;
                PowerTotal = 0.0f;

                // 1. 遍历计算原始总功率
                for (uint8_t i = 0; i < N; i++)
                {
                    this->SetMotorParameters(i, I[i], V[i], K);
                    if(this->GetPowerCalculate(i) < 0)
                    {
                        GenerateElectricity += this->GetPowerCalculate(i);
                    }
                    else 
                    {
                        PowerConsumption += this->GetPowerCalculate(i);
                    }

                    PowerTotal += this->GetPowerCalculate(i);
                }
                PowerTotal = PowerTotal + CorrectionConstant;

                // 2. 判断是否超限
                if(PowerTotal < PowerMax)
                {
                    for(uint8_t i = 0; i < N; i++)
                    {
                        Current[i] = I[i];
                    }
                }
                else
                {
                    // 3. 计算衰减系数 rho
                    if (PowerConsumption > 1e-4f) 
                    {
                        rho = ((PowerMax - CorrectionConstant) - GenerateElectricity) / PowerConsumption;
                    } 
                    else 
                    {
                        rho = 1.0f; 
                    }
                    if(rho < 0) rho = 1;
                    if(rho > 1) rho = 1;

                    // 4. 应用衰减：耗电电机按比例方程求根
                    for(int i = 0; i < N; i++)
                    {
                        a[i] = K[4];
                        b[i] = K[1] + K[3]*V[i];
                        c[i] = (-rho)*this->GetPowerCalculate(i) + K[0] + K[2]*V[i] + K[5]*V[i]*V[i];
                        delta[i] = b[i]*b[i] - 4*a[i]*c[i];

                        if(delta[i] < 0)
                        {
                            Current[i] = 0.0f;
                        }
                        else if(delta[i] == 0)
                        {
                            Current[i] = -b[i]/(2*a[i]);
                        }   
                        else
                        {
                            float X1 = (-b[i] + sqrtf(delta[i])) / (2*a[i]);
                            float X2 = (-b[i] - sqrtf(delta[i])) / (2*a[i]);
                            float Distance1 = fabsf(I[i] - X1);
                            float Distance2 = fabsf(I[i] - X2);
                            if(Distance1 < Distance2)
                            {  
                                Current[i] = X1;
                            }
                            else
                            {
                                Current[i] = X2;
                            }
                        }

                    }
                }
            }

            /**
             * @brief 电流衰减法 (方案B: 全局衰减)
             * @note 推荐使用。通过解一个全局一元二次方程，求出统一的电流衰减比例 eta。
             *       该方法能保持所有电机的电流比例不变，从而保证底盘运动方向不偏移。
             * 
             * @param I 主要控制电流 (如PID输出)
             * @param V 当前转速
             * @param K 功率模型系数
             * @param I_other 辅助电流 (如前馈)，总目标电流 = I + I_other
             * @param CorrectionConstant 功率补偿常数
             * @param PowerMax 最大功率限制
             */
            void DecayingCurrent(float *I, float *V, float *K, float *I_other, float CorrectionConstant, float PowerMax)
            {
                GenerateElectricity = 0.0f;
                PowerConsumption = 0.0f;
                PowerTotal = 0.0f;
                A = 0.0f;
                B = 0.0f;
                C = 0.0f;

                // 1. 遍历计算原始功率，同时准备全局方程参数 A, B, C
                for (uint8_t i = 0; i < N; i++)
                {
                    this->SetMotorParameters(i, I[i], V[i], K); // 这里更新了基类的功率计算
                    if(this->GetPowerCalculate(i) < 0)
                    {
                        GenerateElectricity += this->GetPowerCalculate(i);
                    }
                    else 
                    {
                        PowerConsumption += this->GetPowerCalculate(i);
                    }

                    PowerTotal += this->GetPowerCalculate(i);
                }
                PowerTotal = PowerTotal + CorrectionConstant;

                // 2. 判断是否超限
                if(PowerTotal < PowerMax)
                {
                    for(uint8_t i = 0; i < N; i++)
                    {
                        Current[i] = I[i];
                    }
                }
                else
                {
                    // 3. 构建全局方程 A*eta^2 + B*eta + C = 0
                    for(int i = 0; i < N; i++)
                    {
                        A += K[4] * (I[i]+I_other[i]) * (I[i]+I_other[i]);
                        B += K[1] * (I[i]+I_other[i]) + K[3] * V[i] * (I[i]+I_other[i]);
                        C += K[0] + K[2]*V[i] + K[5]*V[i]*V[i];
                    }
                    
                    // 4. 常数项需要减去(最大功率+补偿值)，即等于0
                    C = C - (PowerMax + CorrectionConstant);
                    Delta = B*B - 4*A*C;

                    if(Delta < 0)
                    {
                        eta = 0.0f;
                    }
                    else
                    {
                        float X1 = (-B + sqrtf(Delta)) / (2*A);
                        float X2 = (-B - sqrtf(Delta)) / (2*A);

                        bool X1_ok = (X1 >= 0.0f && X1 <= 1.0f);
                        bool X2_ok = (X2 >= 0.0f && X2 <= 1.0f);

                        if(X1_ok && X2_ok) eta = (X1 > X2) ? X1 : X2;
                        else if(X1_ok) eta = X1;
                        else if(X2_ok) eta = X2;
                        else eta = 0.0f; // 无合理解
                    }

                    // 5. 应用全局衰减
                    for(int i = 0; i < N; i++)
                    {
                        Current[i] = eta * I[i];
                    }
                }
            }

            /**
             * @brief 获取当前计算后的限制电流
             * @param i 电机编号
             * @return float 
             */
            float getCurrentCalculate(uint8_t i)
            {
                return Current[i];
            }

            /**
             * @brief 获取当前计算的总功率（包含补偿值）
             */
            float getPowerTotal()
            {
                return PowerTotal;
            }

        private:
            float GenerateElectricity;  // 发电电机功率和
            float PowerConsumption;     // 耗电电机功率和
            float PowerTotal;           // 预测总功率
            float PowerMax;             // 最大功率限制
            float rho;                  // 功率衰减因数 (方案A用)
            float eta;                  // 电流衰减因数 (方案B用)
            float Current[N];           // 最终计算出的控制电流
            
            // 方案A的中间变量 (针对每个电机)
            float a[N];                    
            float b[N];
            float c[N];
            float delta[N];             // b^2-4ac
            
            // 方案B的全局中间变量
            float A;
            float B;
            float C;
            float Delta;                // b^2-4ac
            

    };

    /**
     * @brief 能量环控制类
     * @details 根据当前能量状态动态调整底盘最大功率限制，实现智能功率分配
     *          通过设置富足线和贫困线来判断当前能量状态，并据此调整功率输出策略
     */
    class EnergyRing
    {
        public:
            /**
             * @brief 构造函数
             * @param abundanceline 富足线阈值 (J) - 当能量高于此值时认为能量充足
             * @param povertyline 贫困线阈值 (J) - 当能量低于此值时认为能量不足
             * @param pmin_restriction 最小功率限制 (W) - 防止功率过低导致系统死锁
             */
            EnergyRing(float abundanceline, float povertyline, float pmin_restriction)
            {
                PowerMax = 0.0f;
                AbundanceLine = abundanceline;
                PovertyLine = povertyline;
                Pmin_restriction = pmin_restriction;
            }

            /**
             * @brief 能量环主控制算法
             * @details 根据裁判系统能量信息和操作状态，动态计算当前允许的最大功率
             * 
             * @param AbundanceOut 富足环输出
             * @param PovertyOut 贫困环输出
             * @param P_referee 裁判系统允许的最大功率 (W)
             * @param CurrentEnergy 当前剩余能量 (J)
             * @param isShift 是否按下Shift键 (加速模式)
             * @param isPower 是否处于充电模式 (忽略能量环逻辑，强行80%给底盘，20%给超电)
             * 
             * @note 功率分配优先级：
             *       1. 充电模式 → 固定80%功率
             *       2. 能量不足(<贫困线) → 强制80%功率
             *       3. 加速模式（shift按下） → 使用贫困功率限制
             *       4. 能量充足(≥富足线) → 使用富足功率限制
             *       5. 正常模式 → 满等级功率运行
             *       6. 所有情况都需满足最小功率保障
             */
            void energyring(float AbundanceOut, float PovertyOut, float P_referee, float CurrentEnergy, bool isShift, bool isPower)
            {
                if(isPower)
                {
                    // 充电模式：固定使用80%的裁判功率
                    PowerMax = 0.8f * P_referee;
                }
                else
                {
                    // 计算两种功率限制值
                    float PowerMax_abundance = P_referee - AbundanceOut;  // 富足状态下的功率上限
                    float PowerMax_poverty = P_referee - PovertyOut;      // 贫困状态下的功率上限
                    
                    // 1. 如果剩余能量小于贫困线，强制限制为80%
                    if (CurrentEnergy < PovertyLine)
                    {
                        PowerMax = 0.8f * P_referee;
                    }
                    // 2. 如果按下Shift (加速模式)
                    else if (isShift)
                    {
                        PowerMax = PowerMax_poverty;
                    }
                    // 3. Shift未按下的一般情况
                    else
                    {
                        if (CurrentEnergy >= AbundanceLine)
                        {
                            // 能量充足：使用富足功率限制
                            PowerMax = PowerMax_abundance;
                        }
                        else // 在贫困线和富足线之间
                        {
                            // 能量中等：满等级功率运行
                            PowerMax = P_referee;
                        }
                    }

                    // 最小功率保障机制：防止功率过低导致系统死锁
                    Pmin_restriction = 0.7f * P_referee;
                    if(PowerMax < Pmin_restriction)
                    {
                        PowerMax = Pmin_restriction;
                    }
                }
            }

            /**
             * @brief 获取富足线阈值
             * @return float 富足线能量值 (J)
             */
            float GetAbundanceLine()
            {
                return AbundanceLine;
            }

            /**
             * @brief 获取贫困线阈值
             * @return float 贫困线能量值 (J)
             */
            float GetPovertyLine()
            {
                return PovertyLine;
            }

            /**
             * @brief 获取当前计算的最大功率限制
             * @return float 最大功率限制值 (W)
             */
            float GetPowerMax()
            {
                return PowerMax;
            }

        private:
            float PowerMax;             // 当前计算得出的最大功率限制 (W)
            float AbundanceLine;        // 富足线阈值 (J) - 能量充足判断标准
            float PovertyLine;          // 贫困线阈值 (J) - 能量不足判断标准
            float Pmin_restriction;     // 最小功率限制 (W) - 防止死锁的安全下限
    };

    /**
     * @brief 上限功率与剩余能量策略管理类
     */
    class PowerControlStrategy
    {
        public:
            /**
             * @brief 构造函数，初始化功率控制策略
             * @param abundanceline 富足线阈值 (J)，用于超级电容满能量的判断基准
             */
            PowerControlStrategy(float abundanceline)
            {
                last_valid_limit = 60.0f; // 默认功率限制 (W)
                input_limit = 60.0f;      // 初始功率限制 (W)
                input_energy = 0.0f;      // 初始能量反馈 (J)
                AbundanceLine = abundanceline;
            }

            /**
             * @brief 更新 上限功率 与 剩余能量 策略
             * @param isSupercapOnline 超电在线状态
             * @param isRefereeOnline 裁判系统在线状态
             * @param referee_limit 裁判系统功率限制
             * @param referee_buffer 裁判系统缓冲能量
             * @param supercap_energy 超电剩余能量
             */
            void Update(bool isSupercapOnline, bool isRefereeOnline, float referee_limit, float referee_buffer, float supercap_energy)
            {
                // 若裁判系统在线，更新裁判系统功率上限
                if (isRefereeOnline) last_valid_limit = referee_limit;

                // 1. 电容连接，裁判断连
                if (isSupercapOnline && !isRefereeOnline)
                {
                    input_limit = last_valid_limit;
                    input_energy = supercap_energy;
                }
                // 2. 电容断连，裁判连接
                else if (!isSupercapOnline && isRefereeOnline)
                {
                    // 隐形能量池逻辑：若缓冲满(接近60J)，认为电容有电，允许爆发
                    if (referee_buffer > 55.0f)
                    {
                        input_limit = last_valid_limit; 
                        input_energy = AbundanceLine; // 假装能量充足，骗过EnergyRing
                    }
                    else
                    {
                        input_limit = last_valid_limit; // 正常功率
                        input_energy = 0.0f; // 假装能量耗尽，触发保护
                    }
                }
                // 3. 双断连
                else if (!isSupercapOnline && !isRefereeOnline)
                {
                    input_limit = last_valid_limit;
                    input_energy = 0.0f;
                }
                // 4. 正常情况
                else
                {
                    input_limit = referee_limit;
                    input_energy = supercap_energy;
                }
            }

            /**
             * @brief 获取当前使用的功率上限
             * @return float 功率上限 (W)
             */
            float GetInputLimit() const { return input_limit; }

            /**
             * @brief 获取当前使用的能量反馈值
             * @return float 能量值 (J)
             */
            float GetInputEnergy() const { return input_energy; }

        private:
            float last_valid_limit;     // 最后一次有效的功率上限 (W)
            float input_limit;          // 当前使用的功率上限 (W)
            float input_energy;         // 当前使用的能量反馈 (J)
            float AbundanceLine;        // 富足线 (J)，用于伪造满能量状态
    };
}

#endif //  POWERCONTROL
