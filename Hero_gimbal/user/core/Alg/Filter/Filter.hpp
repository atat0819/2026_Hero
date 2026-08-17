#ifndef FILTER_HPP
#define FILTER_HPP

/*  =========================== Kalman Filter ===========================  */
class KalmanFilter
{
private:
    float X_last;
    float X_mid;
    float X_now;
    float P_mid;
    float P_now;
    float P_last;
    float kg;
    float A;
    float Q;
    float R;
    float H;

public:
    KalmanFilter(float T_Q = 0.0001f, float T_R = 0.0001f);

    float filter(float dat);
    void reinit(float T_Q, float T_R);

    float getState() const;
    float getPrediction() const;
    float getGain() const;
};

/*  =========================== Tracking Differentiator Filter ===========================  */
class TDFilter
{
private:
    float v1;
    float v2;
    float R;
    float H;

public:
    TDFilter(float init_R = 100.0f, float init_H = 0.01f);

    float filter(float Input);
    void setParams(float new_R, float new_H);

    float getDerivative() const;
};

/*  =========================== First-order Low-pass Filter ===========================  */
class LPFFilter
{
private:
    float Last_Out;
    float Ratio;

public:
    LPFFilter(float ratio = 0.5f);

    float filter(float Input);
    void setRatio(float ratio);

    float getOutput() const;
    float getRatio() const;
};

/*  =========================== Limit Filter ===========================  */
class LMFFilter
{
private:
    float Last_Out;
    float Limit_Ratio;

public:
    LMFFilter(float limit_ratio = 1.0f);

    float filter(float Input);
    void setLimit(float limit_ratio);

    float getOutput() const;
    float getLimitRatio() const;
};

/*  =========================== Second-order Low-pass Filter ===========================  */
class SecondOrderLPFFilter
{
private:
    float b0;
    float b1;
    float b2;
    float a1;
    float a2;
    float input_1;
    float input_2;
    float output_1;
    float output_2;
    float output;
    float cutoff_frequency;
    float sample_period;
    float damping;

public:
    SecondOrderLPFFilter(float cutoff_frequency_hz = 0.0f,
                         float sample_period_s = 0.001f,
                         float damping_ratio = 0.70710678f);

    void configure(float cutoff_frequency_hz, float sample_period_s,
                   float damping_ratio = 0.70710678f);
    float filter(float input);
    void reset(float initial_value = 0.0f);

    float getOutput() const;
    float getCutoffFrequency() const;
    float getSamplePeriod() const;
    float getDampingRatio() const;

    void Configure(float cutoff_frequency_hz, float sample_period_s,
                   float damping_ratio = 0.70710678f)
    {
        configure(cutoff_frequency_hz, sample_period_s, damping_ratio);
    }

    float Filter(float input)
    {
        return filter(input);
    }

    void Reset(float initial_value = 0.0f)
    {
        reset(initial_value);
    }

    float GetOutput() const { return getOutput(); }
    float GetCutoffFrequency() const { return getCutoffFrequency(); }
    float GetSamplePeriod() const { return getSamplePeriod(); }
    float GetDampingRatio() const { return getDampingRatio(); }
};

#endif
