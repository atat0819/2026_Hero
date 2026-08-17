#include "Filter.hpp"
#include <math.h>

namespace
{
    constexpr float SECOND_ORDER_LPF_PI = 3.141592653589f;

    float clampRatio(float ratio)
    {
        return (ratio >= 0.0f && ratio <= 1.0f) ? ratio : 0.5f;
    }
}

/*  =========================== Kalman Filter ===========================  */
KalmanFilter::KalmanFilter(float T_Q, float T_R)
{
    reinit(T_Q, T_R);
}

float KalmanFilter::filter(float dat)
{
    X_mid = A * X_last;
    P_mid = A * P_last + Q;

    kg = P_mid / (P_mid + R);
    X_now = X_mid + kg * (dat - X_mid);
    P_now = (1.0f - kg) * P_mid;

    P_last = P_now;
    X_last = X_now;

    return X_now;
}

void KalmanFilter::reinit(float T_Q, float T_R)
{
    X_last = 0.0f;
    X_mid = 0.0f;
    X_now = 0.0f;
    P_mid = 0.0f;
    P_now = 0.0f;
    P_last = 1000.0f;
    kg = 0.0f;
    A = 1.0f;
    Q = T_Q;
    R = T_R;
    H = 1.0f;
}

float KalmanFilter::getState() const
{
    return X_now;
}

float KalmanFilter::getPrediction() const
{
    return X_mid;
}

float KalmanFilter::getGain() const
{
    return kg;
}

/*  =========================== Tracking Differentiator Filter ===========================  */
TDFilter::TDFilter(float init_R, float init_H)
{
    v1 = 0.0f;
    v2 = 0.0f;
    R = init_R;
    H = init_H;
}

float TDFilter::filter(float Input)
{
    float fh = -R * R * (v1 - Input) - 2.0f * R * v2;

    v1 += v2 * H;
    v2 += fh * H;

    return v1;
}

void TDFilter::setParams(float new_R, float new_H)
{
    R = new_R;
    H = new_H;
}

float TDFilter::getDerivative() const
{
    return v2;
}

/*  =========================== First-order Low-pass Filter ===========================  */
LPFFilter::LPFFilter(float ratio)
{
    Last_Out = 0.0f;
    Ratio = clampRatio(ratio);
}

float LPFFilter::filter(float Input)
{
    float Out = Ratio * Input + (1.0f - Ratio) * Last_Out;
    Last_Out = Out;
    return Out;
}

void LPFFilter::setRatio(float ratio)
{
    Ratio = clampRatio(ratio);
}

float LPFFilter::getOutput() const
{
    return Last_Out;
}

float LPFFilter::getRatio() const
{
    return Ratio;
}

/*  =========================== Limit Filter ===========================  */
LMFFilter::LMFFilter(float limit_ratio)
{
    Last_Out = 0.0f;
    Limit_Ratio = limit_ratio;
}

float LMFFilter::filter(float Input)
{
    float Error = fabsf(Input - Last_Out);

    if (Error > Limit_Ratio)
    {
        Input = Last_Out;
    }

    Last_Out = Input;
    return Input;
}

void LMFFilter::setLimit(float limit_ratio)
{
    Limit_Ratio = limit_ratio;
}

float LMFFilter::getOutput() const
{
    return Last_Out;
}

float LMFFilter::getLimitRatio() const
{
    return Limit_Ratio;
}

/*  =========================== Second-order Low-pass Filter ===========================  */
SecondOrderLPFFilter::SecondOrderLPFFilter(float cutoff_frequency_hz,
                                           float sample_period_s,
                                           float damping_ratio)
    : b0(1.0f), b1(0.0f), b2(0.0f), a1(0.0f), a2(0.0f),
      input_1(0.0f), input_2(0.0f), output_1(0.0f), output_2(0.0f),
      output(0.0f), cutoff_frequency(0.0f), sample_period(0.0f),
      damping(0.70710678f)
{
    configure(cutoff_frequency_hz, sample_period_s, damping_ratio);
}

void SecondOrderLPFFilter::configure(float cutoff_frequency_hz,
                                     float sample_period_s,
                                     float damping_ratio)
{
    cutoff_frequency = cutoff_frequency_hz;
    sample_period = sample_period_s;
    damping = damping_ratio;

    const float nyquist_frequency =
        sample_period_s > 0.0f ? 0.5f / sample_period_s : 0.0f;
    if (cutoff_frequency_hz <= 0.0f || sample_period_s <= 0.0f ||
        damping_ratio <= 0.0f || cutoff_frequency_hz >= nyquist_frequency)
    {
        b0 = 1.0f;
        b1 = 0.0f;
        b2 = 0.0f;
        a1 = 0.0f;
        a2 = 0.0f;
        reset();
        return;
    }

    const float k = tanf(SECOND_ORDER_LPF_PI *
                         cutoff_frequency_hz * sample_period_s);
    const float k_squared = k * k;
    const float normalizer =
        1.0f / (1.0f + 2.0f * damping_ratio * k + k_squared);

    b0 = k_squared * normalizer;
    b1 = 2.0f * b0;
    b2 = b0;
    a1 = 2.0f * (k_squared - 1.0f) * normalizer;
    a2 = (1.0f - 2.0f * damping_ratio * k + k_squared) * normalizer;
    reset();
}

float SecondOrderLPFFilter::filter(float input)
{
    output = b0 * input + b1 * input_1 + b2 * input_2 -
             a1 * output_1 - a2 * output_2;

    input_2 = input_1;
    input_1 = input;
    output_2 = output_1;
    output_1 = output;
    return output;
}

void SecondOrderLPFFilter::reset(float initial_value)
{
    input_1 = initial_value;
    input_2 = initial_value;
    output_1 = initial_value;
    output_2 = initial_value;
    output = initial_value;
}

float SecondOrderLPFFilter::getOutput() const
{
    return output;
}

float SecondOrderLPFFilter::getCutoffFrequency() const
{
    return cutoff_frequency;
}

float SecondOrderLPFFilter::getSamplePeriod() const
{
    return sample_period;
}

float SecondOrderLPFFilter::getDampingRatio() const
{
    return damping;
}
