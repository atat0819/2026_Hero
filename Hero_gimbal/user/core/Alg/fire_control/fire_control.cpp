#include "fire_control.hpp"

#include <math.h>

namespace ALG
{
namespace FireControl
{
namespace
{
constexpr uint32_t kPlanPeriodMs = 100U;
// 浮点零阈值，用于保护除法和无意义的规划参数。
constexpr float kMinPositive = 0.001f;

float Clamp(float value, float lower, float upper)
{
    if (value < lower)
    {
        return lower;
    }
    if (value > upper)
    {
        return upper;
    }
    return value;
}
} // namespace

FireControlConfig::FireControlConfig()
    : projectile_heat(40.0f),
      max_fire_hz(3.0f),
      safety_reserve_shots(3.0f),
      referee_timeout_ms(500U),
      max_plan_ticks(300U)
{
}

HeatLimitedFireControl::HeatLimitedFireControl(const FireControlConfig &config)
    : config_(config)
{
    Init();
}

void HeatLimitedFireControl::Init()
{
    judge_data_valid_ = false;
    continuous_trigger_active_ = false;
    current_heat_ = 0.0f;
    heat_limit_ = 0.0f;
    remaining_heat_ = 0.0f;
    cooling_rate_ = 0.0f;
    ResetPlan();
}

void HeatLimitedFireControl::SetProjectileHeat(float projectile_heat)
{
    config_.projectile_heat = projectile_heat;
    ResetPlan();
}

void HeatLimitedFireControl::SetMaxFireHz(float max_fire_hz)
{
    config_.max_fire_hz = max_fire_hz;
    ResetPlan();
}

void HeatLimitedFireControl::UpdateJudgeData(bool data_received,
                                              uint16_t cooling_rate,
                                              uint16_t heat_limit,
                                              uint16_t current_heat,
                                              uint32_t last_update_tick_ms,
                                              uint32_t now_tick_ms)
{
    cooling_rate_ = static_cast<float>(cooling_rate);
    heat_limit_ = static_cast<float>(heat_limit);
    current_heat_ = static_cast<float>(current_heat);

    // 热量数据不可信时宁可禁射：既检查通信时效，也拒绝“当前热量大于上限”的异常帧。
    judge_data_valid_ = data_received &&
                        IsConfigurationValid() &&
                        IsRefereeDataFresh(now_tick_ms, last_update_tick_ms) &&
                        current_heat_ <= heat_limit_;

    // 只有有效裁判数据才允许使用剩余热量参与规划，避免旧数据驱动继续发射。
    remaining_heat_ = judge_data_valid_ ? (heat_limit_ - current_heat_) : 0.0f;

    if (!judge_data_valid_)
    {
        continuous_trigger_active_ = false;
        ResetPlan();
    }
}

void HeatLimitedFireControl::Update(bool continuous_trigger_active, uint32_t now_tick_ms)
{
    if (!judge_data_valid_)
    {
        continuous_trigger_active_ = false;
        ResetPlan();
        return;
    }

    if (!continuous_trigger_active)
    {
        continuous_trigger_active_ = false;
        ResetPlan();
        return;
    }

    if (!continuous_trigger_active_)
    {
        // 新一次按住：必须依据本次热量快照重新计算，不能复用上一次的 ShootTime。
        continuous_trigger_active_ = true;
        StartPlan(now_tick_ms);
    }

    if (!plan_active_ || remaining_heat_ <= 0.0f)
    {
        target_fire_hz_ = 0.0f;
        return;
    }

    // 使用 HAL tick 的实际间隔，而不是假设控制循环严格为 5 ms。
    while (static_cast<uint32_t>(now_tick_ms - last_plan_tick_ms_) >= kPlanPeriodMs)
    {
        last_plan_tick_ms_ += kPlanPeriodMs;
        Advance100ms();
    }
}

bool HeatLimitedFireControl::CanStartShot() const
{
    const float reserve_heat = config_.safety_reserve_shots * config_.projectile_heat;
    // 需要至少再容纳一发，预留量不用于正常发射。
    return judge_data_valid_ && remaining_heat_ >= (reserve_heat + config_.projectile_heat);
}

bool HeatLimitedFireControl::IsJudgeDataValid() const
{
    return judge_data_valid_;
}

float HeatLimitedFireControl::GetTargetFireHz() const
{
    return target_fire_hz_;
}

float HeatLimitedFireControl::GetCurrentHeat() const
{
    return current_heat_;
}

float HeatLimitedFireControl::GetHeatLimit() const
{
    return heat_limit_;
}

float HeatLimitedFireControl::GetRemainingHeat() const
{
    return remaining_heat_;
}

float HeatLimitedFireControl::GetCoolingRate() const
{
    return cooling_rate_;
}

float HeatLimitedFireControl::GetPlannedFireHz() const
{
    return planned_fire_hz_;
}

uint32_t HeatLimitedFireControl::GetShootTime() const
{
    return shoot_time_;
}

uint32_t HeatLimitedFireControl::GetShootTimeLimit() const
{
    return shoot_time_limit_;
}

void HeatLimitedFireControl::ResetPlan()
{
    // 松开、数据失效或重配参数后均调用这里，确保下一次按住必定重新规划。
    plan_active_ = false;
    planned_fire_hz_ = 0.0f;
    target_fire_hz_ = 0.0f;
    last_plan_tick_ms_ = 0U;
    shoot_time_ = 0U;
    shoot_time_limit_ = 0U;
}

void HeatLimitedFireControl::StartPlan(uint32_t now_tick_ms)
{
    ResetPlan();
    last_plan_tick_ms_ = now_tick_ms;

    if (!CanStartShot())
    {
        return;
    }

    const float d = config_.projectile_heat; // 单发热量，热量/发
    const float a = cooling_rate_;            // 冷却速度，热量/s
    const float c_limit = config_.max_fire_hz; // 机械频率上限，发/s
    const float reserve_heat = config_.safety_reserve_shots * d;
    const float m = remaining_heat_ - reserve_heat; // 可消耗热量，已扣除安全余量
    const float steady_fire_hz = a / d;

    plan_active_ = true;

    // 当冷却能力已覆盖机械频率上限时，不需要消耗热量预算来做前馈计划。
    if (c_limit <= steady_fire_hz + kMinPositive)
    {
        target_fire_hz_ = c_limit;
        planned_fire_hz_ = c_limit;
        return;
    }

    // d*c*t = m + a*(t - 0.1), t = n/10。
    // n 取上整，保证实际输出不高于机械频率上限。
    const float denominator = d * c_limit - a;
    const float numerator = 10.0f * m - a;
    if (denominator <= kMinPositive || numerator <= 0.0f)
    {
        plan_active_ = false;
        return;
    }

    uint32_t planned_ticks = static_cast<uint32_t>(ceilf(numerator / denominator));
    if (planned_ticks == 0U)
    {
        planned_ticks = 1U;
    }
    if (planned_ticks > config_.max_plan_ticks)
    {
        // 避免异常参数形成过长的单次计划；到期后转为可持续冷却频率。
        planned_ticks = config_.max_plan_ticks;
    }

    shoot_time_limit_ = planned_ticks;
    planned_fire_hz_ = (10.0f * m - a) / (d * static_cast<float>(planned_ticks)) + steady_fire_hz;
    planned_fire_hz_ = Clamp(planned_fire_hz_, 0.0f, c_limit);
    target_fire_hz_ = planned_fire_hz_;
}

void HeatLimitedFireControl::Advance100ms()
{
    if (!plan_active_)
    {
        target_fire_hz_ = 0.0f;
        return;
    }

    if (remaining_heat_ <= 0.0f)
    {
        target_fire_hz_ = 0.0f;
        plan_active_ = false;
        return;
    }

    if (shoot_time_ < shoot_time_limit_)
    {
        ++shoot_time_;
        target_fire_hz_ = planned_fire_hz_;
    }
    else
    {
        // 前馈时段结束后只按可持续冷却频率发射，避免继续消耗热量预算。
        target_fire_hz_ = Clamp(cooling_rate_ / config_.projectile_heat,
                                0.0f,
                                config_.max_fire_hz);
    }
}

bool HeatLimitedFireControl::IsConfigurationValid() const
{
    return config_.projectile_heat > kMinPositive &&
           config_.max_fire_hz > kMinPositive &&
           config_.safety_reserve_shots >= 0.0f &&
           config_.referee_timeout_ms > 0U &&
           config_.max_plan_ticks > 0U;
}

bool HeatLimitedFireControl::IsRefereeDataFresh(uint32_t now_tick_ms,
                                                 uint32_t last_update_tick_ms) const
{
    return static_cast<uint32_t>(now_tick_ms - last_update_tick_ms) <=
           config_.referee_timeout_ms;
}

} // namespace FireControl
} // namespace ALG
