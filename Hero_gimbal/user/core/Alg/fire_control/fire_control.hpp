#ifndef FIRE_CONTROL_HPP
#define FIRE_CONTROL_HPP

#include <stdint.h>

namespace ALG
{
namespace FireControl
{

/**
 * @brief 发射机构热量规划参数。
 *
 * 所有热量量纲均为裁判系统热量单位，频率单位为 Hz。默认单发热量 40
 * 沿用工程中原 Heat_Control 模块的配置；上场前必须按实际枪口和赛季规则标定。
 */
struct FireControlConfig
{
    float projectile_heat;       // d: 单发增加的热量，单位：热量/发
    float max_fire_hz;           // 机械与控制链允许的最高发射频率，单位：Hz
    float safety_reserve_shots;  // 规划中始终预留的弹数，避免裁判数据延迟时超热
    uint32_t referee_timeout_ms; // 超过该时间未收到裁判帧即视为数据失效，单位：ms
    uint32_t max_plan_ticks;     // 单次前馈规划上限，单位：100 ms tick

    FireControlConfig();
};

/**
 * @brief 基于裁判系统热量数据的连发频率规划器。
 *
 * UpdateJudgeData() 在每个控制周期更新裁判系统快照；Update() 仅在按住
 * 连发时运行，并以真实 100 ms 节拍推进 ShootTime。该类只输出目标频率，
 * 不直接操作电机或 PID。
 */
class HeatLimitedFireControl
{
public:
    explicit HeatLimitedFireControl(const FireControlConfig &config = FireControlConfig());

    /// @brief 清空裁判快照和当前规划；上电或需要强制复位时调用。
    void Init();

    /// @brief 修改单发热量参数，并使当前规划立即失效。
    void SetProjectileHeat(float projectile_heat);
    /// @brief 修改机械频率上限，并使当前规划立即失效。
    void SetMaxFireHz(float max_fire_hz);

    /**
     * @brief 更新由 CAN 接收层提供的裁判系统快照。
     * @param data_received        是否至少收到过一帧裁判数据。
     * @param cooling_rate         a: 枪管冷却速度，单位：热量/s。
     * @param heat_limit           枪管热量上限，单位：热量。
     * @param current_heat         当前枪管热量，单位：热量。
     * @param last_update_tick_ms  最近一帧裁判数据的 HAL tick，单位：ms。
     * @param now_tick_ms          当前 HAL tick，单位：ms。
     */
    void UpdateJudgeData(bool data_received,
                         uint16_t cooling_rate,
                         uint16_t heat_limit,
                         uint16_t current_heat,
                         uint32_t last_update_tick_ms,
                         uint32_t now_tick_ms);

    /**
     * @brief 推进连发规划。仅连续触发为真时输出目标频率；松开立即清空规划。
     * @param continuous_trigger_active 拨弹轮 FSM 是否处在实际连发请求中。
     * @param now_tick_ms 当前 HAL tick，内部按真实 100 ms 周期计数。
     */
    void Update(bool continuous_trigger_active, uint32_t now_tick_ms);

    // 用于在拨弹轮 FSM 进入正向发射前做安全门控；反转退弹不受此限制。
    bool CanStartShot() const;
    bool IsJudgeDataValid() const;
    float GetTargetFireHz() const;

    // 调试量，单位依次为热量、热量、Hz、100 ms tick。
    float GetCurrentHeat() const;
    float GetHeatLimit() const;
    float GetRemainingHeat() const;
    float GetCoolingRate() const;
    float GetPlannedFireHz() const;
    uint32_t GetShootTime() const;
    uint32_t GetShootTimeLimit() const;

private:
    void ResetPlan();
    void StartPlan(uint32_t now_tick_ms);
    void Advance100ms();
    bool IsConfigurationValid() const;
    bool IsRefereeDataFresh(uint32_t now_tick_ms, uint32_t last_update_tick_ms) const;

private:
    FireControlConfig config_;

    bool judge_data_valid_;          // 裁判帧存在、未超时且热量数据自洽
    bool continuous_trigger_active_; // 上一周期连发请求状态，用于检测新一次按住
    bool plan_active_;               // 当前按住期间是否已生成可用的前馈规划

    float current_heat_;      // 裁判系统当前热量，单位：热量
    float heat_limit_;        // 裁判系统热量上限，单位：热量
    float remaining_heat_;    // heat_limit_ - current_heat_，单位：热量
    float cooling_rate_;      // 裁判系统冷却速度，单位：热量/s
    float planned_fire_hz_;   // 当前前馈阶段固定使用的频率，单位：Hz
    float target_fire_hz_;    // 交给拨弹轮速度环的目标发射频率，单位：Hz

    uint32_t last_plan_tick_ms_; // 上一个规划 tick 的 HAL 时间，单位：ms
    uint32_t shoot_time_;        // 已经过的规划时间，单位：100 ms tick
    uint32_t shoot_time_limit_;  // 前馈阶段总时长，单位：100 ms tick
};

} // namespace FireControl
} // namespace ALG

#endif // FIRE_CONTROL_HPP
