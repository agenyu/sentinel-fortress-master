#pragma once
// 越野堡垒模式状态机
// 来源：M18-3先导车越野堡垒功能详细说明书V1.2
// 4状态：Off / Active / Warning / Failure
// 6个切换条件：T1-T6

#include <cstdint>
#include <string>
#include <functional>
#include "orsm_settings.h"

namespace sentinel {

enum class OrsmState : uint8_t {
    OFF     = 0,
    ACTIVE  = 1,
    WARNING = 2,
    FAILURE = 3,
};

// ODD条件（来自说明书表格5）
struct OrsmOdd {
    bool power_on{false};        // 电源档位ON及以上
    bool gear_park{false};       // P档
    float soc{0.f};              // 动力电池SOC（%）
    float speed_kmh{0.f};        // 车速（km/h）
    float soc_activate_threshold{30.f};      // T1 激活 SOC 阈值（%）
    float soc_deactivate_threshold{20.f};    // T2 退出 SOC 阈值（%）
    float speed_exit_threshold_kmh{20.f};    // T2 退出车速阈值（km/h）
    bool handbrake_on{false};    // 手刹拉起
    bool had_fault{false};       // 智驾系统故障
    bool sensor_fault{false};    // 传感器故障（雷达/摄像头）
    bool network_ok{true};       // 5G网络连接
    bool sw_enable{false};       // IVI/Tbox软开关是否开启
    OrsmSettingType sw_mode{OrsmSettingType::OFF}; // OFF/单车安防/多车联防

    // 多车联防扩展 ODD（单车模式忽略）
    bool multi_vehicle_team_ready{false};       // 组队链路已就绪
    bool multi_vehicle_team_formed{false};      // 已形成有效车队
    bool vehicle_stationary_over_60s{false};    // 静止/驻车时间满足多车进入条件
    bool external_locked{false};                // 外部锁车完成
    bool ivi_prepared{false};                   // IVI 多车联防准备完成
    TeamStatusEnum tbox_team_status{TeamStatusEnum::NO_TEAM_FORMED};
};

struct OrsmStateEvent {
    OrsmState   from;
    OrsmState   to;
    std::string reason;
    uint64_t    timestamp_ns{0};
};

using OrsmStateCallback = std::function<void(const OrsmStateEvent&)>;

class OrsmStateMachine {
public:
    OrsmStateMachine();
    void update(const OrsmOdd& odd, bool threat_detected, bool fault, uint64_t ts);
    void reset();
    OrsmState currentState() const { return state_; }
    void setStateCallback(OrsmStateCallback cb) { cb_ = std::move(cb); }
    uint64_t warningCount() const { return warning_count_; }

private:
    void transition(OrsmState to, const std::string& reason, uint64_t ts);
    OrsmState        state_{OrsmState::OFF};
    OrsmStateCallback cb_;
    uint64_t         warning_count_{0};
};

}  // namespace sentinel
