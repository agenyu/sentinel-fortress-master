#pragma once

/**
 * 哨兵模式状态机
 *
 * 来源：M18-3先导车哨兵模式功能详细说明书V1.3
 *
 * 状态定义（表格7）：
 *   UNAVAILABLE  - 哨兵模式不可用：系统自检/ODD检测不通过
 *   STANDBY      - 哨兵模式待机：自检通过，ODD通过（除钥匙距离），等待驾驶员离车>10m
 *   ACTIVE       - 哨兵模式激活：所有ODD通过，持续监测，无威胁
 *   WARNING      - 哨兵模式警戒：检测到一般威胁（物体速度≥0.1m/s且距离<3m）
 *   ALARM        - 哨兵模式报警：检测到严重威胁（车辆震动/碰撞）
 *
 * 状态切换条件（表格8）：
 *   M1: 系统自检
 *   M2: 除钥匙距离外的ODD检测
 *   M3: 钥匙距离检测（>10m → ACTIVE）
 *   M4: 用户关闭意愿检测
 *   M5: 一般威胁监测（物体<3m且速度≥0.1m/s → WARNING）
 *   M6: 监测威胁解除（速度≤0.1m/s或距离>10m，持续>10s → ACTIVE）
 *   M7: 严重威胁监测（车辆震动 → ALARM）
 *   M8: 同一威胁超时检测（同一威胁>3min → ACTIVE）
 *
 * 输出信号（表格20）：
 *   WARNING: 中控屏点亮，显示"哨兵模式进入警戒状态，视频正在录制"
 *   ALARM:   中控屏闪烁 + 双闪灯(HAD_HzrdLampReq=2) + 喇叭(HAD_HornReq=1)
 */

#include <cstdint>
#include <string>
#include <functional>
#include <chrono>
#include "sentry_mode_contract.h"

namespace sentinel {

// ---- 哨兵模式状态 ----
enum class SentryState : uint8_t {
    UNAVAILABLE = 0,  // 不可用
    STANDBY     = 1,  // 待机（自检通过，等待离车）
    ACTIVE      = 2,  // 激活（正常监测，无威胁）
    WARNING     = 3,  // 警戒（一般威胁：物体<3m）
    ALARM       = 4,  // 报警（严重威胁：车辆震动/碰撞）
};

// ---- 威胁等级（与 AlertManager 对齐） ----
enum class ThreatLevel : uint8_t {
    NONE   = 0,
    LOW    = 1,  // 目标出现但未进入3m范围
    MEDIUM = 2,  // 一般威胁：物体速度≥0.1m/s且距离<3m → 触发 WARNING
    HIGH   = 3,  // 严重威胁：车辆震动/碰撞 → 触发 ALARM
};

// ---- ODD 条件（来自表格5/12） ----
struct OddConditions {
    bool power_ok{false};        // 电量≥15%（纯电）或电量>10%且油量>20%（混动）
    bool speed_zero{false};      // 车速 = 0 km/h
    bool gear_park{false};       // 档位 P档
    bool handbrake_on{false};    // 手刹拉起
    bool key_far{false};         // 钥匙离车 > 10m
    bool power_off{false};       // 整车下电/熄火
    bool no_sensor_fault{false}; // 无传感器故障
    bool storage_ok{false};      // 存储空间充足（≥5GB临时 + 20GB永久/U盘）
    bool no_occupant{false};     // 车内无人（DMS + 座椅传感器）
    bool vehicle_locked{true};   // 整车落锁
    bool temp_ok{true};          // 温度满足 ODD（-40~85℃）
    bool user_enabled{false};    // 用户开启/保持哨兵模式意愿（M4，不属于 ODD）
    bool user_shutdown_requested{false}; // 用户主动关闭请求（M4，不属于 ODD）

    bool userWantsSentryOn() const {
        return user_enabled && !user_shutdown_requested;
    }

    SentinelODDStatus oddStatus() const {
        if (!power_ok) return SentinelODDStatus::INSUFFICIENT_BATTERY_POWER;
        if (!speed_zero) return SentinelODDStatus::SPEED_REQUIREMENT_NOT_MET;
        if (!gear_park) return SentinelODDStatus::GEAR_POSITION_NOT_MET;
        if (!handbrake_on) return SentinelODDStatus::HANDBRAKE_ENGAGED;
        if (!key_far) return SentinelODDStatus::KEY_OUT_OF_RANGE;
        if (!power_off) return SentinelODDStatus::EV_START_CONDITIONS_NOT_MET;
        if (!no_sensor_fault) return SentinelODDStatus::SENSOR_MALFUNCTION;
        if (!storage_ok) return SentinelODDStatus::TEMPORARY_STORAGE_NOT_AVAILABLE;
        if (!no_occupant) return SentinelODDStatus::NO_DRIVER_DETECTED;
        if (!vehicle_locked) return SentinelODDStatus::ODD_RESERVED;
        if (!temp_ok) return SentinelODDStatus::ODD_RESERVED;
        return SentinelODDStatus::SATISFY_ALL_ODD;
    }

    SentinelEnableFailed enableFailedReason() const {
        if (!vehicle_locked) {
            return SentinelEnableFailed::VEHICLE_NOT_LOCKED;
        }
        switch (oddStatus()) {
        case SentinelODDStatus::SATISFY_ALL_ODD:
            return SentinelEnableFailed::SENTINEL_ENABLE_FAILED_UNKNOWN;
        case SentinelODDStatus::INSUFFICIENT_BATTERY_POWER:
        case SentinelODDStatus::INSUFFICIENT_HYBRID_POWER:
            return SentinelEnableFailed::LOW_BATTERY_LEVEL;
        case SentinelODDStatus::GEAR_POSITION_NOT_MET:
            return SentinelEnableFailed::VEHICLE_NOT_IN_PARK_GEAR;
        case SentinelODDStatus::SPEED_REQUIREMENT_NOT_MET:
        case SentinelODDStatus::HANDBRAKE_ENGAGED:
        case SentinelODDStatus::KEY_OUT_OF_RANGE:
        case SentinelODDStatus::EV_START_CONDITIONS_NOT_MET:
        case SentinelODDStatus::HYBRID_START_CONDITIONS_NOT_MET:
        case SentinelODDStatus::TEMPORARY_STORAGE_NOT_AVAILABLE:
        case SentinelODDStatus::PERMANENT_STORAGE_NOT_AVAILABLE:
        case SentinelODDStatus::NO_PERSON_IN_VEHICLE:
        case SentinelODDStatus::NO_DRIVER_DETECTED:
            return SentinelEnableFailed::ABNORMAL_VEHICLE_STATUS;
        case SentinelODDStatus::SENSOR_MALFUNCTION:
            return SentinelEnableFailed::SYSTEM_MALFUNCTION;
        case SentinelODDStatus::ODD_RESERVED:
        default:
            return SentinelEnableFailed::ENA_RESERVED;
        }
    }

    // 除钥匙距离外的ODD是否全部通过
    bool oddWithoutKey() const {
        return power_ok && speed_zero && gear_park && handbrake_on
            && power_off && no_sensor_fault && storage_ok && no_occupant
            && vehicle_locked && temp_ok;
    }

    // 全部ODD通过（含钥匙距离）
    bool allPassed() const {
        return oddWithoutKey() && key_far;
    }

    bool canEnterSentryFlow() const {
        return userWantsSentryOn() && oddWithoutKey();
    }
};

// ---- 状态变化事件 ----
struct SentryStateEvent {
    SentryState from;
    SentryState to;
    std::string reason;
    uint64_t    timestamp_ns{0};
};

using SentryStateCallback = std::function<void(const SentryStateEvent&)>;

/**
 * @brief 哨兵模式状态机
 *
 * 实现功能说明书中定义的5状态机和8个切换条件。
 * 线程安全：update() 由主循环单线程调用。
 */
class SentryStateMachine {
public:
    SentryStateMachine();

    // 每帧调用，传入当前 ODD 条件和威胁等级
    void update(const OddConditions& odd,
                ThreatLevel          threat,
                int                  threat_id,
                uint64_t             timestamp_ns);

    // 兼容旧调用：无明确威胁ID时按0处理。
    void update(const OddConditions& odd,
                ThreatLevel          threat,
                uint64_t             timestamp_ns) {
        update(odd, threat, 0, timestamp_ns);
    }

    SentryState currentState() const { return state_; }

    // M8 正在跟踪的威胁 ID
    int activeThreatId() const { return active_threat_id_; }

    // 注册状态变化回调
    void setStateCallback(SentryStateCallback cb) { state_cb_ = std::move(cb); }

    // 统计
    uint64_t warningCount() const { return warning_count_; }
    uint64_t alarmCount()   const { return alarm_count_; }

private:
    void transition(SentryState new_state, const std::string& reason,
                    uint64_t timestamp_ns);

    SentryState          state_{SentryState::UNAVAILABLE};
    SentryStateCallback  state_cb_;

    // 威胁持续时间跟踪（用于 M8 同一威胁超时检测）
    uint64_t threat_start_ns_{0};
    ThreatLevel last_threat_{ThreatLevel::NONE};
    int active_threat_id_{0};

    // 威胁解除持续时间跟踪（用于 M6，需持续>10s）
    uint64_t threat_clear_start_ns_{0};

    uint64_t warning_count_{0};
    uint64_t alarm_count_{0};

    static constexpr uint64_t kThreatTimeoutNs  = 3ULL * 60 * 1000000000; // 3min
    static constexpr uint64_t kThreatClearNs    = 10ULL * 1000000000;     // 10s
};

}  // namespace sentinel
