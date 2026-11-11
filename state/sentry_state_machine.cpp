#include "sentry_state_machine.h"
#include <ad_log/ad_log.hpp>
#include <iostream>

namespace sentinel {

namespace {
bool elapsedAtLeast(uint64_t start_ns, uint64_t now_ns, uint64_t threshold_ns) {
    return start_ns > 0 && now_ns >= start_ns && (now_ns - start_ns) >= threshold_ns;
}
}

SentryStateMachine::SentryStateMachine()
    : state_(SentryState::UNAVAILABLE)
{}

void SentryStateMachine::update(const OddConditions& odd,
                                 ThreatLevel          threat,
                                 int                  threat_id,
                                 uint64_t             timestamp_ns)
{
    const bool user_off = !odd.userWantsSentryOn();

    switch (state_) {

    // ---- 不可用 ----
    // M1: 系统自检 + M2: 除钥匙距离外的ODD检测
    case SentryState::UNAVAILABLE:
        if (odd.canEnterSentryFlow()) {
            transition(SentryState::STANDBY,
                       "M1+M2: 系统自检通过，ODD（除钥匙距离）通过",
                       timestamp_ns);
        }
        break;

    // ---- 待机 ----
    // M3: 钥匙距离检测（>10m → ACTIVE）
    case SentryState::STANDBY:
        if (user_off) {
            transition(SentryState::UNAVAILABLE,
                       "M4: 用户关闭哨兵模式",
                       timestamp_ns);
        } else if (!odd.oddWithoutKey()) {
            transition(SentryState::UNAVAILABLE,
                       "M2: ODD条件不满足，回到不可用",
                       timestamp_ns);
        } else if (odd.key_far) {
            transition(SentryState::ACTIVE,
                       "M3: 钥匙离车>10m，激活哨兵模式",
                       timestamp_ns);
        }
        break;

    // ---- 激活（正常监测） ----
    // M4: 用户关闭 → UNAVAILABLE/STANDBY
    // M5: 一般威胁（物体<3m，速度≥0.1m/s）→ WARNING
    case SentryState::ACTIVE:
        if (user_off) {
            transition(SentryState::UNAVAILABLE,
                       "M4: 用户关闭哨兵模式",
                       timestamp_ns);
        } else if (!odd.oddWithoutKey()) {
            transition(SentryState::UNAVAILABLE,
                       "M2: ODD条件失效",
                       timestamp_ns);
        } else if (!odd.key_far) {
            transition(SentryState::STANDBY,
                       "M4: 钥匙靠近，回到待机",
                       timestamp_ns);
        } else if (threat >= ThreatLevel::MEDIUM) {
            // M5: 一般威胁（MEDIUM = 物体<3m且速度≥0.1m/s）
            threat_start_ns_  = timestamp_ns;
            last_threat_      = threat;
            active_threat_id_ = threat_id;
            threat_clear_start_ns_ = 0;
            transition(SentryState::WARNING,
                       "M5: 检测到一般威胁，进入警戒状态",
                       timestamp_ns);
        }
        break;

    // ---- 警戒（一般威胁） ----
    // M4: 用户关闭 → UNAVAILABLE
    // M6: 威胁解除（速度≤0.1m/s或距离>10m，持续>10s）→ ACTIVE
    // M7: 严重威胁（车辆震动）→ ALARM
    case SentryState::WARNING:
        if (user_off) {
            transition(SentryState::UNAVAILABLE,
                       "M4: 用户关闭哨兵模式",
                       timestamp_ns);
        } else if (!odd.oddWithoutKey()) {
            transition(SentryState::UNAVAILABLE,
                       "M2: ODD条件失效",
                       timestamp_ns);
        } else if (!odd.key_far) {
            transition(SentryState::STANDBY,
                       "M4: 钥匙靠近，回到待机",
                       timestamp_ns);
        } else if (threat == ThreatLevel::HIGH) {
            // M7: 严重威胁（震动/碰撞）
            if (active_threat_id_ != threat_id || threat_start_ns_ == 0) {
                threat_start_ns_ = timestamp_ns;
            }
            last_threat_     = threat;
            active_threat_id_ = threat_id;
            threat_clear_start_ns_ = 0;
            transition(SentryState::ALARM,
                       "M7: 检测到严重威胁（震动/碰撞），进入报警状态",
                       timestamp_ns);
        } else if (threat < ThreatLevel::MEDIUM) {
            // M6: 威胁降低，开始计时
            if (threat_clear_start_ns_ == 0) {
                threat_clear_start_ns_ = timestamp_ns;
            } else if (elapsedAtLeast(threat_clear_start_ns_, timestamp_ns, kThreatClearNs)) {
                // 持续>10s无威胁 → 解除
                threat_clear_start_ns_ = 0;
                transition(SentryState::ACTIVE,
                           "M6: 威胁解除持续>10s，回到激活状态",
                           timestamp_ns);
            }
        } else {
            // 威胁仍在，重置解除计时，并对 WARNING 同样执行 M8 同一威胁超时。
            threat_clear_start_ns_ = 0;
            if (active_threat_id_ != threat_id || threat_start_ns_ == 0) {
                active_threat_id_ = threat_id;
                threat_start_ns_ = timestamp_ns;
            } else if (elapsedAtLeast(threat_start_ns_, timestamp_ns, kThreatTimeoutNs)) {
                threat_start_ns_ = 0;
                active_threat_id_ = 0;
                transition(SentryState::ACTIVE,
                           "M8: 同一警戒威胁持续>3min，回到激活状态",
                           timestamp_ns);
            }
            last_threat_ = threat;
        }
        break;

    // ---- 报警（严重威胁） ----
    // M4: 用户关闭 → UNAVAILABLE
    // M6: 威胁解除（持续>10s）→ ACTIVE
    // M8: 同一威胁超时（>3min）→ ACTIVE
    case SentryState::ALARM:
        if (user_off) {
            transition(SentryState::UNAVAILABLE,
                       "M4: 用户关闭哨兵模式",
                       timestamp_ns);
        } else if (!odd.oddWithoutKey()) {
            transition(SentryState::UNAVAILABLE,
                       "M2: ODD条件失效",
                       timestamp_ns);
        } else if (!odd.key_far) {
            transition(SentryState::STANDBY,
                       "M4: 钥匙靠近，回到待机",
                       timestamp_ns);
        } else if (threat < ThreatLevel::MEDIUM) {
            // M6: 威胁解除，开始计时。ALARM 中仍有一般威胁时应持续报警。
            if (threat_clear_start_ns_ == 0) {
                threat_clear_start_ns_ = timestamp_ns;
            } else if (elapsedAtLeast(threat_clear_start_ns_, timestamp_ns, kThreatClearNs)) {
                threat_clear_start_ns_ = 0;
                transition(SentryState::ACTIVE,
                           "M6: 严重威胁解除持续>10s，回到激活状态",
                           timestamp_ns);
            }
        } else {
            // 威胁仍在，重置解除计时
            threat_clear_start_ns_ = 0;

            if (active_threat_id_ != threat_id || threat_start_ns_ == 0) {
                // 新的威胁重新计时，避免 A 威胁超时误停止 B 威胁报警。
                active_threat_id_ = threat_id;
                last_threat_ = threat;
                threat_start_ns_ = timestamp_ns;
                break;
            }
            last_threat_ = threat;

            // M8: 同一威胁超时（>3min）→ 停止报警，回到激活
            if (elapsedAtLeast(threat_start_ns_, timestamp_ns, kThreatTimeoutNs))
            {
                threat_start_ns_ = 0;
                active_threat_id_ = 0;
                transition(SentryState::ACTIVE,
                           "M8: 同一严重威胁持续>3min，停止报警，回到激活状态",
                           timestamp_ns);
            }
        }
        break;
    }
}

void SentryStateMachine::transition(SentryState new_state,
                                     const std::string& reason,
                                     uint64_t timestamp_ns)
{
    if (new_state == state_) return;

    SentryStateEvent evt;
    evt.from         = state_;
    evt.to           = new_state;
    evt.reason       = reason;
    evt.timestamp_ns = timestamp_ns;

    if (new_state == SentryState::WARNING) warning_count_++;
    if (new_state == SentryState::ALARM)   alarm_count_++;

    AD_LINFO(Sentinel) << "[SentryStateMachine] "
               << static_cast<int>(state_) << " → "
               << static_cast<int>(new_state)
               << " | " << reason;

    std::cout << "[SentryState] "
              << static_cast<int>(state_) << " → "
              << static_cast<int>(new_state)
              << " | " << reason << std::endl;

    state_ = new_state;

    if (state_cb_) state_cb_(evt);
}

}  // namespace sentinel
