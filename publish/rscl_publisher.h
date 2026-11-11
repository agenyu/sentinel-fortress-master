#pragma once
// RSCL 消息发布封装
// 哨兵模式：l3_msgs/SentinelResStatus（事件）+ StateBroadcast（/sentry/state_broadcast）+ AlarmCtrl
// 堡垒模式：l3_msgs/OrsmStatus
// 通过 config.yaml enable_rscl_publish 开关控制是否真实发布

#include "orsm/orsm_output.h"
#include "orsm/orsm_settings.h"
#include "alert/alert_manager.h"
#include "state/sentry_mode_contract.h"
#include "rscl_access/rscl_access.h"

#include <string>
#include <atomic>
#include <cstdint>

namespace sentinel {

struct PublishConfig {
    bool enable_rscl_publish{false};  // 总开关：是否执行真实 RSCL publish
    std::string sentry_state_topic{"/sentry/state_broadcast"};
    std::string sentry_event_topic{"/network/sentry_resp"};
    std::string sentry_action_topic{"/sentry/action_request"};
    std::string orsm_state_topic{"/network/fortress_resp"};
    std::string orsm_action_topic{"/orsm/action_request"};
};

/// /sentry/state_broadcast 结构化字段（idl/state_broadcast.capnp，无堡垒）
struct StateBroadcastView {
    uint64_t timestamp_ns{0};
    int sentry_state{0};
    std::string sentry_state_name;
    int threat_level{0};
    std::string threat_level_name;
    int raw_threat_id{0};
    int sm_threat_id{0};
    int local_track_id{0};
    std::string threat_camera;
    int target_count{0};
    int obstacle_type{0};
    std::string obstacle_type_name;
    std::string reason_code;
    std::string reason;
    std::string summary;
};

class RsclPublisher {
public:
    explicit RsclPublisher(const PublishConfig& cfg);

    /// 初始化：advertise 所有 topic（仅在 enable_rscl_publish=true 时执行）
    bool init();

    /// 发布哨兵事件报告
    void publishSentryEvent(const AlertEvent& evt,
                            const SentinelResStatusView& res,
                            uint64_t event_id);

    /// /sentry/state_broadcast：StateBroadcast.capnp（无堡垒字段）
    void publishSentryStateBroadcast(const StateBroadcastView& msg);

    /// 发布哨兵动作请求
    void publishSentryAction(const AlertEvent& evt,
                             const SentinelResStatusView& res);

    /// 发布 ORSM 状态
    void publishOrsmState(const ORSMAlertOutput& output, uint64_t timestamp_ns);

    /// 发布 ORSM 声光/双闪/鸣笛动作请求
    void publishOrsmAction(const ORSMAlertOutput& output, uint64_t timestamp_ns);

    bool enabled() const { return cfg_.enable_rscl_publish && initialized_; }

private:
    PublishConfig cfg_;
    bool initialized_{false};
};

}  // namespace sentinel
