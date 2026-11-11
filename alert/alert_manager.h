#pragma once

#include "../imu/imu_subscriber.h"
#include "../uss/uss_subscriber.h"
#include "../detection/tracker.h"
#include "../state/sentry_state_machine.h"
#include <functional>
#include <string>
#include <chrono>
#include <deque>
#include <unordered_map>

namespace sentinel {

// 报警事件（对应功能说明书输出信号表格21）
struct AlertEvent {
    ThreatLevel level{ThreatLevel::NONE};
    SentryState sentry_state{SentryState::ACTIVE};
    std::string reason;
    uint64_t    timestamp_ns{0};
    bool        imu_triggered{false};
    int         target_count{0};
    int         threat_id{0};       // 同一威胁跟踪ID：track_id 或传感器事件ID

    // SentryMode.proto::SentinelResStatus 对齐字段
    SentinelGeneralThreat sentinel_general_threat{SentinelGeneralThreat::OFF};
    SentinelGeneralThreat sentinel_severe_threat{SentinelGeneralThreat::OFF};
    SentinelGeneralThreat sentinel_audio_visual_alarm{SentinelGeneralThreat::OFF};
    SentinelObstacleType  sentinel_obstacle_type{SentinelObstacleType::UNKNOWN_OBSTACLE};

    // 输出信号（HAD → OIB，来自表格21）
    bool light_req{false};       // HAD_LightReq=2：位置灯请求（警戒状态）
    bool horn_req{false};        // HAD_HornReq=1：喇叭请求（报警状态）
    bool hazard_lamp_req{false}; // HAD_HzrdLampReq=2：双闪灯开（报警状态）
};

using AlertCallback = std::function<void(const AlertEvent&)>;

/**
 * @brief 威胁分级与报警管理器
 *
 * 规则对齐《M18-3先导车哨兵模式功能规范 V1.3》：
 *
 * 一般威胁（表14）：
 *   - 视觉：行人/合成骑行距离<3m 即触发
 *   - 自行车/摩托：不要求人车合成；<3m 且约0.35s 单向净位移（在动即视为有人；静止两轮不报）
 *   - 纯汽车：<3m 且约1s 单向净位移（抑制邻车抖动）
 *   - 视觉+USS：1.3m 内近车身确认；车/两轮同样要求对应单向位移，行人/骑行可直接升级
 *   - IMU：整车/门把手小幅震动（振幅 < severe_threshold）
 *
 * 注意：哨兵模式不使用 LiDAR；LiDAR 仅允许用于越野堡垒(ORSM)目标定位/测速融合。
 * USS 不单独作为报警/录像触发源，避免纯超声波近距噪声或静态障碍导致报警风暴。
 *
 * 严重威胁（表16）：
 *   - IMU：整车/门把手大幅震动（振幅 ≥ severe_threshold）
 *   注意：严重威胁只能从警戒状态(WARNING)进入，由状态机控制
 *
 * 威胁解除（表15）：
 *   - 速度≤0.1m/s 或距离>10m，且持续>10s（由状态机 M6 控制）
 *   - 震动停止且持续>10s
 */
class AlertManager {
public:
    explicit AlertManager(int image_width = 1920, int image_height = 1536);

    void setAlertCallback(AlertCallback cb) { alert_cb_ = std::move(cb); }

    // 设置 IMU 震动阈值（可从 config.yaml 配置）
    void setImuThresholds(float general_accel, float severe_accel,
                          float general_gyro, float severe_gyro,
                          float handle_freq_min, float handle_freq_max);

    void evaluate(const std::vector<TrackedObject>& tracked,
                  const ImuData&    imu,
                  const UssData&    uss,
                  SentryState       state,
                  uint64_t          timestamp);

    ThreatLevel currentLevel() const { return current_level_; }
    int         currentThreatId() const { return current_threat_id_; }
    int         currentTargetCount() const { return current_target_count_; }
    const std::string& currentReason() const { return current_reason_; }
    SentinelObstacleType currentObstacleType() const { return current_obstacle_type_; }
    uint64_t    alertCount()   const { return alert_count_; }

private:
    struct ThreatAssessment {
        ThreatLevel level{ThreatLevel::NONE};
        int threat_id{0};
        std::string reason;
        SentinelObstacleType obstacle_type{SentinelObstacleType::UNKNOWN_OBSTACLE};
    };

    ThreatAssessment assessLevel(const std::vector<TrackedObject>& tracked,
                                 const ImuData&    imu,
                                 const UssData&    uss,
                                 SentryState       state,
                                 uint64_t          timestamp_ns);

    // IMU 震动分类
    enum class VibrationClass { NONE, GENERAL, SEVERE, HANDLE };
    VibrationClass classifyVibration(const ImuData& imu) const;

    int         image_width_;
    int         image_height_;
    ThreatLevel current_level_{ThreatLevel::NONE};
    int         current_threat_id_{0};
    int         current_target_count_{0};
    std::string current_reason_;
    SentinelObstacleType current_obstacle_type_{SentinelObstacleType::UNKNOWN_OBSTACLE};
    uint64_t    alert_count_{0};
    AlertCallback alert_cb_;

    // IMU 震动阈值（设计文档表14/16，振幅 TBD，使用可配置默认值）
    float general_accel_threshold_{2.0f};   // 一般威胁：小幅震动加速度阈值 (m/s²)
    float severe_accel_threshold_{5.0f};    // 严重威胁：大幅震动加速度阈值 (m/s²)
    float general_gyro_threshold_{0.5f};    // 一般威胁：小幅震动角速度阈值 (rad/s)
    float severe_gyro_threshold_{2.0f};     // 严重威胁：大幅震动角速度阈值 (rad/s)
    // 门把手震动频率特征（高频小幅，区别于整车低频大幅）
    float handle_freq_min_hz_{10.0f};       // 门把手震动最低频率
    float handle_freq_max_hz_{50.0f};       // 门把手震动最高频率

    struct TargetKinematics {
        float distance_m{0.f};
        float speed_mps{0.f};
        uint64_t timestamp_ns{0};
    };

    float estimateVisualDistanceM(const TrackedObject& obj) const;
    TargetKinematics estimateTargetKinematics(const TrackedObject& obj, uint64_t timestamp_ns);
    static float nominalObjectHeightM(const std::string& label);

    struct CenterSample {
        uint64_t timestamp_ns{0};
        float cx{0.f};
        float cy{0.f};
    };

    // 汽车约 1s、两轮约 0.35s 窗口内框中心单向净位移确认。
    bool updateUnidirectionalMotion(int track_id,
                                    const cv::Rect& bbox,
                                    uint64_t timestamp_ns,
                                    bool cycle_like);

    std::unordered_map<int, TargetKinematics> target_history_;
    std::unordered_map<int, int> target_moving_streak_;  // track_id → 连续移动帧数
    std::unordered_map<int, std::deque<CenterSample>> target_center_history_;

    // 严重威胁短时锁存：真实碰撞/剐蹭/砸车可能只持续一帧。
    // ACTIVE 首帧仍按规范先进入 WARNING，但锁存窗口内下一次评估可继续升级到 ALARM。
    uint64_t severe_threat_latch_until_ns_{0};
    static constexpr uint64_t kSevereThreatLatchNs = 2ULL * 1000000000ULL;

    // 威胁距离阈值。USS 仅在视觉目标存在时作为 1.3m 内近车身距离增强/升级依据。
    static constexpr float kUssFusionThreatDistanceM = 1.3f;
    static constexpr float kVisualThreatDistanceM = 3.0f;
    static constexpr float kVisualImpactThreatDistanceM = 2.0f;
    static constexpr float kVisualClearDistanceM = 10.0f;
    static constexpr float kNearFieldContactAccelRms = 0.08f;
    static constexpr float kNearFieldContactGyroRms = 0.0010f;
    static constexpr float kNearFieldContactDeltaAccel = 0.30f;
    static constexpr float kNearFieldContactDeltaGyro = 0.004f;
    static constexpr int kNearFieldContactSustainedMinAge = 4;
    static constexpr float kVisualThreatSpeedMps = 0.1f;
    static constexpr int kVisualMovingConfirmFrames = 3;  // 非车类目标连续移动确认帧数
    static constexpr int kVisualImpactMinTrackAge = 3;
    static constexpr uint64_t kVehicleMotionWindowNs = 1000000000ULL;  // 汽车 1s
    static constexpr uint64_t kCycleMotionWindowNs = 350000000ULL;     // 两轮 0.35s（快穿）
    static constexpr float kVehicleNetDisplaceMinPx = 48.f;
    static constexpr float kVehicleNetDisplaceBboxFrac = 0.30f;
    static constexpr float kVehicleNetOverPathMin = 0.55f;  // 净位移/路径长，抑制来回抖
    static constexpr float kCycleNetDisplaceMinPx = 36.f;
    static constexpr float kCycleNetDisplaceBboxFrac = 0.20f;
    static constexpr float kCycleNetOverPathMin = 0.50f;

    static constexpr int kThreatIdImuGeneral = -101;
    static constexpr int kThreatIdImuSevere = -100;
};

}  // namespace sentinel
