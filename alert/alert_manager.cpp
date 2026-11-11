#include "alert_manager.h"
#include <ad_log/ad_log.hpp>
#include <cmath>
#include <iostream>
#include <algorithm>
#include <limits>
#include <unordered_set>
#include <unordered_map>

namespace sentinel {

namespace {

SentinelObstacleType obstacleFromLabel(const std::string& label) {
    if (label == "near_field_contact_motion") {
        return SentinelObstacleType::UNIDENTIFIED_COLLISION;
    }
    if (label == "passing_motion_object") {
        return SentinelObstacleType::UNKNOWN_OBSTACLE;
    }
    if (label == "person" || label == "human" || label == "pedestrian") {
        return SentinelObstacleType::ADULT;
    }
    if (label == "bicycle") return SentinelObstacleType::BICYCLE;
    if (label == "motorcycle") return SentinelObstacleType::MOTORCYCLE;
    if (label == "vehicle" || label == "car") return SentinelObstacleType::SEDAN;
    if (label == "truck" || label == "bus") return SentinelObstacleType::SUV;
    if (label == "animal" || label == "dog" || label == "cat" ||
        label == "horse" || label == "cow" || label == "sheep") {
        return SentinelObstacleType::ANIMAL;
    }
    return SentinelObstacleType::UNKNOWN_OBSTACLE;
}

SentinelObstacleType obstacleFromTracked(const std::vector<TrackedObject>& tracked,
                                         bool imu_triggered) {
    if (imu_triggered) return SentinelObstacleType::UNIDENTIFIED_COLLISION;
    for (const auto& obj : tracked) {
        const auto type = obstacleFromLabel(obj.label);
        if (type != SentinelObstacleType::UNKNOWN_OBSTACLE) return type;
    }
    return SentinelObstacleType::UNKNOWN_OBSTACLE;
}

bool isPersonLabel(const std::string& label) {
    return label == "person" || label == "human" || label == "pedestrian";
}

bool isCycleLikeLabel(const std::string& label) {
    return label == "bicycle" || label == "motorcycle" || label == "e_bike";
}

bool isVehicleLikeLabel(const std::string& label) {
    return label == "vehicle" || label == "car" || label == "sedan" ||
           label == "truck" || label == "bus" || label == "suv" ||
           isCycleLikeLabel(label);
}

bool isNearFieldContactLabel(const std::string& label) {
    return label == "near_field_contact_motion";
}

bool isPassingMotionLabel(const std::string& label) {
    return label == "passing_motion_object";
}

cv::Point2f bboxCenter(const cv::Rect& box) {
    return {box.x + box.width * 0.5f, box.y + box.height * 0.5f};
}

// 同相机人+自行车/摩托近邻 → 记为骑行（人走车类移动规则，障碍物用车/摩托类型）。
std::unordered_map<int, SentinelObstacleType> findRiderAssociations(
    const std::vector<TrackedObject>& tracked) {
    std::unordered_map<int, SentinelObstacleType> riders;
    std::vector<const TrackedObject*> people;
    std::vector<const TrackedObject*> cycles;
    people.reserve(tracked.size());
    cycles.reserve(tracked.size());
    for (const auto& obj : tracked) {
        if (obj.track_id < 0) continue;
        if (isPersonLabel(obj.label)) people.push_back(&obj);
        else if (isCycleLikeLabel(obj.label)) cycles.push_back(&obj);
    }
    for (const auto* person : people) {
        const cv::Point2f pc = bboxCenter(person->bbox);
        const float person_diag = std::sqrt(
            static_cast<float>(person->bbox.width * person->bbox.width +
                               person->bbox.height * person->bbox.height));
        const float max_dist = std::max(40.f, person_diag * 0.75f);
        const TrackedObject* best = nullptr;
        float best_d = max_dist;
        for (const auto* cycle : cycles) {
            if (!person->source_camera.empty() && !cycle->source_camera.empty() &&
                person->source_camera != cycle->source_camera) {
                continue;
            }
            const cv::Point2f cc = bboxCenter(cycle->bbox);
            const float d = std::hypot(pc.x - cc.x, pc.y - cc.y);
            if (d < best_d) {
                best_d = d;
                best = cycle;
            }
        }
        if (best) {
            riders[person->track_id] = obstacleFromLabel(best->label);
            riders[best->track_id] = obstacleFromLabel(best->label);
        }
    }
    return riders;
}

}  // namespace

AlertManager::AlertManager(int image_width, int image_height)
    : image_width_(image_width)
    , image_height_(image_height)
{}

void AlertManager::setImuThresholds(float general_accel, float severe_accel,
                                     float general_gyro, float severe_gyro,
                                     float handle_freq_min, float handle_freq_max) {
    general_accel_threshold_ = general_accel;
    severe_accel_threshold_  = severe_accel;
    general_gyro_threshold_  = general_gyro;
    severe_gyro_threshold_   = severe_gyro;
    handle_freq_min_hz_      = handle_freq_min;
    handle_freq_max_hz_      = handle_freq_max;
    std::cout << "[AlertManager] IMU classification=highpass_rms_flags"
              << " legacy_general_accel=" << general_accel
              << " severe_accel=" << severe_accel
              << " general_gyro=" << general_gyro
              << " severe_gyro=" << severe_gyro
              << std::endl;
}

// ---- IMU 震动分类 ----
// 设计文档：
//   表14 一般威胁：整车震动（振幅≤TBD）、门把手震动（振幅≤TBD）
//   表16 严重威胁：整车震动（振幅≥TBD）、门把手震动（振幅≥TBD）
// 实现：
//   - 小幅震动（general_threshold ≤ accel < severe_threshold）→ 一般威胁
//   - 大幅震动（accel ≥ severe_threshold）→ 严重威胁
//   - 门把手震动特征：高频小幅（gyro 高频分量明显，accel 相对小）
//     当前用角速度/加速度比值近似区分，实车应替换为频域分析

AlertManager::VibrationClass AlertManager::classifyVibration(const ImuData& imu) const {
    // 分类只消费 ImuSubscriber 在 100 Hz 上完成的高通/RMS/持续时间结果。
    // 不在低频相机帧上重新用单点绝对幅值判断，否则会遗漏短时事件。
    if (imu.impact_alert || imu.severe_vibration_alert) {
        return VibrationClass::SEVERE;
    }
    if (imu.vibration_alert) {
        return VibrationClass::GENERAL;
    }
    return VibrationClass::NONE;
}

void AlertManager::evaluate(const std::vector<TrackedObject>& tracked,
                             const ImuData&    imu,
                             const UssData&    uss,
                             SentryState       state,
                             uint64_t          timestamp)
{
    // 仅在激活/警戒/报警状态下评估威胁。
    if (state == SentryState::UNAVAILABLE || state == SentryState::STANDBY) {
        current_level_ = ThreatLevel::NONE;
        current_threat_id_ = 0;
        current_target_count_ = 0;
        current_reason_.clear();
        current_obstacle_type_ = SentinelObstacleType::UNKNOWN_OBSTACLE;
        return;
    }

    ThreatAssessment assessment = assessLevel(tracked, imu, uss, state, timestamp);
    ThreatLevel new_level = assessment.level;
    int new_threat_id = assessment.threat_id;
    current_target_count_ = static_cast<int>(tracked.size());
    current_reason_ = assessment.reason;
    current_obstacle_type_ = assessment.obstacle_type;

    const ThreatLevel prev_level = current_level_;
    const int prev_threat_id = current_threat_id_;

    // current_level_ 是状态机和周期 StatusOutput 的实时输入，必须每帧跟随评估结果更新。
    // 旧逻辑只在升级/换威胁/NONE 时更新，导致 HIGH/MEDIUM 降级为 LOW/MEDIUM 且 threat_id
    // 不变时被锁住，StatusOutput 的 ON/ACTIVE 字段无法回落到 OFF。
    current_level_ = new_level;
    current_threat_id_ = new_level == ThreatLevel::NONE ? 0 : new_threat_id;

    if (new_level > prev_level ||
        new_threat_id != prev_threat_id ||
        (new_level == ThreatLevel::NONE && prev_level != ThreatLevel::NONE))
    {

        if (new_level == ThreatLevel::NONE) return;

        alert_count_++;

        AlertEvent evt;
        evt.level         = new_level;
        evt.sentry_state  = state;
        evt.timestamp_ns  = timestamp;
        evt.imu_triggered = (classifyVibration(imu) != VibrationClass::NONE);
        evt.target_count  = static_cast<int>(tracked.size());
        evt.threat_id     = current_threat_id_;
        evt.sentinel_obstacle_type = evt.imu_triggered
            ? SentinelObstacleType::UNIDENTIFIED_COLLISION
            : assessment.obstacle_type;
        if (evt.sentinel_obstacle_type == SentinelObstacleType::UNKNOWN_OBSTACLE) {
            evt.sentinel_obstacle_type = obstacleFromTracked(tracked, false);
        }

        if (new_level == ThreatLevel::HIGH) {
            // 严重威胁：声光报警（表格21）
            // 注意：只有从 WARNING 状态才能进入 HIGH（由状态机 M7 控制）
            evt.light_req       = true;
            evt.horn_req        = true;
            evt.hazard_lamp_req = true;
            evt.sentinel_general_threat = SentinelGeneralThreat::OFF;
            evt.sentinel_severe_threat = SentinelGeneralThreat::ACTIVE;
            evt.sentinel_audio_visual_alarm = SentinelGeneralThreat::ACTIVE;
            evt.reason = assessment.reason.empty()
                ? "严重威胁：大幅震动/碰撞 → 声光报警 + 录像"
                : assessment.reason;
        } else if (new_level == ThreatLevel::MEDIUM) {
            // 一般威胁：中控警戒，不触发声光（表格21）
            evt.light_req       = false;
            evt.horn_req        = false;
            evt.hazard_lamp_req = false;
            evt.sentinel_general_threat = SentinelGeneralThreat::ACTIVE;
            evt.sentinel_severe_threat = SentinelGeneralThreat::OFF;
            evt.sentinel_audio_visual_alarm = SentinelGeneralThreat::OFF;
            evt.reason = assessment.reason.empty()
                ? "一般威胁：移动目标接近/震动 → 中控警戒 + 录像"
                : assessment.reason;
        } else {
            // 低威胁：目标出现但未满足一般威胁条件
            evt.sentinel_general_threat = SentinelGeneralThreat::ON;
            evt.sentinel_severe_threat = SentinelGeneralThreat::OFF;
            evt.sentinel_audio_visual_alarm = SentinelGeneralThreat::OFF;
            evt.reason = assessment.reason.empty()
                ? "低威胁：目标出现在监测范围内，持续观察"
                : assessment.reason;
        }

        AD_LINFO(Sentinel) << "[AlertManager] " << evt.reason
                   << " targets=" << evt.target_count
                   << " threat_id=" << evt.threat_id
                   << " imu_triggered=" << evt.imu_triggered
                   << " light=" << evt.light_req
                   << " horn=" << evt.horn_req
                   << " hazard=" << evt.hazard_lamp_req;

        std::cout << "[ALERT][" << static_cast<int>(new_level) << "] "
                  << evt.reason << std::endl;

        if (alert_cb_) alert_cb_(evt);
    }
}

float AlertManager::nominalObjectHeightM(const std::string& label) {
    if (label == "person" || label == "human" || label == "pedestrian") return 1.70f;
    if (label == "bicycle" || label == "motorcycle") return 1.40f;
    if (label == "vehicle" || label == "car" || label == "sedan") return 1.50f;
    if (label == "truck" || label == "bus" || label == "suv") return 2.20f;
    if (label == "animal" || label == "dog" || label == "cat") return 0.60f;
    if (label == "horse" || label == "cow" || label == "sheep") return 1.40f;
    if (label == "ball" || label == "sports ball") return 0.25f;
    return 1.0f;
}

float AlertManager::estimateVisualDistanceM(const TrackedObject& obj) const {
    if (obj.has_visual_ground_position &&
        std::isfinite(obj.visual_ground_distance_m) &&
        obj.visual_ground_distance_m > 0.f) {
        return obj.visual_ground_distance_m;
    }

    if (obj.bbox.height <= 0 || image_height_ <= 0) return 0.f;

    // 哨兵模式不使用 LiDAR；这里用类别名义高度 + 鱼眼等效焦距估计距离，
    // 作为没有标定地面投影时的保守 fallback。
    const float object_height_m = nominalObjectHeightM(obj.label);
    const float effective_focal_px = static_cast<float>(image_height_) * 0.45f;
    const float distance = object_height_m * effective_focal_px / static_cast<float>(obj.bbox.height);
    if (!std::isfinite(distance) || distance <= 0.f) return 0.f;
    return distance;
}

AlertManager::TargetKinematics AlertManager::estimateTargetKinematics(const TrackedObject& obj,
                                                                       uint64_t timestamp_ns) {
    TargetKinematics kin;
    if (obj.has_uss_fusion && obj.uss_distance_m > 0.f &&
        obj.uss_distance_m <= kUssFusionThreatDistanceM) {
        // 以视频目标为主体，USS 仅在 1.3m 内提升该视觉目标的位置/距离精度。
        kin.distance_m = obj.uss_distance_m;
    } else {
        kin.distance_m = estimateVisualDistanceM(obj);
    }
    kin.timestamp_ns = timestamp_ns;

    float radial_mps = 0.f;
    if (obj.track_id >= 0) {
        const auto it = target_history_.find(obj.track_id);
        if (it != target_history_.end() && it->second.timestamp_ns > 0 &&
            timestamp_ns > it->second.timestamp_ns && kin.distance_m > 0.f) {
            const float dt = static_cast<float>(timestamp_ns - it->second.timestamp_ns) / 1.0e9f;
            if (dt > 1.0e-3f && dt < 2.0f && it->second.distance_m > 0.f) {
                radial_mps = std::abs(kin.distance_m - it->second.distance_m) / dt;
            }
        }
        target_history_[obj.track_id] = kin;
    }

    // 横向路过时径向速度≈0：始终融合图像平面速度，取 max，避免漏报；
    // 静止目标像素抖动通常过不了连续 3 帧确认。
    const float speed_px = std::sqrt(obj.velocity.x * obj.velocity.x + obj.velocity.y * obj.velocity.y);
    const float effective_focal_px = std::max(1.0f, static_cast<float>(image_height_) * 0.45f);
    const float lateral_mps =
        kin.distance_m > 0.f ? (speed_px / effective_focal_px) * kin.distance_m * 10.0f : 0.f;
    kin.speed_mps = std::max(radial_mps, lateral_mps);
    return kin;
}

bool AlertManager::updateUnidirectionalMotion(int track_id,
                                              const cv::Rect& bbox,
                                              uint64_t timestamp_ns,
                                              bool cycle_like) {
    if (track_id < 0 || bbox.width <= 0 || bbox.height <= 0 || timestamp_ns == 0) {
        return false;
    }

    const uint64_t window_ns = cycle_like ? kCycleMotionWindowNs : kVehicleMotionWindowNs;
    const float min_px = cycle_like ? kCycleNetDisplaceMinPx : kVehicleNetDisplaceMinPx;
    const float bbox_frac =
        cycle_like ? kCycleNetDisplaceBboxFrac : kVehicleNetDisplaceBboxFrac;
    const float net_over_path =
        cycle_like ? kCycleNetOverPathMin : kVehicleNetOverPathMin;

    auto& hist = target_center_history_[track_id];
    hist.push_back({timestamp_ns,
                    bbox.x + bbox.width * 0.5f,
                    bbox.y + bbox.height * 0.5f});
    while (!hist.empty() && timestamp_ns > hist.front().timestamp_ns &&
           (timestamp_ns - hist.front().timestamp_ns) > window_ns) {
        hist.pop_front();
    }
    if (hist.size() < 2) return false;

    float path_len = 0.f;
    for (size_t i = 1; i < hist.size(); ++i) {
        const float dx = hist[i].cx - hist[i - 1].cx;
        const float dy = hist[i].cy - hist[i - 1].cy;
        path_len += std::sqrt(dx * dx + dy * dy);
    }
    const float net_dx = hist.back().cx - hist.front().cx;
    const float net_dy = hist.back().cy - hist.front().cy;
    const float net = std::sqrt(net_dx * net_dx + net_dy * net_dy);
    const float min_net = std::max(min_px, static_cast<float>(bbox.width) * bbox_frac);
    if (net < min_net) return false;
    if (path_len > 1.0e-3f && (net / path_len) < net_over_path) return false;
    return true;
}

AlertManager::ThreatAssessment AlertManager::assessLevel(const std::vector<TrackedObject>& tracked,
                                                         const ImuData&    imu,
                                                         const UssData&    uss,
                                                         SentryState       state,
                                                         uint64_t          timestamp_ns)
{
    // ========== 1. IMU 震动评估 ==========
    // 设计文档：
    //   表14 一般威胁：整车/门把手小幅震动（振幅≤TBD）
    //   表16 严重威胁：整车/门把手大幅震动（振幅≥TBD）
    //   M7: 严重威胁只能从 WARNING 状态进入（状态机控制）
    VibrationClass vib = classifyVibration(imu);

    if (vib == VibrationClass::SEVERE) {
        severe_threat_latch_until_ns_ = timestamp_ns + kSevereThreatLatchNs;
        if (state == SentryState::WARNING || state == SentryState::ALARM) {
            return {ThreatLevel::HIGH, kThreatIdImuSevere,
                    "严重威胁：大幅震动/碰撞 → 声光报警 + 录像",
                    SentinelObstacleType::UNIDENTIFIED_COLLISION};
        }
        return {ThreatLevel::MEDIUM, kThreatIdImuSevere,
                "一般威胁：大幅震动/碰撞预警 → 中控警戒 + 录像",
                SentinelObstacleType::UNIDENTIFIED_COLLISION};
    }

    const bool severe_latched = severe_threat_latch_until_ns_ > 0 &&
        timestamp_ns <= severe_threat_latch_until_ns_;
    if (severe_latched && (state == SentryState::WARNING || state == SentryState::ALARM)) {
        return {ThreatLevel::HIGH, kThreatIdImuSevere,
                "严重威胁：大幅震动/碰撞锁存 → 声光报警 + 录像",
                SentinelObstacleType::UNIDENTIFIED_COLLISION};
    }
    if (severe_threat_latch_until_ns_ > 0 && timestamp_ns > severe_threat_latch_until_ns_) {
        severe_threat_latch_until_ns_ = 0;
    }

    const bool general_vibration = vib == VibrationClass::GENERAL || vib == VibrationClass::HANDLE;
    const bool short_imu_disturbance =
        imu.vibration_accel_rms >= kNearFieldContactAccelRms ||
        imu.vibration_gyro_rms >= kNearFieldContactGyroRms ||
        imu.delta_accel >= kNearFieldContactDeltaAccel ||
        imu.delta_gyro >= kNearFieldContactDeltaGyro;

    // ========== 2. USS 使用原则 ==========
    // 哨兵模式以视频检测目标为危险事件触发前提。
    // PDC raw 近距不独立触发报警；PDC sectorDis 缺少与相机目标的空间关联，
    // 只保留为 USS 近距回调/日志输入。严重威胁仍要求 USS fused 点云写入视觉 track。
    (void)uss;

    // ========== 3. 视觉目标评估（距离/速度时间序列估计，含 USS 融合距离增强） ==========
    // 行人/合成骑行：距离<3m → 一般威胁（不要求移动确认）。
    // 自行车/摩托：不要求人车合成；<3m 且约0.35s 单向净位移（在动即视为有人）。
    // 纯汽车：距离<3m（或 USS 近距）且约1s 单向净位移 → 一般威胁。
    struct VisualCandidate {
        int threat_id{0};
        SentinelObstacleType obstacle_type{SentinelObstacleType::UNKNOWN_OBSTACLE};
        float distance_m{std::numeric_limits<float>::max()};
        float speed_mps{0.f};
        bool uss_near_body{false};
        bool medium{false};
        std::string reason;
    };

    bool has_target = false;
    int low_threat_id = 0;
    SentinelObstacleType low_obstacle = SentinelObstacleType::UNKNOWN_OBSTACLE;
    std::vector<VisualCandidate> medium_candidates;
    medium_candidates.reserve(tracked.size());
    std::vector<VisualCandidate> visual_impact_candidates;
    visual_impact_candidates.reserve(tracked.size());
    std::unordered_map<int, int> active_moving_streaks;
    std::unordered_set<int> active_center_ids;
    const auto rider_assoc = findRiderAssociations(tracked);

    for (const auto& obj : tracked) {
        has_target = true;
        const int obj_threat_id = obj.track_id >= 0 ? obj.track_id : 0;
        auto obj_obstacle = obstacleFromLabel(obj.label);
        const auto rider_it = rider_assoc.find(obj.track_id);
        const bool is_rider = rider_it != rider_assoc.end();
        if (is_rider) obj_obstacle = rider_it->second;

        if (low_threat_id == 0) {
            low_threat_id = obj_threat_id;
            low_obstacle = obj_obstacle;
        }
        // 低威胁/观察状态也尽量保持当前目标，避免 UI 目标在多框间来回跳。
        if (current_threat_id_ != 0 && obj_threat_id == current_threat_id_) {
            low_threat_id = obj_threat_id;
            low_obstacle = obj_obstacle;
        }

        TargetKinematics kin = estimateTargetKinematics(obj, timestamp_ns);
        const bool confirmed_detection = obj.lost <= 0;
        const bool uss_near_body = obj.has_uss_fusion && obj.uss_distance_m > 0.f &&
            obj.uss_distance_m <= kUssFusionThreatDistanceM;
        const bool near = kin.distance_m > 0.f && kin.distance_m < kVisualThreatDistanceM;
        const bool frame_moving = confirmed_detection && kin.speed_mps >= kVisualThreatSpeedMps;
        // 行人、人+车合成骑行：<3m 即报。
        // 自行车/摩托：不要求合成；短窗单向在动即报（静止两轮不报）。
        // 纯汽车：约1s单向净位移确认。
        const bool is_person = isPersonLabel(obj.label) && !is_rider;
        const bool is_cycle = isCycleLikeLabel(obj.label);
        const bool is_near_field_contact = isNearFieldContactLabel(obj.label);
        const bool is_passing_motion = isPassingMotionLabel(obj.label);
        const bool instant_near_threat = is_person || is_rider || is_near_field_contact;
        const bool needs_vehicle_pass_motion =
            !instant_near_threat && !is_passing_motion && isVehicleLikeLabel(obj.label);
        const bool calibrated_near_person_with_vibration =
            (general_vibration || (is_near_field_contact && short_imu_disturbance)) &&
            instant_near_threat &&
            confirmed_detection &&
            obj.age >= kVisualImpactMinTrackAge &&
            obj.has_visual_ground_position &&
            std::isfinite(obj.visual_ground_distance_m) &&
            obj.visual_ground_distance_m > 0.f &&
            obj.visual_ground_distance_m <= kVisualImpactThreatDistanceM;
        int moving_streak = 0;
        if (obj.track_id >= 0) {
            moving_streak = frame_moving ? (target_moving_streak_[obj.track_id] + 1) : 0;
            active_moving_streaks[obj.track_id] = moving_streak;
        }
        const bool confirmed_moving = moving_streak >= kVisualMovingConfirmFrames;
        bool vehicle_pass_moving = false;
        if (needs_vehicle_pass_motion && confirmed_detection && obj.track_id >= 0) {
            active_center_ids.insert(obj.track_id);
            vehicle_pass_moving = updateUnidirectionalMotion(
                obj.track_id, obj.bbox, timestamp_ns, is_cycle);
        }
        if (obj.has_uss_fusion) {
            std::cout << "[AlertManager][USS-Fusion] track=" << obj.track_id
                      << " distance=" << kin.distance_m
                      << " uss_distance=" << obj.uss_distance_m
                      << " near_body=" << obj.near_body_confirmed
                      << " uss_cluster=" << obj.uss_cluster_id
                      << " uss_points=" << obj.uss_point_count
                      << " assoc_cost=" << obj.uss_association_cost
                      << " quality=" << obj.uss_fusion_quality << std::endl;
        }
        const bool vehicle_motion_ok = needs_vehicle_pass_motion ? vehicle_pass_moving
                                                                : confirmed_moving;

        if (calibrated_near_person_with_vibration) {
            visual_impact_candidates.push_back({obj_threat_id, obj_obstacle, kin.distance_m,
                                                kin.speed_mps, false, true,
                                                is_near_field_contact
                                                    ? "严重威胁：近车身图像变化伴随敲击/震动 → 声光报警 + 录像"
                                                    : (is_rider
                                                    ? "严重威胁：近车骑行目标伴随敲击/震动 → 声光报警 + 录像"
                                                    : "严重威胁：近车人员伴随敲击/震动 → 声光报警 + 录像")});
        }

        // 纯车类：USS 近距也不能免检移动；行人/骑行可直接升级。
        if (uss_near_body && (instant_near_threat || vehicle_motion_ok)) {
            medium_candidates.push_back({obj_threat_id, obj_obstacle, kin.distance_m, kin.speed_mps,
                                         true, true,
                                         is_rider
                                             ? "一般威胁：骑行目标+USS近距融合确认 → 中控警戒 + 录像"
                                             : (is_person
                                                    ? "一般威胁：行人+USS近距融合确认 → 中控警戒 + 录像"
                                                    : (is_cycle
                                                           ? "一般威胁：两轮短窗单向移动+USS近距融合确认 → 中控警戒 + 录像"
                                                           : "一般威胁：车辆单向移动+USS近距融合确认 → 中控警戒 + 录像"))});
        } else if (near && instant_near_threat) {
            medium_candidates.push_back({obj_threat_id, obj_obstacle, kin.distance_m, kin.speed_mps,
                                         false, true,
                                         is_near_field_contact
                                             ? "一般威胁：近车身图像变化疑似接触 → 中控警戒 + 录像"
                                             : (is_rider
                                             ? "一般威胁：骑行目标距离<3m → 中控警戒 + 录像"
                                             : "一般威胁：检测到人且距离<3m → 中控警戒 + 录像")});
        } else if (near && is_passing_motion) {
            medium_candidates.push_back({obj_threat_id, obj_obstacle, kin.distance_m, kin.speed_mps,
                                         false, true,
                                         "一般威胁：道路ROI连续运动目标距离<3m → 中控警戒 + 录像"});
        } else if (near && needs_vehicle_pass_motion && vehicle_pass_moving) {
            medium_candidates.push_back({obj_threat_id, obj_obstacle, kin.distance_m, kin.speed_mps,
                                         false, true,
                                         is_cycle
                                             ? "一般威胁：两轮距离<3m且约0.35s内单向净位移确认 → 中控警戒 + 录像"
                                             : "一般威胁：车辆距离<3m且约1s内单向净位移确认 → 中控警戒 + 录像"});
        } else if (near && !needs_vehicle_pass_motion && !instant_near_threat && confirmed_moving) {
            // 其它非人目标保持原移动确认逻辑。
            medium_candidates.push_back({obj_threat_id, obj_obstacle, kin.distance_m, kin.speed_mps,
                                         false, true,
                                         "一般威胁：移动目标距离<3m且连续3帧速度≥0.1m/s → 中控警戒 + 录像"});
        }
    }
    target_moving_streak_.swap(active_moving_streaks);
    for (auto it = target_center_history_.begin(); it != target_center_history_.end();) {
        if (active_center_ids.count(it->first) == 0) it = target_center_history_.erase(it);
        else ++it;
    }

    if (!visual_impact_candidates.empty()) {
        const auto selected = std::min_element(
            visual_impact_candidates.begin(), visual_impact_candidates.end(),
            [](const VisualCandidate& a, const VisualCandidate& b) {
                if (std::abs(a.distance_m - b.distance_m) > 0.2f) return a.distance_m < b.distance_m;
                return a.threat_id < b.threat_id;
            });
        if (state == SentryState::WARNING || state == SentryState::ALARM) {
            return {ThreatLevel::HIGH, selected->threat_id, selected->reason,
                    SentinelObstacleType::UNIDENTIFIED_COLLISION};
        }
        return {ThreatLevel::MEDIUM, selected->threat_id,
                selected->obstacle_type == SentinelObstacleType::UNIDENTIFIED_COLLISION
                    ? "一般威胁：近车身图像变化伴随敲击/震动预警 → 中控警戒 + 录像"
                    : "一般威胁：近车人员伴随敲击/震动预警 → 中控警戒 + 录像",
                selected->obstacle_type};
    }

    if (!medium_candidates.empty()) {
        // 告警目标防跳变：如果当前目标仍然满足 MEDIUM 条件，继续沿用它；只有当前目标
        // 消失/不再构成威胁时才切到新的最佳候选。
        const auto current_it = std::find_if(medium_candidates.begin(), medium_candidates.end(),
            [this](const VisualCandidate& c) {
                return current_threat_id_ != 0 && c.threat_id == current_threat_id_;
            });
        const VisualCandidate* selected = nullptr;
        if (current_it != medium_candidates.end()) {
            selected = &(*current_it);
        } else {
            selected = &(*std::min_element(medium_candidates.begin(), medium_candidates.end(),
                [](const VisualCandidate& a, const VisualCandidate& b) {
                    if (a.uss_near_body != b.uss_near_body) return a.uss_near_body > b.uss_near_body;
                    if (std::abs(a.distance_m - b.distance_m) > 0.2f) return a.distance_m < b.distance_m;
                    if (std::abs(a.speed_mps - b.speed_mps) > 0.1f) return a.speed_mps > b.speed_mps;
                    return a.threat_id < b.threat_id;
                }));
        }
        return {ThreatLevel::MEDIUM, selected->threat_id, selected->reason, selected->obstacle_type};
    }

    if (general_vibration) {
        std::cout << "[AlertManager][IMU-GeneralFallback] visual_targets=" << tracked.size()
                  << std::endl;
        return {ThreatLevel::MEDIUM, kThreatIdImuGeneral,
                "一般威胁：小幅震动/门把手震动 → 中控警戒 + 录像",
                SentinelObstacleType::UNIDENTIFIED_COLLISION};
    }

    if (has_target) {
        return {ThreatLevel::LOW, low_threat_id,
                "低威胁：目标出现在监测范围内，持续观察",
                low_obstacle};
    }
    return {ThreatLevel::NONE, 0, ""};
}

}  // namespace sentinel
