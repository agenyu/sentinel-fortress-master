// 单车越野堡垒管理器实现
// 对齐 OrsmInfo.proto / senseauto-msgs: ad_msg_idl/l3_msgs/orsm_info.capnp.h

#include "single_orsm_manager.h"
#include <algorithm>
#include <cmath>
#include <unordered_set>

namespace sentinel {

namespace {

constexpr int kThreatConfirmWindowFrames = 5;  // 最近5次评估作为威胁确认窗口
constexpr int kThreatConfirmHits = 3;          // 窗口内至少3次威胁后触发 T3，吸收多相机空帧抖动
constexpr int kClearConfirmFrames = 5;         // 连续5帧清空后才允许 T4，抑制逐帧抖动
constexpr int kIntruderConfirmMinTrackAge = 3; // 全局 ORSM track 至少多帧命中后才允许发布
constexpr int kIntruderConfirmStableObservations = 3; // 单个入侵对象至少连续稳定3次才允许发布
constexpr uint64_t kWarningHoldNs = 3000000000ULL; // WARNING 状态最少保持3s，贴合声光报警持续语义
constexpr uint64_t kIntruderOutputHoldNs = 1500000000ULL; // 对外 intruder 仅短时保留，避免目标离开后长时间沿用旧轨迹
constexpr uint64_t kWarningLidarSupportHoldNs = 1500000000ULL; // 已报警目标附近 LiDAR 仅允许短时支撑，超过后必须依赖新轨迹

DetecRangeSetting normalizedRange(DetecRangeSetting range) {
    switch (range) {
    case DetecRangeSetting::RANGE_A:
    case DetecRangeSetting::RANGE_B:
    case DetecRangeSetting::RANGE_C:
        return range;
    case DetecRangeSetting::NONE_RANGE:
    default:
        return DetecRangeSetting::RANGE_A;
    }
}

float squaredDistance(const IntruderInfo& lhs, const IntruderInfo& rhs) {
    const float dx = lhs.intruder_pose_x - rhs.intruder_pose_x;
    const float dy = lhs.intruder_pose_y - rhs.intruder_pose_y;
    return dx * dx + dy * dy;
}

bool samePhysicalIntruder(const IntruderInfo& lhs, const IntruderInfo& rhs,
                          float dedup_distance_m) {
    // 已经完成 LiDAR/车辆坐标融合的目标，用车体坐标做空间聚类。
    // 同一人员/动物被检测成多个框时，位置通常高度接近；不同真实目标相距超过该阈值。
    return squaredDistance(lhs, rhs) <= dedup_distance_m * dedup_distance_m;
}

bool samePhysicalIntruderForDedup(const IntruderInfo& lhs, const IntruderInfo& rhs,
                                  float dedup_distance_m) {
    if (lhs.intruder_id == rhs.intruder_id) return true;
    if (dedup_distance_m <= 0.f) return false;
    if (lhs.intruder_type == rhs.intruder_type) {
        return samePhysicalIntruder(lhs, rhs, dedup_distance_m);
    }

    // 类型不同的目标不能直接套用 1m 去重阈值；否则 human/unknown 在相邻但不同位置时
    // 会被 SingleOrsmManager 合并，造成 intruders_num 出现 2→1→2 闪变。
    // 只允许极近距离重叠框合并，保留“同一目标分类抖动”的兜底能力。
    const float cross_type_dedup_m = std::min(dedup_distance_m * 0.45f, 0.45f);
    return samePhysicalIntruder(lhs, rhs, cross_type_dedup_m);
}

bool shouldReplaceIntruderRepresentative(const IntruderInfo& candidate,
                                         const IntruderInfo& current) {
    const auto candidate_zone = static_cast<uint8_t>(candidate.zone);
    const auto current_zone = static_cast<uint8_t>(current.zone);
    if (candidate_zone != current_zone) return candidate_zone > current_zone;

    const float candidate_speed_sq = candidate.intruder_velocity_x * candidate.intruder_velocity_x +
                                     candidate.intruder_velocity_y * candidate.intruder_velocity_y;
    const float current_speed_sq = current.intruder_velocity_x * current.intruder_velocity_x +
                                   current.intruder_velocity_y * current.intruder_velocity_y;
    constexpr float kVelocityEpsilonSq = 1e-4f;
    if ((candidate_speed_sq > kVelocityEpsilonSq) != (current_speed_sq > kVelocityEpsilonSq)) {
        return candidate_speed_sq > current_speed_sq;
    }

    // 同一聚类内保留距离本车更近的目标，避免重复框把报警等级/位置带偏。
    if (std::fabs(candidate.distance - current.distance) > 1e-3f) {
        return candidate.distance < current.distance;
    }

    return candidate.intruder_id < current.intruder_id;
}

std::vector<IntruderInfo> deduplicateIntrudersByPosition(const std::vector<IntruderInfo>& intruders,
                                                         float dedup_distance_m) {
    if (dedup_distance_m <= 0.f) return intruders;

    std::vector<IntruderInfo> deduped;
    deduped.reserve(intruders.size());

    for (const auto& intruder : intruders) {
        auto match = std::find_if(deduped.begin(), deduped.end(),
                                  [&](const IntruderInfo& kept) {
                                      return samePhysicalIntruderForDedup(intruder, kept, dedup_distance_m);
                                  });
        if (match == deduped.end()) {
            deduped.push_back(intruder);
        } else if (shouldReplaceIntruderRepresentative(intruder, *match)) {
            *match = intruder;
        }
    }

    return deduped;
}

bool hasPreviousPrimaryMatch(const std::vector<IntruderInfo>& intruders,
                             const std::vector<IntruderInfo>& previous_intruders,
                             float primary_latch_distance_m) {
    if (intruders.empty() || previous_intruders.empty()) return false;

    const auto& previous_primary = previous_intruders.front();
    const float latch_distance = std::max(1.5f, primary_latch_distance_m);
    return std::any_of(intruders.begin(), intruders.end(),
                       [&](const IntruderInfo& intruder) {
                           // 多相机/旧 tracker 下 track_id 不保证全局唯一；同 ID 也必须空间接近。
                           return samePhysicalIntruder(intruder, previous_primary, latch_distance);
                       });
}

bool promotePreviousPrimary(std::vector<IntruderInfo>& intruders,
                            const std::vector<IntruderInfo>& previous_intruders,
                            float primary_latch_distance_m) {
    if (intruders.empty() || previous_intruders.empty()) return false;

    const auto& previous_primary = previous_intruders.front();
    const float latch_distance = std::max(1.5f, primary_latch_distance_m);
    auto same_id = std::find_if(intruders.begin(), intruders.end(),
                                [&](const IntruderInfo& intruder) {
                                    return intruder.intruder_id == previous_primary.intruder_id &&
                                           samePhysicalIntruder(intruder, previous_primary, latch_distance);
                                });
    if (same_id != intruders.end()) {
        std::rotate(intruders.begin(), same_id, same_id + 1);
        return true;
    }

    auto same_physical = std::min_element(intruders.begin(), intruders.end(),
        [&](const IntruderInfo& lhs, const IntruderInfo& rhs) {
            return squaredDistance(lhs, previous_primary) < squaredDistance(rhs, previous_primary);
        });
    if (same_physical != intruders.end() &&
        samePhysicalIntruder(*same_physical, previous_primary, latch_distance)) {
        std::rotate(intruders.begin(), same_physical, same_physical + 1);
        return true;
    }

    return false;
}

void stabilizeIntruderOrdering(std::vector<IntruderInfo>& intruders,
                               const std::vector<IntruderInfo>& previous_intruders,
                               float primary_latch_distance_m) {
    if (intruders.empty()) return;

    // 输出层/前端通常会把 intruders_info[0] 当作当前报警目标展示。
    // 车辆坐标聚类只能去掉同帧重复框；如果仍有多个真实/候选目标，vector 原始顺序
    // 随检测/多相机输入变化会导致“报警目标”在不同目标间跳变。这里保持上一帧主目标
    // 仍存在时继续排第一；如果旧模型/旧 bag 导致 tracker 重建 ID，则再用车体坐标
    // 做一次主目标空间锁定，避免同一物理目标换 ID 后主目标跳到其它框。
    if (promotePreviousPrimary(intruders, previous_intruders, primary_latch_distance_m)) return;

    auto best = std::min_element(intruders.begin(), intruders.end(),
        [](const IntruderInfo& lhs, const IntruderInfo& rhs) {
            const auto lhs_zone = static_cast<uint8_t>(lhs.zone);
            const auto rhs_zone = static_cast<uint8_t>(rhs.zone);
            if (lhs_zone != rhs_zone) return lhs_zone > rhs_zone;
            if (std::fabs(lhs.distance - rhs.distance) > 1e-3f) return lhs.distance < rhs.distance;
            return lhs.intruder_id < rhs.intruder_id;
        });
    if (best != intruders.end()) {
        std::rotate(intruders.begin(), best, best + 1);
    }
}

std::vector<IntruderInfo> mergePhysicalOutputIntruders(const std::vector<IntruderInfo>& intruders,
                                                       float dedup_distance_m) {
    if (intruders.size() <= 1) return intruders;

    std::vector<IntruderInfo> merged;
    merged.reserve(intruders.size());

    auto samePhysicalOutput = [&](const IntruderInfo& lhs, const IntruderInfo& rhs) {
        if (lhs.intruder_id == rhs.intruder_id) return true;
        const float range = std::max(lhs.distance, rhs.distance);
        const bool same_type = lhs.intruder_type == rhs.intruder_type;
        const float same_type_gate = std::max(dedup_distance_m * 1.35f,
                                              std::min(1.8f, 0.14f * range));
        const float cross_type_gate = std::min(0.60f, std::max(0.35f, dedup_distance_m * 0.55f));
        return samePhysicalIntruder(lhs, rhs, same_type ? same_type_gate : cross_type_gate);
    };

    for (const auto& intruder : intruders) {
        auto match = std::find_if(merged.begin(), merged.end(),
                                  [&](const IntruderInfo& kept) {
                                      return samePhysicalOutput(intruder, kept);
                                  });
        if (match == merged.end()) {
            merged.push_back(intruder);
        } else if (shouldReplaceIntruderRepresentative(intruder, *match)) {
            *match = intruder;
        }
    }

    return merged;
}

OrsmWarnLevel highestWarnLevel(const std::vector<IntruderInfo>& intruders) {
    OrsmWarnLevel level = OrsmWarnLevel::LEVEL_NONE;
    for (const auto& intruder : intruders) {
        if (static_cast<uint8_t>(intruder.zone) > static_cast<uint8_t>(level)) {
            level = intruder.zone;
        }
    }
    return level;
}

void sortIntrudersByPriority(std::vector<IntruderInfo>& intruders) {
    std::sort(intruders.begin(), intruders.end(),
              [](const IntruderInfo& lhs, const IntruderInfo& rhs) {
                  const auto lhs_zone = static_cast<uint8_t>(lhs.zone);
                  const auto rhs_zone = static_cast<uint8_t>(rhs.zone);
                  if (lhs_zone != rhs_zone) return lhs_zone > rhs_zone;
                  if (std::fabs(lhs.distance - rhs.distance) > 1e-3f) {
                      return lhs.distance < rhs.distance;
                  }
                  return lhs.intruder_id < rhs.intruder_id;
              });
}

bool intruderRecentlySeen(const IntruderInfo& intruder, uint64_t timestamp_ns) {
    if (timestamp_ns == 0 || intruder.timestamp_nsec == 0) return true;
    if (timestamp_ns < intruder.timestamp_nsec) return true;
    return timestamp_ns - intruder.timestamp_nsec <= kIntruderOutputHoldNs;
}

bool lidarSupportsIntruder(const IntruderInfo& intruder,
                           const LidarFrame& lidar,
                           float dedup_distance_m) {
    if (!lidar.valid()) return false;
    const float radius = std::max(1.2f, dedup_distance_m * 1.5f);
    const float radius_sq = radius * radius;
    constexpr int kMinSupportPoints = 6;
    int points = 0;
    for (const auto& p : lidar.points) {
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;
        if (p.z < -1.2f || p.z > 2.5f) continue;
        const float dx = p.x - intruder.intruder_pose_x;
        const float dy = p.y - intruder.intruder_pose_y;
        if (dx * dx + dy * dy > radius_sq) continue;
        if (++points >= kMinSupportPoints) return true;
    }
    return false;
}

bool matchesHeldIntruder(const IntruderInfo& current,
                         const IntruderInfo& previous,
                         float dedup_distance_m) {
    const float latch_distance = std::max(1.5f, dedup_distance_m);
    if (current.intruder_id == previous.intruder_id &&
        samePhysicalIntruder(current, previous, latch_distance)) {
        return true;
    }
    return samePhysicalIntruderForDedup(current, previous, latch_distance);
}

std::vector<IntruderInfo> mergeCandidateIntruders(std::vector<IntruderInfo> current,
                                                  const std::vector<IntruderInfo>& previous,
                                                  uint64_t timestamp_ns,
                                                  float dedup_distance_m,
                                                  const std::unordered_set<uint32_t>& observed_track_ids = {}) {
    if (previous.empty() || !current.empty()) {
        current = deduplicateIntrudersByPosition(current, dedup_distance_m);
        sortIntrudersByPriority(current);
        if (!previous.empty()) stabilizeIntruderOrdering(current, previous, dedup_distance_m);
        return current;
    }
    // Do not turn an observed-but-currently-not-LiDAR-backed track into an old
    // coordinate. The dedicated LiDAR hold below handles the case where the whole
    // upstream tracker is absent; reusing a stale snapshot here makes the point sit
    // still and then teleport when LiDAR is reacquired.
    (void)timestamp_ns;
    (void)observed_track_ids;
    return current;
}

std::vector<IntruderInfo> updateHeldIntruders(std::vector<IntruderInfo> current,
                                              const std::vector<IntruderInfo>& previous,
                                              uint64_t timestamp_ns,
                                              float dedup_distance_m,
                                              const std::unordered_set<uint32_t>& observed_track_ids) {
    if (previous.empty() || !current.empty()) {
        current = deduplicateIntrudersByPosition(current, dedup_distance_m);
        sortIntrudersByPriority(current);
        if (!previous.empty()) stabilizeIntruderOrdering(current, previous, dedup_distance_m);
        return current;
    }
    (void)timestamp_ns;
    (void)observed_track_ids;
    return current;
}

std::vector<IntruderInfo> refreshHeldIntrudersWithLidarBackedCurrent(
    std::vector<IntruderInfo> held,
    const std::vector<IntruderInfo>& current_lidar_backed,
    float dedup_distance_m) {
    if (held.empty() || current_lidar_backed.empty()) return held;

    for (auto& prev : held) {
        auto match = std::find_if(
            current_lidar_backed.begin(), current_lidar_backed.end(),
            [&](const IntruderInfo& cur) {
                // An ID match alone is insufficient: after a LiDAR-absent interval the
                // same global ID can be metres away. Replacing a held snapshot without
                // the spatial check produces an instantaneous jump in OrsmStatus.
                return matchesHeldIntruder(cur, prev, dedup_distance_m);
            });
        if (match != current_lidar_backed.end()) {
            prev = *match;
        }
    }

    held = deduplicateIntrudersByPosition(held, dedup_distance_m);
    sortIntrudersByPriority(held);
    return held;
}

std::vector<IntruderInfo> holdPreviousIntrudersWithLidar(
    const std::vector<IntruderInfo>& previous,
    uint64_t timestamp_ns,
    float dedup_distance_m,
    const LidarFrame& lidar,
    bool allow_hold_without_lidar) {
    std::vector<IntruderInfo> held;
    held.reserve(previous.size());
    for (const auto& prev : previous) {
        if (!intruderRecentlySeen(prev, timestamp_ns)) continue;
        if (!allow_hold_without_lidar &&
            !lidarSupportsIntruder(prev, lidar, dedup_distance_m)) {
            continue;
        }
        // 保留原始观测时间，确保 kIntruderOutputHoldNs 真正限制旧目标最长保持时间；
        // 仅因LiDAR仍在旧位置附近就逐帧刷新时间，会造成目标永久残留。
        held.push_back(prev);
    }
    held = deduplicateIntrudersByPosition(held, dedup_distance_m);
    sortIntrudersByPriority(held);
    stabilizeIntruderOrdering(held, previous, dedup_distance_m);
    return held;
}

}  // namespace

SingleOrsmManager::SingleOrsmManager() = default;

void SingleOrsmManager::resetTracking() {
    track_history_.clear();
    threat_confirm_window_.clear();
    threat_confirm_hits_ = 0;
    clear_confirm_count_ = 0;
    warning_hold_until_ns_ = 0;
    pending_warning_snapshot_ = WarningSnapshot{};
    last_warning_snapshot_ = WarningSnapshot{};
}

void SingleOrsmManager::reset() {
    resetTracking();
    sm_.reset();
}

IntruderType SingleOrsmManager::classifyIntruder(const std::string& label) {
    // 根据检测器输出的标签映射到 OrsmInfo.proto::IntruderType
    if (label == "person" || label == "human" || label == "pedestrian") {
        return IntruderType::INTRUDER_HUMANKIND;
    }
    if (label == "animal" || label == "dog" || label == "cat" ||
        label == "bird"   || label == "horse" || label == "cow" ||
        label == "sheep"  || label == "bear") {
        return IntruderType::INTRUDER_ANIMAL;
    }
    return IntruderType::INTRUDER_UNKNOWN;
}

OrsmWarnLevel SingleOrsmManager::distanceToZone(float distance,
                                          float zone_a, float zone_b, float zone_c) {
    // C区（核心）< B区（中间）< A区（最外围）
    if (distance <= zone_c) return OrsmWarnLevel::WARNLEVEL_3;
    if (distance <= zone_b) return OrsmWarnLevel::WARNLEVEL_2;
    if (distance <= zone_a) return OrsmWarnLevel::WARNLEVEL_1;
    return OrsmWarnLevel::LEVEL_NONE;
}

ORSMAlertOutput SingleOrsmManager::update(const std::vector<TrackedObject>& tracked,
                                    const ImuData& imu,
                                    const UssData& uss,
                                    const LidarFrame& lidar,
                                    const OrsmSettings& settings,
                                    const OrsmOdd& odd,
                                    uint64_t timestamp) {
    // 堡垒模式的入侵目标位置/速度以视频 + LiDAR 融合为主。
    // IMU 可作为撞击/晃动等辅助事件输入预留，但不用于外部目标定位。
    (void)imu;
    (void)uss;

    ORSMAlertOutput output;
    output.timestamp_ns = timestamp;

    // 感知/状态机路径只生成运行态与预警轨迹。设置反馈由
    // SentinelNode 的 IVI/TBOX 设置回调维护，避免旧 settings 快照回写。

    // 根据用户设置确定有效检测范围
    const float effective_zone_a = settings.zone_a_radius;
    const float effective_zone_b = settings.zone_b_radius;
    const float effective_zone_c = settings.zone_c_radius;

    // DetecRangeSetting 限制最大检测范围
    // RANGE_A=全范围，RANGE_B=仅B+C，RANGE_C=仅C
    float max_range = effective_zone_a;
    switch (normalizedRange(settings.detec_range_setting)) {
    case DetecRangeSetting::RANGE_B:
        max_range = effective_zone_b;
        break;
    case DetecRangeSetting::RANGE_C:
        max_range = effective_zone_c;
        break;
    case DetecRangeSetting::RANGE_A:
    case DetecRangeSetting::NONE_RANGE:
    default:
        max_range = effective_zone_a;
        break;
    }

    struct TargetSnapshot {
        IntruderType type{IntruderType::INTRUDER_UNKNOWN};
        float distance{0.f};
        float pose_x{0.f};
        float pose_y{0.f};
        float velocity_x{0.f};
        float velocity_y{0.f};
    };

    auto snapshotTarget = [&](const TrackedObject& obj, TargetSnapshot& snap) -> bool {
        snap.type = classifyIntruder(obj.label);

        // 根据用户设置过滤目标类型
        if (snap.type == IntruderType::INTRUDER_HUMANKIND &&
            !switchOn(settings.humans_recog_setting)) return false;
        if (snap.type == IntruderType::INTRUDER_ANIMAL &&
            !switchOn(settings.animals_recog_setting)) return false;
        // 堡垒模式以多帧稳定视觉 track 作为报警主体；LiDAR 融合只用于提升
        // 位置/距离精度。哨兵 detection fallback 会生成负 track_id，仍禁止
        // 直接驱动 ORSM，避免单帧/短时误检造成抖动报警。
        if (obj.track_id < 0) return false;

        snap.distance = obj.distance_m;
        if (snap.distance <= 0.f && obj.bbox.height > 0) {
            snap.distance = 6120.0f / static_cast<float>(obj.bbox.height);
        }
        // 设计文档口径为“危险生物：人/动物/夜间生物入侵”。普通 unknown 不能
        // 直接预警；远距 unknown 默认还需要图像运动证据和 LiDAR 支撑，后续若
        // 视频语义观测关联上再补入侵类型。
        if (snap.type == IntruderType::INTRUDER_UNKNOWN) {
            const float far_min_distance =
                std::max(0.0f, std::max(settings.lidar_only_far_output_min_distance_m,
                                        settings.far_dynamic_min_distance_m));
            if (!settings.lidar_only_far_output_enabled ||
                !obj.has_lidar_fusion ||
                (settings.far_image_motion_required &&
                 !obj.orsm_far_image_motion_lidar_confirmed) ||
                (!obj.orsm_far_image_motion_lidar_confirmed &&
                 snap.distance < far_min_distance)) {
                return false;
            }
        }
        snap.pose_x = obj.position_3d.x;
        snap.pose_y = obj.position_3d.y;
        snap.velocity_x = obj.velocity_3d.x;
        snap.velocity_y = obj.velocity_3d.y;
        constexpr float kMaxBiologicalSpeedMps = 12.0f;
        const float speed_mps = std::hypot(snap.velocity_x, snap.velocity_y);
        if (speed_mps > kMaxBiologicalSpeedMps) return false;

        return snap.distance > 0.f;
    };

    auto isMovingAwayFromHistory = [&](int track_id, float distance) -> bool {
        const auto it = track_history_.find(track_id);
        if (it == track_history_.end()) return false;
        constexpr float kMovingAwayEpsilonM = 0.25f;
        return distance > it->second.last_distance + kMovingAwayEpsilonM;
    };

    auto isPublishableIntruderCandidate = [&](const TrackedObject& obj,
                                              const TargetSnapshot& snap,
                                              OrsmWarnLevel zone) -> bool {
        // Upstream grants this flag only for a confirmed Inbox/LiDAR match
        // or its active lease. Do not reconfirm an already granted lease.
        if (obj.has_lidar_fusion && obj.orsm_far_image_motion_lidar_confirmed) {
            return true;
        }
        const auto it = track_history_.find(obj.track_id);
        if (it == track_history_.end()) return false;
        // 上游只发布携带 LiDAR 融合的轨迹；无融合观测不应进入发布判定。
        if (!obj.has_lidar_fusion) {
            return false;
        }
        if (obj.age < kIntruderConfirmMinTrackAge) return false;
        if (it->second.stable_observations < kIntruderConfirmStableObservations) return false;

        if (zone != OrsmWarnLevel::WARNLEVEL_1) return true;
        // A 区是最远预警圈，在全局多帧 + LiDAR 背书基础上继续做更严格的稳定性约束。

        const float speed_mps = std::hypot(snap.velocity_x, snap.velocity_y);
        const float step_m = std::hypot(snap.pose_x - it->second.last_pose_x,
                                        snap.pose_y - it->second.last_pose_y);
        const bool outer_a_zone = snap.distance > effective_zone_b + 5.0f;
        const int stable_min = outer_a_zone ? 60 : 8;
        const float max_speed_mps = outer_a_zone ? 1.5f : 3.0f;
        const float max_step_m = outer_a_zone ? 1.0f : 2.0f;
        return it->second.stable_observations >= stable_min &&
               speed_mps <= max_speed_mps &&
               step_m <= max_step_m;
    };

    auto makeIntruderInfo = [&](const TrackedObject& obj,
                                const TargetSnapshot& snap,
                                OrsmWarnLevel zone) -> IntruderInfo {
        IntruderInfo info;
        info.intruder_id = static_cast<uint32_t>(obj.track_id);
        info.intruder_type = snap.type;
        info.intruder_pose_x = snap.pose_x;
        info.intruder_pose_y = snap.pose_y;
        info.intruder_velocity_x = snap.velocity_x;
        info.intruder_velocity_y = snap.velocity_y;
        info.timestamp_nsec = timestamp;
        info.distance = snap.distance;
        info.zone = zone;
        return info;
    };

    auto updateTrackHistory = [&](int track_id, const TargetSnapshot& snap, uint64_t ts) -> TrackHistory& {
        auto& history = track_history_[track_id];
        constexpr float kStableDistanceDeltaM = 1.5f;
        constexpr float kStablePoseDeltaM = 2.0f;
        const bool had_history = history.last_timestamp_ns > 0;
        const float distance_delta = std::fabs(snap.distance - history.last_distance);
        const float pose_delta = std::hypot(snap.pose_x - history.last_pose_x,
                                            snap.pose_y - history.last_pose_y);
        if (!had_history || distance_delta > kStableDistanceDeltaM || pose_delta > kStablePoseDeltaM) {
            history.stable_observations = 1;
        } else {
            history.stable_observations = std::min(history.stable_observations + 1, 1000);
        }
        history.last_distance = snap.distance;
        history.last_pose_x = snap.pose_x;
        history.last_pose_y = snap.pose_y;
        history.last_timestamp_ns = ts;
        return history;
    };

    auto isThreatCandidate = [&](const TrackedObject& obj) -> bool {
        TargetSnapshot snap;
        if (!snapshotTarget(obj, snap)) return false;

        if (snap.distance > max_range) return false;

        const OrsmWarnLevel zone = distanceToZone(snap.distance,
                                                  effective_zone_a,
                                                  effective_zone_b,
                                                  effective_zone_c);
        if (zone == OrsmWarnLevel::LEVEL_NONE) return false;

        if (!obj.orsm_far_image_motion_lidar_confirmed &&
            zone == OrsmWarnLevel::WARNLEVEL_1 &&
            isMovingAwayFromHistory(obj.track_id, snap.distance)) {
            return false;
        }

        return true;
    };

    auto warningSnapshotHasLidarSupport = [&](const WarningSnapshot& snapshot) -> bool {
        if (snapshot.intruders.empty()) return false;
        if (timestamp > 0 && snapshot.timestamp_ns > 0) {
            if (timestamp < snapshot.timestamp_ns) return false;
            if (timestamp - snapshot.timestamp_ns > kWarningLidarSupportHoldNs) return false;
        }
        return std::any_of(snapshot.intruders.begin(), snapshot.intruders.end(),
                           [&](const IntruderInfo& intruder) {
                               return lidarSupportsIntruder(intruder, lidar,
                                                            settings.intruder_dedup_distance_m);
                           });
    };

    // ---- 1. 先计算原始威胁，再对 threat_detected 输入做确认/保持 ----
    bool raw_threat_detected = false;
    for (const auto& obj : tracked) {
        if (isThreatCandidate(obj)) {
            raw_threat_detected = true;
            break;
        }
    }
    const bool direct_lidar_lease_observed = std::any_of(
        tracked.begin(), tracked.end(), [](const TrackedObject& obj) {
            return obj.has_lidar_fusion &&
                   obj.orsm_far_image_motion_lidar_confirmed;
        });

    threat_confirm_window_.push_back(raw_threat_detected);
    if (raw_threat_detected) {
        threat_confirm_hits_++;
    }
    while (static_cast<int>(threat_confirm_window_.size()) > kThreatConfirmWindowFrames) {
        if (threat_confirm_window_.front()) {
            threat_confirm_hits_--;
        }
        threat_confirm_window_.pop_front();
    }

    if (raw_threat_detected) {
        clear_confirm_count_ = 0;
    } else {
        clear_confirm_count_ = std::min(clear_confirm_count_ + 1, kClearConfirmFrames);
    }

    const OrsmState previous_state = sm_.currentState();
    const bool was_warning = previous_state == OrsmState::WARNING;
    bool threat_detected = false;
    if (was_warning) {
        const bool held_warning_has_lidar_support =
            warningSnapshotHasLidarSupport(last_warning_snapshot_) ||
            warningSnapshotHasLidarSupport(pending_warning_snapshot_);
        // WARNING 态下保持到最小报警时长，且必须连续多帧无威胁后才解除。
        threat_detected = raw_threat_detected ||
                          held_warning_has_lidar_support ||
                          (timestamp > 0 && timestamp < warning_hold_until_ns_) ||
                          clear_confirm_count_ < kClearConfirmFrames;
    } else {
        // ACTIVE 态下使用滑动窗口确认，避免单帧 LiDAR/视觉关联抖动直接 T3，
        // 同时不让多相机空帧把 T,-,T,-,T 这类真实威胁反复清零。
        threat_detected = direct_lidar_lease_observed ||
                          threat_confirm_hits_ >= kThreatConfirmHits;
    }

    const bool has_fault = odd.had_fault || odd.sensor_fault;
    sm_.update(odd, threat_detected, has_fault, timestamp);

    if (sm_.currentState() == OrsmState::WARNING && raw_threat_detected && timestamp > 0) {
        warning_hold_until_ns_ = std::max(warning_hold_until_ns_, timestamp + kWarningHoldNs);
    }

    OrsmState state = sm_.currentState();
    if (state == OrsmState::OFF || state == OrsmState::FAILURE) {
        resetTracking();
    }
    if (was_warning && state != OrsmState::WARNING) {
        pending_warning_snapshot_ = WarningSnapshot{};
        last_warning_snapshot_ = WarningSnapshot{};
    }

    // ---- 2. 遍历目标，生成内部追踪候选，并从中筛选状态帧发布对象 ----
    std::vector<IntruderInfo> internal_intruders;
    std::vector<IntruderInfo> lidar_backed_intruders;
    std::vector<IntruderInfo> published_intruders;
    std::unordered_set<uint32_t> observed_track_ids;
    observed_track_ids.reserve(tracked.size());
    OrsmWarnLevel highest_level = OrsmWarnLevel::LEVEL_NONE;

    if (state == OrsmState::ACTIVE || state == OrsmState::WARNING) {
        for (const auto& obj : tracked) {
            if (obj.track_id >= 0) {
                observed_track_ids.insert(static_cast<uint32_t>(obj.track_id));
            }

            TargetSnapshot snap;
            if (!snapshotTarget(obj, snap)) continue;

            const bool moving_away = isMovingAwayFromHistory(obj.track_id, snap.distance);
            auto& history = updateTrackHistory(obj.track_id, snap, timestamp);

            if (snap.distance > max_range) continue;

            OrsmWarnLevel zone = distanceToZone(snap.distance,
                                                effective_zone_a,
                                                effective_zone_b,
                                                effective_zone_c);
            if (zone == OrsmWarnLevel::LEVEL_NONE) continue;

            // A 区非安全目标若距离趋势增大（远离本车），不触发报警。
            if (!obj.orsm_far_image_motion_lidar_confirmed &&
                zone == OrsmWarnLevel::WARNLEVEL_1 && moving_away) continue;

            IntruderInfo info = makeIntruderInfo(obj, snap, zone);
            internal_intruders.push_back(info);
            if (obj.has_lidar_fusion) {
                lidar_backed_intruders.push_back(info);
            }

            if (!isPublishableIntruderCandidate(obj, snap, zone)) continue;
            published_intruders.push_back(info);

            if (static_cast<uint8_t>(zone) > static_cast<uint8_t>(highest_level)) {
                highest_level = zone;
            }
        }
    }

    if (!internal_intruders.empty()) {
        internal_intruders = deduplicateIntrudersByPosition(internal_intruders,
                                                            settings.intruder_dedup_distance_m);
    }

    if (!published_intruders.empty()) {
        published_intruders = deduplicateIntrudersByPosition(published_intruders,
                                                             settings.intruder_dedup_distance_m);
        highest_level = highestWarnLevel(published_intruders);
        // intruders_info 是多目标数组，不存在“主对象”语义；排序只影响展示顺序。
        // 直接按 stable_id / intruder_id 排序，避免历史主目标 latch 改变多目标输出语义。
        std::sort(published_intruders.begin(), published_intruders.end(),
                  [](const IntruderInfo& lhs, const IntruderInfo& rhs) {
                      return lhs.intruder_id < rhs.intruder_id;
                  });
    }
    if (!lidar_backed_intruders.empty()) {
        lidar_backed_intruders = deduplicateIntrudersByPosition(lidar_backed_intruders,
                                                                settings.intruder_dedup_distance_m);
        sortIntrudersByPriority(lidar_backed_intruders);
    }

    if (state != OrsmState::WARNING) {
        if (raw_threat_detected && !published_intruders.empty()) {
            auto pending = mergeCandidateIntruders(published_intruders,
                                                   pending_warning_snapshot_.intruders,
                                                   timestamp,
                                                   settings.intruder_dedup_distance_m,
                                                   observed_track_ids);
            if (!pending.empty()) {
                pending_warning_snapshot_.intruders = pending;
                pending_warning_snapshot_.level = highestWarnLevel(pending);
                pending_warning_snapshot_.timestamp_ns = timestamp;
            }
        } else if (clear_confirm_count_ >= kClearConfirmFrames) {
            pending_warning_snapshot_ = WarningSnapshot{};
        }
    }

    // A confirmed far-image/LiDAR association is already an explicit lease
    // grant from the Inbox path.  It must be visible on fortress_resp in the
    // same cycle; do not make it wait for the ordinary ACTIVE -> WARNING
    // threat-confirmation window.
    const bool direct_lidar_lease_output = direct_lidar_lease_observed;

    // ---- 3. 填充 OrsmStatus 输出信号 ----
    state = sm_.currentState();

    std::vector<IntruderInfo> output_intruders;
    OrsmWarnLevel output_level = OrsmWarnLevel::LEVEL_NONE;
    if (state == OrsmState::WARNING) {
        const auto& previous_output = last_warning_snapshot_.intruders.empty()
                                      ? pending_warning_snapshot_.intruders
                                      : last_warning_snapshot_.intruders;
        output_intruders = last_warning_snapshot_.intruders.empty()
                           ? mergeCandidateIntruders(published_intruders,
                                                     previous_output,
                                                     timestamp,
                                                     settings.intruder_dedup_distance_m,
                                                     observed_track_ids)
                           : updateHeldIntruders(published_intruders,
                                                 previous_output,
                                                 timestamp,
                                                 settings.intruder_dedup_distance_m,
                                                 observed_track_ids);
        output_level = highestWarnLevel(output_intruders);
        if (!output_intruders.empty() && !lidar_backed_intruders.empty()) {
            output_intruders = refreshHeldIntrudersWithLidarBackedCurrent(
                output_intruders,
                lidar_backed_intruders,
                settings.intruder_dedup_distance_m);
            output_intruders = mergePhysicalOutputIntruders(
                output_intruders, settings.intruder_dedup_distance_m);
            output_level = highestWarnLevel(output_intruders);
        }
        // 不使用全部原始 LiDAR 候选回填，以免绕过分区确认策略。
        // 已发布目标在上游短时缺失时由下方 LiDAR 保持逻辑处理。
        // 当前帧出现另一个目标时，不应立即让上一帧仍有LiDAR支撑的物理目标从多目标数组消失。
        // 最多按 kIntruderOutputHoldNs 保留，并保持原始时间戳，避免把不同目标轮流显示成“跳点”。
        if (!output_intruders.empty() && !previous_output.empty()) {
            auto held_previous = holdPreviousIntrudersWithLidar(
                previous_output, timestamp, settings.intruder_dedup_distance_m,
                lidar, false);
            for (const auto& held : held_previous) {
                // 当前同步组以 ORSM 全局轨迹为准，原始点不能恢复已释放的旧 ID。
                // 当前目标全部缺失时，下面的分支仍提供短时 LiDAR 保持。
                if (observed_track_ids.count(held.intruder_id) == 0) continue;
                const bool already_present = std::any_of(
                    output_intruders.begin(), output_intruders.end(),
                    [&](const IntruderInfo& current) {
                        return matchesHeldIntruder(current, held,
                                                   settings.intruder_dedup_distance_m);
                    });
                if (!already_present) output_intruders.push_back(held);
            }
            output_intruders = deduplicateIntrudersByPosition(
                output_intruders, settings.intruder_dedup_distance_m);
            output_intruders = mergePhysicalOutputIntruders(
                output_intruders, settings.intruder_dedup_distance_m);
            sortIntrudersByPriority(output_intruders);
            stabilizeIntruderOrdering(output_intruders, previous_output,
                                      settings.intruder_dedup_distance_m);
            output_level = highestWarnLevel(output_intruders);
        }

        if (output_intruders.empty() && !previous_output.empty() &&
            observed_track_ids.empty() &&
            (raw_threat_detected || !lidar_backed_intruders.empty())) {
            // warning_hold_until_ns_ 仅保持 WARNING 和声光语义，不保持旧坐标。
            output_intruders = holdPreviousIntrudersWithLidar(previous_output,
                                                             timestamp,
                                                             settings.intruder_dedup_distance_m,
                                                             lidar,
                                                             false);
            output_intruders = mergePhysicalOutputIntruders(
                output_intruders, settings.intruder_dedup_distance_m);
            output_level = highestWarnLevel(output_intruders);
        }
        if (!output_intruders.empty()) {
            output_intruders = mergePhysicalOutputIntruders(
                output_intruders, settings.intruder_dedup_distance_m);
            output_level = highestWarnLevel(output_intruders);
            last_warning_snapshot_.intruders = output_intruders;
            last_warning_snapshot_.level = output_level;
            last_warning_snapshot_.timestamp_ns = timestamp;
            pending_warning_snapshot_ = WarningSnapshot{};
        }
    } else if (direct_lidar_lease_output) {
        // 租约发布独立于告警状态切换；非租约候选仍使用常规确认策略。
        output_intruders = published_intruders;
        output_level = highestWarnLevel(output_intruders);
    }

    switch (state) {
    case OrsmState::OFF:
        // ODD 未满足时返回启用失败（3），而不是用户主动关闭（0）。
        output.orsm_setting_st_fb = (odd.sw_enable && orsmEnabled(odd.sw_mode))
                                        ? OrsmSettingType::FUNCTION_LIMITED_ENABLE_FAILED
                                        : OrsmSettingType::OFF;
        break;
    case OrsmState::ACTIVE:
    case OrsmState::WARNING:
        output.orsm_setting_st_fb = odd.sw_mode;
        break;
    case OrsmState::FAILURE:
        output.orsm_setting_st_fb = OrsmSettingType::FUNCTION_LIMITED_ENABLE_FAILED;
        break;
    }

    // OrsmMessageReminder 状态文字提醒
    if (state == OrsmState::ACTIVE || state == OrsmState::WARNING) {
        output.orsm_message_rem = OrsmMessageReminder::SECURITY_MODE_ON;
        if (odd.soc < odd.soc_activate_threshold && odd.soc >= odd.soc_deactivate_threshold) {
            output.orsm_message_rem = OrsmMessageReminder::LOW_BATTERY;
        }
    } else if (state == OrsmState::OFF && odd.sw_enable && orsmEnabled(odd.sw_mode)) {
        if (!odd.power_on) {
            output.orsm_message_rem = OrsmMessageReminder::NOT_IN_ON_READY_STATE_FUNCTION_ENTRY_FAILED;
        } else if (!odd.gear_park) {
            output.orsm_message_rem = OrsmMessageReminder::GEAR_NOT_IN_P_POSITION_FUNCTION_ENTRY_FAILED;
        } else if (odd.soc < odd.soc_activate_threshold) {
            output.orsm_message_rem = OrsmMessageReminder::LOW_BATTERY_FUNCTION_ENTRY_FAILED;
        } else if (odd.speed_kmh > odd.speed_exit_threshold_kmh) {
            output.orsm_message_rem = OrsmMessageReminder::VEHICLE_SPEED_TOO_HIGH_FUNCTION_CANNOT_BE_ACTIVATED;
        }
    } else if (state == OrsmState::FAILURE) {
        if (odd.had_fault) {
            output.orsm_message_rem = OrsmMessageReminder::INTELLIGENT_DRIVING_SYSTEM_MALFUNCTION_FUNCTION_ENTRY_FAILED;
        } else if (odd.sensor_fault) {
            output.orsm_message_rem = OrsmMessageReminder::RADAR_CAMERA_MALFUNCTION_FUNCTION_ENTRY_FAILED;
        }
    }

    // OrsmWarningStatus / OrsmWarnLevel / IntrudersInfo
    if ((state == OrsmState::WARNING || direct_lidar_lease_output) &&
        !output_intruders.empty()) {
        output.orsm_warning_st = OrsmWarningStatus::WARNING_ON;
        output.orsm_warn_level = output_level;
        output.intruders_info = std::move(output_intruders);

        if (output.intruders_info.size() == 1) {
            output.intruders_num = IntrudersNumType::SINGLE_INTRUDER;
        } else {
            output.intruders_num = IntrudersNumType::MULTIPLE_INTRUDERS;
        }
    } else {
        output.orsm_warning_st = OrsmWarningStatus::WARNING_OFF;
        output.orsm_warn_level = OrsmWarnLevel::LEVEL_NONE;
        output.intruders_num = IntrudersNumType::NO_INTRUDER;
        output.intruders_info.clear();
    }

    return output;
}

}  // namespace sentinel
