#include "sentinel_node.h"
#include "orsm/tracking_safety.h"
#include <ad_msg_idl/avp_uss/uss_perception.capnp.h>
#include <ad_log/ad_log.hpp>
#include <iostream>
#include <filesystem>
#include <type_traits>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>
#include <chrono>
#include <limits>
#include <cstdlib>
#include <cctype>
#include <iomanip>
#include <unordered_set>
#include <ctime>
#include <functional>
#include <stdexcept>

namespace fs = std::filesystem;

namespace sentinel {

namespace {

uint64_t absDiffNs(uint64_t a, uint64_t b) {
    return a > b ? a - b : b - a;
}

uint64_t currentSteadyTimeNs() {
    return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count());
}

uint64_t currentSteadyTimeMs() {
    return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count());
}

uint64_t currentSystemTimeMs() {
    return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count());
}

uint64_t extendLeaseExpiryNs(uint64_t current_expiry_ns,
                             uint64_t timestamp_ns,
                             int lease_ms) {
    if (timestamp_ns == 0) return current_expiry_ns;
    const uint64_t duration_ns = static_cast<uint64_t>(std::max(0, lease_ms)) * 1000000ULL;
    const uint64_t max_uint64 = std::numeric_limits<uint64_t>::max();
    const uint64_t requested_expiry_ns = timestamp_ns > max_uint64 - duration_ns
        ? max_uint64
        : timestamp_ns + duration_ns;
    // A shorter evidence refresh must never shorten a longer active lease.
    return std::max(current_expiry_ns, requested_expiry_ns);
}

constexpr uint64_t kThreatSessionMergeNs = 10ULL * 1000000000ULL;
constexpr uint64_t kVideoRecordSuppressWindowNs = 180ULL * 1000000000ULL;
constexpr double kAlgorithmProcessTimeLimitMs = 100.0;
constexpr const char* kAlgorithmProcessTimeMetric = "process.total_ms";

bool isAlgorithmProcessMetric(const std::string& name) {
    return name.rfind("process.", 0) == 0;
}

std::string safeFileToken(const std::string& value) {
    std::string out = value.empty() ? "camera" : value;
    for (char& ch : out) {
        const bool ok = (ch >= 'a' && ch <= 'z') ||
                        (ch >= 'A' && ch <= 'Z') ||
                        (ch >= '0' && ch <= '9') ||
                        ch == '_' || ch == '-';
        if (!ok) ch = '_';
    }
    return out;
}

const char* algorithmProcessMetricDisplayName(const std::string& name) {
    if (name == "process.total_ms") return "总耗时";
    if (name == "process.undistort_ms") return "鱼眼去畸变";
    if (name == "process.detect_ms") return "目标检测";
    if (name == "process.track_fallback_ms") return "目标跟踪/兜底";
    if (name == "process.uss_snapshot_ms") return "USS快照";
    if (name == "process.lidar_merge_ms") return "LiDAR合并";
    if (name == "process.uss_fusion_ms") return "USS视觉融合";
    if (name == "process.lidar_fusion_ms") return "视觉LiDAR融合";
    if (name == "process.orsm_global_track_ms") return "堡垒全局跟踪";
    if (name == "process.sentry_group_track_ms") return "哨兵组级跟踪";
    if (name == "process.orsm_update_publish_ms") return "堡垒状态更新";
    if (name == "process.sentry_alert_state_ms") return "哨兵告警状态";
    if (name == "process.visualize_record_ms") return "可视化/调试保存";
    return "其他处理阶段";
}

bool elapsedAtLeastNs(uint64_t start_ns, uint64_t now_ns, uint64_t duration_ns) {
    return start_ns > 0 && now_ns >= start_ns && (now_ns - start_ns) >= duration_ns;
}

long long mergeVoxelKey(int ix, int iy, int iz) {
    constexpr long long kOffset = 1LL << 20;
    constexpr long long kMask = (1LL << 21) - 1LL;
    return (((static_cast<long long>(ix) + kOffset) & kMask) << 42) |
           (((static_cast<long long>(iy) + kOffset) & kMask) << 21) |
           (((static_cast<long long>(iz) + kOffset) & kMask));
}

std::string topicBasename(const std::string& topic) {
    const auto slash = topic.find_last_of('/');
    return (slash == std::string::npos) ? topic : topic.substr(slash + 1);
}

float nominalVisualObjectHeightM(const std::string& label) {
    if (label == "person" || label == "human" || label == "pedestrian") return 1.70f;
    if (label == "bicycle" || label == "motorcycle") return 1.40f;
    if (label == "vehicle" || label == "car" || label == "sedan") return 1.50f;
    if (label == "truck" || label == "bus" || label == "suv") return 2.20f;
    if (label == "animal" || label == "dog" || label == "cat") return 0.60f;
    if (label == "horse" || label == "cow" || label == "sheep") return 1.40f;
    return 1.0f;
}

float estimateVisualDistanceM(const TrackedObject& obj, int image_height) {
    if (obj.bbox.height <= 0 || image_height <= 0) return 0.f;
    const float effective_focal_px = static_cast<float>(image_height) * 0.45f;
    const float distance = nominalVisualObjectHeightM(obj.label) * effective_focal_px /
                           static_cast<float>(obj.bbox.height);
    return (std::isfinite(distance) && distance > 0.f) ? distance : 0.f;
}

uint64_t pathSizeBytes(const fs::path& path) {
    std::error_code ec;
    if (!fs::exists(path, ec)) return 0;
    if (fs::is_regular_file(path, ec)) {
        const auto size = fs::file_size(path, ec);
        return ec ? 0 : static_cast<uint64_t>(size);
    }
    if (!fs::is_directory(path, ec)) return 0;

    uint64_t total = 0;
    for (fs::recursive_directory_iterator it(path, fs::directory_options::skip_permission_denied, ec), end;
         !ec && it != end; it.increment(ec)) {
        std::error_code item_ec;
        if (!it->is_regular_file(item_ec)) continue;
        const auto size = it->file_size(item_ec);
        if (!item_ec) total += static_cast<uint64_t>(size);
    }
    return total;
}

bool startsWith(const std::string& text, const std::string& prefix) {
    return text.size() >= prefix.size() && text.compare(0, prefix.size(), prefix) == 0;
}

const char* sentryStateNameZh(SentryState state) {
    switch (state) {
    case SentryState::UNAVAILABLE: return "不可用";
    case SentryState::STANDBY:     return "待机";
    case SentryState::ACTIVE:      return "已进入哨兵模式";
    case SentryState::WARNING:     return "哨兵警戒";
    case SentryState::ALARM:       return "哨兵报警";
    }
    return "未知";
}

const char* sentryStateNameEn(SentryState state) {
    switch (state) {
    case SentryState::UNAVAILABLE: return "UNAVAILABLE";
    case SentryState::STANDBY:     return "STANDBY";
    case SentryState::ACTIVE:      return "ACTIVE";
    case SentryState::WARNING:     return "WARNING";
    case SentryState::ALARM:       return "ALARM";
    }
    return "UNKNOWN";
}

const char* threatLevelNameEn(ThreatLevel level) {
    switch (level) {
    case ThreatLevel::NONE:   return "NONE";
    case ThreatLevel::LOW:    return "LOW";
    case ThreatLevel::MEDIUM: return "MEDIUM";
    case ThreatLevel::HIGH:   return "HIGH";
    }
    return "UNKNOWN";
}

const char* obstacleTypeNameEn(SentinelObstacleType type) {
    switch (type) {
    case SentinelObstacleType::UNKNOWN_OBSTACLE:      return "UNKNOWN";
    case SentinelObstacleType::SEDAN:                 return "SEDAN";
    case SentinelObstacleType::SUV:                   return "SUV";
    case SentinelObstacleType::ADULT:                 return "ADULT";
    case SentinelObstacleType::CHILD:                 return "CHILD";
    case SentinelObstacleType::BICYCLE:               return "BICYCLE";
    case SentinelObstacleType::MOTORCYCLE:            return "MOTORCYCLE";
    case SentinelObstacleType::E_BIKE:                return "E_BIKE";
    case SentinelObstacleType::ANIMAL:                return "ANIMAL";
    case SentinelObstacleType::UNIDENTIFIED_COLLISION: return "COLLISION";
    case SentinelObstacleType::UNIDENTIFIED_SCRAPING:  return "SCRAPING";
    }
    return "UNKNOWN";
}

const char* sentryStateFrameToken(SentryState state) {
    switch (state) {
    case SentryState::UNAVAILABLE: return "UNAV";
    case SentryState::STANDBY:     return "STBY";
    case SentryState::ACTIVE:      return "ACT";
    case SentryState::WARNING:     return "WARN";
    case SentryState::ALARM:       return "ALRM";
    }
    return "?";
}

const char* threatLevelFrameToken(ThreatLevel level) {
    switch (level) {
    case ThreatLevel::NONE:   return "NONE";
    case ThreatLevel::LOW:    return "LOW";
    case ThreatLevel::MEDIUM: return "MED";
    case ThreatLevel::HIGH:   return "HIGH";
    }
    return "?";
}

const char* obstacleFrameToken(SentinelObstacleType type) {
    switch (type) {
    case SentinelObstacleType::UNKNOWN_OBSTACLE:       return "-";
    case SentinelObstacleType::SEDAN:                  return "SED";
    case SentinelObstacleType::SUV:                    return "SUV";
    case SentinelObstacleType::ADULT:                  return "ADU";
    case SentinelObstacleType::CHILD:                  return "CHD";
    case SentinelObstacleType::BICYCLE:                return "BIKE";
    case SentinelObstacleType::MOTORCYCLE:             return "MOTO";
    case SentinelObstacleType::E_BIKE:                 return "EBIK";
    case SentinelObstacleType::ANIMAL:                 return "ANI";
    case SentinelObstacleType::UNIDENTIFIED_COLLISION: return "COL";
    case SentinelObstacleType::UNIDENTIFIED_SCRAPING:  return "SCR";
    }
    return "?";
}

const char* threatReasonFrameToken(const std::string& reason) {
    if (reason.empty()) return "-";
    if (reason.find("锁存") != std::string::npos) return "IMU_LATCH";
    if (reason.find("大幅震动") != std::string::npos ||
        reason.find("碰撞") != std::string::npos) return "IMU_HIT";
    if (reason.find("门把手") != std::string::npos ||
        reason.find("小幅震动") != std::string::npos) return "IMU_VIB";
    if (reason.find("USS") != std::string::npos) return "USS_NEAR";
    if (reason.find("检测到人") != std::string::npos ||
        reason.find("行人") != std::string::npos) return "P<3m";
    if (reason.find("骑行") != std::string::npos) return "RIDE";
    if (reason.find("移动目标") != std::string::npos ||
        reason.find("连续3帧") != std::string::npos) return "MV<3m";
    if (reason.find("低威胁") != std::string::npos ||
        reason.find("持续观察") != std::string::npos) return "WATCH";
    if (reason.find("严重") != std::string::npos) return "HIGH";
    if (reason.find("一般") != std::string::npos) return "MED";
    return "OTH";
}

const char* orsmStateNameZh(OrsmState state) {
    switch (state) {
    case OrsmState::OFF:     return "堡垒关闭";
    case OrsmState::ACTIVE:  return "已进入堡垒模式";
    case OrsmState::WARNING: return "堡垒预警";
    case OrsmState::FAILURE: return "堡垒故障";
    }
    return "未知";
}

const char* sentryLifecycleActionZh(SentryState from, SentryState to) {
    if (to == SentryState::ACTIVE &&
        (from == SentryState::UNAVAILABLE || from == SentryState::STANDBY)) {
        return "进入哨兵模式";
    }
    if ((from == SentryState::ACTIVE || from == SentryState::WARNING || from == SentryState::ALARM) &&
        (to == SentryState::UNAVAILABLE || to == SentryState::STANDBY)) {
        return "退出哨兵模式";
    }
    if (to == SentryState::WARNING) return "哨兵进入警戒";
    if (to == SentryState::ALARM) return "哨兵进入报警";
    if ((from == SentryState::WARNING || from == SentryState::ALARM) &&
        to == SentryState::ACTIVE) {
        return "哨兵解除预警";
    }
    return "哨兵状态变化";
}

const char* orsmLifecycleActionZh(OrsmState from, OrsmState to) {
    if (from == OrsmState::OFF && to == OrsmState::ACTIVE) return "进入堡垒模式";
    if ((from == OrsmState::ACTIVE || from == OrsmState::WARNING) && to == OrsmState::OFF) {
        return "退出堡垒模式";
    }
    if (to == OrsmState::WARNING) return "堡垒进入预警";
    if (from == OrsmState::WARNING && to == OrsmState::ACTIVE) return "堡垒解除预警";
    if (to == OrsmState::FAILURE) return "堡垒进入故障";
    if (from == OrsmState::FAILURE && to == OrsmState::ACTIVE) return "堡垒故障恢复";
    return "堡垒状态变化";
}

bool isEventRecordDirectory(const fs::path& path) {
    std::error_code ec;
    if (!fs::is_directory(path, ec)) return false;
    if (fs::exists(path / "manifest.json", ec)) return true;
    const std::string name = path.filename().string();
    return startsWith(name, "Sentry-") || startsWith(name, "event_");
}

std::string canonicalPathString(const fs::path& path) {
    std::error_code ec;
    const auto canonical = fs::weakly_canonical(path, ec);
    if (!ec) return canonical.string();
    std::error_code abs_ec;
    const auto absolute = fs::absolute(path, abs_ec);
    return (abs_ec ? path : absolute).string();
}

void enrichVisualGeometry(TrackedObject& obj,
                          const FisheyeCalib* calib,
                          int image_width,
                          int image_height) {
    if (obj.has_lidar_fusion || obj.distance_m > 0.f) return;

    cv::Vec2d calibrated_ray_xy(0.0, 0.0);
    cv::Vec2d calibrated_origin_xy(0.0, 0.0);
    bool has_calibrated_ray = false;

    // The detector runs on the rectified image, so K_new maps its pixels to a camera ray.
    // Intersect the bbox bottom-centre ray with the vehicle ground plane using the calibrated
    // camera->car transform.  This places rear/left/right visual-only observations in their
    // correct vehicle quadrant instead of assigning every camera a positive vehicle X.
    if (calib && !calib->K_new.empty() && !calib->extrinsic.empty() &&
        calib->K_new.rows == 3 && calib->K_new.cols == 3 &&
        calib->extrinsic.rows == 4 && calib->extrinsic.cols == 4) {
        cv::Mat K_new, T_cam_to_car;
        calib->K_new.convertTo(K_new, CV_64F);
        calib->extrinsic.convertTo(T_cam_to_car, CV_64F);
        const double fx = K_new.at<double>(0, 0);
        const double fy = K_new.at<double>(1, 1);
        const double cx = K_new.at<double>(0, 2);
        const double cy = K_new.at<double>(1, 2);
        const double u = std::clamp(
            static_cast<double>(obj.bbox.x) + obj.bbox.width * 0.5,
            0.0, static_cast<double>(std::max(0, image_width - 1)));
        const double v = std::clamp(
            static_cast<double>(obj.bbox.y + obj.bbox.height),
            0.0, static_cast<double>(std::max(0, image_height - 1)));
        if (fx > 1e-6 && fy > 1e-6) {
            const cv::Vec3d ray_cam((u - cx) / fx, (v - cy) / fy, 1.0);
            const cv::Matx33d R(
                T_cam_to_car.at<double>(0, 0), T_cam_to_car.at<double>(0, 1), T_cam_to_car.at<double>(0, 2),
                T_cam_to_car.at<double>(1, 0), T_cam_to_car.at<double>(1, 1), T_cam_to_car.at<double>(1, 2),
                T_cam_to_car.at<double>(2, 0), T_cam_to_car.at<double>(2, 1), T_cam_to_car.at<double>(2, 2));
            const cv::Vec3d origin(T_cam_to_car.at<double>(0, 3),
                                   T_cam_to_car.at<double>(1, 3),
                                   T_cam_to_car.at<double>(2, 3));
            const cv::Vec3d ray_car = R * ray_cam;
            const double ray_xy_norm = std::hypot(ray_car[0], ray_car[1]);
            if (ray_xy_norm > 1e-6) {
                calibrated_ray_xy = cv::Vec2d(ray_car[0] / ray_xy_norm,
                                               ray_car[1] / ray_xy_norm);
                calibrated_origin_xy = cv::Vec2d(origin[0], origin[1]);
                has_calibrated_ray = true;
            }
            if (ray_car[2] < -1e-4 && origin[2] > 0.05) {
                const double scale = -origin[2] / ray_car[2];
                const cv::Vec3d ground = origin + ray_car * scale;
                const double range = std::hypot(ground[0], ground[1]);
                if (scale > 0.0 && std::isfinite(range) && range >= 0.3 && range <= 30.0) {
                    obj.position_3d = cv::Point3f(static_cast<float>(ground[0]),
                                                  static_cast<float>(ground[1]), 0.f);
                    obj.distance_m = static_cast<float>(range);
                    obj.has_visual_ground_position = true;
                    obj.visual_ground_position_3d = obj.position_3d;
                    obj.visual_ground_distance_m = obj.distance_m;
                    return;
                }
            }
        }
    }

    const float distance = estimateVisualDistanceM(obj, image_height);
    if (distance <= 0.f) return;
    obj.distance_m = distance;

    if (has_calibrated_ray) {
        // The ground-plane intersection can fall beyond the supported range for
        // a tiny box near the horizon. Keep the bbox-height range estimate, but
        // place it along this camera's calibrated direction in vehicle space.
        const double dx = calibrated_ray_xy[0];
        const double dy = calibrated_ray_xy[1];
        const double ox = calibrated_origin_xy[0];
        const double oy = calibrated_origin_xy[1];
        const double range = static_cast<double>(distance);
        const double b = ox * dx + oy * dy;
        const double discriminant = b * b + range * range - ox * ox - oy * oy;
        if (discriminant >= 0.0) {
            const double along_ray = -b + std::sqrt(discriminant);
            obj.position_3d = cv::Point3f(
                static_cast<float>(ox + along_ray * dx),
                static_cast<float>(oy + along_ray * dy), 0.f);
            obj.has_visual_ground_position = true;
            obj.visual_ground_position_3d = obj.position_3d;
            obj.visual_ground_distance_m = distance;
            return;
        }
    }

    // Calibration was unavailable/invalid. Keep the legacy approximation as a clearly marked
    // fallback; callers must not treat this as a reliable cross-camera vehicle coordinate.
    const float cx = obj.bbox.x + obj.bbox.width * 0.5f;
    const float norm_x = image_width > 0 ? (cx - image_width * 0.5f) / (image_width * 0.5f) : 0.f;
    obj.position_3d = {distance, norm_x * distance * 0.6f, 0.f};
}

int cameraScopedTrackId(const std::string& camera_name, int track_id) {
    if (track_id < 0) return track_id;
    const uint32_t camera_hash = static_cast<uint32_t>(std::hash<std::string>{}(camera_name) % 9000U);
    return static_cast<int>((camera_hash + 1U) * 100000U + static_cast<uint32_t>(track_id % 100000));
}

std::string shortCameraToken(const std::string& camera_name) {
    if (camera_name.find("front") != std::string::npos) return "F";
    if (camera_name.find("rear") != std::string::npos) return "R";
    if (camera_name.find("left") != std::string::npos) return "L";
    if (camera_name.find("right") != std::string::npos) return "Ri";
    if (camera_name.empty()) return "-";
    return camera_name.size() > 8 ? camera_name.substr(0, 8) : camera_name;
}

std::string cameraNameFromScopedThreatId(int threat_id,
                                         const std::vector<std::string>& camera_names) {
    if (threat_id <= 0) return "";
    const int local = threat_id % 100000;
    for (const auto& name : camera_names) {
        if (cameraScopedTrackId(name, local) == threat_id) return name;
    }
    const int prefix = threat_id / 100000;
    for (const auto& name : camera_names) {
        const uint32_t camera_hash =
            static_cast<uint32_t>(std::hash<std::string>{}(name) % 9000U);
        if (static_cast<int>(camera_hash + 1U) == prefix) return name;
    }
    return "";
}

std::vector<TrackedObject> stableVisualTargets(const std::vector<TrackedObject>& input,
                                               bool allow_negative_fallback,
                                               int min_age,
                                               int max_lost,
                                               float min_confidence) {
    std::vector<TrackedObject> out;
    out.reserve(input.size());
    const int effective_min_age = std::max(1, min_age);
    const int effective_max_lost = std::max(0, max_lost);
    for (const auto& obj : input) {
        if (obj.confidence < min_confidence) continue;
        if (obj.track_id < 0) {
            if (allow_negative_fallback && obj.lost == 0 && obj.age >= effective_min_age) {
                out.push_back(obj);
            }
            continue;
        }
        if (obj.age < effective_min_age) continue;
        if (obj.lost > effective_max_lost) continue;
        out.push_back(obj);
    }
    return out;
}

float pointDistance2D(const cv::Point3f& a, const cv::Point3f& b) {
    const float dx = a.x - b.x;
    const float dy = a.y - b.y;
    return std::sqrt(dx * dx + dy * dy);
}

float circleOverlapRatioOfFirst(const cv::Point2f& first_center,
                                float first_radius,
                                const cv::Point2f& second_center,
                                float second_radius) {
    if (!(first_radius > 0.f) || !(second_radius > 0.f)) return 0.f;
    const double dx = static_cast<double>(first_center.x - second_center.x);
    const double dy = static_cast<double>(first_center.y - second_center.y);
    const double distance = std::hypot(dx, dy);
    const double r1 = static_cast<double>(first_radius);
    const double r2 = static_cast<double>(second_radius);
    if (distance >= r1 + r2) return 0.f;
    if (distance <= std::abs(r1 - r2)) {
        if (r1 <= r2) return 1.f;
        return static_cast<float>((r2 * r2) / (r1 * r1));
    }

    const double d2 = distance * distance;
    const double r1_2 = r1 * r1;
    const double r2_2 = r2 * r2;
    const double first_angle = std::acos(std::clamp(
        (d2 + r1_2 - r2_2) / (2.0 * distance * r1), -1.0, 1.0));
    const double second_angle = std::acos(std::clamp(
        (d2 + r2_2 - r1_2) / (2.0 * distance * r2), -1.0, 1.0));
    const double radical = std::max(
        0.0,
        (-distance + r1 + r2) * (distance + r1 - r2) *
        (distance - r1 + r2) * (distance + r1 + r2));
    const double intersection =
        r1_2 * first_angle + r2_2 * second_angle - 0.5 * std::sqrt(radical);
    const double first_area = CV_PI * r1_2;
    return first_area > 1.e-9
        ? static_cast<float>(std::clamp(intersection / first_area, 0.0, 1.0))
        : 0.f;
}

float pointNorm2D(const cv::Point3f& p) {
    return std::sqrt(p.x * p.x + p.y * p.y);
}

bool finitePoint2D(const cv::Point3f& p) {
    return std::isfinite(p.x) && std::isfinite(p.y);
}

bool orsmBiologicalLabel(const std::string& label) {
    return label == "person" || label == "human" || label == "pedestrian" ||
           label == "animal" || label == "dog" || label == "cat" ||
           label == "horse" || label == "cow" || label == "sheep";
}

// A track has a "real" class once any detector/difference box has given it a
// usable label.  Such a label is treated as write-once: an unknown track may
// still be upgraded to a real class, but a real class must never be replaced
// (neither by "unknown" nor by a different class).  See
// cfg_.orsm_class_label_lock_enabled.
bool isUnknownOrsmLabel(const std::string& label, int class_id) {
    return class_id < 0 || label.empty() || label == "unknown";
}

bool sameOrsmBiologicalClass(int lhs_class_id,
                             const std::string& lhs_label,
                             int rhs_class_id,
                             const std::string& rhs_label) {
    if (!orsmBiologicalLabel(lhs_label) || !orsmBiologicalLabel(rhs_label)) return false;
    if (lhs_class_id >= 0 && rhs_class_id >= 0) return lhs_class_id == rhs_class_id;
    return lhs_label == rhs_label;
}

float medianFloat(std::vector<float>& values) {
    if (values.empty()) return 0.f;
    const size_t mid = values.size() / 2;
    std::nth_element(values.begin(), values.begin() + mid, values.end());
    float m = values[mid];
    if (values.size() % 2 == 0 && mid > 0) {
        const auto lower = std::max_element(values.begin(), values.begin() + mid);
        if (lower != values.begin() + mid) m = 0.5f * (m + *lower);
    }
    return m;
}

void logSentryUssFusionBeforeEvaluate(const std::vector<TrackedObject>& tracks,
                                      const std::string& camera_name,
                                      uint64_t frame_timestamp_ns,
                                      SentryState state,
                                      bool suppress_sentry,
                                      const UssPointCloudSnapshot& uss_cloud) {
    static std::atomic<uint64_t> log_count{0};
    int fused_count = 0;
    int first_track_id = -1;
    int first_cluster_id = -1;
    float first_uss_distance = 0.f;
    for (const auto& obj : tracks) {
        if (!obj.has_uss_fusion) continue;
        fused_count++;
        if (first_track_id < 0) {
            first_track_id = obj.track_id;
            first_cluster_id = obj.uss_cluster_id;
            first_uss_distance = obj.uss_distance_m;
        }
    }

    const uint64_t idx = ++log_count;
    if (fused_count > 0 || idx <= 20 || idx % 120 == 0) {
        const double uss_delta_ms = uss_cloud.valid()
            ? static_cast<double>(absDiffNs(uss_cloud.timestamp_ns, frame_timestamp_ns)) / 1.0e6
            : -1.0;
        std::cout << "[SentinelNode][PreAlertEvaluate] camera=" << camera_name
                  << " state=" << static_cast<int>(state)
                  << " suppress=" << suppress_sentry
                  << " tracks=" << tracks.size()
                  << " uss_fused_tracks=" << fused_count
                  << " first_track=" << first_track_id
                  << " first_cluster=" << first_cluster_id
                  << " first_uss_dist=" << first_uss_distance
                  << " uss_cloud_valid=" << uss_cloud.valid()
                  << " uss_points=" << uss_cloud.points.size()
                  << " uss_delta_ms=" << uss_delta_ms
                  << std::endl;
    }
}

SentinelResStatusView makeSentinelResStatus(const OddConditions& odd, const AlertEvent& evt) {
    SentinelResStatusView status;
    status.sentinel_odd_status = odd.oddStatus();
    status.sentinel_enable_failed = odd.enableFailedReason();
    // 周期状态帧/告警状态帧不是哨兵请求响应帧；响应类型仅在
    // publishSentryRequestResponse() 中按实际 SentinelRequest 赋值。
    status.sentinel_res_msg_type = SentinelResMsgType::UNKNOWN_SENTINEL_RES;
    status.sentinel_general_threat = evt.sentinel_general_threat;
    status.sentinel_severe_threat = evt.sentinel_severe_threat;
    status.sentinel_audio_visual_alarm = evt.sentinel_audio_visual_alarm;
    status.sentinel_obstacle_type = evt.sentinel_obstacle_type;
    return status;
}

template <typename T>
using void_t = void;

#define DEFINE_HAS_GETTER(NAME, METHOD) \
    template <typename T, typename = void> struct NAME : std::false_type {}; \
    template <typename T> struct NAME<T, void_t<decltype(std::declval<T>().METHOD())>> : std::true_type {}

DEFINE_HAS_GETTER(HasGetSpeed, getSpeed);
DEFINE_HAS_GETTER(HasGetVehicleSpeed, getVehicleSpeed);
DEFINE_HAS_GETTER(HasGetVehSpeed, getVehSpeed);
DEFINE_HAS_GETTER(HasGetSoc, getSoc);
DEFINE_HAS_GETTER(HasGetBmsSoc, getBmsSoc);
DEFINE_HAS_GETTER(HasGetBatterySoc, getBatterySoc);
DEFINE_HAS_GETTER(HasGetGearPark, getGearPark);
DEFINE_HAS_GETTER(HasGetGearP, getGearP);
DEFINE_HAS_GETTER(HasGetParkingGear, getParkingGear);
DEFINE_HAS_GETTER(HasGetHandbrakeOn, getHandbrakeOn);
DEFINE_HAS_GETTER(HasGetParkingBrakeOn, getParkingBrakeOn);
DEFINE_HAS_GETTER(HasGetEpbActive, getEpbActive);
DEFINE_HAS_GETTER(HasGetPowerOff, getPowerOff);
DEFINE_HAS_GETTER(HasGetVehiclePowerOff, getVehiclePowerOff);
DEFINE_HAS_GETTER(HasGetSensorFault, getSensorFault);
DEFINE_HAS_GETTER(HasGetNoSensorFault, getNoSensorFault);
DEFINE_HAS_GETTER(HasGetNoOccupant, getNoOccupant);
DEFINE_HAS_GETTER(HasGetCabinEmpty, getCabinEmpty);
DEFINE_HAS_GETTER(HasGetBcmDriverSeatSt, getBcmDriverSeatSt);
DEFINE_HAS_GETTER(HasGetDkmBleKeyPosn, getDkmBleKeyPosn);
DEFINE_HAS_GETTER(HasGetBcmUsageMode, getBcmUsageMode);
DEFINE_HAS_GETTER(HasGetMultiVehicleTeamSetting, getMultiVehicleTeamSetting);

#undef DEFINE_HAS_GETTER

template <typename V>
float numericValue(V v) { return static_cast<float>(v); }

template <typename V>
bool boolValue(V v) { return static_cast<bool>(v); }

template <typename Reader>
std::optional<float> readSpeedKmh(const Reader& r) {
    if constexpr (HasGetVehicleSpeed<Reader>::value) return numericValue(r.getVehicleSpeed());
    else if constexpr (HasGetVehSpeed<Reader>::value) return numericValue(r.getVehSpeed());
    else if constexpr (HasGetSpeed<Reader>::value) return numericValue(r.getSpeed());
    else return std::nullopt;
}

template <typename Reader>
std::optional<float> readSocPercent(const Reader& r) {
    if constexpr (HasGetBmsSoc<Reader>::value) return numericValue(r.getBmsSoc());
    else if constexpr (HasGetBatterySoc<Reader>::value) return numericValue(r.getBatterySoc());
    else if constexpr (HasGetSoc<Reader>::value) return numericValue(r.getSoc());
    else return std::nullopt;
}

std::optional<float> normalizeSocPercent(float raw_soc) {
    if (!std::isfinite(raw_soc) || raw_soc < 0.f) return std::nullopt;
    if (raw_soc > 0.f && raw_soc <= 1.f) return raw_soc * 100.f;
    if (raw_soc > 100.f && raw_soc <= 1000.f) return raw_soc * 0.1f;
    if (raw_soc <= 100.f) return raw_soc;
    return std::nullopt;
}

template <typename Reader>
std::optional<bool> readGearPark(const Reader& r) {
    if constexpr (HasGetGearPark<Reader>::value) return boolValue(r.getGearPark());
    else if constexpr (HasGetGearP<Reader>::value) return boolValue(r.getGearP());
    else if constexpr (HasGetParkingGear<Reader>::value) return boolValue(r.getParkingGear());
    else return std::nullopt;
}

template <typename Reader>
std::optional<bool> readHandbrakeOn(const Reader& r) {
    if constexpr (HasGetHandbrakeOn<Reader>::value) return boolValue(r.getHandbrakeOn());
    else if constexpr (HasGetParkingBrakeOn<Reader>::value) return boolValue(r.getParkingBrakeOn());
    else if constexpr (HasGetEpbActive<Reader>::value) return boolValue(r.getEpbActive());
    else return std::nullopt;
}

template <typename Reader>
std::optional<bool> readPowerOff(const Reader& r) {
    if constexpr (HasGetPowerOff<Reader>::value) return boolValue(r.getPowerOff());
    else if constexpr (HasGetVehiclePowerOff<Reader>::value) return boolValue(r.getVehiclePowerOff());
    else return std::nullopt;
}

template <typename Reader>
std::optional<bool> readNoSensorFault(const Reader& r) {
    if constexpr (HasGetNoSensorFault<Reader>::value) return boolValue(r.getNoSensorFault());
    else if constexpr (HasGetSensorFault<Reader>::value) return !boolValue(r.getSensorFault());
    else return std::nullopt;
}

template <typename Reader>
std::optional<bool> readNoOccupant(const Reader& r) {
    if constexpr (HasGetNoOccupant<Reader>::value) return boolValue(r.getNoOccupant());
    else if constexpr (HasGetCabinEmpty<Reader>::value) return boolValue(r.getCabinEmpty());
    else return std::nullopt;
}

template <typename Reader>
std::optional<bool> readDriverSeatEmpty(const Reader& r) {
    if constexpr (HasGetBcmDriverSeatSt<Reader>::value) {
        // bcm_driver_seat_st: 0=无人/未占用，非0=有人/占用（以信号清单口径接入，枚举待实车/IDL确认）
        return static_cast<int>(r.getBcmDriverSeatSt()) == 0;
    } else {
        return std::nullopt;
    }
}

template <typename Reader>
std::optional<bool> readKeyFar(const Reader& r) {
    if constexpr (HasGetDkmBleKeyPosn<Reader>::value) {
        // dkm_ble_key_posn: 0=unknown, 1=inside, 2=far (>10m)
        return static_cast<int>(r.getDkmBleKeyPosn()) == 2;
    } else {
        return std::nullopt;
    }
}

template <typename Reader>
std::optional<bool> readNonOtaMode(const Reader& r) {
    if constexpr (HasGetBcmUsageMode<Reader>::value) {
        // bcm_usage_mode: 0=Normal mode, 非0=OTA/Factory/Transport mode
        return static_cast<int>(r.getBcmUsageMode()) == 0;
    } else {
        return std::nullopt;
    }
}

template <typename Reader>
std::optional<MultiVehicleTeamSettingEnum> readMultiVehicleTeamSetting(const Reader& r) {
    if constexpr (HasGetMultiVehicleTeamSetting<Reader>::value) {
        return static_cast<MultiVehicleTeamSettingEnum>(
            static_cast<uint16_t>(r.getMultiVehicleTeamSetting()));
    } else {
        return std::nullopt;
    }
}

std::string summarizeOrsmIntruders(const std::vector<IntruderInfo>& intruders) {
    if (intruders.empty()) return "[]";

    std::ostringstream os;
    os << std::fixed << std::setprecision(2) << "[";
    for (size_t i = 0; i < intruders.size(); ++i) {
        const auto& intruder = intruders[i];
        const float speed = std::hypot(intruder.intruder_velocity_x,
                                       intruder.intruder_velocity_y);
        if (i > 0) os << "; ";
        os << "#" << intruder.intruder_id
           << " type=" << static_cast<int>(intruder.intruder_type)
           << " zone=" << static_cast<int>(intruder.zone)
           << " pos=(" << intruder.intruder_pose_x
           << "," << intruder.intruder_pose_y << ")"
           << " vel=(" << intruder.intruder_velocity_x
           << "," << intruder.intruder_velocity_y << ")"
           << " speed=" << speed;
    }
    os << "]";
    return os.str();
}

}  // namespace

SentinelNode::SentinelNode(const SentinelConfig& cfg)
    : cfg_(cfg)
{}

bool SentinelNode::start(int argc, char* argv[]) {
    if (started_) return true;

    std::cout << "[SentinelNode] Starting..." << std::endl;
    std::cout << "[SentinelNode][VisualLiDARGroundGate] enabled="
              << (cfg_.orsm_visual_lidar_ground_gate_enabled ? 1 : 0)
              << " min_m=" << cfg_.orsm_visual_lidar_ground_gate_min_m
              << " range_ratio=" << cfg_.orsm_visual_lidar_ground_gate_range_ratio
              << std::endl;
    std::cout << "[SentinelNode][LiDARAssociation] visual_roi="
              << (cfg_.orsm_lidar_visual_roi_enabled ? 1 : 0)
              << " history_anchor=" << (cfg_.orsm_lidar_history_anchor_enabled ? 1 : 0)
              << " history_max_age_ms=" << cfg_.orsm_lidar_history_anchor_max_age_ms
              << " history_gate_m=" << cfg_.orsm_lidar_history_anchor_gate_m
              << std::endl;
    std::cout << "[SentinelNode][FrontFixedGroundPlane] enabled="
              << (cfg_.orsm_lidar_primary_front_fixed_ground_plane_enabled ? 1 : 0)
              << " a=" << cfg_.orsm_lidar_primary_front_fixed_ground_plane_a
              << " b=" << cfg_.orsm_lidar_primary_front_fixed_ground_plane_b
              << " c=" << cfg_.orsm_lidar_primary_front_fixed_ground_plane_c
              << " min_height_m=" << cfg_.orsm_lidar_primary_front_fixed_ground_plane_min_height_m
              << " max_height_m=" << cfg_.orsm_lidar_primary_max_z_m
              << std::endl;
    std::cout << "[SentinelNode][LiDARTimeSync] mode=" << cfg_.lidar_time_sync_mode
              << " effective_max_age_ms=" << cfg_.effectiveLidarMaxAgeMs()
              << " real_vehicle_max_age_ms=" << cfg_.lidar_max_age_ms
              << " bag_replay_max_age_ms=" << cfg_.lidar_bag_replay_max_age_ms
              << std::endl;
    if (cfg_.lidar_time_sync_mode == "bag_replay") {
        std::cout << "[SentinelNode][LiDARTimeSync][Warn] bag_replay mode is for offline bag validation only; "
                  << "fix/check LiDAR-Camera timestamp synchronization before real-vehicle deployment."
                  << std::endl;
    }

    // 1. 初始化 RSCL
    RsclAccess::Instance()->init(argc, argv, "sentinel_node");

    // CUDA shared-memory subscriber construction is sensitive to RSCL runtime state.
    // Create CUDA camera subscribers before starting high-rate normal subscribers
    // (vehicle/USS/LiDAR), otherwise callbacks from those topics can race with
    // CudaSubscriber construction and corrupt the RSCL/CUDA allocator state.
    ImuDetectionConfig imu_detection_cfg;
    imu_detection_cfg.highpass_cutoff_hz = cfg_.imu_highpass_cutoff_hz;
    imu_detection_cfg.rms_window_ms = cfg_.imu_rms_window_ms;
    imu_detection_cfg.min_duration_ms = cfg_.imu_vibration_min_duration_ms;
    imu_detection_cfg.latch_ms = cfg_.imu_vibration_latch_ms;
    imu_detection_cfg.accel_rms_threshold = cfg_.imu_accel_rms_threshold;
    imu_detection_cfg.gyro_rms_threshold = cfg_.imu_gyro_rms_threshold;
    imu_detection_cfg.severe_accel_rms_threshold = cfg_.imu_accel_severe_rms_threshold;
    imu_detection_cfg.severe_gyro_rms_threshold = cfg_.imu_gyro_severe_rms_threshold;
    imu_detection_cfg.accel_guard_rms = cfg_.imu_accel_guard_rms;
    imu_ = std::make_shared<ImuSubscriber>(imu_detection_cfg);
    const CameraInputMode camera_input_mode = parseCameraInputMode(cfg_.camera_input_mode);
    const HevcDecoderBackend camera_decoder_backend =
        parseHevcDecoderBackend(cfg_.camera_decoder_backend);
    CameraDecodeDebugConfig decode_debug_cfg;
    decode_debug_cfg.enabled = cfg_.camera_decode_debug_enabled;
    decode_debug_cfg.output_dir = cfg_.camera_decode_debug_dir;
    decode_debug_cfg.max_frames_per_camera = cfg_.camera_decode_debug_max_frames_per_camera;
    decode_debug_cfg.save_encoded_hevc = cfg_.camera_decode_debug_save_hevc;
    decode_debug_cfg.save_decoded_bgr = cfg_.camera_decode_debug_save_decoded_bgr;
    if (decode_debug_cfg.enabled) {
        std::cout << "[SentinelNode][CameraDecodeDebug] enabled dir="
                  << decode_debug_cfg.output_dir
                  << " max_frames_per_camera=" << decode_debug_cfg.max_frames_per_camera
                  << " save_node_images=" << (cfg_.camera_decode_debug_save_node_images ? 1 : 0)
                  << std::endl;
    }
    const bool early_cuda_camera_subscribe = (camera_input_mode == CameraInputMode::CUDA_FRAME);
    if (early_cuda_camera_subscribe) {
        std::cout << "[SentinelNode] Camera input mode: "
                  << cameraInputModeName(camera_input_mode)
                  << " (early subscribe before high-rate topics)" << std::endl;
        for (size_t i = 0; i < cfg_.camera_topics.size(); ++i) {
            const auto& topic = cfg_.camera_topics[i];
            std::string cam_name = (i < cfg_.camera_names.size()) ? cfg_.camera_names[i] : topicBasename(topic);
            auto cam = std::make_shared<CameraSubscriber>(imu_,
                                                           cfg_.camera_width,
                                                           cfg_.camera_height,
                                                           cfg_.gpu_id,
                                                           camera_input_mode,
                                                           camera_decoder_backend,
                                                           decode_debug_cfg);
            cam->setFrameCallback([this](const DecodedFrame& f) { onFrame(f); });
            cam->subscribe(topic, cam_name);
            cameras_.push_back(cam);
        }
        if (!cameras_.empty()) camera_ = cameras_.front();
    }

    // 2. 车辆/IVI/TBOX 状态订阅：更新 ODD 快照与模式设置闭环。
    RsclAccess::Instance()->subscribe<senseAD::msg::vehicle::VehicleReport>(
        cfg_.vehicle_report_topic, &SentinelNode::onVehicleReport, this);
#ifndef SENTINEL_DIAGNOSTIC_NO_L3
    RsclAccess::Instance()->subscribe<senseAD::msg::l3::ChassisInfoM18V2>(
        cfg_.chassis_info_topic, &SentinelNode::onChassisInfo, this);
    RsclAccess::Instance()->subscribe<senseAD::msg::l3::SentinelRequest>(
        cfg_.sentry_request_topic, &SentinelNode::onSentryRequest, this);
    RsclAccess::Instance()->subscribe<senseAD::msg::l3::SetOffRoadMdInfoReq>(
        cfg_.orsm_setting_topic, &SentinelNode::onOrsmSettingRequest, this);
    RsclAccess::Instance()->subscribe<senseAD::msg::l3::OffRoadSetInfoStruct>(
        cfg_.orsm_set_info_topic, &SentinelNode::onOrsmSetInfo, this);
#else
    std::cout << "[SentinelNode][Diagnostic] L3 request/status subscriptions disabled" << std::endl;
#endif

    // 3. 传感器标定参数加载（LiDAR / IMU / 车辆参数）
    sensor_calib_ = std::make_shared<SensorCalibManager>();
    sensor_calib_->loadAll(cfg_.calib_base_dir);

    // 4. IMU 订阅器（应用标定补偿）
    if (!imu_) {
        imu_ = std::make_shared<ImuSubscriber>(imu_detection_cfg);
    }
    if (auto* imu_calib = sensor_calib_->getImu()) {
        ImuBiasCompensation comp;
        comp.accel_bias_x  = static_cast<float>(imu_calib->accel_bias.x);
        comp.accel_bias_y  = static_cast<float>(imu_calib->accel_bias.y);
        comp.accel_bias_z  = static_cast<float>(imu_calib->accel_bias.z);
        comp.gyro_bias_x   = static_cast<float>(imu_calib->gyro_bias.x);
        comp.gyro_bias_y   = static_cast<float>(imu_calib->gyro_bias.y);
        comp.gyro_bias_z   = static_cast<float>(imu_calib->gyro_bias.z);
        comp.accel_scale_x = static_cast<float>(imu_calib->accel_scale.x);
        comp.accel_scale_y = static_cast<float>(imu_calib->accel_scale.y);
        comp.accel_scale_z = static_cast<float>(imu_calib->accel_scale.z);
        comp.gyro_scale_x  = static_cast<float>(imu_calib->gyro_scale.x);
        comp.gyro_scale_y  = static_cast<float>(imu_calib->gyro_scale.y);
        comp.gyro_scale_z  = static_cast<float>(imu_calib->gyro_scale.z);
        imu_->setCalibration(comp);
        std::cout << "[SentinelNode] IMU bias compensation applied" << std::endl;
    }
    imu_->subscribe(cfg_.imu_topic);

    // 4. USS 超声波订阅器
    uss_ = std::make_shared<UssSubscriber>(cfg_.uss_alert_threshold_mm);
    uss_->setAlertCallback([this](const UssData& /*data*/, uint16_t min_dist) {
        std::cout << "[SentinelNode][USS] Proximity alert! min_dist="
                  << min_dist << "mm" << std::endl;
    });
    uss_->subscribe(cfg_.uss_pdc_topic, cfg_.uss_meb_topic);
    const bool needs_uss_fused = cfg_.enable_uss_vision_fusion;
    if (needs_uss_fused) {
        RsclAccess::Instance()->subscribe<senseAD::msg::avp_perception::UssFusedStaticMap>(
            cfg_.uss_fused_topic, &SentinelNode::onUssFusedStaticMap, this);
        if (!cfg_.uss_fused_legacy_topic.empty() && cfg_.uss_fused_legacy_topic != cfg_.uss_fused_topic) {
            RsclAccess::Instance()->subscribe<senseAD::msg::avp_perception::UssFusedStaticMap>(
                cfg_.uss_fused_legacy_topic, &SentinelNode::onUssFusedStaticMap, this);
        }
    } else {
        std::cout << "[SentinelNode] USS fused subscription disabled; PDC/MEB remain subscribed." << std::endl;
    }

    // 6. LiDAR 订阅器（支持多路 LiDAR）。yaw=0兼容已是自车坐标系的输入；
    // 对仍处于各雷达自身坐标系的bag，可按topic仅补偿安装方向。
    for (size_t i = 0; i < cfg_.lidar_topics.size(); ++i) {
        const auto& topic = cfg_.lidar_topics[i];
        std::string lidar_name = (i < cfg_.lidar_names.size()) ? cfg_.lidar_names[i] : std::string{};
        if (lidar_name.empty()) {
            const auto slash = topic.find_last_of('/');
            lidar_name = (slash == std::string::npos) ? topic : topic.substr(slash + 1);
        }

        const std::string point_layout = i < cfg_.lidar_point_layouts.size()
            ? cfg_.lidar_point_layouts[i]
            : std::string{};
        if (point_layout.empty()) {
            throw std::runtime_error("Missing lidar_point_layouts entry for " + lidar_name);
        }
        const float yaw_degrees = i < cfg_.lidar_yaw_degrees.size()
            ? cfg_.lidar_yaw_degrees[i]
            : 0.0f;
        auto lidar = std::make_shared<LidarSubscriber>(cfg_.lidar_near_distance_m,
                                                       cfg_.lidar_near_min_points,
                                                       lidarPointLayoutFromString(point_layout),
                                                       yaw_degrees,
                                                       static_cast<uint8_t>(i));
        if (i < cfg_.lidar_xy_transforms.size()) {
            const auto& transform = cfg_.lidar_xy_transforms[i];
            lidar->setXYTransform(transform[0], transform[1],
                                  transform[2], transform[3]);
        }
        std::cout << "[SentinelNode] LiDAR topic: " << lidar_name
                  << " <= " << topic
                  << " point_layout=" << point_layout
                  << " yaw_degrees=" << yaw_degrees << std::endl;
        lidar->setFrameCallback([this](const LidarFrame&) {
            lidar_last_receive_steady_ns_.store(currentSteadyTimeNs(), std::memory_order_relaxed);
        });
        lidar->subscribe(topic);
        lidars_.push_back(lidar);
    }
    if (!lidars_.empty()) lidar_ = lidars_.front();

    // 6. 鱼眼去畸变管理器
    fisheye_mgr_ = std::make_shared<FisheyeUndistortManager>();
    fisheye_mgr_->loadAll(cfg_.calib_base_dir, cfg_.camera_names);
    // Inbox 差值框的 LiDAR 投影使用独立外参；图像去畸变和其它视觉/LiDAR
    // 融合路径继续使用上面的普通标定。目录缺失时才回退，避免程序无法启动。
    inbox_fisheye_mgr_ = std::make_shared<FisheyeUndistortManager>();
    if (!inbox_fisheye_mgr_->loadAll(cfg_.inbox_calib_dir, cfg_.camera_names)) {
        std::cout << "[SentinelNode][InboxCalib] calibration unavailable at "
                  << cfg_.inbox_calib_dir << ", fallback to "
                  << cfg_.calib_base_dir << std::endl;
        inbox_fisheye_mgr_->loadAll(cfg_.calib_base_dir, cfg_.camera_names);
    }

    // 7. 摄像头订阅器（4 路鱼眼 FOV195，各自独立解码/转码线程）
    if (!early_cuda_camera_subscribe) {
        std::cout << "[SentinelNode] Camera input mode: "
                  << cameraInputModeName(camera_input_mode)
                  << " decoder_backend="
                  << hevcDecoderBackendName(camera_decoder_backend)
                  << std::endl;
        for (size_t i = 0; i < cfg_.camera_topics.size(); ++i) {
            const auto& topic = cfg_.camera_topics[i];
            std::string cam_name = (i < cfg_.camera_names.size()) ? cfg_.camera_names[i] : topicBasename(topic);
            auto cam = std::make_shared<CameraSubscriber>(imu_,
                                                           cfg_.camera_width,
                                                           cfg_.camera_height,
                                                           cfg_.gpu_id,
                                                           camera_input_mode,
                                                           camera_decoder_backend,
                                                           decode_debug_cfg);
            cam->setFrameCallback([this](const DecodedFrame& f) { onFrame(f); });
            cam->subscribe(topic, cam_name);
            cameras_.push_back(cam);
        }
        if (!cameras_.empty()) camera_ = cameras_.front();
    }

    // 8. 检测器（TensorRT）
    detector_ = std::make_shared<Detector>();
    std::cout << "[SentinelNode][Detector] init begin"
              << " engine=" << cfg_.engine_path
              << " model_type=" << cfg_.detector_model_type
              << " conf_threshold=" << cfg_.conf_threshold
              << std::endl;
    const bool detector_ready = detector_->init(cfg_.engine_path,
                                                cfg_.conf_threshold,
                                                cfg_.nms_threshold,
                                                cfg_.detector_model_type);
    detector_ready_.store(detector_ready, std::memory_order_release);
    std::cout << "[SentinelNode][Detector] init result ready="
              << (detector_ready ? 1 : 0) << std::endl;
    if (!detector_ready) {
        std::cerr << "[SentinelNode][Detector][ERROR] TensorRT detector init failed"
                  << " engine=" << cfg_.engine_path
                  << " model_type=" << cfg_.detector_model_type
                  << " conf_threshold=" << cfg_.conf_threshold
                  << ". Visual class detection will be unavailable; sentry will rely on"
                  << " IMU, near-field contact motion, and passing-motion fallback." << std::endl;
    }

    // 9. 跟踪器：4 路环视相机视角差异很大，需按相机独立维护轨迹，避免跨视角错误关联。
    trackers_by_camera_.clear();

    // 9.5 告警视频录制器（独立线程，环形缓冲 + H.265 编码）
    if (cfg_.enable_event_recording) {
        VideoRecorderConfig rec_cfg;
        rec_cfg.pre_seconds  = cfg_.event_pre_seconds;
        rec_cfg.post_seconds = cfg_.event_post_seconds;
        rec_cfg.fps          = std::max(1, cfg_.event_record_fps);
        rec_cfg.queue_capacity = std::max(1, cfg_.event_record_fps) *
                                 (std::max(1, cfg_.event_pre_seconds) + 1);
        // 保持4路鱼眼原图分辨率录制；环形缓冲内部转JPEG，避免缓存BGR原图导致内存爆涨。
        rec_cfg.max_frame_width = 0;
        rec_cfg.jpeg_quality = 85;
        rec_cfg.bitrate_kbps = 2000;
        rec_cfg.output_dir   = cfg_.event_record_dir;
        video_recorder_ = std::make_shared<VideoRecorder>(rec_cfg);
        video_recorder_->setDoneCallback([this](uint64_t eid, const std::vector<std::string>& paths) {
            handleRecordedVideoDone(eid, paths);
        });
        video_recorder_->start();
        cleanupEventRecordingStorage();
    }

    // 10. 报警管理器
    alert_manager_ = std::make_shared<AlertManager>(cfg_.camera_width,
                                                     cfg_.camera_height);
    alert_manager_->setImuThresholds(cfg_.imu_accel_threshold,
                                      cfg_.imu_accel_severe_threshold,
                                      cfg_.imu_gyro_threshold,
                                      cfg_.imu_gyro_severe_threshold,
                                      cfg_.imu_handle_freq_min,
                                      cfg_.imu_handle_freq_max);
    // 10. 报警管理器
    alert_manager_->setAlertCallback([this](const AlertEvent& e) { onAlert(e); });

    // 11. 哨兵状态机（哨兵模式 / 越野堡垒模式共用）
    state_machine_ = std::make_shared<SentryStateMachine>();
    state_machine_->setStateCallback([this](const SentryStateEvent& e) {
        std::cout << "[SentryState] "
                  << static_cast<int>(e.from) << " → "
                  << static_cast<int>(e.to)
                  << " | " << e.reason << std::endl;
        logSentryModeLifecycle(e);
        updateCachedSentryStatus();
        publishSentryStateBroadcastTick();
        if (!statusPublishFixed10Hz() && !statusPublishPerFrame() &&
            (e.to == SentryState::WARNING || e.to == SentryState::ALARM)) {
            publishSentryStatusTick();
        }
    });

    // 12. 越野堡垒管理器（单车安防）
    orsm_settings_.zone_a_radius = cfg_.orsm_zone_a_radius;
    orsm_settings_.zone_b_radius = cfg_.orsm_zone_b_radius;
    orsm_settings_.zone_c_radius = cfg_.orsm_zone_c_radius;
    orsm_settings_.intruder_dedup_distance_m = cfg_.orsm_intruder_dedup_distance_m;
    orsm_settings_.lidar_only_far_output_enabled = cfg_.orsm_lidar_only_far_output_enabled;
    orsm_settings_.lidar_only_far_output_min_distance_m =
        cfg_.orsm_lidar_only_far_output_min_distance_m;
    orsm_settings_.far_dynamic_min_distance_m = cfg_.orsm_far_dynamic_min_distance_m;
    orsm_settings_.far_image_motion_required = cfg_.orsm_far_image_motion_required;
    orsm_settings_.orsm_setting_st = static_cast<OrsmSettingType>(cfg_.orsm_default_setting_st);
    orsm_settings_.humans_recog_setting = static_cast<BasicSwitchReq>(cfg_.orsm_default_humans_recog_setting);
    orsm_settings_.animals_recog_setting = static_cast<BasicSwitchReq>(cfg_.orsm_default_animals_recog_setting);
    orsm_settings_.detec_range_setting = static_cast<DetecRangeSetting>(cfg_.orsm_default_detec_range_setting);
    orsm_settings_.alarm_light_setting = static_cast<BasicSwitchReq>(cfg_.orsm_default_alarm_light_setting);
    orsm_settings_.alarm_horn_setting = static_cast<BasicSwitchReq>(cfg_.orsm_default_alarm_horn_setting);
    orsm_settings_.alarm_sound_setting = static_cast<AlarmSoundType>(cfg_.orsm_default_alarm_sound_setting);
    orsm_settings_.alarm_snd_dura_setting = static_cast<AlarmSoundDuration>(cfg_.orsm_default_alarm_snd_dura_setting);
    orsm_settings_.multi_vehicle_team_setting = static_cast<MultiVehicleTeamSettingEnum>(cfg_.orsm_default_multi_vehicle_team_setting);
    orsm_settings_.multi_vehicle_team_req = static_cast<MultiVehicleTeamReqEnum>(cfg_.orsm_default_multi_vehicle_team_req);
    orsm_settings_.multi_vehicle_team_prepared = static_cast<MultiVehicleTeamPreparedEnum>(cfg_.orsm_default_multi_vehicle_team_prepared);
    orsm_settings_.tbox_team_status_info = static_cast<TeamStatusEnum>(cfg_.orsm_default_tbox_team_status_info);
    orsm_settings_.multi_vehicle_role = static_cast<MultiVehicleRole>(cfg_.orsm_default_multi_vehicle_role);
    orsm_settings_.multi_vehicle_enable_network_check = cfg_.orsm_multi_vehicle_enable_network_check;
    orsm_settings_.multi_vehicle_speed_entry_max_kmh = cfg_.orsm_multi_vehicle_speed_entry_max_kmh;
    orsm_settings_.multi_vehicle_speed_exit_max_kmh = cfg_.orsm_multi_vehicle_speed_exit_max_kmh;
    orsm_settings_.multi_vehicle_soc_entry_min = cfg_.orsm_multi_vehicle_soc_entry_min;
    orsm_settings_.multi_vehicle_soc_exit_min = cfg_.orsm_multi_vehicle_soc_exit_min;
    orsm_settings_.multi_vehicle_stationary_required_sec = cfg_.orsm_multi_vehicle_stationary_required_sec;
    orsm_settings_.multi_vehicle_security_side = cfg_.orsm_multi_vehicle_security_side;
    orsm_settings_.multi_vehicle_external_radius = cfg_.orsm_multi_vehicle_external_radius;
    orsm_settings_.multi_vehicle_internal_radius = cfg_.orsm_multi_vehicle_internal_radius;
    orsm_settings_.multi_vehicle_no_alarm_radius = cfg_.orsm_multi_vehicle_no_alarm_radius;
    orsm_manager_ = std::make_shared<OrsmManager>();
    orsm_manager_->setStateCallback([this](const OrsmStateEvent& e) {
        std::cout << "[OrsmState] "
                  << "transition_steady_ms=" << currentSteadyTimeMs()
                  << " transition_system_ms=" << currentSystemTimeMs()
                  << " event_ts_ns=" << e.timestamp_ns
                  << " "
                  << static_cast<int>(e.from) << " → "
                  << static_cast<int>(e.to)
                  << " | " << e.reason << std::endl;
        logOrsmModeLifecycle(e);
        if (e.to == OrsmState::OFF) {
            clearOrsmStaticFootprints("fortress_state_off");
        }
        updateCachedSentryStatus();
        publishSentryStateBroadcastTick();
    });
    std::cout << "[SentinelNode]   ORSM zones: A=" << orsm_settings_.zone_a_radius
              << "m B=" << orsm_settings_.zone_b_radius
              << "m C=" << orsm_settings_.zone_c_radius
              << "m intruder_dedup=" << orsm_settings_.intruder_dedup_distance_m << "m" << std::endl;
    std::cout << "[SentinelNode]   ORSM safety defaults: "
              << (cfg_.orsm_safety_defaults_enabled ? "enabled" : "disabled")
              << " conf_threshold=" << cfg_.conf_threshold
              << std::endl;

    // 13. RSCL 发布器（可通过 config.yaml enable_rscl_publish 开关控制）
    {
        PublishConfig pub_cfg;
        pub_cfg.enable_rscl_publish = cfg_.enable_rscl_publish;
        pub_cfg.sentry_state_topic = cfg_.sentry_state_topic;
        pub_cfg.sentry_event_topic = cfg_.sentry_event_topic;
        pub_cfg.sentry_action_topic = cfg_.sentry_action_topic;
        pub_cfg.orsm_state_topic = cfg_.orsm_state_topic;
        pub_cfg.orsm_action_topic = cfg_.orsm_action_topic;
        rscl_publisher_ = std::make_shared<RsclPublisher>(pub_cfg);
        rscl_publisher_->init();
    }

    frame_processing_running_ = true;
    frame_processing_thread_ = std::thread(&SentinelNode::frameProcessingLoop, this);

    state_tick_running_ = true;
    state_tick_thread_ = std::thread(&SentinelNode::stateTickLoop, this);

    if (statusPublishFixed10Hz()) {
        status_publish_running_ = true;
        status_publish_thread_ = std::thread(&SentinelNode::statusPublishLoop, this);
        std::cout << "[SentinelNode] Status publish mode: fixed_10hz" << std::endl;
    } else if (statusPublishPerFrame()) {
        status_publish_running_ = false;
        std::cout << "[SentinelNode] Status publish mode: per_frame (image timestamp)" << std::endl;
    } else {
        status_publish_running_ = false;
        std::cout << "[SentinelNode] Status publish mode: event_driven" << std::endl;
    }

    orsm_status_publish_running_ = true;
    orsm_status_publish_thread_ = std::thread(&SentinelNode::orsmStatusPublishLoop, this);
    std::cout << "[SentinelNode] ORSM status publish mode: fixed_10hz on "
              << cfg_.orsm_state_topic << std::endl;

    // enable_rscl_publish 时始终 ~10Hz 广播状态机到 /sentry/state_broadcast。
    // fixed_10hz 模式下复用 statusPublishLoop，避免双线程重复发布。
    if (cfg_.enable_rscl_publish && !statusPublishFixed10Hz()) {
        state_broadcast_running_ = true;
        state_broadcast_thread_ = std::thread(&SentinelNode::stateBroadcastLoop, this);
        std::cout << "[SentinelNode] State broadcast: StateBroadcast Cap'n Proto 10Hz on " << cfg_.sentry_state_topic << std::endl;
    } else if (cfg_.enable_rscl_publish && statusPublishFixed10Hz()) {
        std::cout << "[SentinelNode] State broadcast: via fixed_10hz status thread on "
                  << cfg_.sentry_state_topic << std::endl;
    }

    started_ = true;
    std::cout << "[SentinelNode] All modules started." << std::endl;
    std::cout << "[SentinelNode]   IMU    : " << cfg_.imu_topic      << std::endl;
    std::cout << "[SentinelNode]   USS PDC: " << cfg_.uss_pdc_topic  << std::endl;
    std::cout << "[SentinelNode]   USS MEB: " << cfg_.uss_meb_topic  << std::endl;
    std::cout << "[SentinelNode]   USS Fused: " << cfg_.uss_fused_topic << std::endl;
    if (!cfg_.uss_fused_legacy_topic.empty() && cfg_.uss_fused_legacy_topic != cfg_.uss_fused_topic) {
        std::cout << "[SentinelNode]   USS Fused legacy: " << cfg_.uss_fused_legacy_topic << std::endl;
    }
    for (const auto& t : cfg_.lidar_topics)
        std::cout << "[SentinelNode]   LiDAR  : " << t << std::endl;
    for (const auto& t : cfg_.camera_topics)
        std::cout << "[SentinelNode]   Camera : " << t << std::endl;
    std::cout << "[SentinelNode] External topic contract (single-node):" << std::endl;
    std::cout << "[SentinelNode]   Vehicle report : " << cfg_.vehicle_report_topic << std::endl;
    std::cout << "[SentinelNode]   Chassis info   : " << cfg_.chassis_info_topic << std::endl;
    std::cout << "[SentinelNode]   Sentry request : " << cfg_.sentry_request_topic << std::endl;
    std::cout << "[SentinelNode]   Sentry state   : " << cfg_.sentry_state_topic << std::endl;
    std::cout << "[SentinelNode]   Sentry event   : " << cfg_.sentry_event_topic << std::endl;
    std::cout << "[SentinelNode]   Sentry action  : " << cfg_.sentry_action_topic << std::endl;
    std::cout << "[SentinelNode]   Media cmd/result: " << cfg_.sentry_media_cmd_topic
              << " / " << cfg_.sentry_media_result_topic << std::endl;
    std::cout << "[SentinelNode]   ORSM request/state/action: " << cfg_.orsm_setting_topic
              << " / " << cfg_.orsm_state_topic
              << " / " << cfg_.orsm_action_topic << std::endl;
    std::cout << "[SentinelNode]   Engine : " << cfg_.engine_path    << std::endl;
    return true;
}

void SentinelNode::stop() {
    if (!started_) return;
    state_tick_running_ = false;
    if (state_tick_thread_.joinable()) state_tick_thread_.join();

    status_publish_running_ = false;
    if (status_publish_thread_.joinable()) status_publish_thread_.join();

    orsm_status_publish_running_ = false;
    if (orsm_status_publish_thread_.joinable()) orsm_status_publish_thread_.join();

    state_broadcast_running_ = false;
    if (state_broadcast_thread_.joinable()) state_broadcast_thread_.join();

    // Stop camera workers first so no new decoded frames enter the processing queue.
    camera_.reset();
    cameras_.clear();

    frame_processing_running_ = false;
    frame_processing_cv_.notify_all();
    if (frame_processing_thread_.joinable()) frame_processing_thread_.join();
    {
        std::lock_guard<std::mutex> lk(frame_processing_mutex_);
        frame_processing_queue_.clear();
    }

    imu_.reset();
    uss_.reset();
    lidars_.clear();
    lidar_.reset();
    if (video_recorder_) { video_recorder_->stop(); video_recorder_.reset(); }
    {
        std::lock_guard<std::mutex> lk(camera_sync_mutex_);
        camera_sync_buffers_.clear();
        camera_sync_last_frames_.clear();
        camera_sync_half_rate_kept_.clear();
        camera_sync_latest_ts_ns_ = 0;
    }
    orsm_manager_.reset();
    rscl_publisher_.reset();
    started_ = false;
    std::cout << "[SentinelNode] Stopped." << std::endl;
}


LidarFrame SentinelNode::mergedLidarFrame(uint64_t target_timestamp_ns) const {
    LidarFrame merged;
    const float effective_max_age_ms = cfg_.effectiveLidarMaxAgeMs();
    const uint64_t max_age_ns = static_cast<uint64_t>(std::max(0.0f, effective_max_age_ms) * 1.0e6f);
    const float leaf = cfg_.lidar_merge_voxel_leaf_m;
    std::unordered_set<long long> occupied_voxels;
    for (const auto& lidar : lidars_) {
        if (!lidar) continue;
        LidarFrame frame = target_timestamp_ns > 0
            ? lidar->closestFrame(target_timestamp_ns)
            : lidar->latestFrame();
        if (!frame.valid()) continue;
        lidar_frame_candidate_count_++;
        if (target_timestamp_ns > 0) {
            recordPerfMetric("lidar_camera_delta_ms.candidate", static_cast<double>(absDiffNs(frame.timestamp_ns, target_timestamp_ns)) / 1.0e6);
        }
        if (target_timestamp_ns > 0 && max_age_ns > 0 && absDiffNs(frame.timestamp_ns, target_timestamp_ns) > max_age_ns) {
            const uint64_t idx = ++lidar_stale_skip_count_;
            recordPerfMetric("lidar_camera_delta_ms.stale", static_cast<double>(absDiffNs(frame.timestamp_ns, target_timestamp_ns)) / 1.0e6);
            if (idx <= 10 || idx % 120 == 0) {
                std::cout << "[LiDARMerge][Skip] stale frame_ts=" << frame.timestamp_ns
                          << " target_ts=" << target_timestamp_ns
                          << " delta_ms=" << static_cast<double>(absDiffNs(frame.timestamp_ns, target_timestamp_ns)) / 1.0e6
                          << " max_age_ms=" << effective_max_age_ms
                          << " mode=" << cfg_.lidar_time_sync_mode << std::endl;
            }
            continue;
        }
        if (!merged.valid() || frame.timestamp_ns > merged.timestamp_ns) {
            merged.timestamp_ns = frame.timestamp_ns;
            merged.width = frame.width;
            merged.height = frame.height;
            merged.parity_flag = frame.parity_flag;
        }
        if (leaf > 0.0f) {
            occupied_voxels.reserve(occupied_voxels.size() + frame.points.size() / 2 + 1);
            for (const auto& point : frame.points) {
                if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) continue;
                const int ix = static_cast<int>(std::floor(point.x / leaf));
                const int iy = static_cast<int>(std::floor(point.y / leaf));
                const int iz = static_cast<int>(std::floor(point.z / leaf));
                if (!occupied_voxels.insert(mergeVoxelKey(ix, iy, iz)).second) continue;
                merged.points.push_back(point);
            }
        } else {
            merged.points.insert(merged.points.end(), frame.points.begin(), frame.points.end());
        }
    }
    if (merged.valid()) lidar_merge_output_count_++;
    return merged;
}

void SentinelNode::recordPerfMetric(const std::string& name, double value_ms) const {
    if (!std::isfinite(value_ms) || value_ms < 0.0) return;
    std::lock_guard<std::mutex> lk(perf_mutex_);
    auto it = perf_metrics_.find(name);
    if (it == perf_metrics_.end()) {
        auto [ins, _] = perf_metrics_.emplace(name, PerfMetricStats{});
        it = ins;
    }
    auto& metric = it->second;
    metric.count++;
    metric.sum_ms += value_ms;
    metric.min_ms = std::min(metric.min_ms, value_ms);
    metric.max_ms = std::max(metric.max_ms, value_ms);
    if (name == kAlgorithmProcessTimeMetric && value_ms > kAlgorithmProcessTimeLimitMs) {
        metric.over_threshold_count++;
    }
    if (metric.samples_ms.size() < 200000) metric.samples_ms.push_back(value_ms);
}

void SentinelNode::logPerfSummaryIfNeeded(uint64_t processed_frames) {
    const uint64_t perf_summary_interval_frames =
        static_cast<uint64_t>(std::max(1, cfg_.algorithm_time_log_interval_frames));
    if (processed_frames == 0 || processed_frames - last_perf_summary_frame_ < perf_summary_interval_frames) return;
    last_perf_summary_frame_ = processed_frames;

    std::lock_guard<std::mutex> lk(perf_mutex_);
    auto percentile = [](std::vector<double> samples, double ratio) -> double {
        if (samples.empty()) return 0.0;
        std::sort(samples.begin(), samples.end());
        const size_t idx = std::min(samples.size() - 1,
                                    static_cast<size_t>(std::llround(ratio * static_cast<double>(samples.size() - 1))));
        return samples[idx];
    };

    std::cout << "[PerfSummary] frames=" << processed_frames
              << " sync_ok=" << camera_sync_group_ok_count_.load()
              << " sync_timeout=" << camera_sync_timeout_count_.load()
              << " sync_buffer_overflow=" << camera_sync_buffer_overflow_count_.load()
              << " sync_old_master_drop=" << camera_sync_unmatchable_master_drop_count_.load()
              << " sync_old_left_drop=" << camera_sync_unmatchable_left_drop_count_.load()
              << " sync_half_rate_drop=" << camera_sync_half_rate_drop_count_.load()
              << " frame_queue_overflow=" << frame_processing_overflow_count_.load()
              << " interval_drop_groups=" << frame_interval_drop_group_count_.load()
              << " interval_drop_frames=" << frame_interval_drop_count_.load()
              << " orsm_latest_replace_groups=" << orsm_latest_group_replace_count_.load()
              << " orsm_latest_drop_frames=" << orsm_latest_group_drop_frame_count_.load()
              << " sentry_latest_replace_groups=" << sentry_latest_group_replace_count_.load()
              << " sentry_latest_drop_frames=" << sentry_latest_group_drop_frame_count_.load()
              << " lidar_candidates=" << lidar_frame_candidate_count_.load()
              << " lidar_stale_skip=" << lidar_stale_skip_count_.load()
              << " lidar_merge_output=" << lidar_merge_output_count_.load()
              << std::endl;

    for (const auto& kv : perf_metrics_) {
        const auto& name = kv.first;
        const auto& metric = kv.second;
        if (metric.count == 0) continue;
        if (!cfg_.enable_algorithm_time_log && isAlgorithmProcessMetric(name)) continue;
        const double avg = metric.sum_ms / static_cast<double>(metric.count);
        const double min_v = metric.min_ms == std::numeric_limits<double>::max() ? 0.0 : metric.min_ms;
        const double p50 = percentile(metric.samples_ms, 0.50);
        const double p90 = percentile(metric.samples_ms, 0.90);
        const double p99 = percentile(metric.samples_ms, 0.99);
        std::cout << std::fixed << std::setprecision(2)
                  << "[PerfMetric] name=" << name
                  << " count=" << metric.count
                  << " avg=" << avg
                  << " p50=" << p50
                  << " p90=" << p90
                  << " p99=" << p99
                  << " min=" << min_v
                  << " max=" << metric.max_ms
                  << " ms" << std::endl;
        if (cfg_.enable_algorithm_time_log && isAlgorithmProcessMetric(name)) {
            std::cout << std::fixed << std::setprecision(2)
                      << "[算法模块耗时统计] 模块=" << algorithmProcessMetricDisplayName(name)
                      << " 指标=" << name
                      << " 样本数=" << metric.count
                      << " 平均=" << avg << "ms"
                      << " P50=" << p50 << "ms"
                      << " P90=" << p90 << "ms"
                      << " P99=" << p99 << "ms"
                      << " 最小=" << min_v << "ms"
                      << " 最大=" << metric.max_ms << "ms"
                      << std::endl;
        }
        if (name == kAlgorithmProcessTimeMetric) {
            const double over_ratio = metric.count > 0
                ? static_cast<double>(metric.over_threshold_count) * 100.0 / static_cast<double>(metric.count)
                : 0.0;
            if (cfg_.enable_algorithm_time_log) {
                std::cout << std::fixed << std::setprecision(2)
                          << "[算法耗时统计] 总帧数=" << processed_frames
                          << " 指标=" << name
                          << " 阈值=" << kAlgorithmProcessTimeLimitMs << "ms"
                          << " 样本数=" << metric.count
                          << " 平均=" << avg << "ms"
                          << " P50=" << p50 << "ms"
                          << " P90=" << p90 << "ms"
                          << " P99=" << p99 << "ms"
                          << " 最大=" << metric.max_ms << "ms"
                          << " 超限帧数=" << metric.over_threshold_count
                          << " 超限率=" << over_ratio << "%"
                          << std::endl;
            }
        }
    }

    for (const auto& kv : camera_sync_timeout_missing_counts_) {
        std::cout << "[PerfCounter] sync_timeout_missing camera=" << kv.first
                  << " count=" << kv.second << std::endl;
    }
    for (const auto& kv : frame_processing_drop_counts_) {
        std::cout << "[PerfCounter] frame_queue_drop camera=" << kv.first
                  << " count=" << kv.second << std::endl;
    }
}

// ---- 帧处理（在 CameraSubscriber 解码线程调用） ----

std::string SentinelNode::cameraSyncKey(const DecodedFrame& frame) const {
    if (!frame.camera_name.empty()) return frame.camera_name;
    return "camera";
}

int64_t SentinelNode::cameraSyncOffsetNs(const std::string& camera_key) const {
    for (size_t i = 0; i < cfg_.camera_names.size(); ++i) {
        const std::string key = cfg_.camera_names[i].empty()
            ? (i < cfg_.camera_topics.size() ? topicBasename(cfg_.camera_topics[i]) : std::string{})
            : cfg_.camera_names[i];
        if (key == camera_key && i < cfg_.camera_sync_offsets_ms.size()) {
            return static_cast<int64_t>(cfg_.camera_sync_offsets_ms[i]) * 1000000LL;
        }
    }
    return 0;
}

bool SentinelNode::isCameraSyncHalfRate(const std::string& camera_key) const {
    return std::find(cfg_.camera_sync_half_rate_cameras.begin(),
                     cfg_.camera_sync_half_rate_cameras.end(),
                     camera_key) != cfg_.camera_sync_half_rate_cameras.end();
}

std::vector<DecodedFrame> SentinelNode::pushFrameAndCollectSynchronized(const DecodedFrame& frame) {
    std::vector<DecodedFrame> ready;
    const std::string key = cameraSyncKey(frame);
    if (frame.timestamp_ns > 0) {
        std::lock_guard<std::mutex> perf_lk(perf_mutex_);
        auto& last_ts = camera_last_timestamp_ns_[key];
        if (last_ts > 0 && frame.timestamp_ns > last_ts) {
            const double delta_ms = static_cast<double>(frame.timestamp_ns - last_ts) / 1e6;
            auto& metric = perf_metrics_["camera_delta_ms." + key];
            metric.count++;
            metric.sum_ms += delta_ms;
            metric.min_ms = std::min(metric.min_ms, delta_ms);
            metric.max_ms = std::max(metric.max_ms, delta_ms);
            if (metric.samples_ms.size() < 200000) metric.samples_ms.push_back(delta_ms);
        }
        last_ts = frame.timestamp_ns;
    }

    if (!cfg_.enable_camera_sync || cfg_.camera_topics.size() <= 1) {
        ready.push_back(frame);
        return ready;
    }

    const uint64_t tolerance_ns = static_cast<uint64_t>(std::max(0, cfg_.camera_sync_tolerance_ms)) * 1000000ULL;
    const uint64_t timeout_ns = static_cast<uint64_t>(std::max(0, cfg_.camera_sync_timeout_ms)) * 1000000ULL;
    const size_t max_buffered = static_cast<size_t>(std::max(1, cfg_.camera_sync_max_buffered_frames));

    const std::vector<std::string> expected = [&] {
        std::vector<std::string> keys;
        keys.reserve(cfg_.camera_topics.size());
        for (size_t i = 0; i < cfg_.camera_topics.size(); ++i) {
            if (i < cfg_.camera_names.size() && !cfg_.camera_names[i].empty()) keys.push_back(cfg_.camera_names[i]);
            else keys.push_back(topicBasename(cfg_.camera_topics[i]));
        }
        return keys;
    }();
    if (expected.empty()) {
        ready.push_back(frame);
        return ready;
    }
    const std::string base_key = expected.front();  // front camera is the synchronization master.

    std::lock_guard<std::mutex> lk(camera_sync_mutex_);

    // 右后约20Hz → 感知同步队列约10Hz。以前视时间戳相位划分100ms槽；
    // 同槽保留更靠近前视锚点的一帧，避免盲隔帧造成约100ms半拍错位。
    if (isCameraSyncHalfRate(key) && frame.timestamp_ns > 0) {
        constexpr uint64_t kPeriodNs = 100000000ULL;
        constexpr uint64_t kHalfPeriodNs = kPeriodNs / 2ULL;
        const auto circularDistance = [](uint64_t lhs, uint64_t rhs, uint64_t period) {
            const uint64_t distance = lhs > rhs ? lhs - rhs : rhs - lhs;
            return std::min(distance, period - distance);
        };

        uint64_t front_timestamp_ns = 0;
        const auto front_last = camera_sync_last_frames_.find(base_key);
        if (front_last != camera_sync_last_frames_.end() &&
            front_last->second.frame.timestamp_ns > 0) {
            front_timestamp_ns = front_last->second.frame.timestamp_ns;
        } else {
            const auto front_buffer = camera_sync_buffers_.find(base_key);
            if (front_buffer != camera_sync_buffers_.end() &&
                !front_buffer->second.empty()) {
                front_timestamp_ns = front_buffer->second.back().frame.timestamp_ns;
            }
        }

        bool drop = false;
        if (front_timestamp_ns > 0) {
            const uint64_t reference_phase = front_timestamp_ns % kPeriodNs;
            const uint64_t frame_phase = frame.timestamp_ns % kPeriodNs;
            const uint64_t reference_distance =
                circularDistance(frame_phase, reference_phase, kPeriodNs);
            const uint64_t opposite_distance = circularDistance(
                frame_phase, (reference_phase + kHalfPeriodNs) % kPeriodNs, kPeriodNs);
            drop = opposite_distance < reference_distance;
        }

        const uint64_t phase_origin =
            front_timestamp_ns > 0 ? front_timestamp_ns % kPeriodNs : 0ULL;
        const uint64_t slot =
            (frame.timestamp_ns + kPeriodNs - phase_origin) / kPeriodNs;
        const uint64_t new_error = front_timestamp_ns > 0
            ? absDiffNs(frame.timestamp_ns, front_timestamp_ns)
            : 0ULL;

        auto kept = camera_sync_half_rate_kept_.find(key);
        bool replace_kept = false;
        if (!drop && kept != camera_sync_half_rate_kept_.end() &&
            kept->second.slot == slot) {
            const uint64_t anchor = front_timestamp_ns > 0
                ? front_timestamp_ns
                : kept->second.front_anchor_ts;
            const uint64_t old_error = anchor > 0
                ? absDiffNs(kept->second.frame_ts, anchor)
                : 0ULL;
            if (anchor > 0 && new_error < old_error) replace_kept = true;
            else drop = true;
        }

        if (drop) {
            const uint64_t dropped = ++camera_sync_half_rate_drop_count_;
            if (dropped <= 10 || dropped % 100 == 0) {
                std::cout << "[CameraSync] half-rate drop camera=" << key
                          << " seq=" << frame.seq
                          << " dropped=" << dropped << std::endl;
            }
            return ready;
        }

        if (replace_kept) {
            auto buffer = camera_sync_buffers_.find(key);
            if (buffer != camera_sync_buffers_.end() && !buffer->second.empty()) {
                const auto& previous = buffer->second.back().frame;
                if (previous.seq == kept->second.frame_seq &&
                    previous.timestamp_ns == kept->second.frame_ts) {
                    buffer->second.pop_back();
                }
            }
        }

        camera_sync_half_rate_kept_[key] = HalfRateKeep{
            slot, frame.timestamp_ns, front_timestamp_ns, frame.seq};
    }

    camera_sync_latest_ts_ns_ = std::max(camera_sync_latest_ts_ns_, frame.timestamp_ns);

    auto& q = camera_sync_buffers_[key];
    const int64_t sync_ts = static_cast<int64_t>(frame.timestamp_ns) + cameraSyncOffsetNs(key);
    q.push_back(CameraSyncFrame{frame, currentSteadyTimeNs(), sync_ts});
    camera_sync_last_frames_[key] = q.back();
    while (q.size() > max_buffered) {
        std::cout << "[CameraSync] buffer overflow camera=" << key
                  << ", drop seq=" << q.front().frame.seq << std::endl;
        q.pop_front();
        camera_sync_buffer_overflow_count_++;
    }

    while (true) {
        auto base_it = camera_sync_buffers_.find(base_key);
        if (base_it == camera_sync_buffers_.end() || base_it->second.empty()) break;

        auto& base_buf = base_it->second;
        const auto& base = base_buf.front();
        const int64_t base_ts = base.sync_timestamp_ns;

        std::vector<DecodedFrame> group;
        std::vector<std::string> missing;
        std::unordered_set<std::string> group_from_buffer;
        group.reserve(expected.size());

        for (const auto& cam_key : expected) {
            auto it = camera_sync_buffers_.find(cam_key);
            if (it == camera_sync_buffers_.end() || it->second.empty()) {
                missing.push_back(cam_key);
                continue;
            }

            auto& buf = it->second;
            while (buf.size() > 1 && buf[1].sync_timestamp_ns <= base_ts &&
                   std::llabs(buf[1].sync_timestamp_ns - base_ts) <=
                       std::llabs(buf.front().sync_timestamp_ns - base_ts)) {
                std::cout << "[CameraSync] drop stale camera=" << cam_key
                          << " seq=" << buf.front().frame.seq << std::endl;
                buf.pop_front();
            }

            if (static_cast<uint64_t>(std::llabs(buf.front().sync_timestamp_ns - base_ts)) <= tolerance_ns) {
                group.push_back(buf.front().frame);
                group_from_buffer.insert(cam_key);
            } else {
                missing.push_back(cam_key);
            }
        }

        const bool complete = group.size() == expected.size();
        if (!complete) {
            // If a missing camera's oldest buffered frame is already newer than
            // the master beyond the tolerance window, no future frame from that
            // camera can match this master timestamp. Drop the old master
            // immediately instead of waiting for timeout/degrade and letting
            // faster camera buffers overflow.
            std::vector<std::string> too_new_cameras;
            int64_t min_too_new_delta_ns = std::numeric_limits<int64_t>::max();
            for (const auto& cam_key : missing) {
                auto it = camera_sync_buffers_.find(cam_key);
                if (it == camera_sync_buffers_.end() || it->second.empty()) continue;
                const int64_t delta_ns = it->second.front().sync_timestamp_ns - base_ts;
                if (delta_ns > static_cast<int64_t>(tolerance_ns)) {
                    too_new_cameras.push_back(cam_key);
                    min_too_new_delta_ns = std::min(min_too_new_delta_ns, delta_ns);
                }
            }
            bool dropped_unmatchable = false;
            if (!too_new_cameras.empty()) {
                const uint64_t drop_count =
                    ++camera_sync_unmatchable_master_drop_count_;
                if (drop_count <= 20 || drop_count % 100 == 0) {
                    std::cout << "[CameraSync] drop old master base_camera=" << base_key
                              << " base_ts=" << base.frame.timestamp_ns
                              << " got=" << group.size() << "/" << expected.size()
                              << " too_new=";
                    for (size_t i = 0; i < too_new_cameras.size(); ++i) {
                        std::cout << (i ? "," : "") << too_new_cameras[i];
                    }
                    std::cout << " min_delta_ms="
                              << static_cast<double>(min_too_new_delta_ns) / 1e6
                              << " dropped_old_masters=" << drop_count
                              << std::endl;
                }
                base_buf.pop_front();
                dropped_unmatchable = true;
            }

            // 右/后等路最老帧已比左视新超过 tolerance → 左视配不上，立刻 pop 左视。
            std::string left_key;
            for (const auto& cam_key : expected) {
                if (cam_key == base_key) continue;
                if (cam_key.find("left") != std::string::npos) {
                    left_key = cam_key;
                    break;
                }
            }
            if (!left_key.empty()) {
                auto left_it = camera_sync_buffers_.find(left_key);
                if (left_it != camera_sync_buffers_.end() && !left_it->second.empty()) {
                    const int64_t left_ts = left_it->second.front().sync_timestamp_ns;
                    std::vector<std::string> too_new_vs_left;
                    int64_t min_left_delta_ns = std::numeric_limits<int64_t>::max();
                    for (const auto& cam_key : expected) {
                        if (cam_key == left_key) continue;
                        auto it = camera_sync_buffers_.find(cam_key);
                        if (it == camera_sync_buffers_.end() || it->second.empty()) continue;
                        const int64_t delta_ns = it->second.front().sync_timestamp_ns - left_ts;
                        if (delta_ns > static_cast<int64_t>(tolerance_ns)) {
                            too_new_vs_left.push_back(cam_key);
                            min_left_delta_ns = std::min(min_left_delta_ns, delta_ns);
                        }
                    }
                    if (!too_new_vs_left.empty()) {
                        const auto& left_front = left_it->second.front();
                        const uint64_t drop_count =
                            ++camera_sync_unmatchable_left_drop_count_;
                        if (drop_count <= 20 || drop_count % 100 == 0) {
                            std::cout << "[CameraSync] drop old left left_camera=" << left_key
                                      << " left_ts=" << left_front.frame.timestamp_ns
                                      << " got=" << group.size() << "/" << expected.size()
                                      << " too_new=";
                            for (size_t i = 0; i < too_new_vs_left.size(); ++i) {
                                std::cout << (i ? "," : "") << too_new_vs_left[i];
                            }
                            std::cout << " min_delta_ms="
                                      << static_cast<double>(min_left_delta_ns) / 1e6
                                      << " dropped_old_lefts=" << drop_count
                                      << std::endl;
                        }
                        left_it->second.pop_front();
                        dropped_unmatchable = true;
                    }
                }
            }

            if (dropped_unmatchable) continue;
        }
        const uint64_t now_steady_ns = currentSteadyTimeNs();
        const uint64_t degrade_ns = static_cast<uint64_t>(std::max(0, cfg_.camera_sync_degrade_ms)) * 1000000ULL;
        bool stream_time_expired = !missing.empty();
        for (const auto& cam_key : missing) {
            auto last_it = camera_sync_last_frames_.find(cam_key);
            if (last_it == camera_sync_last_frames_.end() ||
                last_it->second.sync_timestamp_ns <= base_ts + static_cast<int64_t>(tolerance_ns)) {
                stream_time_expired = false;
                break;
            }
        }
        const bool wall_time_timeout = now_steady_ns > base.arrival_steady_ns &&
                                       now_steady_ns - base.arrival_steady_ns >= timeout_ns;
        const bool timeout = wall_time_timeout || stream_time_expired;
        const bool degrade_timeout = cfg_.camera_sync_enable_degrade &&
            (stream_time_expired ||
             (now_steady_ns > base.arrival_steady_ns &&
              now_steady_ns - base.arrival_steady_ns >= std::max(timeout_ns, degrade_ns)));
        bool held_complete = complete;
        bool degraded_group = false;
        if (!complete && timeout && cfg_.camera_sync_enable_hold) {
            const uint64_t hold_ns = static_cast<uint64_t>(std::max(0, cfg_.camera_sync_hold_ms)) * 1000000ULL;
            std::vector<std::string> still_missing;
            for (const auto& cam_key : missing) {
                auto last_it = camera_sync_last_frames_.find(cam_key);
                if (last_it == camera_sync_last_frames_.end() || hold_ns == 0) {
                    still_missing.push_back(cam_key);
                    continue;
                }
                const uint64_t delta_ns = static_cast<uint64_t>(std::llabs(last_it->second.sync_timestamp_ns - base_ts));
                if (delta_ns <= hold_ns) {
                    DecodedFrame held = last_it->second.frame;
                    held.recording_group_timestamp_ns = static_cast<uint64_t>(std::max<int64_t>(1, base_ts));
                    group.push_back(std::move(held));
                    std::cout << "[CameraSync] hold camera=" << cam_key
                              << " delta_ms=" << static_cast<double>(delta_ns) / 1e6
                              << " base_ts=" << base.frame.timestamp_ns
                              << std::endl;
                } else {
                    still_missing.push_back(cam_key);
                }
            }
            missing.swap(still_missing);
            held_complete = group.size() == expected.size();
        }
        if (!held_complete && timeout && !degrade_timeout) break;
        if (!held_complete && degrade_timeout && group.size() >= 2) {
            degraded_group = true;
            std::cout << "[CameraSync] degraded group base_camera=" << base_key
                      << " base_ts=" << base.frame.timestamp_ns
                      << " got=" << group.size() << "/" << expected.size()
                      << " missing=";
            for (size_t i = 0; i < missing.size(); ++i) {
                std::cout << (i ? "," : "") << missing[i];
            }
            std::cout << std::endl;
        }
        if (!held_complete && !degraded_group) {
            if (!timeout) break;
        }

        if (!held_complete && !degraded_group) {
            std::cout << "[CameraSync] timeout base_camera=" << base_key
                      << " base_ts=" << base.frame.timestamp_ns
                      << " base_sync_ts=" << base_ts
                      << " got=" << group.size() << "/" << expected.size()
                      << " missing=";
            for (size_t i = 0; i < missing.size(); ++i) {
                std::cout << (i ? "," : "") << missing[i];
            }
            std::cout << ", drop unsynchronized group" << std::endl;

            camera_sync_timeout_count_++;
            recordPerfMetric("camera_sync_timeout_wait_ms",
                             static_cast<double>(now_steady_ns - base.arrival_steady_ns) / 1e6);
            {
                std::lock_guard<std::mutex> perf_lk(perf_mutex_);
                for (const auto& cam : missing) camera_sync_timeout_missing_counts_[cam]++;
            }

            // 四路相机是时间同步强约束：同步不齐不能单路降级处理，否则会破坏BEV/位置判断。
            base_buf.pop_front();
            continue;
        }

        int64_t min_sync_ts = group.front().timestamp_ns ? static_cast<int64_t>(group.front().timestamp_ns) : base_ts;
        int64_t max_sync_ts = min_sync_ts;
        for (const auto& f : group) {
            const int64_t ts = static_cast<int64_t>(f.timestamp_ns) + cameraSyncOffsetNs(cameraSyncKey(f));
            min_sync_ts = std::min(min_sync_ts, ts);
            max_sync_ts = std::max(max_sync_ts, ts);
        }
        camera_sync_group_ok_count_++;
        recordPerfMetric("camera_sync_group_delta_ms", static_cast<double>(max_sync_ts - min_sync_ts) / 1e6);

        for (auto& f : group) {
            f.recording_group_size = group.size();
        }

        for (const auto& f : group) {
            const auto cam_key = cameraSyncKey(f);
            if (!group_from_buffer.count(cam_key)) continue;
            auto it = camera_sync_buffers_.find(cameraSyncKey(f));
            if (it == camera_sync_buffers_.end()) continue;
            auto& buf = it->second;
            while (!buf.empty()) {
                const bool matched = buf.front().frame.seq == f.seq && buf.front().frame.timestamp_ns == f.timestamp_ns;
                buf.pop_front();
                if (matched) break;
            }
        }

        ready.insert(ready.end(), group.begin(), group.end());
    }

    return ready;
}

uint64_t SentinelNode::frameGroupSyncTimestampNs(const std::vector<DecodedFrame>& group) const {
    uint64_t group_ts = std::numeric_limits<uint64_t>::max();
    for (const auto& frame : group) {
        if (frame.timestamp_ns == 0) continue;
        const int64_t adjusted = static_cast<int64_t>(frame.timestamp_ns) + cameraSyncOffsetNs(cameraSyncKey(frame));
        if (adjusted <= 0) continue;
        group_ts = std::min(group_ts, static_cast<uint64_t>(adjusted));
    }
    return group_ts == std::numeric_limits<uint64_t>::max() ? 0 : group_ts;
}

bool SentinelNode::shouldProcessFrameGroup(const std::vector<DecodedFrame>& group) {
    if (group.empty()) return false;

    const bool orsm_requested = fortressModeActiveForProcessing();
    const int interval_ms = orsm_requested ? cfg_.orsm_process_interval_ms : cfg_.sentry_process_interval_ms;
    if (interval_ms <= 0) return true;

    const uint64_t group_ts = frameGroupSyncTimestampNs(group);
    if (group_ts == 0) return true;
    const uint64_t interval_ns = static_cast<uint64_t>(interval_ms) * 1000000ULL;

    std::lock_guard<std::mutex> lk(frame_process_interval_mutex_);
    if (last_processed_group_sync_ts_ns_ == 0 || group_ts >= last_processed_group_sync_ts_ns_ + interval_ns) {
        last_processed_group_sync_ts_ns_ = group_ts;
        return true;
    }

    frame_interval_drop_group_count_++;
    frame_interval_drop_count_ += group.size();
    const uint64_t dropped_groups = frame_interval_drop_group_count_.load();
    if (dropped_groups <= 10 || dropped_groups % 100 == 0) {
        std::cout << "[FrameProcessing] interval drop group size=" << group.size()
                  << " mode=" << (orsm_requested ? "orsm" : "sentry")
                  << " interval_ms=" << interval_ms
                  << " group_ts=" << group_ts
                  << " last_processed_ts=" << last_processed_group_sync_ts_ns_
                  << " dropped_groups=" << dropped_groups
                  << " dropped_frames=" << frame_interval_drop_count_.load()
                  << std::endl;
    }
    return false;
}

void SentinelNode::onFrame(const DecodedFrame& frame) {
    camera_last_receive_steady_ns_.store(currentSteadyTimeNs(), std::memory_order_relaxed);
    // 告警录像保存四路原始鱼眼画面；按event_record_fps抽帧由VideoRecorder完成，
    // 不经过上面的感知半帧率路径。
    pushFrameForEventRecording(frame);

    auto ready = pushFrameAndCollectSynchronized(frame);
    if (ready.empty()) return;

    for (size_t i = 0; i < ready.size();) {
        const size_t group_size = std::max<size_t>(1, ready[i].recording_group_size);
        const size_t end = std::min(ready.size(), i + group_size);
        std::vector<DecodedFrame> group(ready.begin() + static_cast<std::ptrdiff_t>(i),
                                        ready.begin() + static_cast<std::ptrdiff_t>(end));
        i = end;
        // Select a *temporal* cadence before the expensive four-camera work is
        // queued.  Without this call the intended ORSM interval was inert and
        // overload was handled solely by latest-replace: arbitrary middle
        // groups were discarded, breaking short visual/LiDAR evidence chains.
        // Sampling whole synchronized groups keeps the selected observations
        // evenly spaced and never mixes camera times.
        if (!shouldProcessFrameGroup(group)) continue;
        const uint64_t recording_group_id = next_recording_group_id_++;
        uint64_t recording_group_ts = frameGroupSyncTimestampNs(group);
        if (recording_group_ts == 0 && !group.empty()) recording_group_ts = group.front().timestamp_ns;
        for (auto& synced_frame : group) {
            synced_frame.recording_group_id = recording_group_id;
            synced_frame.recording_group_timestamp_ns = recording_group_ts;
            synced_frame.recording_group_size = group.size();
        }
        enqueueFrameGroupForProcessing(group);
    }
}

void SentinelNode::enqueueFrameForProcessing(const DecodedFrame& frame) {
    {
        std::lock_guard<std::mutex> lk(frame_processing_mutex_);
        if (frame_processing_queue_.size() >= kMaxFrameProcessingQueueSize) {
            const auto& dropped = frame_processing_queue_.front();
            const std::string dropped_camera = dropped.camera_name.empty() ? "camera" : dropped.camera_name;
            std::cout << "[FrameProcessing] queue overflow, drop camera="
                      << dropped_camera
                      << " seq=" << dropped.seq << std::endl;
            frame_processing_queue_.pop_front();
            frame_processing_overflow_count_++;
            {
                std::lock_guard<std::mutex> perf_lk(perf_mutex_);
                frame_processing_drop_counts_[dropped_camera]++;
            }
        }
        DecodedFrame queued = frame;
        queued.processing_enqueue_steady_ns = currentSteadyTimeNs();
        recordPerfMetric("frame_queue_size_before_push", static_cast<double>(frame_processing_queue_.size()));
        frame_processing_queue_.push_back(std::move(queued));
    }
    frame_processing_cv_.notify_one();
}

void SentinelNode::enqueueFrameGroupForProcessing(const std::vector<DecodedFrame>& group) {
    if (group.empty()) return;

    const bool orsm_requested = fortressModeActiveForProcessing();
    // Offline validation must preserve every *selected* synchronized group.
    // The realtime latest-group policy is useful in a vehicle, but it makes a
    // deterministic bag replay depend on incidental logging/CPU timing.
    const bool offline_drain_queue =
        std::getenv("ORSM_OFFLINE_DRAIN_QUEUE") != nullptr;
    size_t dropped_frames = 0;
    size_t dropped_groups = 0;
    {
        std::lock_guard<std::mutex> lk(frame_processing_mutex_);
        const uint64_t current_group_id = current_processing_group_id_.load(std::memory_order_relaxed);
        std::unordered_set<uint64_t> dropped_group_ids;

        auto keep = [&](const DecodedFrame& queued) {
            return current_group_id > 0 && queued.recording_group_id == current_group_id;
        };
        if (!offline_drain_queue) {
            for (auto it = frame_processing_queue_.begin(); it != frame_processing_queue_.end();) {
                if (keep(*it)) {
                    ++it;
                    continue;
                }
                if (it->recording_group_id > 0) dropped_group_ids.insert(it->recording_group_id);
                it = frame_processing_queue_.erase(it);
                dropped_frames++;
            }
        }

        dropped_groups = dropped_group_ids.size();
        if (dropped_groups > 0) {
            auto& replace_counter = orsm_requested
                ? orsm_latest_group_replace_count_
                : sentry_latest_group_replace_count_;
            auto& drop_counter = orsm_requested
                ? orsm_latest_group_drop_frame_count_
                : sentry_latest_group_drop_frame_count_;
            replace_counter.fetch_add(dropped_groups);
            drop_counter.fetch_add(dropped_frames);
            const uint64_t total_replaced = replace_counter.load();
            if (total_replaced <= 10 || total_replaced % 100 == 0) {
                std::cout << "[FrameProcessing] latest group replace"
                          << " mode=" << (orsm_requested ? "orsm" : "sentry")
                          << " dropped_groups=" << dropped_groups
                          << " dropped_frames=" << dropped_frames
                          << " current_group=" << current_group_id
                          << " total_replaced_groups=" << total_replaced
                          << " total_dropped_frames=" << drop_counter.load()
                          << std::endl;
            }
        }

        for (auto frame : group) {
            frame.processing_enqueue_steady_ns = currentSteadyTimeNs();
            recordPerfMetric("frame_queue_size_before_push", static_cast<double>(frame_processing_queue_.size()));
            frame_processing_queue_.push_back(std::move(frame));
        }
    }
    frame_processing_cv_.notify_one();
}

void SentinelNode::frameProcessingLoop() {
    while (true) {
        DecodedFrame frame;
        {
            std::unique_lock<std::mutex> lk(frame_processing_mutex_);
            frame_processing_cv_.wait(lk, [this] {
                return !frame_processing_queue_.empty() || !frame_processing_running_.load();
            });
            if (!frame_processing_running_.load() && frame_processing_queue_.empty()) break;
            frame = std::move(frame_processing_queue_.front());
            frame_processing_queue_.pop_front();
        }
        processFrame(frame);
    }
    std::cout << "[FrameProcessing] Worker exiting." << std::endl;
}

namespace {

cv::Rect nearFieldMotionRoiForCamera(const std::string& camera_name, const cv::Size& size) {
    const int w = std::max(1, size.width);
    const int h = std::max(1, size.height);
    const int roi_w = std::max(1, static_cast<int>(w * 0.35f));
    const int roi_h = std::max(1, static_cast<int>(h * 0.75f));
    if (camera_name.find("left") != std::string::npos) {
        return cv::Rect(0, 0, roi_w, roi_h);
    }
    if (camera_name.find("right") != std::string::npos) {
        return cv::Rect(w - roi_w, 0, roi_w, roi_h);
    }
    return cv::Rect();
}

cv::Point3f nearFieldMotionPositionForCamera(const std::string& camera_name) {
    if (camera_name.find("left") != std::string::npos) return {0.f, 0.8f, 0.f};
    if (camera_name.find("right") != std::string::npos) return {0.f, -0.8f, 0.f};
    if (camera_name.find("rear") != std::string::npos) return {-0.8f, 0.f, 0.f};
    return {0.8f, 0.f, 0.f};
}

bool supportsPassingMotionCamera(const std::string& camera_name) {
    return camera_name.find("front") != std::string::npos ||
           camera_name.find("rear") != std::string::npos ||
           camera_name.find("left") != std::string::npos ||
           camera_name.find("right") != std::string::npos;
}

cv::Rect passingMotionRoiForCamera(const std::string& camera_name, const cv::Size& size) {
    const int w = std::max(1, size.width);
    const int h = std::max(1, size.height);
    if (camera_name.find("left") != std::string::npos ||
        camera_name.find("right") != std::string::npos) {
        const int y = static_cast<int>(h * 0.18f);
        return cv::Rect(0, y, w, h - y);
    }
    const int y = camera_name.find("rear") != std::string::npos
        ? static_cast<int>(h * 0.34f)
        : static_cast<int>(h * 0.42f);
    const int roi_h = std::max(1, h - y);
    return cv::Rect(0, y, w, roi_h);
}

cv::Point3f passingMotionPositionForCamera(const std::string& camera_name) {
    if (camera_name.find("rear") != std::string::npos) return {-2.5f, 0.f, 0.f};
    if (camera_name.find("left") != std::string::npos) return {0.f, 2.5f, 0.f};
    if (camera_name.find("right") != std::string::npos) return {0.f, -2.5f, 0.f};
    return {2.5f, 0.f, 0.f};
}

}  // namespace

std::optional<TrackedObject> SentinelNode::updateNearFieldMotionCandidate(const DecodedFrame& frame,
                                                                          const cv::Mat& image) {
    if (!cfg_.sentry_near_field_motion_enabled || image.empty()) return std::nullopt;

    const std::string camera_name = frame.camera_name.empty() ? "camera" : frame.camera_name;
    cv::Rect roi = nearFieldMotionRoiForCamera(camera_name, image.size()) &
                   cv::Rect(0, 0, image.cols, image.rows);
    if (roi.width < 16 || roi.height < 16) return std::nullopt;

    cv::Mat gray;
    if (image.channels() == 1) {
        gray = image;
    } else {
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    }

    cv::Mat roi_gray = gray(roi);
    const int scaled_w = 320;
    const int scaled_h = std::max(80, roi.height * scaled_w / std::max(1, roi.width));
    cv::Mat scaled;
    cv::resize(roi_gray, scaled, cv::Size(scaled_w, scaled_h), 0.0, 0.0, cv::INTER_AREA);
    cv::GaussianBlur(scaled, scaled, cv::Size(5, 5), 0.0);

    std::lock_guard<std::mutex> lk(near_field_motion_mutex_);
    auto& state = near_field_motion_states_[camera_name];
    if (state.previous_gray.empty()) {
        state.previous_gray = scaled.clone();
        state.previous_motion_center = {roi.x + roi.width * 0.5f, roi.y + roi.height * 0.5f};
        state.last_timestamp_ns = frame.timestamp_ns;
        return std::nullopt;
    }

    if (frame.timestamp_ns > state.last_timestamp_ns &&
        frame.timestamp_ns - state.last_timestamp_ns > 2ULL * 1000000000ULL) {
        state.previous_gray = scaled.clone();
        state.streak = 0;
        state.last_timestamp_ns = frame.timestamp_ns;
        return std::nullopt;
    }

    cv::Mat diff;
    cv::absdiff(scaled, state.previous_gray, diff);
    cv::Mat mask;
    cv::threshold(diff, mask, 22.0, 255.0, cv::THRESH_BINARY);
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, cv::Mat(), cv::Point(-1, -1), 1);

    const double changed_ratio = static_cast<double>(cv::countNonZero(mask)) /
        static_cast<double>(std::max(1, mask.rows * mask.cols));
    const double mean_delta = cv::mean(diff)[0];
    const bool motion_hit =
        changed_ratio >= std::max(0.0f, cfg_.sentry_near_field_motion_min_changed_ratio) &&
        mean_delta >= std::max(0.0f, cfg_.sentry_near_field_motion_min_mean_delta);

    cv::Point2f motion_center{roi.x + roi.width * 0.5f, roi.y + roi.height * 0.5f};
    const cv::Moments moments = cv::moments(mask, true);
    if (moments.m00 > 1.0) {
        const float sx = static_cast<float>(moments.m10 / moments.m00);
        const float sy = static_cast<float>(moments.m01 / moments.m00);
        motion_center.x = roi.x + sx * static_cast<float>(roi.width) / static_cast<float>(scaled_w);
        motion_center.y = roi.y + sy * static_cast<float>(roi.height) / static_cast<float>(scaled_h);
    }

    if (motion_hit) {
        state.streak++;
    } else {
        state.streak = 0;
    }

    std::optional<TrackedObject> candidate;
    const int min_streak = std::max(1, cfg_.sentry_near_field_motion_min_streak);
    if (state.streak >= min_streak) {
        TrackedObject obj;
        const int local_id = 87000 + static_cast<int>(std::hash<std::string>{}(camera_name) % 1000U);
        obj.track_id = cameraScopedTrackId(camera_name, local_id);
        obj.class_id = -10;
        obj.label = "near_field_contact_motion";
        obj.bbox = roi;
        obj.confidence = static_cast<float>(std::clamp(
            std::max(changed_ratio * 8.0, mean_delta / 32.0), 0.0, 0.99));
        obj.velocity = motion_center - state.previous_motion_center;
        obj.age = state.streak;
        obj.lost = 0;
        obj.source_camera = camera_name;
        obj.has_visual_ground_position = true;
        obj.visual_ground_position_3d = nearFieldMotionPositionForCamera(camera_name);
        obj.visual_ground_distance_m = std::hypot(obj.visual_ground_position_3d.x,
                                                  obj.visual_ground_position_3d.y);
        obj.position_3d = obj.visual_ground_position_3d;
        obj.distance_m = obj.visual_ground_distance_m;
        obj.origin = ObservationOrigin::OriginSentryNearMotion;
        candidate = obj;

        if (state.streak == min_streak || state.streak % 20 == 0) {
            std::cout << "[SentinelNode][NearFieldMotion] camera=" << camera_name
                      << " streak=" << state.streak
                      << " changed_ratio=" << changed_ratio
                      << " mean_delta=" << mean_delta
                      << " bbox=(" << roi.x << "," << roi.y << ","
                      << roi.width << "," << roi.height << ")"
                      << " confidence=" << obj.confidence
                      << std::endl;
        }
    }

    state.previous_gray = scaled.clone();
    state.previous_motion_center = motion_center;
    state.last_timestamp_ns = frame.timestamp_ns;
    return candidate;
}

std::optional<TrackedObject> SentinelNode::updatePassingMotionCandidate(const DecodedFrame& frame,
                                                                        const cv::Mat& image) {
    if (!cfg_.sentry_passing_motion_enabled || image.empty()) return std::nullopt;

    const std::string camera_name = frame.camera_name.empty() ? "camera" : frame.camera_name;
    if (!supportsPassingMotionCamera(camera_name)) return std::nullopt;

    cv::Rect roi = passingMotionRoiForCamera(camera_name, image.size()) &
                   cv::Rect(0, 0, image.cols, image.rows);
    if (roi.width < 16 || roi.height < 16) return std::nullopt;

    cv::Mat gray;
    if (image.channels() == 1) {
        gray = image;
    } else {
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    }

    cv::Mat roi_gray = gray(roi);
    const int scaled_w = 360;
    const int scaled_h = std::max(100, roi.height * scaled_w / std::max(1, roi.width));
    cv::Mat scaled;
    cv::resize(roi_gray, scaled, cv::Size(scaled_w, scaled_h), 0.0, 0.0, cv::INTER_AREA);
    cv::GaussianBlur(scaled, scaled, cv::Size(5, 5), 0.0);

    std::lock_guard<std::mutex> lk(passing_motion_mutex_);
    auto& state = passing_motion_states_[camera_name];
    if (state.previous_gray.empty()) {
        state.previous_gray = scaled.clone();
        state.previous_motion_center = {roi.x + roi.width * 0.5f, roi.y + roi.height * 0.5f};
        state.last_timestamp_ns = frame.timestamp_ns;
        return std::nullopt;
    }

    if (frame.timestamp_ns > state.last_timestamp_ns &&
        frame.timestamp_ns - state.last_timestamp_ns > 1500ULL * 1000000ULL) {
        state.previous_gray = scaled.clone();
        state.streak = 0;
        state.last_timestamp_ns = frame.timestamp_ns;
        return std::nullopt;
    }

    cv::Mat diff;
    cv::absdiff(scaled, state.previous_gray, diff);
    cv::Mat mask;
    cv::threshold(diff, mask, 18.0, 255.0, cv::THRESH_BINARY);
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, cv::Mat(), cv::Point(-1, -1), 1);
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, cv::Mat(), cv::Point(-1, -1), 2);

    const int mask_area = std::max(1, mask.rows * mask.cols);
    const double changed_ratio = static_cast<double>(cv::countNonZero(mask)) /
        static_cast<double>(mask_area);
    const double mean_delta = cv::mean(diff)[0];

    cv::Mat labels;
    cv::Mat stats;
    cv::Mat centroids;
    const int components = cv::connectedComponentsWithStats(mask, labels, stats, centroids, 8);
    int best_label = -1;
    int best_area = 0;
    for (int label = 1; label < components; ++label) {
        const int area = stats.at<int>(label, cv::CC_STAT_AREA);
        if (area > best_area) {
            best_area = area;
            best_label = label;
        }
    }
    const double component_ratio = static_cast<double>(best_area) / static_cast<double>(mask_area);
    const bool motion_hit =
        changed_ratio >= std::max(0.0f, cfg_.sentry_passing_motion_min_changed_ratio) &&
        mean_delta >= std::max(0.0f, cfg_.sentry_passing_motion_min_mean_delta) &&
        component_ratio >= std::max(0.0f, cfg_.sentry_passing_motion_min_component_ratio);
    const bool strong_motion_hit =
        changed_ratio >= std::max(0.02f, cfg_.sentry_passing_motion_min_changed_ratio * 4.0f) &&
        mean_delta >= std::max(1.6f, cfg_.sentry_passing_motion_min_mean_delta * 2.0f) &&
        component_ratio >= std::max(0.002f, cfg_.sentry_passing_motion_min_component_ratio * 3.0f);

    cv::Point2f motion_center{roi.x + roi.width * 0.5f, roi.y + roi.height * 0.5f};
    cv::Rect motion_box = roi;
    if (best_label > 0) {
        const float sx = static_cast<float>(centroids.at<double>(best_label, 0));
        const float sy = static_cast<float>(centroids.at<double>(best_label, 1));
        motion_center.x = roi.x + sx * static_cast<float>(roi.width) / static_cast<float>(scaled_w);
        motion_center.y = roi.y + sy * static_cast<float>(roi.height) / static_cast<float>(scaled_h);

        const int bx = stats.at<int>(best_label, cv::CC_STAT_LEFT);
        const int by = stats.at<int>(best_label, cv::CC_STAT_TOP);
        const int bw = stats.at<int>(best_label, cv::CC_STAT_WIDTH);
        const int bh = stats.at<int>(best_label, cv::CC_STAT_HEIGHT);
        motion_box = cv::Rect(
            roi.x + bx * roi.width / scaled_w,
            roi.y + by * roi.height / scaled_h,
            std::max(2, bw * roi.width / scaled_w),
            std::max(2, bh * roi.height / scaled_h)) &
            cv::Rect(0, 0, image.cols, image.rows);
    }

    if (strong_motion_hit) {
        state.streak = std::max(state.streak + 1,
                                std::max(1, cfg_.sentry_passing_motion_min_streak));
    } else if (motion_hit) {
        state.streak++;
    } else {
        state.streak = 0;
    }

    std::optional<TrackedObject> candidate;
    const int min_streak = std::max(1, cfg_.sentry_passing_motion_min_streak);
    if (state.streak >= min_streak) {
        TrackedObject obj;
        const int local_id = 88000 + static_cast<int>(std::hash<std::string>{}(camera_name) % 1000U);
        obj.track_id = cameraScopedTrackId(camera_name, local_id);
        obj.class_id = -11;
        obj.label = "passing_motion_object";
        obj.bbox = motion_box;
        obj.confidence = static_cast<float>(std::clamp(
            std::max({changed_ratio * 12.0, mean_delta / 12.0, component_ratio * 30.0}),
            0.0, 0.99));
        obj.velocity = motion_center - state.previous_motion_center;
        obj.age = state.streak;
        obj.lost = 0;
        obj.source_camera = camera_name;
        obj.has_visual_ground_position = true;
        obj.visual_ground_position_3d = passingMotionPositionForCamera(camera_name);
        obj.visual_ground_distance_m = std::hypot(obj.visual_ground_position_3d.x,
                                                  obj.visual_ground_position_3d.y);
        obj.position_3d = obj.visual_ground_position_3d;
        obj.distance_m = obj.visual_ground_distance_m;
        obj.origin = ObservationOrigin::OriginSentryRoadMotion;
        candidate = obj;

        if (state.streak == min_streak || state.streak % 20 == 0) {
            std::cout << "[SentinelNode][PassingMotion] camera=" << camera_name
                      << " streak=" << state.streak
                      << " strong=" << (strong_motion_hit ? 1 : 0)
                      << " changed_ratio=" << changed_ratio
                      << " mean_delta=" << mean_delta
                      << " component_ratio=" << component_ratio
                      << " bbox=(" << motion_box.x << "," << motion_box.y << ","
                      << motion_box.width << "," << motion_box.height << ")"
                      << " confidence=" << obj.confidence
                      << std::endl;
        }
    }

    state.previous_gray = scaled.clone();
    state.previous_motion_center = motion_center;
    state.last_timestamp_ns = frame.timestamp_ns;
    return candidate;
}

// 差值框局部放大二次检测（crop-YOLO）：全图 YOLO 对 10~20m 侧向小目标常零
// 检出（bag-1008 实测），但差值框已把运动目标定位到稳定 bbox。把该框周边
// orsm_crop_yolo_pad_ratio 比例外扩后裁剪送检测器，目标像素占比大幅提高；
// 命中生物类则生成 OriginYolo 语义观测，与真实 YOLO 检测走同一条
// anchor→L# 认领→20s 语义租约通道，为 lidar-only 轨迹提供类别支持。
// 节流：每相机每 orsm_crop_yolo_every_n 帧最多一次；已有 YOLO 生物框覆盖
// 差值框(IoU≥0.3)时跳过；输出条数受 orsm_crop_yolo_max_boxes 限制。
void SentinelNode::runOrsmCropYoloSemantic(const DecodedFrame& frame,
                                           const cv::Mat& rectified,
                                           const TrackedObject& diff_box,
                                           const FisheyeCalib* calib,
                                           const std::vector<TrackedObject>& yolo_entries,
                                           std::vector<TrackedObject>& out) {
    if (!cfg_.orsm_crop_yolo_enabled ||
        !detector_ready_.load(std::memory_order_acquire) ||
        !detector_ || !detector_->isInitialized()) {
        return;
    }
    if (rectified.empty() || diff_box.bbox.width <= 0 || diff_box.bbox.height <= 0) {
        return;
    }
    if (diff_box.age < std::max(1, cfg_.orsm_crop_yolo_min_streak)) {
        return; // 差值框尚未稳定，抑制瞬时噪声框触发推理
    }
    // 远场裁剪语义不可信：远距静止物体（停车簇/植被）被误检为 person/animal
    // 会经语义租约发布假目标，超过上限的差值框不二次检测（0=不设限）。
    if (cfg_.orsm_crop_yolo_max_distance_m > 0.f &&
        diff_box.distance_m > cfg_.orsm_crop_yolo_max_distance_m) {
        return;
    }
    // 已有 YOLO 生物框覆盖该差值框时无需二次检测（小框面积重叠比）
    for (const auto& entry : yolo_entries) {
        if (!orsmBiologicalLabel(entry.label)) continue;
        const float intersection = static_cast<float>((entry.bbox & diff_box.bbox).area());
        const float smaller = static_cast<float>(std::min(entry.bbox.area(), diff_box.bbox.area()));
        if (smaller > 0.f && intersection / smaller >= 0.30f) {
            return;
        }
    }
    // 每相机节流：每 orsm_crop_yolo_every_n 帧最多推理一次
    const std::string cam = frame.camera_name.empty() ? std::string("camera") : frame.camera_name;
    const uint64_t tick = orsm_crop_yolo_frame_counter_[cam]++;
    const int every = std::max(1, cfg_.orsm_crop_yolo_every_n_frames);
    if (tick % static_cast<uint64_t>(every) != 0) {
        return;
    }
    // 裁剪区域：差值框四周各外扩 pad_ratio，并限制最小尺寸
    const float pad_ratio = std::clamp(cfg_.orsm_crop_yolo_pad_ratio, 0.f, 2.f);
    const int pad_w = std::max(24, static_cast<int>(diff_box.bbox.width * pad_ratio));
    const int pad_h = std::max(24, static_cast<int>(diff_box.bbox.height * pad_ratio));
    cv::Rect crop_box(diff_box.bbox.x - pad_w, diff_box.bbox.y - pad_h,
                      diff_box.bbox.width + 2 * pad_w, diff_box.bbox.height + 2 * pad_h);
    crop_box &= cv::Rect(0, 0, rectified.cols, rectified.rows);
    if (crop_box.width < 48 || crop_box.height < 48) {
        return;
    }
    const std::vector<Detection> crop_dets = detector_->detect(rectified(crop_box));
    const float min_conf = std::clamp(cfg_.orsm_crop_yolo_min_confidence, 0.05f, 0.95f);
    std::vector<const Detection*> biological;
    for (const auto& det : crop_dets) {
        if (!orsmBiologicalLabel(det.label)) continue;
        if (det.confidence < min_conf) continue;
        biological.push_back(&det);
    }
    if (biological.empty()) {
        return;
    }
    std::sort(biological.begin(), biological.end(),
              [](const Detection* a, const Detection* b) {
                  return a->confidence > b->confidence;
              });
    const int max_boxes = std::max(1, cfg_.orsm_crop_yolo_max_boxes);
    const int local_id = 97000 + static_cast<int>(std::hash<std::string>{}(cam) % 1000U);
    int emitted = 0;
    for (const Detection* best : biological) {
        if (emitted >= max_boxes) break;
        TrackedObject obj;
        obj.track_id = cameraScopedTrackId(cam, local_id);
        obj.class_id = best->class_id;
        obj.label = best->label;
        obj.bbox = best->bbox + crop_box.tl();
        obj.confidence = best->confidence;
        obj.age = 1;
        obj.lost = 0;
        obj.source_camera = cam;
        enrichVisualGeometry(obj, calib, cfg_.camera_width, cfg_.camera_height);
        if (obj.distance_m <= 0.f || !finitePoint2D(obj.position_3d)) continue;
        // 发射侧同口径距离门：裁剪框内检测位置与差值框底边像素不同，far ray
        // 每像素 0.5~0.75m，仅门控差值框估距会漏出 20.3~24.2m 的发射（v4b 实测）。
        if (cfg_.orsm_crop_yolo_max_distance_m > 0.f &&
            obj.distance_m > cfg_.orsm_crop_yolo_max_distance_m) {
            continue;
        }
        obj.origin = ObservationOrigin::OriginYolo;
        std::cout << "[ORSMCropYoloSemantic] camera=" << cam
                  << " seq=" << frame.seq
                  << " diff_box=(" << diff_box.bbox.x << ',' << diff_box.bbox.y << ','
                  << diff_box.bbox.width << ',' << diff_box.bbox.height << ')'
                  << " crop_box=(" << crop_box.x << ',' << crop_box.y << ','
                  << crop_box.width << ',' << crop_box.height << ')'
                  << " label=" << obj.label
                  << " conf=" << obj.confidence
                  << " dist=" << obj.distance_m
                  << std::endl;
        out.push_back(std::move(obj));
        ++emitted;
    }
}

std::optional<TrackedObject> SentinelNode::updateOrsmFarImageMotionCandidate(const DecodedFrame& frame,
                                                                             const cv::Mat& image) {
    if (!cfg_.orsm_far_image_motion_tracking_enabled ||
        !cfg_.orsm_far_image_motion_allow_lidar_backed_new_targets ||
        image.empty() || frame.timestamp_ns == 0) {
        return std::nullopt;
    }

    const std::string camera_name = frame.camera_name.empty() ? "camera" : frame.camera_name;
    cv::Mat gray;
    if (image.channels() == 1) {
        gray = image;
    } else {
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    }

    cv::Mat scaled;
    // Keep the far Inbox difference mask at the original camera resolution,
    // matching master_newest.  Downscaling to 360x288 made small distant
    // targets occupy too few pixels and introduced avoidable box quantization.
    cv::GaussianBlur(gray, scaled, cv::Size(5, 5), 0.0);
    const int scaled_w = scaled.cols;
    const int scaled_h = scaled.rows;
    cv::GaussianBlur(scaled, scaled, cv::Size(5, 5), 0.0);

    std::lock_guard<std::mutex> lk(orsm_far_image_motion_mutex_);
    auto& state = orsm_far_image_motion_states_[camera_name];
    if (state.previous_gray.empty()) {
        state.previous_gray = scaled.clone();
        state.previous_motion_center = {scaled_w * 0.5f, scaled_h * 0.5f};
        state.last_timestamp_ns = frame.timestamp_ns;
        return std::nullopt;
    }

    const uint64_t max_age_ns =
        static_cast<uint64_t>(std::max(0, cfg_.orsm_far_image_motion_max_age_ms)) *
        1000000ULL;
    if (max_age_ns > 0 &&
        frame.timestamp_ns > state.last_timestamp_ns &&
        frame.timestamp_ns - state.last_timestamp_ns > max_age_ns) {
        state.previous_gray = scaled.clone();
        state.streak = 0;
        state.last_timestamp_ns = frame.timestamp_ns;
        return std::nullopt;
    }

    cv::Mat diff;
    cv::absdiff(scaled, state.previous_gray, diff);
    cv::Mat mask;
    cv::threshold(diff, mask, 18.0, 255.0, cv::THRESH_BINARY);
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, cv::Mat(), cv::Point(-1, -1), 1);
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, cv::Mat(), cv::Point(-1, -1), 1);

    const int mask_area = std::max(1, mask.rows * mask.cols);
    const double changed_ratio = static_cast<double>(cv::countNonZero(mask)) /
        static_cast<double>(mask_area);
    const double mean_delta = cv::mean(diff)[0];

    cv::Mat labels;
    cv::Mat stats;
    cv::Mat centroids;
    const int components = cv::connectedComponentsWithStats(mask, labels, stats, centroids, 8);
    int best_label = -1;
    int best_area = 0;
    int largest_unfiltered_area = 0;
    const double max_box_area_ratio =
        std::max(0.0f, cfg_.orsm_far_image_motion_max_box_area_ratio);
    // Ignore motion components above the measured road region before choosing
    // the largest component.  This lets a lower valid component in the same
    // frame become the winner instead of dropping the whole frame after the
    // largest (too-high) component has already been selected.
    int min_bottom_y = 0;
    if (camera_name == "front_camera_fov195") {
        min_bottom_y = 470;
    } else if (camera_name == "rear_camera_fov195") {
        min_bottom_y = 512;
    } else if (camera_name == "left_camera_fov195") {
        min_bottom_y = 510;
    }
    const int min_bottom_y_scaled = image.rows > 0
        ? min_bottom_y * scaled_h / image.rows
        : min_bottom_y;
    const FisheyeCalib* calib = inbox_fisheye_mgr_
        ? inbox_fisheye_mgr_->getCalib(camera_name)
        : nullptr;
    // Do not reject a motion component just because its bbox-bottom ray cannot
    // intersect the ground plane. Ground-ray distance is evaluated later by
    // enrichVisualGeometry(), which has a fallback estimator for this case.
    for (int label = 1; label < components; ++label) {
        const int area = stats.at<int>(label, cv::CC_STAT_AREA);
        largest_unfiltered_area = std::max(largest_unfiltered_area, area);
        const int top = stats.at<int>(label, cv::CC_STAT_TOP);
        const int width = stats.at<int>(label, cv::CC_STAT_WIDTH);
        const int height = stats.at<int>(label, cv::CC_STAT_HEIGHT);
        const double box_area_ratio =
            static_cast<double>(width) * static_cast<double>(height) /
            static_cast<double>(mask_area);
        // Reject an oversized motion region before selecting the winner.  This
        // allows the largest remaining valid component (for example, a distant
        // pedestrian) to survive instead of being discarded with a near-field
        // foreground blob and only rejected later in the LiDAR association.
        if (max_box_area_ratio > 0.0 && box_area_ratio > max_box_area_ratio) {
            continue;
        }
        if (min_bottom_y_scaled > 0 && top + height < min_bottom_y_scaled) {
            continue;
        }
        if (area > best_area) {
            best_area = area;
            best_label = label;
        }
    }
    const double component_ratio = static_cast<double>(best_area) /
        static_cast<double>(mask_area);
    const double largest_unfiltered_component_ratio =
        static_cast<double>(largest_unfiltered_area) / static_cast<double>(mask_area);
    const bool global_flash =
        changed_ratio > 0.60 && largest_unfiltered_component_ratio > 0.40;
    const bool motion_hit =
        best_label > 0 && !global_flash &&
        changed_ratio >= std::max(0.0f, cfg_.orsm_far_image_motion_min_changed_ratio) &&
        mean_delta >= std::max(0.0f, cfg_.orsm_far_image_motion_min_mean_delta) &&
        component_ratio >= std::max(0.0f, cfg_.orsm_far_image_motion_min_component_ratio);

    cv::Point2f motion_center{scaled_w * 0.5f, scaled_h * 0.5f};
    cv::Rect motion_box_scaled(0, 0, scaled_w, scaled_h);
    if (best_label > 0) {
        const float sx = static_cast<float>(centroids.at<double>(best_label, 0));
        const float sy = static_cast<float>(centroids.at<double>(best_label, 1));
        motion_center = {sx, sy};
        motion_box_scaled = cv::Rect(stats.at<int>(best_label, cv::CC_STAT_LEFT),
                                     stats.at<int>(best_label, cv::CC_STAT_TOP),
                                     stats.at<int>(best_label, cv::CC_STAT_WIDTH),
                                     stats.at<int>(best_label, cv::CC_STAT_HEIGHT));
    }

    if (motion_hit) {
        state.streak++;
    } else {
        state.streak = 0;
    }

    std::optional<TrackedObject> candidate;
    const int min_streak = std::max(1, cfg_.orsm_far_image_motion_min_streak);
    if (state.streak >= min_streak && best_label > 0) {
        // Keep the raw connected-component box.  Any downward padding moves the
        // bottom edge away from the target's feet, and the ground-plane ray is
        // extremely sensitive to that pixel (about 0.5--0.75 m per pixel near
        // 20 m), which previously under-estimated the box range by several
        // metres.
        cv::Rect motion_box(
            motion_box_scaled.x * image.cols / scaled_w,
            motion_box_scaled.y * image.rows / scaled_h,
            std::max(2, motion_box_scaled.width * image.cols / scaled_w),
            std::max(2, motion_box_scaled.height * image.rows / scaled_h));
        motion_box &= cv::Rect(0, 0, image.cols, image.rows);

        TrackedObject obj;
        const int local_id = 99000 + static_cast<int>(std::hash<std::string>{}(camera_name) % 1000U);
        obj.track_id = cameraScopedTrackId(camera_name, local_id);
        obj.class_id = -1;
        obj.label = "unknown";
        obj.bbox = motion_box;
        const double max_motion_confidence = std::clamp(
            static_cast<double>(cfg_.orsm_far_image_motion_confidence), 0.05, 0.99);
        obj.confidence = static_cast<float>(std::clamp(
            std::max({changed_ratio * 20.0, mean_delta / 10.0, component_ratio * 80.0}),
            0.0, max_motion_confidence));
        obj.velocity = motion_center - state.previous_motion_center;
        obj.age = state.streak;
        obj.lost = 0;
        obj.source_camera = camera_name;

        // The difference box feeds the Inbox association, so its geometry must be
        // derived from the same dedicated extrinsic set that Inbox later uses to
        // project clusters (cfg_.inbox_calib_dir).  Using the generic calibration
        // here made the box's own range estimate inconsistent with the projection.
        enrichVisualGeometry(obj, calib, cfg_.camera_width, cfg_.camera_height);
        obj.origin = ObservationOrigin::OriginDiffbox;
        const float bbox_center_u = obj.bbox.x + 0.5f * obj.bbox.width;
        const float bbox_bottom_v = obj.bbox.y + obj.bbox.height;
        const float override_distance_m =
            std::max(0.0f, cfg_.orsm_rear_motion_range_override_distance_m);
        const bool rear_range_override =
            cfg_.orsm_rear_motion_range_override_enabled &&
            camera_name == "rear_camera_fov195" &&
            bbox_center_u >= cfg_.orsm_rear_motion_range_override_min_center_u_px &&
            bbox_center_u <= cfg_.orsm_rear_motion_range_override_max_center_u_px &&
            bbox_bottom_v >= cfg_.orsm_rear_motion_range_override_min_bottom_v_px &&
            bbox_bottom_v <= cfg_.orsm_rear_motion_range_override_max_bottom_v_px &&
            override_distance_m > 0.0f;
        if (rear_range_override) {
            const float original_distance_m =
                obj.has_visual_ground_position && obj.visual_ground_distance_m > 0.0f
                ? obj.visual_ground_distance_m
                : obj.distance_m;
            const float position_range_m = std::hypot(obj.position_3d.x, obj.position_3d.y);
            if (position_range_m > 1.e-3f) {
                const float scale = override_distance_m / position_range_m;
                obj.position_3d.x *= scale;
                obj.position_3d.y *= scale;
            }
            obj.distance_m = override_distance_m;
            if (obj.has_visual_ground_position) {
                const float visual_range_m = std::hypot(
                    obj.visual_ground_position_3d.x,
                    obj.visual_ground_position_3d.y);
                if (visual_range_m > 1.e-3f) {
                    const float scale = override_distance_m / visual_range_m;
                    obj.visual_ground_position_3d.x *= scale;
                    obj.visual_ground_position_3d.y *= scale;
                }
                obj.visual_ground_distance_m = override_distance_m;
            }
            std::cout << "[ORSMFarRangeOverride] camera=" << camera_name
                      << " center_u=" << bbox_center_u
                      << " bottom_v=" << bbox_bottom_v
                      << " original=" << original_distance_m
                      << " corrected=" << override_distance_m
                      << std::endl;
        }
        const float motion_distance = obj.has_visual_ground_position && obj.visual_ground_distance_m > 0.f
            ? obj.visual_ground_distance_m
            : obj.distance_m;
        // The difference box is only an image-region proposal.  Its bottom-edge
        // ground-plane estimate is now trustworthy (the box is no longer padded),
        // so Inbox uses it for the upper sanity cap and the relative near filter;
        // the absolute distance gate is still applied to the LiDAR cluster.
        candidate = obj;
        if (cfg_.orsm_far_image_motion_debug_every_frame ||
            state.streak == min_streak || state.streak % 10 == 0) {
            std::cout << "[ORSMFarImageMotionProposal] camera=" << camera_name
                      << " streak=" << state.streak
                      << " ts=" << frame.timestamp_ns
                      << " changed_ratio=" << changed_ratio
                      << " mean_delta=" << mean_delta
                      << " component_ratio=" << component_ratio
                      << " bbox=(" << obj.bbox.x << "," << obj.bbox.y << ","
                      << obj.bbox.width << "," << obj.bbox.height << ")"
                      << " bbox_height_px=" << obj.bbox.height
                      << " bbox_bottom_y_px=" << (obj.bbox.y + obj.bbox.height)
                      << " visual_distance=" << motion_distance
                      << std::endl;
        }
    }

    state.previous_gray = scaled.clone();
    state.previous_motion_center = motion_center;
    state.last_timestamp_ns = frame.timestamp_ns;
    return candidate;
}

void SentinelNode::processFrame(const DecodedFrame& frame) {
    const uint64_t total_start_ns = currentSteadyTimeNs();
    if (frame.recording_group_id > 0) {
        current_processing_group_id_.store(frame.recording_group_id, std::memory_order_relaxed);
    }
    if (frame.processing_enqueue_steady_ns > 0 && total_start_ns > frame.processing_enqueue_steady_ns) {
        recordPerfMetric("frame_queue_wait_ms",
                         static_cast<double>(total_start_ns - frame.processing_enqueue_steady_ns) / 1e6);
    }

    std::lock_guard<std::mutex> process_lk(processing_mutex_);
    frame_count_++;
    if (frame.timestamp_ns > 0) {
        last_sensor_time_ns_.store(frame.timestamp_ns, std::memory_order_relaxed);
    }

    evaluateSentryRequestTriggers(frame.timestamp_ns);
    OddConditions odd = buildOdd();
    latest_odd_ = odd;

    if (cfg_.camera_decode_debug_enabled && cfg_.camera_decode_debug_save_node_images) {
        saveCameraDecodeDebugImage("node_raw_bgr", frame, frame.image);
        if (fisheye_mgr_ && !frame.camera_name.empty() &&
            fisheye_mgr_->hasCamera(frame.camera_name)) {
            saveCameraDecodeDebugImage("undistorted_bgr", frame,
                                       fisheye_mgr_->undistort(frame.camera_name, frame.image));
        }
    }

    if (!shouldRunHeavyPerception(odd)) {
        const uint64_t sentry_stage_start_ns = currentSteadyTimeNs();
        const SentryState state_before_update = state_machine_ ? state_machine_->currentState() : SentryState::UNAVAILABLE;
        if (state_machine_) {
            state_machine_->update(odd, ThreatLevel::NONE, 0, frame.timestamp_ns);
        }
        recordPerfMetric("process.sentry_alert_state_ms",
                         static_cast<double>(currentSteadyTimeNs() - sentry_stage_start_ns) / 1e6);

        static std::atomic<uint64_t> perception_gate_skip_log_count{0};
        const uint64_t skip_idx = ++perception_gate_skip_log_count;
        if (skip_idx <= 10 || skip_idx % 100 == 0) {
            std::cout << "[PerceptionGate] skip_heavy_perception"
                      << " frame_count=" << frame_count_.load()
                      << " camera=" << (frame.camera_name.empty() ? "camera" : frame.camera_name)
                      << " seq=" << frame.seq
                      << " state_before=" << static_cast<int>(state_before_update)
                      << " state_after=" << (state_machine_ ? static_cast<int>(state_machine_->currentState()) : 0)
                      << " odd_status=" << static_cast<int>(odd.oddStatus())
                      << " user_enabled=" << (odd.user_enabled ? 1 : 0)
                      << " user_shutdown=" << (odd.user_shutdown_requested ? 1 : 0)
                      << " fortress_requested=0"
                      << std::endl;
        }

        const double process_total_ms = static_cast<double>(currentSteadyTimeNs() - total_start_ns) / 1e6;
        recordPerfMetric(kAlgorithmProcessTimeMetric, process_total_ms);
        logPerfSummaryIfNeeded(frame_count_.load());
        return;
    }

    // 鱼眼去畸变：解码后的原始图像经过 remap 去除径向畸变
    uint64_t stage_start_ns = currentSteadyTimeNs();
    cv::Mat rectified;
    if (fisheye_mgr_ && !frame.camera_name.empty() &&
        fisheye_mgr_->hasCamera(frame.camera_name)) {
        rectified = fisheye_mgr_->undistort(frame.camera_name, frame.image);
    } else {
        rectified = frame.image;  // 无标定参数时回退为原图
    }
    recordPerfMetric("process.undistort_ms", static_cast<double>(currentSteadyTimeNs() - stage_start_ns) / 1e6);

    // 目标检测（去畸变后的图像送入检测器）
    stage_start_ns = currentSteadyTimeNs();
    std::vector<Detection> detections;
    if (detector_ready_.load(std::memory_order_acquire)) {
        detections = detector_->detect(rectified);
    } else {
        static std::atomic<uint64_t> detector_unavailable_log_count{0};
        const uint64_t idx = ++detector_unavailable_log_count;
        if (idx <= 10 || idx % 100 == 0) {
            std::cout << "[SentinelNode][DetectorUnavailable]"
                      << " camera=" << (frame.camera_name.empty() ? "camera" : frame.camera_name)
                      << " seq=" << frame.seq
                      << " fallback=near_field_contact_motion,passing_motion"
                      << std::endl;
        }
    }
    recordPerfMetric("process.detect_ms", static_cast<double>(currentSteadyTimeNs() - stage_start_ns) / 1e6);

    if (cfg_.orsm_debug_track) {
        std::cout << "[PerceptionStageDiag][Detector]"
                  << " camera=" << (frame.camera_name.empty() ? "camera" : frame.camera_name)
                  << " seq=" << frame.seq
                  << " ts=" << frame.timestamp_ns
                  << " count=" << detections.size()
                  << " details=[";
        for (size_t i = 0; i < detections.size(); ++i) {
            const auto& det = detections[i];
            if (i > 0) std::cout << ";";
            std::cout << det.label << "#" << det.class_id
                      << " conf=" << det.confidence
                      << " bbox=(" << det.bbox.x << "," << det.bbox.y << ","
                      << det.bbox.width << "," << det.bbox.height << ")";
        }
        std::cout << "]" << std::endl;
    }

    // Camera-local tracking gives ORSM a stable visual identity and explicitly
    // distinguishes a real detector hit from a short tracker prediction.
    stage_start_ns = currentSteadyTimeNs();
    const std::string tracker_key = frame.camera_name.empty() ? std::string("default_camera") : frame.camera_name;
    std::vector<TrackedObject> tracked;
    {
        auto& tracker = trackers_by_camera_[tracker_key];
        if (!tracker) {
            tracker = std::make_shared<Tracker>(30,
                                                fortressModeActiveForProcessing() ? 1 : 3,
                                                cfg_.visual_tracker_iou_threshold,
                                                cfg_.visual_track_max_lost_frames,
                                                cfg_.visual_tracker_high_conf_threshold,
                                                cfg_.visual_tracker_low_conf_threshold,
                                                cfg_.visual_tracker_new_track_threshold,
                                                cfg_.visual_tracker_low_match_iou_threshold);
        }
        tracked = tracker->update(detections);
        for (auto& obj : tracked) {
            obj.origin = obj.lost == 0
                ? ObservationOrigin::OriginYolo
                : ObservationOrigin::OriginTrackerPrediction;
        }
        if (cfg_.orsm_debug_track) {
            std::cout << "[PerceptionStageDiag][ByteTrack]"
                      << " camera=" << tracker_key
                      << " seq=" << frame.seq
                      << " ts=" << frame.timestamp_ns
                      << " detection_count=" << detections.size()
                      << " track_count=" << tracked.size()
                      << " details=[";
            for (size_t i = 0; i < tracked.size(); ++i) {
                const auto& obj = tracked[i];
                if (i > 0) std::cout << ";";
                std::cout << "id=" << obj.track_id
                          << " " << obj.label << "#" << obj.class_id
                          << " conf=" << obj.confidence
                          << " age=" << obj.age
                          << " lost=" << obj.lost
                          << " bbox=(" << obj.bbox.x << "," << obj.bbox.y << ","
                          << obj.bbox.width << "," << obj.bbox.height << ")";
            }
            std::cout << "]" << std::endl;
        }

        // Detection fallback：设计要求人员接近/拉门类事件不能完全依赖已确认 track。
        // 当同一相机短时间连续有检测但 tracker 尚未确认时，生成临时目标参与低/一般威胁评估。
        if (!fortressModeActiveForProcessing() && tracked.empty()) {
            auto& streak = detection_fallback_streak_[tracker_key];
            if (!detections.empty()) {
                streak++;
                if (streak >= std::max(1, cfg_.visual_fallback_min_streak)) {
                    for (size_t i = 0; i < detections.size() && tracked.size() < 3; ++i) {
                        const auto& det = detections[i];
                        TrackedObject obj;
                        obj.track_id = -1000 - static_cast<int>(std::hash<std::string>{}(tracker_key) % 900) - static_cast<int>(i);
                        obj.class_id = det.class_id;
                        obj.label = det.label;
                        obj.bbox = det.bbox;
                        obj.confidence = det.confidence;
                        obj.age = streak;
                        obj.lost = 0;
                        tracked.push_back(obj);
                    }
                    if (!tracked.empty()) {
                        std::cout << "[SentinelNode][DetectionFallback] camera=" << tracker_key
                                  << " detections=" << detections.size()
                                  << " fallback_targets=" << tracked.size();
                        if (cfg_.orsm_debug_track) {
                            std::cout << " details=[";
                            for (size_t j = 0; j < tracked.size(); ++j) {
                                if (j > 0) std::cout << ";";
                                std::cout << tracked[j].label << "#" << tracked[j].class_id
                                          << " conf=" << tracked[j].confidence
                                          << " bbox=(" << tracked[j].bbox.x << ","
                                          << tracked[j].bbox.y << ","
                                          << tracked[j].bbox.width << ","
                                          << tracked[j].bbox.height << ")";
                            }
                            std::cout << "]";
                        }
                        std::cout << std::endl;
                    }
                }
            } else {
                streak = 0;
            }
        } else if (!fortressModeActiveForProcessing()) {
            detection_fallback_streak_[tracker_key] = 0;
        }
    }
    recordPerfMetric("process.track_fallback_ms", static_cast<double>(currentSteadyTimeNs() - stage_start_ns) / 1e6);

    // 读取最新传感器快照
    stage_start_ns = currentSteadyTimeNs();
    UssData    uss_snap   = uss_   ? uss_->latestPdc()       : UssData{};
    UssPointCloudSnapshot uss_cloud = uss_
        ? uss_->closestPointCloud(frame.timestamp_ns, cfg_.uss_fusion_max_age_ms)
        : UssPointCloudSnapshot{};
    UssClusterSnapshot uss_clusters = uss_
        ? uss_->closestClusters(frame.timestamp_ns, cfg_.uss_fusion_max_age_ms)
        : UssClusterSnapshot{};
    if (uss_cloud.valid() && frame.timestamp_ns > 0) {
        recordPerfMetric("uss_delta_ms", static_cast<double>(absDiffNs(uss_cloud.timestamp_ns, frame.timestamp_ns)) / 1.0e6);
    }
    recordPerfMetric("process.uss_snapshot_ms", static_cast<double>(currentSteadyTimeNs() - stage_start_ns) / 1e6);

    const bool orsm_lidar_fusion_enabled = fortressModeActiveForProcessing();
    const bool use_group_lidar_fusion =
        orsm_lidar_fusion_enabled && cfg_.enable_camera_sync && frame.recording_group_size > 1;
    const uint64_t lidar_fusion_period_ns =
        static_cast<uint64_t>(std::max(0, cfg_.orsm_lidar_fusion_period_ms)) * 1000000ULL;
    const uint64_t lidar_fusion_now_ns = currentSteadyTimeNs();
    const bool lidar_fusion_due = orsm_lidar_fusion_enabled &&
        (last_lidar_fusion_steady_ns_ == 0 || lidar_fusion_period_ns == 0 ||
         lidar_fusion_now_ns - last_lidar_fusion_steady_ns_ >= lidar_fusion_period_ns);

    LidarFrame lidar_snap;
    stage_start_ns = currentSteadyTimeNs();
    if (lidar_fusion_due && !use_group_lidar_fusion) {
        lidar_snap = mergedLidarFrame(frame.timestamp_ns);
        if (lidar_snap.valid() && frame.timestamp_ns > 0) {
            recordPerfMetric("lidar_delta_ms", static_cast<double>(absDiffNs(lidar_snap.timestamp_ns, frame.timestamp_ns)) / 1.0e6);
        }
        // LiDAR merge/cache 本身也按周期推进；即使当前没有视觉候选，也不应逐帧重复 merge。
        last_lidar_fusion_steady_ns_ = lidar_fusion_now_ns;
    }
    recordPerfMetric("process.lidar_merge_ms", static_cast<double>(currentSteadyTimeNs() - stage_start_ns) / 1e6);

    auto sentry_tracked = stableVisualTargets(tracked,
                                              true,
                                              cfg_.visual_track_confirm_min_age,
                                              cfg_.visual_track_max_lost_frames,
                                              cfg_.visual_track_min_confidence);
    stage_start_ns = currentSteadyTimeNs();
    if (cfg_.enable_uss_vision_fusion && uss_cloud.valid() && uss_clusters.valid()) {
        sentry_tracked = uss_vision_fusion_.fuse(
            sentry_tracked,
            uss_cloud,
            uss_clusters,
            frame.camera_name,
            frame.timestamp_ns,
            cfg_.camera_height,
            cfg_.uss_fusion_max_range_m,
            cfg_.uss_fusion_candidate_vision_range_m,
            cfg_.uss_fusion_max_age_ms,
            cfg_.uss_fusion_confidence_boost);
    }
    for (auto& obj : sentry_tracked) {
        const FisheyeCalib* calib = fisheye_mgr_
            ? fisheye_mgr_->getCalib(frame.camera_name)
            : nullptr;
        enrichVisualGeometry(obj, calib, cfg_.camera_width, cfg_.camera_height);
        obj.source_camera = frame.camera_name;
        obj.track_id = cameraScopedTrackId(frame.camera_name, obj.track_id);
    }
    if (auto near_field_motion = updateNearFieldMotionCandidate(frame, frame.image)) {
        sentry_tracked.push_back(*near_field_motion);
    }
    if (auto passing_motion = updatePassingMotionCandidate(frame, frame.image)) {
        sentry_tracked.push_back(*passing_motion);
    }
    recordPerfMetric("process.uss_fusion_ms", static_cast<double>(currentSteadyTimeNs() - stage_start_ns) / 1e6);

    std::vector<TrackedObject> orsm_tracked;
    if (fortressModeActiveForProcessing()) {
        // Fortress YOLO range uses the same Inbox extrinsics as the
        // difference-box/LiDAR projection path, with normal calibration fallback.
        const FisheyeCalib* calib = inbox_fisheye_mgr_
            ? inbox_fisheye_mgr_->getCalib(frame.camera_name)
            : nullptr;
        if (!calib && fisheye_mgr_) {
            calib = fisheye_mgr_->getCalib(frame.camera_name);
        }
        orsm_tracked.reserve(detections.size());
        for (const auto& det : detections) {
            if (det.confidence < cfg_.visual_track_min_confidence) continue;
            const TrackedObject* matched_track = nullptr;
            float best_iou = 0.f;
            for (const auto& visual_track : tracked) {
                if (visual_track.lost != 0 || visual_track.class_id != det.class_id) continue;
                const cv::Rect intersection = visual_track.bbox & det.bbox;
                const float union_area = static_cast<float>(visual_track.bbox.area() +
                    det.bbox.area() - intersection.area());
                const float iou = union_area > 0.f
                    ? static_cast<float>(intersection.area()) / union_area
                    : 0.f;
                if (iou > best_iou) {
                    best_iou = iou;
                    matched_track = &visual_track;
                }
            }
            TrackedObject obj;
            obj.track_id = cameraScopedTrackId(
                frame.camera_name,
                matched_track && best_iou >= 0.30f
                    ? matched_track->track_id
                    : 70000 + static_cast<int>(orsm_tracked.size()));
            obj.class_id = det.class_id;
            obj.label = det.label;
            obj.bbox = det.bbox;
            obj.confidence = det.confidence;
            obj.age = matched_track ? matched_track->age : 1;
            obj.lost = 0;
            obj.origin = ObservationOrigin::OriginYolo;
            obj.source_camera = frame.camera_name;
            obj.orsm_visual_track_id = obj.track_id;
            enrichVisualGeometry(obj, calib, cfg_.camera_width, cfg_.camera_height);
            if (obj.distance_m <= 0.f || !finitePoint2D(obj.position_3d)) continue;
            orsm_tracked.push_back(std::move(obj));
        }
        if (auto far_motion = updateOrsmFarImageMotionCandidate(frame, rectified)) {
            orsm_tracked.push_back(*far_motion);
            // crop-YOLO：全图 YOLO 对远距小目标常零检出（bag-1008 实测），差值框
            // 已把运动目标定位到稳定 bbox；裁剪周边放大后二次检测，命中生物类则
            // 生成 OriginYolo 观测，与真实 YOLO 检测共用 anchor→L# 认领→语义租约
            // 通道（默认关闭，配置 crop_yolo_enabled 开启）。
            std::vector<TrackedObject> crop_semantics;
            runOrsmCropYoloSemantic(frame, rectified, *far_motion, calib,
                                    orsm_tracked, crop_semantics);
            for (auto& semantic_obj : crop_semantics) {
                orsm_tracked.push_back(std::move(semantic_obj));
            }
        }
    }
    // 视觉-LiDAR 融合结果仅供越野堡垒(ORSM)目标定位/测速使用。
    // 哨兵模式不允许使用 LiDAR；USS fused点云不可用时，AlertManager只消费视频跟踪结果 + IMU。
    // 因此必须先判定 ORSM 被请求，并按配置限频，避免逐 camera frame 全量点云融合拖垮处理队列。
    stage_start_ns = currentSteadyTimeNs();
    recordPerfMetric("process.lidar_fusion_ms", static_cast<double>(currentSteadyTimeNs() - stage_start_ns) / 1e6);

    stage_start_ns = currentSteadyTimeNs();
    std::vector<TrackedObject> orsm_group_candidates;
    LidarFrame orsm_group_lidar = lidar_snap;
    bool run_orsm_group_update = false;
    const uint64_t orsm_group_ts = frame.recording_group_timestamp_ns > 0
        ? frame.recording_group_timestamp_ns
        : frame.timestamp_ns;
    if (fortressModeActiveForProcessing()) {
        const uint64_t group_id = frame.recording_group_id > 0
            ? frame.recording_group_id
            : (frame.timestamp_ns > 0 ? frame.timestamp_ns : frame.seq);
        const size_t expected_group_size = std::max<size_t>(1, frame.recording_group_size);
        auto& pending = pending_orsm_groups_[group_id];
        if (pending.processed_count == 0) {
            pending.group_id = group_id;
            pending.timestamp_ns = orsm_group_ts;
            pending.expected_size = expected_group_size;
        }
        pending.processed_count++;
        pending.candidates.insert(pending.candidates.end(), orsm_tracked.begin(), orsm_tracked.end());
        if (lidar_snap.valid()) pending.lidar = lidar_snap;
        if (pending.processed_count >= pending.expected_size) {
            auto group_candidates = pending.candidates;
            if (cfg_.orsm_debug_track) {
                std::cout << "[PerceptionStageDiag][GroupPreLiDAR]"
                          << " group_id=" << group_id
                          << " ts=" << orsm_group_ts
                          << " candidates=" << group_candidates.size()
                          << " details=[";
                for (size_t i = 0; i < group_candidates.size(); ++i) {
                    const auto& obj = group_candidates[i];
                    if (i > 0) std::cout << ";";
                    std::cout << obj.source_camera << ":id=" << obj.track_id
                              << " " << obj.label << "#" << obj.class_id
                              << " conf=" << obj.confidence
                              << " age=" << obj.age
                              << " lost=" << obj.lost
                              << " bbox=(" << obj.bbox.x << "," << obj.bbox.y << ","
                              << obj.bbox.width << "," << obj.bbox.height << ")";
                }
                std::cout << "]" << std::endl;
            }
            LidarFrame group_lidar = mergedLidarFrame(orsm_group_ts);
            if (group_lidar.valid()) {
                pending.lidar = group_lidar;
                recordPerfMetric("lidar_delta_ms", static_cast<double>(absDiffNs(group_lidar.timestamp_ns, orsm_group_ts)) / 1.0e6);
            }
            if (cfg_.orsm_debug_track) {
                const auto lidar_matched = std::count_if(
                    group_candidates.begin(), group_candidates.end(),
                    [](const TrackedObject& obj) { return obj.has_lidar_fusion; });
                std::cout << "[PerceptionStageDiag][GroupPostLiDAR]"
                          << " group_id=" << group_id
                          << " ts=" << orsm_group_ts
                          << " candidates=" << group_candidates.size()
                          << " lidar_matched=" << lidar_matched
                          << " details=[";
                for (size_t i = 0; i < group_candidates.size(); ++i) {
                    const auto& obj = group_candidates[i];
                    if (i > 0) std::cout << ";";
                    std::cout << obj.source_camera << ":id=" << obj.track_id
                              << " lidar=" << (obj.has_lidar_fusion ? 1 : 0)
                              << " dist=" << obj.distance_m
                              << " pos=(" << obj.position_3d.x << "," << obj.position_3d.y
                              << "," << obj.position_3d.z << ")";
                }
                std::cout << "]" << std::endl;
            }
            orsm_group_candidates = group_candidates;
            if (cfg_.orsm_debug_track) {
                std::cout << "[PerceptionStageDiag][GroupPostDedupe]"
                          << " group_id=" << group_id
                          << " ts=" << orsm_group_ts
                          << " before=" << group_candidates.size()
                          << " after=" << orsm_group_candidates.size()
                          << std::endl;
            }
            if (pending.lidar.valid()) orsm_group_lidar = pending.lidar;
            pending_orsm_groups_.erase(group_id);
            run_orsm_group_update = true;
        }
        if (pending_orsm_groups_.size() > 16) {
            std::cout << "[ORSMGroupTrack] pending overflow, clear stale groups size="
                      << pending_orsm_groups_.size() << std::endl;
            pending_orsm_groups_.clear();
        }
    } else {
        if (!orsm_global_tracks_.empty() || !orsm_lidar_temporal_tracks_.empty()) {
            resetOrsmGlobalTracks();
        } else {
            clearOrsmStaticFootprints("fortress_inactive");
        }
        pending_orsm_groups_.clear();
    }

    std::vector<TrackedObject> orsm_global_tracked;
    if (run_orsm_group_update) {
        orsm_global_tracked = updateOrsmGlobalTracks(orsm_group_candidates, orsm_group_lidar,
                                                     "sync_group", orsm_group_ts);
    }
    recordPerfMetric("process.orsm_global_track_ms", static_cast<double>(currentSteadyTimeNs() - stage_start_ns) / 1e6);

    stage_start_ns = currentSteadyTimeNs();
    std::vector<TrackedObject> sentry_published_targets;
    ImuData sentry_group_imu = frame.imu;
    UssData sentry_group_uss = uss_snap;
    bool run_sentry_group_update = false;
    const uint64_t sentry_group_ts = frame.recording_group_timestamp_ns > 0
        ? frame.recording_group_timestamp_ns
        : frame.timestamp_ns;
    if (!fortressModeActiveForProcessing()) {
        const uint64_t group_id = frame.recording_group_id > 0
            ? frame.recording_group_id
            : (frame.timestamp_ns > 0 ? frame.timestamp_ns : frame.seq);
        const size_t expected_group_size = std::max<size_t>(1, frame.recording_group_size);
        auto& pending = pending_sentry_groups_[group_id];
        if (pending.processed_count == 0) {
            pending.group_id = group_id;
            pending.timestamp_ns = sentry_group_ts;
            pending.expected_size = expected_group_size;
        }
        pending.processed_count++;
        pending.internal_tracks.insert(pending.internal_tracks.end(), sentry_tracked.begin(), sentry_tracked.end());
        // 同一四相机同步组内保留最强 IMU 快照，不允许最后一路普通样本
        // 覆盖前一路已捕获的锁存振动。
        const auto imu_rank = [](const ImuData& sample) {
            if (sample.impact_alert || sample.severe_vibration_alert) return 2;
            if (sample.vibration_accel_rms >= 0.08f ||
                sample.vibration_gyro_rms >= 0.0010f ||
                sample.delta_accel >= 0.30f ||
                sample.delta_gyro >= 0.004f) return 1;
            return sample.vibration_alert ? 1 : 0;
        };
        const auto imu_short_disturbance_score = [](const ImuData& sample) {
            return std::max({sample.vibration_accel_rms / 0.08f,
                             sample.vibration_gyro_rms / 0.0010f,
                             sample.delta_accel / 0.30f,
                             sample.delta_gyro / 0.004f});
        };
        const int frame_imu_rank = imu_rank(frame.imu);
        const int pending_imu_rank = imu_rank(pending.imu);
        if (pending.processed_count == 1 || frame_imu_rank > pending_imu_rank ||
            (frame_imu_rank == pending_imu_rank &&
             imu_short_disturbance_score(frame.imu) > imu_short_disturbance_score(pending.imu))) {
            pending.imu = frame.imu;
        }
        pending.uss = uss_snap;
        if (pending.processed_count >= pending.expected_size) {
            sentry_published_targets = dedupeSentryGroupCandidates(pending.internal_tracks);
            sentry_group_imu = pending.imu;
            sentry_group_uss = pending.uss;
            pending_sentry_groups_.erase(group_id);
            run_sentry_group_update = true;
        }
        if (pending_sentry_groups_.size() > 16) {
            std::cout << "[SentryGroupTrack] pending overflow, clear stale groups size="
                      << pending_sentry_groups_.size() << std::endl;
            pending_sentry_groups_.clear();
        }
    } else {
        pending_sentry_groups_.clear();
    }
    recordPerfMetric("process.sentry_group_track_ms", static_cast<double>(currentSteadyTimeNs() - stage_start_ns) / 1e6);

    // ---- 越野堡垒模式（ORSM）处理 ----
    // 仅当 ORSM 管理器存在时执行；哨兵模式通过全局仲裁被更高优先级模式抑制。
    stage_start_ns = currentSteadyTimeNs();
    if (orsm_manager_ && run_orsm_group_update) {
        OrsmSettings settings_snapshot;
        uint64_t settings_generation = 0;
        {
            std::lock_guard<std::mutex> lk(orsm_settings_mutex_);
            settings_snapshot = orsm_settings_;
            settings_generation = orsm_settings_generation_;
        }
        if (cfg_.orsm_replay_force_entry_odd &&
            settings_snapshot.orsm_setting_st == OrsmSettingType::OFF) {
            settings_snapshot.orsm_setting_st = OrsmSettingType::SINGLE_AUTO_SECURITY_ON;
        }

        OrsmOdd orsm_odd;
        {
            std::lock_guard<std::mutex> lk(vehicle_mutex_);
            // ORSM 的 power_on 单独支持覆盖，不复用 Sentinel 的 power_ok/power_off 覆盖。
            orsm_odd.power_on = cfg_.override_orsm_power_on
                ? cfg_.default_orsm_power_on
                : !latest_vehicle_.power_off.value_or(cfg_.default_power_off);
            orsm_odd.gear_park     = latest_vehicle_.gear_park.value_or(cfg_.default_gear_park);
            orsm_odd.soc           = latest_vehicle_.soc_percent.value_or(100.f);
            orsm_odd.speed_kmh     = latest_vehicle_.speed_kmh.value_or(0.f);
            orsm_odd.soc_activate_threshold = cfg_.soc_min_orsm;
            orsm_odd.soc_deactivate_threshold = cfg_.soc_exit_orsm;
            orsm_odd.speed_exit_threshold_kmh = cfg_.orsm_speed_exit_threshold_kmh;
            orsm_odd.handbrake_on = cfg_.override_handbrake_on
                ? cfg_.default_handbrake_on
                : latest_vehicle_.handbrake_on.value_or(cfg_.default_handbrake_on);
            orsm_odd.had_fault     = false;
            orsm_odd.sensor_fault = cfg_.override_no_sensor_fault
                ? !cfg_.default_no_sensor_fault
                : !latest_vehicle_.no_sensor_fault.value_or(cfg_.default_no_sensor_fault);
            orsm_odd.network_ok    = true;
            orsm_odd.sw_enable     = (settings_snapshot.orsm_setting_st != OrsmSettingType::OFF);
            orsm_odd.sw_mode       = settings_snapshot.orsm_setting_st;
            // 文档约定：队长/队员请求先到 TBOX/云端，HAD 以 TBOX 队伍状态为最终组队依据。
            // MultiVehicleTeamSetting/Req 只作为角色与用户意图记录，不单独判定“组队完成”。
            orsm_odd.multi_vehicle_team_ready = cfg_.orsm_default_multi_vehicle_team_ready ||
                                                settings_snapshot.tbox_team_status_info == TeamStatusEnum::TEAM_UP_COMPLETED;
            orsm_odd.multi_vehicle_team_formed = settings_snapshot.tbox_team_status_info == TeamStatusEnum::TEAM_UP_COMPLETED;
            orsm_odd.vehicle_stationary_over_60s = cfg_.orsm_default_vehicle_stationary_over_60s;
            orsm_odd.external_locked = latest_vehicle_.vehicle_locked.value_or(cfg_.orsm_default_external_locked);
            orsm_odd.ivi_prepared = settings_snapshot.multi_vehicle_team_prepared == MultiVehicleTeamPreparedEnum::PREPARED;
            orsm_odd.tbox_team_status = settings_snapshot.tbox_team_status_info;
        }
        if (cfg_.orsm_replay_force_entry_odd) {
            orsm_odd.power_on = true;
            orsm_odd.gear_park = true;
            orsm_odd.soc = std::max(orsm_odd.soc, orsm_odd.soc_activate_threshold);
            orsm_odd.speed_kmh = 0.f;
            orsm_odd.handbrake_on = true;
            orsm_odd.had_fault = false;
            orsm_odd.sensor_fault = false;
            orsm_odd.network_ok = true;
            orsm_odd.sw_enable = true;
            orsm_odd.vehicle_stationary_over_60s = true;
            orsm_odd.external_locked = true;
        }
        // 本地无相机/雷达数据仍视为传感器故障（在 force_entry 之后，避免被回放 ODD 覆盖）。
        applyLocalSensorDataFault(orsm_odd);

        ImuData imu_snap = imu_ ? imu_->latestData() : ImuData{};
        logOrsmOddIfNeeded(orsm_odd, orsm_manager_->currentState(), orsm_group_ts);
        auto orsm_output = orsm_manager_->update(orsm_global_tracked, imu_snap, uss_snap,
                                                   orsm_group_lidar, settings_snapshot,
                                                   orsm_odd, orsm_group_ts);

        const int orsm_state_value = static_cast<int>(orsm_manager_->currentState());
        const int orsm_warning_value = static_cast<int>(orsm_output.orsm_warning_st);
        const int orsm_warn_level_value = static_cast<int>(orsm_output.orsm_warn_level);
        const int orsm_msg_rem_value = static_cast<int>(orsm_output.orsm_message_rem);
        const size_t orsm_intruder_count = orsm_output.intruders_info.size();
        static int last_orsm_state_value = -1;
        static int last_orsm_warning_value = -1;
        static int last_orsm_warn_level_value = -1;
        static int last_orsm_msg_rem_value = -1;
        const bool orsm_changed =
            orsm_state_value != last_orsm_state_value ||
            orsm_warning_value != last_orsm_warning_value ||
            orsm_warn_level_value != last_orsm_warn_level_value ||
            orsm_msg_rem_value != last_orsm_msg_rem_value;
        if (frame_count_ % 100 == 1 || orsm_changed) {
            std::cout << "[ORSM] state=" << orsm_state_value
                      << " warn_level=" << orsm_warn_level_value
                      << " intruders=" << orsm_intruder_count
                      << " msg_rem=" << orsm_msg_rem_value
                      << " intruder_details=" << summarizeOrsmIntruders(orsm_output.intruders_info)
                      << std::endl;
            last_orsm_state_value = orsm_state_value;
            last_orsm_warning_value = orsm_warning_value;
            last_orsm_warn_level_value = orsm_warn_level_value;
            last_orsm_msg_rem_value = orsm_msg_rem_value;
        }

        // 堡垒模式不触发录像；录像仅用于哨兵模式预警事件。

        // 只合并状态机与感知所有的运行态/预警字段。设置反馈由
        // IVI/TBOX 回调维护，禁止用较早的 settings_snapshot 整帧覆盖。
        {
            // Keep the settings lock until the cache merge is complete. A
            // callback that changes settings increments the generation under
            // this same lock, so a result computed from an older snapshot can
            // never land after the callback's feedback update.
            std::lock_guard<std::mutex> settings_lk(orsm_settings_mutex_);
            if (settings_generation == orsm_settings_generation_) {
                std::lock_guard<std::mutex> output_lk(latest_orsm_output_mutex_);
                applyOrsmRuntimeOutput(latest_orsm_output_, orsm_output);
                latest_orsm_output_ts_ns_ = orsm_group_ts;
            }
        }
    }
    recordPerfMetric("process.orsm_update_publish_ms", static_cast<double>(currentSteadyTimeNs() - stage_start_ns) / 1e6);

    // 更新状态机 ODD 条件（实际项目中由车身信号填充）
    stage_start_ns = currentSteadyTimeNs();
    const bool suppress_sentry = sentrySuppressedByHigherPriorityMode();
    const SentryState state_before_eval = state_machine_->currentState();
    if (run_sentry_group_update) {
        logSentryUssFusionBeforeEvaluate(sentry_published_targets, "sync_group", sentry_group_ts,
                                         state_before_eval, suppress_sentry, uss_cloud);
    }
    if (!suppress_sentry && run_sentry_group_update) {
        // 威胁评估
        alert_manager_->evaluate(sentry_published_targets, sentry_group_imu, sentry_group_uss,
                                  state_before_eval,
                                  sentry_group_ts);
    }

    // 驱动状态机
    ThreatLevel sentry_level = suppress_sentry ? ThreatLevel::NONE : alert_manager_->currentLevel();
    int sentry_threat_id = suppress_sentry ? 0 : alert_manager_->currentThreatId();
    if (suppress_sentry || run_sentry_group_update) {
        state_machine_->update(odd, sentry_level, sentry_threat_id,
                               run_sentry_group_update ? sentry_group_ts : frame.timestamp_ns);
    }

    // 评估先于状态机执行会造成 ACTIVE 严重威胁首帧只能进入 WARNING。
    // 状态变为 WARNING 后立即用同一帧重评一次，配合严重威胁锁存，同帧完成 WARNING→ALARM。
    if (!suppress_sentry && run_sentry_group_update && state_before_eval == SentryState::ACTIVE &&
        state_machine_->currentState() == SentryState::WARNING) {
        logSentryUssFusionBeforeEvaluate(sentry_published_targets, "sync_group", sentry_group_ts,
                                         state_machine_->currentState(), suppress_sentry, uss_cloud);
        alert_manager_->evaluate(sentry_published_targets, sentry_group_imu, sentry_group_uss,
                                  state_machine_->currentState(),
                                  sentry_group_ts);
        sentry_level = alert_manager_->currentLevel();
        sentry_threat_id = alert_manager_->currentThreatId();
        if (sentry_level == ThreatLevel::HIGH) {
            state_machine_->update(odd, sentry_level, sentry_threat_id, sentry_group_ts);
        }
    }
    recordPerfMetric("process.sentry_alert_state_ms", static_cast<double>(currentSteadyTimeNs() - stage_start_ns) / 1e6);

    // 可视化（使用去畸变后的图像）
    stage_start_ns = currentSteadyTimeNs();
    cv::Mat vis = rectified.clone();
    visualize(vis, frame, run_sentry_group_update ? sentry_published_targets : sentry_tracked,
              sentry_level,
              state_machine_->currentState());

    if (cfg_.enable_display) {
        cv::namedWindow("Sentinel", cv::WINDOW_NORMAL);
        cv::resizeWindow("Sentinel", 1920, 1080);
        cv::imshow("Sentinel", vis);
        cv::waitKey(1);
    }

    if (cfg_.enable_save && frame.seq % cfg_.save_interval == 0) {
        if (!fs::exists(cfg_.save_dir)) fs::create_directories(cfg_.save_dir);
        char path[512];
        const std::string cam = frame.camera_name.empty() ? "camera" : frame.camera_name;
        std::snprintf(path, sizeof(path), "%s/%s_seq_%lu%s.jpg",
                      cfg_.save_dir.c_str(), cam.c_str(), frame.seq,
                      frame.imu.vibration_alert ? "_ALERT" : "");
        cv::imwrite(path, vis, {cv::IMWRITE_JPEG_QUALITY, 90});
    }

    // 告警录像已在 onFrame() 收到原始鱼眼帧时写入环形缓存；这里的 vis 仅用于显示/调试截图，
    // 不能作为正式事件录像源，避免保存去畸变/叠加后的非原始视频。
    recordPerfMetric("process.visualize_record_ms", static_cast<double>(currentSteadyTimeNs() - stage_start_ns) / 1e6);

    // ORSM/堡垒模式状态与哨兵模式独立。这里仅更新缓存，状态帧由固定10Hz线程发送，
    // 避免处理帧频率变化导致 /network/fortress_resp 发布周期抖动。

    const double process_total_ms = static_cast<double>(currentSteadyTimeNs() - total_start_ns) / 1e6;
    recordPerfMetric(kAlgorithmProcessTimeMetric, process_total_ms);
    if (cfg_.enable_algorithm_time_log && process_total_ms > kAlgorithmProcessTimeLimitMs) {
        static std::atomic<uint64_t> algo_overrun_log_count{0};
        const uint64_t overrun_idx = ++algo_overrun_log_count;
        if (overrun_idx <= 20 || overrun_idx % 50 == 0) {
            std::cout << std::fixed << std::setprecision(2)
                      << "[算法耗时超限] 阈值=" << kAlgorithmProcessTimeLimitMs << "ms"
                      << " 总耗时=" << process_total_ms << "ms"
                      << " 帧计数=" << frame_count_.load()
                      << " 相机=" << (frame.camera_name.empty() ? "camera" : frame.camera_name)
                      << " seq=" << frame.seq
                      << " ts=" << frame.timestamp_ns
                      << " group_id=" << frame.recording_group_id
                      << " group_size=" << frame.recording_group_size
                      << " 超限累计=" << overrun_idx
                      << std::endl;
        }
    }

    logPerfSummaryIfNeeded(frame_count_.load());
}

void SentinelNode::saveCameraDecodeDebugImage(const char* stage,
                                              const DecodedFrame& frame,
                                              const cv::Mat& image) {
    if (!cfg_.camera_decode_debug_enabled ||
        !cfg_.camera_decode_debug_save_node_images ||
        !stage || !*stage ||
        image.empty()) {
        return;
    }
    if (cfg_.camera_decode_debug_max_frames_per_camera <= 0) {
        return;
    }

    const std::string cam = safeFileToken(frame.camera_name);
    const std::string stage_token = safeFileToken(stage);
    const std::string key = cam + ":" + stage_token;
    uint64_t& saved = camera_decode_debug_node_save_counts_[key];
    if (saved >= static_cast<uint64_t>(cfg_.camera_decode_debug_max_frames_per_camera)) {
        return;
    }
    ++saved;

    const fs::path dir = fs::path(cfg_.camera_decode_debug_dir) / cam;
    std::error_code ec;
    fs::create_directories(dir, ec);
    if (ec) {
        std::cerr << "[SentinelNode][CameraDecodeDebug] create_dir_failed dir="
                  << dir.string() << " error=" << ec.message() << std::endl;
        return;
    }

    std::ostringstream filename;
    filename << std::setw(3) << std::setfill('0') << saved
             << "_seq_" << frame.seq
             << "_ts_" << frame.timestamp_ns
             << "_" << stage_token << ".png";
    const fs::path path = dir / filename.str();
    if (!cv::imwrite(path.string(), image)) {
        std::cerr << "[SentinelNode][CameraDecodeDebug] write_image_failed path="
                  << path.string() << std::endl;
        return;
    }

    std::cout << "[SentinelNode][CameraDecodeDebug] saved stage=" << stage_token
              << " camera=" << cam
              << " index=" << saved
              << " seq=" << frame.seq
              << " image=" << image.cols << "x" << image.rows
              << " path=" << path.string()
              << std::endl;
}

// ---- 车辆状态回调 / ODD 条件构建 ----

void SentinelNode::onVehicleReport(
    const std::shared_ptr<ReceivedMsg<senseAD::msg::vehicle::VehicleReport>>& msg)
{
    if (!msg) return;

    const auto& report = msg->Msg();
    VehicleOddSnapshot snapshot;
    {
        std::lock_guard<std::mutex> lk(vehicle_mutex_);
        snapshot = latest_vehicle_;
        snapshot.sequence = latest_vehicle_.sequence + 1;
    }
    snapshot.timestamp_ns = currentSteadyTimeNs();

    // V2.3.0: fields are nested in sub-messages, not top-level.
    // Speed: Chassis.vehicleMps (m/s) → km/h; fallback DiagSnapshot.vehicleSpeed (uint16 km/h)
    snapshot.speed_kmh = report.getChassis().getVehicleMps() * 3.6f;

    // SOC: DiagSnapshot.socCalculate (uint8, percent)
    const float raw_soc = static_cast<float>(report.getDiagSnapshot().getSocCalculate());
    if (const auto soc = normalizeSocPercent(raw_soc)) {
        snapshot.soc_percent = *soc;
        snapshot.soc_timestamp_ns = snapshot.timestamp_ns;
    }

    // Gear Park: GearReport.actual == GEAR_PARK
    snapshot.gear_park = (report.getGear().getActual() ==
                          senseAD::msg::vehicle::GearCommand::GEAR_PARK);

    // Handbrake / EPB: BrakeReport.parkingBrakeActual
    snapshot.handbrake_on = report.getBrake().getParkingBrakeActual();

    // Power off: VehicleReport 无直接字段；优先保留 chassis_info_m18v2，缺失时回退 false。
    if (!snapshot.power_off) snapshot.power_off = false;

    // no_sensor_fault 仅表示雷达/摄像头故障；VehicleReport 无该字段时默认无故障。
    if (!snapshot.no_sensor_fault) snapshot.no_sensor_fault = true;

    // No occupant: keep config default
    if (!snapshot.no_occupant) snapshot.no_occupant = cfg_.default_no_occupant;
    if (!snapshot.key_far) snapshot.key_far = cfg_.default_key_far;
    if (!snapshot.user_enabled) snapshot.user_enabled = cfg_.user_enabled;
    if (!snapshot.user_shutdown_requested) snapshot.user_shutdown_requested = cfg_.user_shutdown_requested;
    {
        std::lock_guard<std::mutex> lk(vehicle_mutex_);
        latest_vehicle_ = snapshot;
    }

    std::cout << "[SentinelNode][VehicleReport] seq=" << snapshot.sequence
              << " speed=" << (snapshot.speed_kmh ? std::to_string(*snapshot.speed_kmh) : "n/a")
              << " soc=" << (snapshot.soc_percent ? std::to_string(*snapshot.soc_percent) : "n/a")
              << " raw_soc=" << raw_soc
              << " gear_park=" << (snapshot.gear_park ? (*snapshot.gear_park ? "true" : "false") : "n/a")
              << std::endl;
    evaluateSentryRequestTriggers(snapshot.timestamp_ns);
}

#ifndef SENTINEL_DIAGNOSTIC_NO_L3
void SentinelNode::onChassisInfo(
    const std::shared_ptr<ReceivedMsg<senseAD::msg::l3::ChassisInfoM18V2>>& msg)
{
    if (!msg) return;
    const auto& chassis = msg->Msg();
    static std::atomic<uint64_t> chassis_info_log_count{0};
    const uint64_t chassis_info_log_index = ++chassis_info_log_count;
    const bool log_chassis_info =
        chassis_info_log_index <= 5 || chassis_info_log_index % 100 == 0;

    VehicleOddSnapshot snapshot;
    {
        std::lock_guard<std::mutex> lk(vehicle_mutex_);
        snapshot = latest_vehicle_;
        snapshot.sequence = latest_vehicle_.sequence + 1;
    }

    snapshot.timestamp_ns = currentSteadyTimeNs();
    snapshot.chassis_timestamp_ns = snapshot.timestamp_ns;

    // 按《ODD信号源需求清单》接入 /driver/chassis_info_m18v2，报文类型为 ChassisInfoM18V2。
    const float raw_soc = static_cast<float>(chassis.getPdcuBmsSocCalculate());
    if (const auto soc = normalizeSocPercent(raw_soc)) {
        const uint64_t soc_hold_ns =
            static_cast<uint64_t>(std::max(0.f, cfg_.vehicle_report_timeout_sec) * 1e9f);
        const bool has_recent_nonzero_soc =
            snapshot.soc_percent && *snapshot.soc_percent > 0.f &&
            snapshot.soc_timestamp_ns > 0 &&
            snapshot.timestamp_ns >= snapshot.soc_timestamp_ns &&
            snapshot.timestamp_ns - snapshot.soc_timestamp_ns <= soc_hold_ns;
        if (*soc == 0.f && has_recent_nonzero_soc) {
            if (log_chassis_info) {
                std::cout << "[SentinelNode][ChassisInfo] ignore zero chassis soc, keep="
                          << *snapshot.soc_percent << std::endl;
            }
        } else {
            snapshot.soc_percent = *soc;
            snapshot.soc_timestamp_ns = snapshot.timestamp_ns;
        }
    } else if (log_chassis_info) {
        std::cout << "[SentinelNode][ChassisInfo] ignore invalid chassis soc raw="
                  << raw_soc << std::endl;
    }

    // 驾驶座无人：M18 使用 bcm_driver_seat_st。
    const auto driver_seat_empty = readDriverSeatEmpty(chassis);
    if (driver_seat_empty) snapshot.no_occupant = *driver_seat_empty;

    snapshot.vehicle_locked = (chassis.getBcmDoorLockSt() != 0);
    snapshot.power_off = (chassis.getBcmIgnitionSt() == 0);

    // 非 OTA 模式：bcm_usage_mode，0=Normal。
    const auto non_ota_mode = readNonOtaMode(chassis);
    if (non_ota_mode) snapshot.non_ota_mode = *non_ota_mode;

    // no_sensor_fault 仅表示雷达/摄像头故障，不接光照、温度和雨量灯。
    // CAN 侧暂无专用相机/雷达故障位时默认无故障；本地收数由 applyLocalSensorDataFault 判定。
    snapshot.no_sensor_fault = true;

    // 钥匙距离：dkm_ble_key_posn (0=unknown, 1=inside, 2=far>10m)
    // SDK 暴露后自动使用；未暴露则回退配置文件默认值。
    const auto key_far = readKeyFar(chassis);
    snapshot.key_far = key_far.value_or(cfg_.default_key_far);

    {
        std::lock_guard<std::mutex> lk(vehicle_mutex_);
        latest_vehicle_ = snapshot;
    }

    if (log_chassis_info) {
        std::cout << "[SentinelNode][ChassisInfo] seq=" << snapshot.sequence
                  << " soc=" << snapshot.soc_percent.value_or(-1.f)
                  << " raw_soc=" << raw_soc
                  << " no_occupant=" << snapshot.no_occupant.value_or(false)
                  << " locked=" << snapshot.vehicle_locked.value_or(false)
                  << " power_off=" << snapshot.power_off.value_or(false)
                  << " non_ota=" << snapshot.non_ota_mode.value_or(false)
                  << " key_far=" << (snapshot.key_far ? (*snapshot.key_far ? "true" : "false") : "cfg")
                  << std::endl;
    }
    evaluateSentryRequestTriggers(snapshot.timestamp_ns);
}
#endif

#ifndef SENTINEL_DIAGNOSTIC_NO_L3
void SentinelNode::onSentryRequest(
    const std::shared_ptr<ReceivedMsg<senseAD::msg::l3::SentinelRequest>>& msg)
{
    if (!msg) return;
    const auto req = msg->Msg().getSentinelReqMsgType();
    const auto req_value = static_cast<uint16_t>(req);

    OddConditions odd = buildOdd();
    bool approve = sentinelRequestKnown(req_value);

    const uint64_t response_timestamp_ns = static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count());
    const long long request_time_s = static_cast<long long>(std::time(nullptr));

    bool request_latched = false;
    bool run_trigger_eval = false;

    if (req_value == static_cast<uint16_t>(SentinelReqMsgType::ENABLE_SENTINEL_IMMEDIATELY)) {
        // “立即开启”先校验除下电/锁车/无人外的 ODD；实际进入等待用户下电锁车后再做完整 ODD。
        OddConditions immediate_odd = oddIgnoringDeferredEntryConditions(odd);
        approve = immediate_odd.oddStatus() == SentinelODDStatus::SATISFY_ALL_ODD;
        if (!approve) odd = immediate_odd;
    } else if (req_value == static_cast<uint16_t>(SentinelReqMsgType::ENABLE_SENTINEL_AUTOMATICALLY) ||
               req_value == static_cast<uint16_t>(SentinelReqMsgType::ENABLE_SENTINEL_SCHEDULED_START)) {
        // 自动/预约开启是未来触发行为，收到请求即确认；触发时再核查完整 ODD 并按需发送 decline。
        approve = true;
    } else if (sentinelRequestNeedsOddApproval(req_value)) {
        approve = approve && odd.oddStatus() == SentinelODDStatus::SATISFY_ALL_ODD;
    }

    {
        std::lock_guard<std::mutex> lk(sentry_request_mutex_);

        switch (req_value) {
        case 1:  // ENABLE_SENTINEL_IMMEDIATELY
            if (approve) {
                sentry_request_control_.immediate_enable_pending_once = true;
                sentry_request_control_.sentry_enabled_by_scheduled_request = false;
                sentry_request_control_.sentry_enabled_by_automatic_request = false;
                request_latched = true;
                run_trigger_eval = true;
            }
            break;
        case 3:  // ENABLE_SENTINEL_AUTOMATICALLY
            if (approve) {
                sentry_request_control_.automatic_enable_enabled = true;
                sentry_request_control_.automatic_enable_pending_once = true;
                sentry_request_control_.sentry_enabled_by_scheduled_request = false;
                request_latched = true;
                run_trigger_eval = true;
            }
            break;
        case 5: {  // ENABLE_SENTINEL_SCHEDULED_START
            if (approve) {
                const auto sched = msg->Msg().getSentinelScheduledStartTime();
                const auto day = static_cast<uint8_t>(sched.getSentinelScheduledStartDday());
                const auto hour = sched.getSentinelScheduledStartHour();
                const auto minute = sched.getSentinelScheduledStartMinute();
                if (scheduledStartIsFuture(day, hour, minute, request_time_s)) {
                    sentry_request_control_.scheduled_start_day = day;
                    sentry_request_control_.scheduled_start_hour = hour;
                    sentry_request_control_.scheduled_start_minute = minute;
                    sentry_request_control_.scheduled_request_time_s = request_time_s;
                    sentry_request_control_.scheduled_enable_pending = true;
                    sentry_request_control_.sentry_enabled_by_automatic_request = false;
                    request_latched = true;
                    run_trigger_eval = true;
                } else {
                    approve = false;
                    sentry_request_control_.scheduled_enable_pending = false;
                    std::cout << "[SentinelNode][SentryRequest] scheduled start rejected: target is not in future"
                              << " day=" << static_cast<int>(day)
                              << " hour=" << hour
                              << " minute=" << minute
                              << std::endl;
                }
            }
            break;
        }
        case 2:  // DISABLE_SENTINEL_IMMEDIATELY：关闭当前激活窗口，不影响自动/预约配置。
            sentry_request_control_.sentry_enabled_by_request = false;
            sentry_request_control_.sentry_enabled_by_scheduled_request = false;
            sentry_request_control_.sentry_enabled_by_automatic_request = false;
            sentry_request_control_.immediate_enable_pending_once = false;
            request_latched = true;
            run_trigger_eval = true;
            break;
        case 4:  // DISABLE_SENTINEL_AUTOMATICALLY
            sentry_request_control_.automatic_enable_enabled = false;
            sentry_request_control_.automatic_enable_pending_once = false;
            if (sentry_request_control_.sentry_enabled_by_automatic_request) {
                sentry_request_control_.sentry_enabled_by_request = false;
                sentry_request_control_.sentry_enabled_by_automatic_request = false;
                sentry_request_control_.sentry_enabled_by_scheduled_request = false;
                run_trigger_eval = true;
            }
            request_latched = true;
            break;
        case 6:  // DISABLE_SENTINEL_SCHEDULED_START
            sentry_request_control_.scheduled_enable_pending = false;
            request_latched = true;
            break;
        default:
            break;
        }
    }

    std::cout << "[SentinelNode][SentryRequest] type=" << req_value
              << " approve=" << approve
              << " latched=" << request_latched
              << std::endl;

    publishSentryRequestResponse(req_value, response_timestamp_ns, approve, odd);
    if (run_trigger_eval) {
        evaluateSentryRequestTriggers(response_timestamp_ns);
    }
}
#endif

void SentinelNode::publishSentryRequestResponse(uint16_t req_value, uint64_t timestamp_ns,
                                                bool approve, const OddConditions& odd,
                                                std::optional<SentinelEnableFailed> forced_failed) {
    if (!rscl_publisher_) return;

    SentinelResStatusView res;
    res.sentinel_res_msg_type = sentinelResponseTypeForRequest(req_value, approve);
    res.sentinel_odd_status = odd.oddStatus();
    res.sentinel_enable_failed = forced_failed.value_or(
        approve ? SentinelEnableFailed::SENTINEL_ENABLE_FAILED_UNKNOWN
                : odd.enableFailedReason());
    res.sentinel_general_threat = SentinelGeneralThreat::OFF;
    res.sentinel_severe_threat = SentinelGeneralThreat::OFF;
    res.sentinel_audio_visual_alarm = SentinelGeneralThreat::OFF;
    res.sentinel_obstacle_type = SentinelObstacleType::UNKNOWN_OBSTACLE;

    AlertEvent evt;
    evt.timestamp_ns = timestamp_ns > 0 ? timestamp_ns : currentSteadyTimeNs();
    rscl_publisher_->publishSentryEvent(evt, res, 0);

    {
        std::lock_guard<std::mutex> lk(latest_sentry_status_mutex_);
        latest_sentry_status_event_ = evt;
        latest_sentry_status_res_ = res;
    }

    std::cout << "[SentinelNode][SentryRequestResponse] topic=" << cfg_.sentry_event_topic
              << " req=" << req_value
              << " approve=" << approve
              << " res=" << static_cast<int>(res.sentinel_res_msg_type)
              << " odd=" << static_cast<int>(res.sentinel_odd_status)
              << " failed=" << static_cast<int>(res.sentinel_enable_failed)
              << std::endl;
}

#ifndef SENTINEL_DIAGNOSTIC_NO_L3
void SentinelNode::onOrsmSettingRequest(
    const std::shared_ptr<ReceivedMsg<senseAD::msg::l3::SetOffRoadMdInfoReq>>& msg)
{
    if (!msg) return;
    const uint64_t recv_steady_ms = currentSteadyTimeMs();
    const uint64_t recv_system_ms = currentSystemTimeMs();
    const auto& req = msg->Msg();
    const auto requested_mode =
        static_cast<OrsmSettingType>(static_cast<uint16_t>(req.getOrsmSettingSt()));
    {
        std::lock_guard<std::mutex> lk(orsm_settings_mutex_);
        orsm_settings_.orsm_setting_st = requested_mode;
        const auto humans_recog_setting =
            static_cast<BasicSwitchReq>(static_cast<uint16_t>(req.getHumansRecogSetting()));
        const auto animals_recog_setting =
            static_cast<BasicSwitchReq>(static_cast<uint16_t>(req.getAnimalsRecogSetting()));
        if (humans_recog_setting != BasicSwitchReq::NONE) {
            orsm_settings_.humans_recog_setting = humans_recog_setting;
        }
        if (animals_recog_setting != BasicSwitchReq::NONE) {
            orsm_settings_.animals_recog_setting = animals_recog_setting;
        }
        const auto requested_range =
            static_cast<DetecRangeSetting>(static_cast<uint16_t>(req.getDetecRangeSetting()));
        if (requested_range != DetecRangeSetting::NONE_RANGE) {
            orsm_settings_.detec_range_setting = requested_range;
        }
        const auto alarm_light_setting =
            static_cast<BasicSwitchReq>(static_cast<uint16_t>(req.getAlarmLightSetting()));
        const auto alarm_horn_setting =
            static_cast<BasicSwitchReq>(static_cast<uint16_t>(req.getAlarmHornSetting()));
        if (alarm_light_setting != BasicSwitchReq::NONE) {
            orsm_settings_.alarm_light_setting = alarm_light_setting;
        }
        if (alarm_horn_setting != BasicSwitchReq::NONE) {
            orsm_settings_.alarm_horn_setting = alarm_horn_setting;
        }
        orsm_settings_.alarm_sound_setting = static_cast<AlarmSoundType>(static_cast<uint16_t>(req.getAlarmSoundSetting()));
        orsm_settings_.alarm_snd_dura_setting = static_cast<AlarmSoundDuration>(static_cast<uint16_t>(req.getAlarmSndDuraSetting()));
        orsm_settings_.multi_vehicle_team_setting = static_cast<MultiVehicleTeamSettingEnum>(static_cast<uint16_t>(req.getMultiVehicleTeamSetting()));
        orsm_settings_.multi_vehicle_team_req = static_cast<MultiVehicleTeamReqEnum>(static_cast<uint16_t>(req.getMultiVehicleTeamReq()));
        orsm_settings_.multi_vehicle_team_prepared = static_cast<MultiVehicleTeamPreparedEnum>(static_cast<uint16_t>(req.getMultiVehicleTeamPrepared()));
        if (orsm_settings_.multi_vehicle_team_setting != MultiVehicleTeamSettingEnum::TEAM_NO_REQUEST) {
            // 设计文档：MultiVehicleTeamSetting 为队长车辆使用，IVI→TBOX。
            orsm_settings_.multi_vehicle_role = MultiVehicleRole::LEADER;
        } else if (orsm_settings_.multi_vehicle_team_req != MultiVehicleTeamReqEnum::TEAM_REQ_NO_REQUEST) {
            // 设计文档：MultiVehicleTeamReq 为成员车辆使用，IVI→TBOX。
            orsm_settings_.multi_vehicle_role = MultiVehicleRole::MEMBER;
        }
        ++orsm_settings_generation_;
    }
    std::cout << "[SentinelNode][ORSMSettingReq] recv_steady_ms=" << recv_steady_ms
              << " recv_system_ms=" << recv_system_ms
              << " mode=" << static_cast<uint16_t>(req.getOrsmSettingSt())
              << " humans=" << static_cast<uint16_t>(req.getHumansRecogSetting())
              << " animals=" << static_cast<uint16_t>(req.getAnimalsRecogSetting())
              << " team_setting=" << static_cast<uint16_t>(req.getMultiVehicleTeamSetting())
              << " team_req=" << static_cast<uint16_t>(req.getMultiVehicleTeamReq())
              << " prepared=" << static_cast<uint16_t>(req.getMultiVehicleTeamPrepared())
              << std::endl;
    // A close request must take effect immediately even if the matching TBOX
    // message is delayed or lost. TBOX OFF remains idempotent.
    if (requested_mode == OrsmSettingType::OFF) {
        forceOrsmOffFeedback(currentSteadyTimeNs(), "orsm_setting_req_off");
    } else {
        refreshCachedOrsmSettingsFeedback();
    }
}

void SentinelNode::onOrsmSetInfo(
    const std::shared_ptr<ReceivedMsg<senseAD::msg::l3::OffRoadSetInfoStruct>>& msg)
{
    if (!msg) return;
    const uint64_t recv_steady_ms = currentSteadyTimeMs();
    const uint64_t recv_system_ms = currentSystemTimeMs();
    const auto& info = msg->Msg();
    const auto requested_mode =
        static_cast<OrsmSettingType>(static_cast<uint16_t>(info.getOrsmSettingSt()));
    {
        std::lock_guard<std::mutex> lk(orsm_settings_mutex_);
        orsm_settings_.orsm_setting_st = requested_mode;
        const auto humans_recog_setting =
            static_cast<BasicSwitchReq>(static_cast<uint16_t>(info.getHumansRecogSetting()));
        const auto animals_recog_setting =
            static_cast<BasicSwitchReq>(static_cast<uint16_t>(info.getAnimalsRecogSetting()));
        if (humans_recog_setting != BasicSwitchReq::NONE) {
            orsm_settings_.humans_recog_setting = humans_recog_setting;
        }
        if (animals_recog_setting != BasicSwitchReq::NONE) {
            orsm_settings_.animals_recog_setting = animals_recog_setting;
        }
        const auto requested_range =
            static_cast<DetecRangeSetting>(static_cast<uint16_t>(info.getDetecRangeSetting()));
        if (requested_range != DetecRangeSetting::NONE_RANGE) {
            orsm_settings_.detec_range_setting = requested_range;
        }
        const auto alarm_light_setting =
            static_cast<BasicSwitchReq>(static_cast<uint16_t>(info.getAlarmLightSetting()));
        const auto alarm_horn_setting =
            static_cast<BasicSwitchReq>(static_cast<uint16_t>(info.getAlarmHornSetting()));
        if (alarm_light_setting != BasicSwitchReq::NONE) {
            orsm_settings_.alarm_light_setting = alarm_light_setting;
        }
        if (alarm_horn_setting != BasicSwitchReq::NONE) {
            orsm_settings_.alarm_horn_setting = alarm_horn_setting;
        }
        orsm_settings_.alarm_sound_setting = static_cast<AlarmSoundType>(static_cast<uint16_t>(info.getAlarmSoundSetting()));
        orsm_settings_.alarm_snd_dura_setting = static_cast<AlarmSoundDuration>(static_cast<uint16_t>(info.getAlarmSndDuraSetting()));
        orsm_settings_.tbox_team_status_info = static_cast<TeamStatusEnum>(static_cast<uint16_t>(info.getTboxTeamStatusInfo()));

        // IVI→TBOX→HAD 多车联防组队设置。
        // multi_vehicle_team_setting 与 tbox_team_status_info 均通过 TBOX 主题下发；
        // 旧 SDK 未暴露该字段时保留 IVI 请求侧/默认设置。
        if (const auto team_setting = readMultiVehicleTeamSetting(info)) {
            orsm_settings_.multi_vehicle_team_setting = *team_setting;
        }
        ++orsm_settings_generation_;
    }
    std::cout << "[SentinelNode][ORSMSetInfo] recv_steady_ms=" << recv_steady_ms
              << " recv_system_ms=" << recv_system_ms
              << " mode=" << static_cast<uint16_t>(info.getOrsmSettingSt())
              << " humans=" << static_cast<uint16_t>(info.getHumansRecogSetting())
              << " animals=" << static_cast<uint16_t>(info.getAnimalsRecogSetting())
              << " tbox_team_status=" << static_cast<uint16_t>(info.getTboxTeamStatusInfo())
              << std::endl;
    // TBOX is the source of immediate mode feedback. Repeated OFF messages are
    // intentionally not edge-filtered: every TBOX OFF refreshes and publishes
    // a coherent closed-state snapshot.
    if (requested_mode == OrsmSettingType::OFF) {
        forceOrsmOffFeedback(currentSteadyTimeNs(), "orsm_set_info_off");
    } else {
        refreshCachedOrsmSettingsFeedback();
        publishOrsmStatusTick();
    }
}
#endif

bool SentinelNode::localSensorDataMissing() const {
    const int timeout_ms = cfg_.orsm_sensor_data_timeout_ms;
    if (timeout_ms <= 0) return false;

    const bool need_camera = !cfg_.camera_topics.empty();
    const bool need_lidar = !cfg_.lidar_topics.empty();
    if (!need_camera && !need_lidar) return false;

    const uint64_t timeout_ns = static_cast<uint64_t>(timeout_ms) * 1000000ULL;
    const uint64_t now_ns = currentSteadyTimeNs();
    auto stale = [&](uint64_t last_ns) {
        if (last_ns == 0) return true;
        return now_ns > last_ns && (now_ns - last_ns) > timeout_ns;
    };

    if (need_camera &&
        stale(camera_last_receive_steady_ns_.load(std::memory_order_relaxed))) {
        return true;
    }
    if (need_lidar &&
        stale(lidar_last_receive_steady_ns_.load(std::memory_order_relaxed))) {
        return true;
    }
    return false;
}

void SentinelNode::applyLocalSensorDataFault(OrsmOdd& odd) const {
    if (localSensorDataMissing()) {
        odd.sensor_fault = true;
    }
}

void SentinelNode::refreshCachedOrsmSettingsFeedback(uint64_t timestamp_ns) {
    OrsmSettings settings_snapshot;
    {
        std::lock_guard<std::mutex> lk(orsm_settings_mutex_);
        settings_snapshot = orsm_settings_;
    }

    // 开启请求的快速反馈只做车辆/TBOX ODD 预检查，不运行完整的目标关联和
    // 入侵物计算；完整 ORSM 感知更新仍由正常感知流程执行。
    OrsmOdd odd;
    {
        std::lock_guard<std::mutex> lk(vehicle_mutex_);
        odd.power_on = cfg_.override_orsm_power_on
            ? cfg_.default_orsm_power_on
            : !latest_vehicle_.power_off.value_or(cfg_.default_power_off);
        odd.gear_park = latest_vehicle_.gear_park.value_or(cfg_.default_gear_park);
        odd.soc = latest_vehicle_.soc_percent.value_or(100.f);
        odd.speed_kmh = latest_vehicle_.speed_kmh.value_or(0.f);
        odd.soc_activate_threshold = cfg_.soc_min_orsm;
        odd.soc_deactivate_threshold = cfg_.soc_exit_orsm;
        odd.speed_exit_threshold_kmh = cfg_.orsm_speed_exit_threshold_kmh;
        odd.handbrake_on = cfg_.override_handbrake_on
            ? cfg_.default_handbrake_on
            : latest_vehicle_.handbrake_on.value_or(cfg_.default_handbrake_on);
        odd.had_fault = false;
        odd.sensor_fault = cfg_.override_no_sensor_fault
            ? !cfg_.default_no_sensor_fault
            : !latest_vehicle_.no_sensor_fault.value_or(cfg_.default_no_sensor_fault);
        odd.network_ok = true;
        odd.sw_enable = settings_snapshot.orsm_setting_st != OrsmSettingType::OFF;
        odd.sw_mode = settings_snapshot.orsm_setting_st;
        odd.multi_vehicle_team_ready =
            cfg_.orsm_default_multi_vehicle_team_ready ||
            settings_snapshot.tbox_team_status_info == TeamStatusEnum::TEAM_UP_COMPLETED;
        odd.multi_vehicle_team_formed =
            settings_snapshot.tbox_team_status_info == TeamStatusEnum::TEAM_UP_COMPLETED;
        odd.vehicle_stationary_over_60s = cfg_.orsm_default_vehicle_stationary_over_60s;
        odd.external_locked = latest_vehicle_.vehicle_locked.value_or(cfg_.orsm_default_external_locked);
        odd.ivi_prepared =
            settings_snapshot.multi_vehicle_team_prepared == MultiVehicleTeamPreparedEnum::PREPARED;
        odd.tbox_team_status = settings_snapshot.tbox_team_status_info;
    }

    if (cfg_.orsm_replay_force_entry_odd) {
        odd.power_on = true;
        odd.gear_park = true;
        odd.soc = std::max(odd.soc, odd.soc_activate_threshold);
        odd.speed_kmh = 0.f;
        odd.handbrake_on = true;
        odd.had_fault = false;
        odd.sensor_fault = false;
        odd.network_ok = true;
        odd.sw_enable = true;
        odd.vehicle_stationary_over_60s = true;
        odd.external_locked = true;
    }
    // 本地无相机/雷达数据仍视为传感器故障（在 force_entry 之后，避免被回放 ODD 覆盖）。
    applyLocalSensorDataFault(odd);

    bool enable_ready = false;
    OrsmMessageReminder enable_failure = OrsmMessageReminder::ORSM_NONE;
    const bool requested_enable =
        odd.sw_enable && orsmEnabled(settings_snapshot.orsm_setting_st);
    if (requested_enable &&
        settings_snapshot.orsm_setting_st == OrsmSettingType::SINGLE_AUTO_SECURITY_ON) {
        enable_ready = odd.power_on &&
                       odd.gear_park &&
                       odd.soc >= odd.soc_activate_threshold &&
                       odd.handbrake_on &&
                       !odd.had_fault &&
                       !odd.sensor_fault;
        if (!enable_ready) {
            if (!odd.power_on) {
                enable_failure = OrsmMessageReminder::NOT_IN_ON_READY_STATE_FUNCTION_ENTRY_FAILED;
            } else if (!odd.gear_park) {
                enable_failure = OrsmMessageReminder::GEAR_NOT_IN_P_POSITION_FUNCTION_ENTRY_FAILED;
            } else if (odd.soc < odd.soc_activate_threshold) {
                enable_failure = OrsmMessageReminder::LOW_BATTERY_FUNCTION_ENTRY_FAILED;
            } else if (odd.speed_kmh > odd.speed_exit_threshold_kmh) {
                enable_failure = OrsmMessageReminder::VEHICLE_SPEED_TOO_HIGH_FUNCTION_CANNOT_BE_ACTIVATED;
            } else if (odd.had_fault) {
                enable_failure = OrsmMessageReminder::INTELLIGENT_DRIVING_SYSTEM_MALFUNCTION_FUNCTION_ENTRY_FAILED;
            } else if (odd.sensor_fault) {
                enable_failure = OrsmMessageReminder::RADAR_CAMERA_MALFUNCTION_FUNCTION_ENTRY_FAILED;
            }
        }
    } else if (requested_enable &&
               settings_snapshot.orsm_setting_st == OrsmSettingType::MULTI_VEHICLE_JOINT_SECURITY_ON) {
        const bool team_formed = odd.multi_vehicle_team_formed ||
            settings_snapshot.tbox_team_status_info == TeamStatusEnum::TEAM_UP_COMPLETED ||
            odd.tbox_team_status == TeamStatusEnum::TEAM_UP_COMPLETED;
        const bool ivi_prepared = odd.ivi_prepared ||
            settings_snapshot.multi_vehicle_team_prepared == MultiVehicleTeamPreparedEnum::PREPARED;
        const bool base_ready = odd.power_on &&
                                odd.gear_park &&
                                odd.soc >= settings_snapshot.multi_vehicle_soc_entry_min &&
                                odd.speed_kmh <= settings_snapshot.multi_vehicle_speed_entry_max_kmh &&
                                !odd.had_fault &&
                                !odd.sensor_fault &&
                                (!settings_snapshot.multi_vehicle_enable_network_check || odd.network_ok);
        enable_ready = base_ready &&
                       team_formed &&
                       ivi_prepared &&
                       odd.multi_vehicle_team_ready &&
                       odd.vehicle_stationary_over_60s &&
                       odd.external_locked;
        if (!enable_ready) {
            if (!odd.power_on) {
                enable_failure = OrsmMessageReminder::NOT_IN_ON_READY_STATE_FUNCTION_ENTRY_FAILED;
            } else if (!odd.gear_park) {
                enable_failure = OrsmMessageReminder::GEAR_NOT_IN_P_POSITION_FUNCTION_ENTRY_FAILED;
            } else if (odd.had_fault) {
                enable_failure = OrsmMessageReminder::INTELLIGENT_DRIVING_SYSTEM_MALFUNCTION_FUNCTION_ENTRY_FAILED;
            } else if (odd.sensor_fault) {
                enable_failure = OrsmMessageReminder::RADAR_CAMERA_MALFUNCTION_FUNCTION_ENTRY_FAILED;
            } else if (settings_snapshot.multi_vehicle_enable_network_check && !odd.network_ok) {
                enable_failure = OrsmMessageReminder::NETWORK_CONNECTION_DISCONNECTED_JOINT_SECURITY_FUNCTION_EXITED;
            } else if (odd.speed_kmh > settings_snapshot.multi_vehicle_speed_entry_max_kmh) {
                enable_failure = OrsmMessageReminder::VEHICLE_SPEED_TOO_HIGH_FUNCTION_CANNOT_BE_ACTIVATED;
            } else if (odd.soc < settings_snapshot.multi_vehicle_soc_entry_min) {
                enable_failure = OrsmMessageReminder::LOW_BATTERY_FUNCTION_ENTRY_FAILED;
            } else {
                enable_failure = OrsmMessageReminder::MULTI_VEHICLE_JOINT_SECURITY_FUNCTION_EXITED;
            }
        }
    }

    const OrsmState state = orsm_manager_ ? orsm_manager_->currentState() : OrsmState::OFF;
    const bool manager_off = state == OrsmState::OFF;
    uint64_t ts_ns = timestamp_ns > 0
        ? timestamp_ns
        : last_sensor_time_ns_.load(std::memory_order_relaxed);

    std::lock_guard<std::mutex> lk(latest_orsm_output_mutex_);
    if (ts_ns == 0) {
        ts_ns = latest_orsm_output_ts_ns_;
    }
    latest_orsm_output_.timestamp_ns = ts_ns;
    applyOrsmSettingsFeedback(latest_orsm_output_, settings_snapshot);

    if (requested_enable) {
        latest_orsm_output_.orsm_setting_st_fb =
            enable_ready ? settings_snapshot.orsm_setting_st
                         : OrsmSettingType::FUNCTION_LIMITED_ENABLE_FAILED;
        // The running manager owns the success reminder (SECURITY_MODE_ON).
        // Repeated IVI/TBOX setting frames must not overwrite it with ORSM_NONE.
        applyOrsmEnableMessageFeedback(latest_orsm_output_, enable_ready, enable_failure);
    } else if (manager_off) {
        latest_orsm_output_.orsm_setting_st_fb =
            settings_snapshot.orsm_setting_st == OrsmSettingType::OFF
                ? OrsmSettingType::OFF
                : OrsmSettingType::FUNCTION_LIMITED_ENABLE_FAILED;
        latest_orsm_output_.orsm_warning_st = OrsmWarningStatus::WARNING_OFF;
        latest_orsm_output_.intruders_num = IntrudersNumType::NO_INTRUDER;
        latest_orsm_output_.orsm_warn_level = OrsmWarnLevel::LEVEL_NONE;
        latest_orsm_output_.intruders_info.clear();
    }
    latest_orsm_output_ts_ns_ = ts_ns;
}

void SentinelNode::forceOrsmOffFeedback(uint64_t timestamp_ns, const char* reason) {
    const uint64_t force_steady_ms = currentSteadyTimeMs();
    const uint64_t force_system_ms = currentSystemTimeMs();
    if (orsm_manager_) {
        orsm_manager_->resetTracking();
    }
    clearOrsmStaticFootprints(reason ? reason : "fortress_force_off");

    OrsmSettings settings_snapshot;
    {
        std::lock_guard<std::mutex> lk(orsm_settings_mutex_);
        settings_snapshot = orsm_settings_;
    }

    ORSMAlertOutput off_output;
    off_output.timestamp_ns = timestamp_ns > 0 ? timestamp_ns : currentSteadyTimeNs();
    // 关闭只清理运行时报警和目标状态；用户设置反馈沿用最后一次有效配置。
    applyOrsmSettingsFeedback(off_output, settings_snapshot);
    off_output.orsm_setting_st_fb = OrsmSettingType::OFF;
    off_output.orsm_warning_st = OrsmWarningStatus::WARNING_OFF;
    off_output.intruders_num = IntrudersNumType::NO_INTRUDER;
    off_output.orsm_warn_level = OrsmWarnLevel::LEVEL_NONE;
    off_output.orsm_message_rem = OrsmMessageReminder::ORSM_NONE;
    off_output.intruders_info.clear();
    {
        std::lock_guard<std::mutex> lk(latest_orsm_output_mutex_);
        latest_orsm_output_ = off_output;
        latest_orsm_output_ts_ns_ = off_output.timestamp_ns;
    }

    std::cout << "[SentinelNode][ORSMForceOff]"
              << " force_steady_ms=" << force_steady_ms
              << " force_system_ms=" << force_system_ms
              << " reason=" << (reason ? reason : "unknown")
              << " ts=" << off_output.timestamp_ns
              << std::endl;
    publishOrsmStatusTick(off_output.timestamp_ns);
}

bool SentinelNode::fortressModeRequested() const {
    std::lock_guard<std::mutex> lk(orsm_settings_mutex_);
    return orsm_settings_.orsm_setting_st == OrsmSettingType::SINGLE_AUTO_SECURITY_ON ||
           orsm_settings_.orsm_setting_st == OrsmSettingType::MULTI_VEHICLE_JOINT_SECURITY_ON;
}

bool SentinelNode::fortressModeActiveForProcessing() const {
    return cfg_.orsm_replay_force_entry_odd || fortressModeRequested();
}

bool SentinelNode::sentrySuppressedByHigherPriorityMode() const {
    // 当前设计范围仅三种模式：哨兵、单车堡垒、多车堡垒。
    // 只要任一堡垒模式被请求，哨兵模式就不再激活；单车/多车由 OrsmManager 内部路由。
    return fortressModeActiveForProcessing();
}

bool SentinelNode::shouldRunHeavyPerception(const OddConditions& odd) const {
    if (fortressModeActiveForProcessing()) return true;
    if (!state_machine_) return odd.userWantsSentryOn() && odd.oddStatus() == SentinelODDStatus::SATISFY_ALL_ODD;

    const SentryState state = state_machine_->currentState();
    if (state == SentryState::ACTIVE ||
        state == SentryState::WARNING ||
        state == SentryState::ALARM) {
        return true;
    }

    return odd.userWantsSentryOn() && odd.oddStatus() == SentinelODDStatus::SATISFY_ALL_ODD;
}

void SentinelNode::resetOrsmGlobalTracks() {
    orsm_global_tracks_.clear();
    next_orsm_global_track_id_ = 1;
    orsm_lidar_temporal_tracks_.clear();
    next_orsm_lidar_temporal_track_id_ = 1;
    clearOrsmStaticFootprints("orsm_track_reset");
}

void SentinelNode::clearOrsmStaticFootprints(const char* reason) {
    size_t cleared = 0;
    {
        std::lock_guard<std::mutex> lk(orsm_static_footprints_mutex_);
        cleared = orsm_static_footprints_.size();
        orsm_static_footprints_.clear();
    }
    if (cleared > 0) {
        std::cout << "[ORSMStaticFootprintClear] count=" << cleared
                  << " reason=" << (reason ? reason : "unknown")
                  << std::endl;
    }
}

void SentinelNode::associateOrsmLidarTemporalTracks(
    std::vector<TrackedObject>& candidates, uint64_t timestamp_ns) {
    if (timestamp_ns == 0 || candidates.empty()) return;

    // 关联门限允许聚类中心小幅抖动，同时阻止相邻静态片段接管原轨迹。
    constexpr float kBaseGateM = 1.35f;
    constexpr float kMaxSpeedMps = 3.0f;
    constexpr uint64_t kPruneNs = 3000ULL * 1000000ULL;
    constexpr float kMinStaticFootprintRadiusM = 0.20f;
    constexpr float kStaticFootprintOverlapRatio = 0.30f;
    std::unordered_set<int> claimed;

    // lidar-only 运动历史供裸簇准入和发布判定共用。
    const uint64_t output_motion_window_ns =
        static_cast<uint64_t>(std::max(1000, cfg_.orsm_lidar_only_output_motion_window_ms)) *
        1000000ULL;
    auto recordOutputMotion = [&](OrsmLidarTemporalTrack& track) {
        auto& history = track.output_motion_history;
        // 多路雷达时间戳可能乱序，删除不早于当前观测的样本以保持历史有序。
        while (!history.empty() && history.back().first >= timestamp_ns) {
            history.pop_back();
        }
        while (!history.empty() &&
               timestamp_ns > history.front().first &&
               timestamp_ns - history.front().first > output_motion_window_ns) {
            history.pop_front();
        }
        history.emplace_back(timestamp_ns, track.position);
    };

    auto rememberStaticFootprint = [&](const OrsmLidarTemporalTrack& track,
                                       const TrackedObject& candidate) {
        const float cluster_radius = std::max(
            kMinStaticFootprintRadiusM, candidate.lidar_cluster_radius_m);
        // 静态区域以 L# 初始位置为中心，并计入观测到的中心抖动。
        const cv::Point2f center(track.initial_position.x, track.initial_position.y);
        const float occupied_radius = cluster_radius + track.max_displacement_m;
        bool inserted = false;
        float stored_radius = occupied_radius;
        {
            std::lock_guard<std::mutex> lk(orsm_static_footprints_mutex_);
            const auto existing = std::find_if(
                orsm_static_footprints_.begin(), orsm_static_footprints_.end(),
                [&](const OrsmStaticFootprint& footprint) {
                    return footprint.source_lidar_temporal_id == track.stable_id;
                });
            if (existing == orsm_static_footprints_.end()) {
                orsm_static_footprints_.push_back(OrsmStaticFootprint{
                    track.stable_id, center, occupied_radius, timestamp_ns, timestamp_ns});
                inserted = true;
            } else {
                existing->center = center;
                existing->radius_m = std::max(existing->radius_m, occupied_radius);
                existing->last_seen_ts_ns = timestamp_ns;
                stored_radius = existing->radius_m;
            }
        }
        if (inserted) {
            std::cout << "[ORSMStaticFootprintAdd] ts=" << timestamp_ns
                      << " L=" << track.stable_id
                      << " center=(" << center.x << ',' << center.y << ')'
                      << " cluster_radius=" << cluster_radius
                      << " displacement=" << track.max_displacement_m
                      << " footprint_radius=" << stored_radius
                      << std::endl;
        }
    };

    auto findStaticFootprintOverlap = [&](const TrackedObject& candidate,
                                          int& source_lidar_id,
                                          float& overlap_ratio,
                                          float& center_distance,
                                          float& footprint_radius) {
        const cv::Point2f center(candidate.position_3d.x, candidate.position_3d.y);
        const float candidate_radius = std::max(
            kMinStaticFootprintRadiusM, candidate.lidar_cluster_radius_m);
        source_lidar_id = 0;
        overlap_ratio = 0.f;
        center_distance = 0.f;
        footprint_radius = 0.f;
        std::lock_guard<std::mutex> lk(orsm_static_footprints_mutex_);
        for (const auto& footprint : orsm_static_footprints_) {
            const float ratio = circleOverlapRatioOfFirst(
                center, candidate_radius, footprint.center, footprint.radius_m);
            if (ratio <= overlap_ratio) continue;
            overlap_ratio = ratio;
            source_lidar_id = footprint.source_lidar_temporal_id;
            center_distance = std::hypot(
                center.x - footprint.center.x, center.y - footprint.center.y);
            footprint_radius = footprint.radius_m;
        }
        return overlap_ratio >= kStaticFootprintOverlapRatio;
    };

    for (auto& candidate : candidates) {
        if (!candidate.has_lidar_fusion || !finitePoint2D(candidate.position_3d)) continue;
        int best_id = 0;
        float best_distance = std::numeric_limits<float>::max();
        for (const auto& [id, track] : orsm_lidar_temporal_tracks_) {
            if (claimed.count(id) || timestamp_ns < track.last_seen_ts_ns) continue;
            const float dt = static_cast<float>(timestamp_ns - track.last_seen_ts_ns) * 1.e-9f;
            if (dt > 3.f) continue;
            cv::Point3f predicted = track.position;
            predicted.x += track.velocity.x * std::min(dt, 1.f);
            predicted.y += track.velocity.y * std::min(dt, 1.f);
            const float gate = std::min(1.8f, kBaseGateM + kMaxSpeedMps * dt);
            const float distance = pointDistance2D(candidate.position_3d, predicted);
            if (distance <= gate && distance < best_distance) {
                best_id = id;
                best_distance = distance;
            }
        }
        if (best_id == 0) {
            OrsmLidarTemporalTrack track;
            track.stable_id = next_orsm_lidar_temporal_track_id_++;
            track.position = candidate.position_3d;
            track.initial_position = candidate.position_3d;
            track.first_seen_ts_ns = timestamp_ns;
            track.last_seen_ts_ns = timestamp_ns;
            track.hits = 1;
            track.output_motion_history.emplace_back(timestamp_ns, track.position);
            best_id = track.stable_id;
            orsm_lidar_temporal_tracks_.emplace(best_id, std::move(track));
        } else {
            auto& track = orsm_lidar_temporal_tracks_.at(best_id);
            const float dt = static_cast<float>(timestamp_ns - track.last_seen_ts_ns) * 1.e-9f;
            if (dt > 1.e-3f && dt < 1.5f) {
                const cv::Point3f observed{(candidate.position_3d.x - track.position.x) / dt,
                                            (candidate.position_3d.y - track.position.y) / dt, 0.f};
                track.velocity.x = 0.6f * track.velocity.x + 0.4f * observed.x;
                track.velocity.y = 0.6f * track.velocity.y + 0.4f * observed.y;
            }
            track.position = candidate.position_3d;
            track.max_displacement_m = std::max(
                track.max_displacement_m,
                pointDistance2D(track.position, track.initial_position));
            track.last_seen_ts_ns = timestamp_ns;
            ++track.hits;
            recordOutputMotion(track);
        }
        auto& track = orsm_lidar_temporal_tracks_.at(best_id);
        // Static-background rejection must happen before an unlabeled
        // difference-box observation is allowed to grant a publication lease.
        // Otherwise the lease itself exempts the same cluster from the later
        // background filter and turns a wall/tree fragment into an intruder.
        const int static_background_min_hits =
            std::max(2, cfg_.orsm_lidar_primary_static_background_min_hits);
        const float static_background_max_displacement_m = std::max(
            0.05f, cfg_.orsm_lidar_primary_static_background_max_displacement_m);
        const float static_background_max_speed_mps = std::max(
            0.05f, cfg_.orsm_lidar_primary_static_background_max_speed_mps);
        const bool unlabeled_static_background =
            cfg_.orsm_lidar_primary_static_background_filter_enabled &&
            !orsmBiologicalLabel(candidate.label) &&
            !orsmBiologicalLabel(track.label) &&
            track.hits >= static_background_min_hits &&
            track.max_displacement_m <= static_background_max_displacement_m &&
            pointNorm2D(track.velocity) <= static_background_max_speed_mps;
        if (unlabeled_static_background) {
            rememberStaticFootprint(track, candidate);
        }

        int footprint_source_lidar_id = 0;
        float footprint_overlap_ratio = 0.f;
        float footprint_center_distance = 0.f;
        float footprint_radius = 0.f;
        const bool unknown_diffbox_candidate =
            candidate.orsm_far_image_motion_lidar_confirmed &&
            candidate.lidar_claim == LidarClaim::LidarClaimDiffbox &&
            !orsmBiologicalLabel(candidate.label) &&
            !orsmBiologicalLabel(track.label);
        const bool overlaps_static_footprint =
            unknown_diffbox_candidate &&
            findStaticFootprintOverlap(candidate,
                                       footprint_source_lidar_id,
                                       footprint_overlap_ratio,
                                       footprint_center_distance,
                                       footprint_radius);
        if (candidate.orsm_far_image_motion_lidar_confirmed &&
            (unlabeled_static_background || overlaps_static_footprint)) {
            candidate.orsm_far_image_motion_lidar_confirmed = false;
            candidate.lidar_claim = LidarClaim::LidarClaimNone;
            track.far_image_motion_streak = 0;
            track.last_far_image_motion_ts_ns = 0;
            track.lidar_publication_lease_until_ts_ns = 0;
            if (track.bound_global_track_id != 0) {
                const auto global_it = orsm_global_tracks_.find(track.bound_global_track_id);
                if (global_it != orsm_global_tracks_.end() &&
                    !orsmBiologicalLabel(global_it->second.label)) {
                    global_it->second.lidar_publication_lease_until_ts_ns = 0;
                }
            }
            std::cout << "[ORSMFarMotionStaticReject] ts=" << timestamp_ns
                      << " L=" << best_id
                      << " reason="
                      << (unlabeled_static_background
                          ? "current_static_track" : "persisted_static_footprint")
                      << " hits=" << track.hits
                      << " displacement=" << track.max_displacement_m
                      << " speed=" << pointNorm2D(track.velocity)
                      << " pos=(" << track.position.x << ',' << track.position.y << ')'
                      << " footprint_source_L=" << footprint_source_lidar_id
                      << " footprint_overlap=" << footprint_overlap_ratio
                      << " footprint_center_distance=" << footprint_center_distance
                      << " footprint_radius=" << footprint_radius
                      << " cluster_radius="
                      << std::max(kMinStaticFootprintRadiusM,
                                  candidate.lidar_cluster_radius_m)
                      << std::endl;
        }
        if (candidate.orsm_far_image_motion_lidar_confirmed) {
            constexpr uint64_t kFarMotionMatchGapNs = 1500ULL * 1000000ULL;
            if (track.last_far_image_motion_ts_ns > 0 &&
                timestamp_ns > track.last_far_image_motion_ts_ns &&
                timestamp_ns - track.last_far_image_motion_ts_ns <= kFarMotionMatchGapNs) {
                ++track.far_image_motion_streak;
            } else if (track.last_far_image_motion_ts_ns != timestamp_ns) {
                track.far_image_motion_streak = 1;
            }
            track.last_far_image_motion_ts_ns = timestamp_ns;
            // A difference box that inherited a real detector class is already
            // semantically anchored by the overlapping YOLO box.  It only needs
            // one InBox match to earn the lease.  An unlabeled difference box
            // still requires the configured consecutive matches so a single
            // noisy motion component cannot publish on its own.
            const bool classed_motion_candidate =
                cfg_.orsm_far_image_motion_classed_single_match_enabled &&
                orsmBiologicalLabel(candidate.label);
            const int min_assoc_streak = classed_motion_candidate
                ? 1
                : std::max(1, cfg_.orsm_far_image_motion_lidar_assoc_min_streak);
            if (track.far_image_motion_streak < min_assoc_streak) {
                candidate.orsm_far_image_motion_lidar_confirmed = false;
            } else {
                // Difference-box evidence refreshes the shared publication lease
                // with its shorter, configured lifetime.  Never shorten a
                // longer YOLO semantic lease that is already active.
                track.lidar_publication_lease_until_ts_ns = extendLeaseExpiryNs(
                    track.lidar_publication_lease_until_ts_ns,
                    timestamp_ns,
                    cfg_.orsm_far_image_motion_lease_ms);
                std::cout << "[ORSMFarMotionLease] ts=" << timestamp_ns
                          << " L=" << best_id
                          << " streak=" << track.far_image_motion_streak
                          << " expires=" << track.lidar_publication_lease_until_ts_ns
                          << " min_streak=" << min_assoc_streak
                          << " classed=" << (classed_motion_candidate ? 1 : 0)
                          << " label=" << candidate.label
                          << std::endl;
            }
        } else if (track.last_far_image_motion_ts_ns == 0 ||
                   timestamp_ns > track.last_far_image_motion_ts_ns + 1500ULL * 1000000ULL) {
            track.far_image_motion_streak = 0;
        }

        if (orsmBiologicalLabel(candidate.label) && candidate.class_id >= 0) {
            // 类别按置信度单调更新，unknown 不覆盖已有类别。
            const bool may_adopt_class =
                !cfg_.orsm_class_label_lock_enabled ||
                isUnknownOrsmLabel(track.label, track.class_id) ||
                track.class_id == candidate.class_id ||
                candidate.confidence > track.class_confidence;
            if (may_adopt_class) {
                track.class_id = candidate.class_id;
                track.label = candidate.label;
                track.class_confidence = std::max(track.class_confidence, candidate.confidence);
                track.confidence = std::max(track.confidence, candidate.confidence);
            }
        }
        if (candidate.origin == ObservationOrigin::OriginLidarCluster &&
            candidate.lidar_claim == LidarClaim::LidarClaimYolo &&
            orsmBiologicalLabel(candidate.label)) {
            track.last_semantic_ts_ns = timestamp_ns;
            ++track.semantic_hits;
            if (cfg_.orsm_lidar_semantic_latch_output_enabled) {
                // YOLO 与差值框共用租约，取较晚截止时间，短租约不缩短现有长租约。
                track.lidar_publication_lease_until_ts_ns = extendLeaseExpiryNs(
                    track.lidar_publication_lease_until_ts_ns,
                    timestamp_ns,
                    cfg_.orsm_lidar_semantic_latch_refresh_ms);
                if (cfg_.orsm_debug_track) {
                    std::cout << "[ORSMYoloLease] ts=" << timestamp_ns
                              << " L=" << track.stable_id
                              << " lease_ms=" << cfg_.orsm_lidar_semantic_latch_refresh_ms
                              << " expires=" << track.lidar_publication_lease_until_ts_ns
                              << " label=" << track.label << std::endl;
                }
            }
        }
        candidate.orsm_lidar_temporal_track_id = best_id;
        claimed.insert(best_id);
    }

    if (cfg_.orsm_lidar_primary_static_background_filter_enabled) {
        const int min_hits = std::max(2, cfg_.orsm_lidar_primary_static_background_min_hits);
        const float max_displacement = std::max(
            0.05f, cfg_.orsm_lidar_primary_static_background_max_displacement_m);
        const float max_speed = std::max(
            0.05f, cfg_.orsm_lidar_primary_static_background_max_speed_mps);
        candidates.erase(std::remove_if(candidates.begin(), candidates.end(),
            [&](const TrackedObject& candidate) {
                if (orsmBiologicalLabel(candidate.label)) return false;
                const auto it = orsm_lidar_temporal_tracks_.find(
                    candidate.orsm_lidar_temporal_track_id);
                if (it == orsm_lidar_temporal_tracks_.end()) return false;
                auto& track = it->second;
                if (orsmBiologicalLabel(track.label)) return false;
                const bool static_background =
                    track.hits >= min_hits &&
                    track.max_displacement_m <= max_displacement &&
                    pointNorm2D(track.velocity) <= max_speed;
                if (static_background) {
                    // 静态/未知分类优先于旧租约；移除候选前清除两侧租约。
                    track.lidar_publication_lease_until_ts_ns = 0;
                    if (track.bound_global_track_id != 0) {
                        const auto global_it = orsm_global_tracks_.find(
                            track.bound_global_track_id);
                        if (global_it != orsm_global_tracks_.end() &&
                            !orsmBiologicalLabel(global_it->second.label)) {
                            global_it->second.lidar_publication_lease_until_ts_ns = 0;
                        }
                    }
                }
                if (static_background && std::getenv("ORSM_LIDAR_CLUSTER_DEBUG") != nullptr) {
                    std::cout << "[ORSMLidarBackgroundFilter] ts=" << timestamp_ns
                              << " L=" << track.stable_id
                              << " hits=" << track.hits
                              << " displacement=" << track.max_displacement_m
                              << " speed=" << pointNorm2D(track.velocity)
                              << " center=(" << candidate.position_3d.x << ','
                              << candidate.position_3d.y << ')' << std::endl;
                }
                return static_background;
            }), candidates.end());
    }

    // 前向垂直接近兜底：只对同一 L# 的真实 LiDAR 簇生效，不从 ROI/图像
    // 创建目标。当前车辆坐标约定为 X 前、Y 左，因此要求 X>20m、Y 位于
    // [-10,10]m 走廊内且连续向 X=0 靠近；确认后只发放 3s 短租约。
    // 该判断放在静态背景过滤之后，避免租约反过来豁免静态背景过滤。
    if (cfg_.orsm_front_lidar_motion_lease_enabled) {
        const float min_x = std::max(0.0f, cfg_.orsm_front_lidar_motion_min_x_m);
        const float max_abs_y = std::max(0.0f, cfg_.orsm_front_lidar_motion_max_abs_y_m);
        const float max_y_span = std::max(0.0f, cfg_.orsm_front_lidar_motion_max_y_span_m);
        const float min_progress = std::max(0.0f, cfg_.orsm_front_lidar_motion_min_x_progress_m);
        const size_t min_points = static_cast<size_t>(std::max(2, cfg_.orsm_front_lidar_motion_min_history_points));
        const int min_toward_steps = std::max(1, cfg_.orsm_front_lidar_motion_min_toward_steps);
        const uint64_t max_age_ns = static_cast<uint64_t>(
            std::max(1, cfg_.orsm_front_lidar_motion_history_max_age_ms)) * 1000000ULL;
        for (auto& candidate : candidates) {
            if (!candidate.has_lidar_fusion || !finitePoint2D(candidate.position_3d)) continue;
            const int temporal_id = candidate.orsm_lidar_temporal_track_id;
            const auto temporal_it = orsm_lidar_temporal_tracks_.find(temporal_id);
            if (temporal_it == orsm_lidar_temporal_tracks_.end()) continue;
            auto& track = temporal_it->second;

            // 这条规则只在当前点仍位于 20m 外、且处于前向横向走廊时工作。
            // 一旦进入 20m 内或离开走廊，清掉特殊规则的短历史，避免重新
            // 回到外侧时拼接一段已经不连续的旧轨迹。
            if (candidate.position_3d.x <= min_x ||
                std::fabs(candidate.position_3d.y) > max_abs_y) {
                track.front_motion_history.clear();
                continue;
            }
            auto& history = track.front_motion_history;
            if (!history.empty() && timestamp_ns <= history.back().first) {
                history.clear();
            }
            history.emplace_back(timestamp_ns, candidate.position_3d);
            while (!history.empty() &&
                   timestamp_ns >= history.front().first &&
                   timestamp_ns - history.front().first > max_age_ns) {
                history.pop_front();
            }
            if (history.size() < min_points) continue;

            float min_y = history.front().second.y;
            float max_y = min_y;
            int toward_steps = 0;
            int transitions = 0;
            for (size_t i = 1; i < history.size(); ++i) {
                const auto& previous = history[i - 1].second;
                const auto& current = history[i].second;
                if (std::fabs(current.y) > max_abs_y || current.x <= min_x) {
                    min_y = max_y = 0.0f;
                    transitions = -1;
                    break;
                }
                min_y = std::min(min_y, current.y);
                max_y = std::max(max_y, current.y);
                ++transitions;
                // LiDAR 中心有抖动，给每一步留 5cm 死区；只有 X 明显减小
                // 才计为朝车头靠近。
                if (previous.x - current.x >= 0.05f) ++toward_steps;
            }
            if (transitions < 1) continue;
            const float x_progress = history.front().second.x - history.back().second.x;
            const bool front_approach =
                history.size() >= min_points &&
                std::fabs(history.front().second.y) <= max_abs_y &&
                std::fabs(history.back().second.y) <= max_abs_y &&
                max_y - min_y <= max_y_span &&
                x_progress >= min_progress &&
                toward_steps >= min_toward_steps;
            if (!front_approach) continue;

            const uint64_t lease_ms = static_cast<uint64_t>(
                std::max(0, cfg_.orsm_front_lidar_motion_lease_ms));
            track.lidar_publication_lease_until_ts_ns = extendLeaseExpiryNs(
                track.lidar_publication_lease_until_ts_ns,
                timestamp_ns,
                static_cast<int>(lease_ms));
            candidate.orsm_far_image_motion_lidar_confirmed = true;
            std::cout << "[ORSMFrontApproachLease] ts=" << timestamp_ns
                      << " L=" << temporal_id
                      << " x=" << candidate.position_3d.x
                      << " y=" << candidate.position_3d.y
                      << " y_span=" << (max_y - min_y)
                      << " x_progress=" << x_progress
                      << " toward_steps=" << toward_steps << "/" << transitions
                      << " lease_ms=" << lease_ms
                      << " expires=" << track.lidar_publication_lease_until_ts_ns
                      << std::endl;
        }
    }

    for (auto it = orsm_lidar_temporal_tracks_.begin(); it != orsm_lidar_temporal_tracks_.end();) {
        if (timestamp_ns > it->second.last_seen_ts_ns &&
            timestamp_ns - it->second.last_seen_ts_ns > kPruneNs) it = orsm_lidar_temporal_tracks_.erase(it);
        else ++it;
    }
}

std::vector<TrackedObject> SentinelNode::dedupeOrsmGroupCandidates(
    const std::vector<TrackedObject>& candidates,
    const char* log_tag) const {
    std::vector<TrackedObject> merged;
    merged.reserve(candidates.size());

    auto preferCandidate = [](const TrackedObject& a, const TrackedObject& b) {
        if (a.has_lidar_fusion != b.has_lidar_fusion) return a.has_lidar_fusion;
        if (a.age != b.age) return a.age > b.age;
        return a.confidence > b.confidence;
    };

    for (const auto& obj : candidates) {
        if (obj.track_id < 0 || obj.distance_m <= 0.f || !finitePoint2D(obj.position_3d)) continue;

        int best_idx = -1;
        float best_dist = std::numeric_limits<float>::max();
        for (size_t i = 0; i < merged.size(); ++i) {
            const auto& existing = merged[i];
            const bool both_semantic = obj.class_id >= 0 && existing.class_id >= 0;
            if (both_semantic &&
                !sameOrsmBiologicalClass(obj.class_id, obj.label,
                                         existing.class_id, existing.label)) {
                continue;
            }
            const float d = pointDistance2D(obj.position_3d, existing.position_3d);
            const float range = std::max(obj.distance_m, existing.distance_m);
            float gate = std::max(0.45f, 0.12f * range);
            if (obj.has_lidar_fusion || existing.has_lidar_fusion) {
                gate = std::max(gate, 0.80f);
            } else if (range <= 2.0f) {
                gate = std::max(gate, 0.55f);
            }
            if (d <= gate && d < best_dist) {
                best_dist = d;
                best_idx = static_cast<int>(i);
            }
        }

        if (best_idx < 0) {
            merged.push_back(obj);
            continue;
        }

        auto& dst = merged[static_cast<size_t>(best_idx)];
        if (preferCandidate(obj, dst)) {
            TrackedObject selected = obj;
            selected.confidence = std::max(obj.confidence, dst.confidence);
            selected.age = std::max(obj.age, dst.age);
            dst = selected;
        } else {
            dst.confidence = std::max(dst.confidence, obj.confidence);
            dst.age = std::max(dst.age, obj.age);
        }
    }

    const bool debug_track = cfg_.orsm_debug_track || std::getenv("ORSM_DEBUG_TRACK") != nullptr;
    if ((!candidates.empty() && candidates.size() != merged.size()) || debug_track) {
        const int lidar_count = static_cast<int>(std::count_if(
            merged.begin(), merged.end(), [](const TrackedObject& obj) { return obj.has_lidar_fusion; }));
        std::cout << "[" << (log_tag ? log_tag : "ORSMDedupe") << "] candidates=" << candidates.size()
                  << " merged=" << merged.size()
                  << " lidar_merged=" << lidar_count
                  << std::endl;
    }

    return merged;
}

std::vector<TrackedObject> SentinelNode::buildLidarPrimaryOrsmCandidates(
    const LidarFrame& lidar,
    const std::vector<TrackedObject>& visual_tracks,
    uint64_t timestamp_ns) const {
    if (!lidar.valid()) return {};

    struct LidarSeed {
        const LidarPoint* point{nullptr};
        int cluster_id{-1};
    };
    struct LidarClusterCandidate {
        std::vector<int> indices;
        cv::Point3f center{0.f, 0.f, 0.f};
        float distance_m{0.f};
        float radius_m{0.f};
    };

    const float near_tol = std::max(0.10f, cfg_.orsm_lidar_primary_cluster_tolerance_m);
    const float far_tol = std::max(near_tol, cfg_.orsm_lidar_primary_far_cluster_tolerance_m);
    const float far_distance_m = std::max(0.0f, cfg_.orsm_lidar_primary_far_min_distance_m);
    const float grid_cell = far_tol;
    const float near_voxel_leaf = std::max(0.02f, cfg_.orsm_lidar_primary_voxel_leaf_m);
    const float far_voxel_leaf = std::max(0.02f, cfg_.orsm_lidar_primary_far_voxel_leaf_m);
    const auto clusterToleranceForRange = [near_tol, far_tol, far_distance_m](float range) {
        return range > far_distance_m ? far_tol : near_tol;
    };
    const auto voxelLeafForRange = [near_voxel_leaf, far_voxel_leaf, far_distance_m](float range) {
        return range > far_distance_m ? far_voxel_leaf : near_voxel_leaf;
    };
    const float max_range_m = std::max(cfg_.orsm_zone_a_radius,
                                       cfg_.orsm_multi_vehicle_external_radius) + 2.0f;
    const float min_z = std::min(cfg_.orsm_lidar_primary_min_z_m,
                                 cfg_.orsm_lidar_primary_max_z_m);
    const float max_z = std::max(cfg_.orsm_lidar_primary_min_z_m,
                                 cfg_.orsm_lidar_primary_max_z_m);
    const float source_z_split_distance_m =
        std::max(0.0f, cfg_.orsm_lidar_primary_source_z_split_distance_m);
    const float side_rear_min_z = std::min(
        min_z, cfg_.orsm_lidar_primary_side_rear_min_z_m);
    const float left_min_z = std::min(
        min_z, cfg_.orsm_lidar_primary_left_min_z_m);
    const bool front_fixed_ground_plane_enabled =
        cfg_.orsm_lidar_primary_front_fixed_ground_plane_enabled &&
        std::isfinite(cfg_.orsm_lidar_primary_front_fixed_ground_plane_a) &&
        std::isfinite(cfg_.orsm_lidar_primary_front_fixed_ground_plane_b) &&
        std::isfinite(cfg_.orsm_lidar_primary_front_fixed_ground_plane_c);
    const float front_ground_a = cfg_.orsm_lidar_primary_front_fixed_ground_plane_a;
    const float front_ground_b = cfg_.orsm_lidar_primary_front_fixed_ground_plane_b;
    const float front_ground_c = cfg_.orsm_lidar_primary_front_fixed_ground_plane_c;
    const float front_ground_min_height = std::max(
        0.0f, cfg_.orsm_lidar_primary_front_fixed_ground_plane_min_height_m);
    const float base_semantic_gate = std::max(0.5f, cfg_.orsm_lidar_primary_semantic_gate_m);
    // 重叠框归属已在前序阶段确定，YOLO 标签不再受近场距离上限约束。

    struct SemanticAnchor {
        cv::Point3f position{0.f, 0.f, 0.f};
        float distance_m{0.f};
        bool has_visual_ground_position{false};
        cv::Point3f visual_ground_position{0.f, 0.f, 0.f};
        float visual_ground_distance_m{0.f};
        std::string label;
        int class_id{-1};
        float confidence{0.f};
        cv::Rect bbox;
        std::string source_camera;
        bool fallback_vision{false};
        int anchor_id{-1};
        int visual_track_id{-1};
        bool far_image_motion{false};
        bool lidar_backed{false};
        int expected_lidar_points{0};
        float expected_lidar_radius_m{0.f};
        bool history_anchor{false};
    };
    std::vector<SemanticAnchor> anchors;
    anchors.reserve(visual_tracks.size() + orsm_lidar_temporal_tracks_.size());
    int next_anchor_id = 1;
    for (const auto& visual : visual_tracks) {
        const bool far_image_motion =
            cfg_.orsm_far_image_motion_allow_lidar_backed_new_targets &&
            visual.origin == ObservationOrigin::OriginDiffbox;
        const bool detector_confirmed =
            visual.origin == ObservationOrigin::OriginYolo && visual.lost == 0;
        if (!detector_confirmed && !far_image_motion) continue;
        if (!orsmBiologicalLabel(visual.label) && !far_image_motion) continue;
        const bool fallback_vision = visual.track_id < 0;
        if (fallback_vision &&
            visual.confidence < cfg_.orsm_lidar_primary_fallback_min_confidence) {
            continue;
        }
        const cv::Point3f anchor_position = visual.has_lidar_fusion
            ? visual.position_3d
            : (visual.has_visual_ground_position
                   ? visual.visual_ground_position_3d
                   : visual.position_3d);
        const float anchor_distance = visual.has_lidar_fusion
            ? visual.distance_m
            : (visual.has_visual_ground_position
                   ? visual.visual_ground_distance_m
                   : visual.distance_m);
        // 差值框由 InBox 与真实 LiDAR 簇校验，不要求有效的视觉地面位置。
        if ((!finitePoint2D(anchor_position) || anchor_distance <= 0.f) &&
            !far_image_motion) continue;
        const int visual_track_id = visual.track_id;
        // 当前帧差值观测不携带语义类别。
        anchors.push_back(SemanticAnchor{anchor_position, anchor_distance,
                                         visual.has_visual_ground_position,
                                         visual.visual_ground_position_3d,
                                         visual.visual_ground_distance_m,
                                         detector_confirmed ? visual.label : "unknown",
                                         detector_confirmed ? visual.class_id : -1,
                                         visual.confidence, visual.bbox,
                                         visual.source_camera, fallback_vision,
                                         visual_track_id >= 0 ? visual_track_id : next_anchor_id++,
                                         visual_track_id,
                                         far_image_motion, visual.has_lidar_fusion,
                                         visual.lidar_point_count,
                                         visual.lidar_cluster_radius_m});
    }

    // 历史锚点仅辅助延续语义 L# 附近的稀疏簇，不传递类别、不创建候选或延长租约。
    if (cfg_.orsm_lidar_history_anchor_enabled &&
        cfg_.orsm_lidar_history_anchor_max_age_ms > 0) {
        const uint64_t max_age_ns = static_cast<uint64_t>(
            cfg_.orsm_lidar_history_anchor_max_age_ms) * 1000000ULL;
        for (const auto& [id, track] : orsm_lidar_temporal_tracks_) {
            if (track.semantic_hits <= 0 || !orsmBiologicalLabel(track.label) ||
                track.last_seen_ts_ns == 0 || timestamp_ns < track.last_seen_ts_ns ||
                timestamp_ns - track.last_seen_ts_ns > max_age_ns ||
                !finitePoint2D(track.position)) {
                continue;
            }
            const float dt = std::min(
                1.5f, static_cast<float>(timestamp_ns - track.last_seen_ts_ns) * 1.0e-9f);
            cv::Point3f predicted = track.position;
            if (pointNorm2D(track.velocity) <= 3.0f) {
                predicted.x += track.velocity.x * dt;
                predicted.y += track.velocity.y * dt;
            }
            SemanticAnchor history;
            history.position = predicted;
            history.distance_m = pointNorm2D(predicted);
            history.label = track.label;
            history.class_id = track.class_id;
            history.confidence = track.confidence;
            history.source_camera = "lidar_history";
            history.anchor_id = 900000000 + id;
            history.expected_lidar_points = 0;
            history.expected_lidar_radius_m = 0.f;
            history.history_anchor = true;
            anchors.push_back(std::move(history));
        }
    }

    // 保留旧版 15m 归属、类别传递和带类别差值框逻辑，当前流程不调用。
    auto legacyYoloDiffboxOwnership = [&]() {
            const float min_overlap =
                std::clamp(cfg_.orsm_far_image_motion_class_transfer_min_overlap, 0.01f, 1.0f);
            const float ownership_split_m = std::max(
                0.0f, cfg_.orsm_far_image_motion_lidar_assoc_min_distance_m);
            const bool split_ownership =
                cfg_.orsm_far_image_motion_classed_box_split_ownership_enabled;
            auto boxCoverage = [](const cv::Rect& a, const cv::Rect& b) {
                const float intersection = static_cast<float>((a & b).area());
                const float smaller = static_cast<float>(std::min(a.area(), b.area()));
                return smaller > 0.f ? intersection / smaller : 0.f;
            };
            std::vector<char> anchor_dropped(anchors.size(), 0);
            std::vector<char> detector_claimed(anchors.size(), 0);
            std::vector<char> detector_kept(anchors.size(), 0);
            for (size_t motion_index = 0; motion_index < anchors.size(); ++motion_index) {
                auto& motion_anchor = anchors[motion_index];
                if (!motion_anchor.far_image_motion) continue;
                if (motion_anchor.bbox.width <= 0 || motion_anchor.bbox.height <= 0) continue;
                if (orsmBiologicalLabel(motion_anchor.label)) continue;
                size_t best_detector_index = anchors.size();
                float best_overlap = min_overlap;
                for (size_t detector_index = 0; detector_index < anchors.size(); ++detector_index) {
                    const auto& detector_anchor = anchors[detector_index];
                    if (detector_anchor.far_image_motion) continue;
                    if (!orsmBiologicalLabel(detector_anchor.label)) continue;
                    if (detector_anchor.source_camera != motion_anchor.source_camera) continue;
                    if (detector_anchor.bbox.width <= 0 || detector_anchor.bbox.height <= 0) continue;
                    if (detector_anchor.anchor_id == motion_anchor.anchor_id) continue;
                    const float overlap = boxCoverage(motion_anchor.bbox, detector_anchor.bbox);
                    if (overlap > best_overlap) {
                        best_overlap = overlap;
                        best_detector_index = detector_index;
                    }
                }
                if (best_detector_index == anchors.size()) continue;
                const auto& best_detector = anchors[best_detector_index];
                motion_anchor.label = best_detector.label;
                motion_anchor.class_id = best_detector.class_id;
                motion_anchor.confidence =
                    std::max(motion_anchor.confidence, best_detector.confidence);
                const float detector_range_m = best_detector.distance_m > 0.f
                    ? best_detector.distance_m
                    : best_detector.visual_ground_distance_m;
                const float motion_range_m = motion_anchor.distance_m > 0.f
                    ? motion_anchor.distance_m
                    : motion_anchor.visual_ground_distance_m;
                const bool detector_range_valid = detector_range_m > 0.f;
                const float ownership_range_m = detector_range_valid
                    ? detector_range_m
                    : motion_range_m;
                const bool box_owns_target = split_ownership &&
                    ownership_range_m > ownership_split_m;
                detector_claimed[best_detector_index] = 1;
                if (box_owns_target) {
                    anchor_dropped[motion_index] = 0;
                } else {
                    anchor_dropped[motion_index] = 1;
                    detector_kept[best_detector_index] = 1;
                }
            }
            if (split_ownership) {
                for (size_t i = 0; i < anchors.size(); ++i) {
                    if (detector_claimed[i] == 1 && detector_kept[i] == 0) anchor_dropped[i] = 1;
                }
                size_t keep = 0;
                for (size_t i = 0; i < anchors.size(); ++i) {
                    if (anchor_dropped[i] == 1) continue;
                    if (keep != i) anchors[keep] = std::move(anchors[i]);
                    ++keep;
                }
                anchors.resize(keep);
            }
    };
    (void)legacyYoloDiffboxOwnership;

    // Current behavior: when a YOLO box and a difference box overlap, YOLO
    // always owns the target and the difference-box anchor is discarded.
    {
        const float min_overlap =
            std::clamp(cfg_.orsm_far_image_motion_class_transfer_min_overlap, 0.01f, 1.0f);
        auto boxCoverage = [](const cv::Rect& a, const cv::Rect& b) {
            const float intersection = static_cast<float>((a & b).area());
            const float smaller = static_cast<float>(std::min(a.area(), b.area()));
            return smaller > 0.f ? intersection / smaller : 0.f;
        };
        std::vector<char> anchor_dropped(anchors.size(), 0);
        for (size_t motion_index = 0; motion_index < anchors.size(); ++motion_index) {
            const auto& motion_anchor = anchors[motion_index];
            if (!motion_anchor.far_image_motion ||
                motion_anchor.bbox.width <= 0 || motion_anchor.bbox.height <= 0) {
                continue;
            }
            size_t best_detector_index = anchors.size();
            float best_overlap = min_overlap;
            for (size_t detector_index = 0; detector_index < anchors.size(); ++detector_index) {
                const auto& detector_anchor = anchors[detector_index];
                if (detector_anchor.far_image_motion ||
                    !orsmBiologicalLabel(detector_anchor.label) ||
                    detector_anchor.source_camera != motion_anchor.source_camera ||
                    detector_anchor.bbox.width <= 0 || detector_anchor.bbox.height <= 0 ||
                    detector_anchor.anchor_id == motion_anchor.anchor_id) {
                    continue;
                }
                const float overlap = boxCoverage(motion_anchor.bbox, detector_anchor.bbox);
                if (overlap > best_overlap) {
                    best_overlap = overlap;
                    best_detector_index = detector_index;
                }
            }
            if (best_detector_index == anchors.size()) continue;
            anchor_dropped[motion_index] = 1;
            if (cfg_.orsm_debug_track || std::getenv("ORSM_DEBUG_TRACK") != nullptr) {
                std::cout << "[ORSMFarMotionYoloWins] camera=" << motion_anchor.source_camera
                          << " motion_anchor=" << motion_anchor.anchor_id
                          << " detector_anchor=" << anchors[best_detector_index].anchor_id
                          << " overlap=" << best_overlap
                          << " owner=detector_box" << std::endl;
            }
        }
        size_t keep = 0;
        for (size_t i = 0; i < anchors.size(); ++i) {
            if (anchor_dropped[i] == 1) continue;
            if (keep != i) anchors[keep] = std::move(anchors[i]);
            ++keep;
        }
        anchors.resize(keep);
    }

    std::vector<LidarSeed> seeds;
    seeds.reserve(std::min<size_t>(lidar.points.size(), 30000));
    std::unordered_map<long long, std::vector<int>> grid;
    grid.reserve(lidar.points.size() / 4 + 1);
    std::unordered_set<long long> occupied_voxels;
    if (near_voxel_leaf > 0.f || far_voxel_leaf > 0.f) {
        occupied_voxels.reserve(lidar.points.size() / 8 + 1);
    }

    // Project the current LiDAR points into the same rectified camera image in
    // which the detector bbox was produced.  This is an association gate only:
    // it never creates a cluster from ROI pixels and it is disabled gracefully
    // when a camera calibration is unavailable.
    auto cameraCalibForRoi = [&](const std::string& camera_name) -> const FisheyeCalib* {
        if (fisheye_mgr_) {
            if (const auto* calib = fisheye_mgr_->getCalib(camera_name)) return calib;
        }
        return inbox_fisheye_mgr_ ? inbox_fisheye_mgr_->getCalib(camera_name) : nullptr;
    };
    auto projectRoiPoint = [](const FisheyeCalib* calib,
                              const cv::Point3f& point,
                              cv::Point2f& uv) {
        if (!calib || calib->K_new.empty() || calib->extrinsic.empty() ||
            calib->K_new.rows != 3 || calib->K_new.cols != 3 ||
            calib->extrinsic.rows != 4 || calib->extrinsic.cols != 4) {
            return false;
        }
        cv::Mat transform;
        calib->extrinsic.convertTo(transform, CV_64F);
        const cv::Mat point_cam = transform.inv() *
            (cv::Mat_<double>(4, 1) << point.x, point.y, point.z, 1.0);
        const double x = point_cam.at<double>(0, 0);
        const double y = point_cam.at<double>(1, 0);
        const double z = point_cam.at<double>(2, 0);
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) || z <= 0.05) {
            return false;
        }
        const double u = calib->K_new.at<double>(0, 0) * x / z + calib->K_new.at<double>(0, 2);
        const double v = calib->K_new.at<double>(1, 1) * y / z + calib->K_new.at<double>(1, 2);
        if (!std::isfinite(u) || !std::isfinite(v) || u < 0.0 || v < 0.0 ||
            u >= calib->img_w || v >= calib->img_h) {
            return false;
        }
        uv = cv::Point2f(static_cast<float>(u), static_cast<float>(v));
        return true;
    };
    auto clusterTouchesVisualRoi = [&](const LidarClusterCandidate& cluster,
                                       const SemanticAnchor& anchor) {
        if (!cfg_.orsm_lidar_visual_roi_enabled || anchor.history_anchor ||
            anchor.far_image_motion || anchor.bbox.width <= 0 || anchor.bbox.height <= 0) {
            return true;
        }
        const auto* calib = cameraCalibForRoi(anchor.source_camera);
        if (!calib) return true;
        const int pad_x = std::max(8, static_cast<int>(std::lround(anchor.bbox.width * 0.10f)));
        const int pad_y = std::max(8, static_cast<int>(std::lround(anchor.bbox.height * 0.10f)));
        cv::Rect roi(anchor.bbox.x - pad_x, anchor.bbox.y - pad_y,
                     anchor.bbox.width + 2 * pad_x, anchor.bbox.height + 2 * pad_y);
        roi &= cv::Rect(0, 0, calib->img_w, calib->img_h);
        if (roi.width <= 0 || roi.height <= 0) return false;
        auto inRoi = [&](const cv::Point3f& point) {
            cv::Point2f uv;
            if (!projectRoiPoint(calib, point, uv)) return false;
            return roi.contains(cv::Point(static_cast<int>(std::lround(uv.x)),
                                          static_cast<int>(std::lround(uv.y))));
        };
        if (inRoi(cluster.center)) return true;
        for (const int index : cluster.indices) {
            const auto* point = seeds[static_cast<size_t>(index)].point;
            if (point && inRoi(cv::Point3f(point->x, point->y, point->z))) return true;
        }
        return false;
    };
    auto visualRangeConsistent = [&](const LidarClusterCandidate& cluster,
                                      const SemanticAnchor& anchor) {
        if (!cfg_.orsm_visual_lidar_ground_gate_enabled || anchor.history_anchor ||
            anchor.far_image_motion || !anchor.has_visual_ground_position ||
            anchor.visual_ground_distance_m <= 0.f) {
            return true;
        }
        const float tolerance = std::max(
            std::max(0.f, cfg_.orsm_visual_lidar_ground_gate_min_m),
            std::max(0.f, cfg_.orsm_visual_lidar_ground_gate_range_ratio) *
                std::max(1.f, anchor.visual_ground_distance_m));
        return std::fabs(cluster.distance_m - anchor.visual_ground_distance_m) <= tolerance;
    };

    size_t front_ground_plane_rejected_points = 0;
    for (const auto& point : lidar.points) {
        if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) continue;
        const float range = std::hypot(point.x, point.y);
        if (range < 0.45f || range > max_range_m) continue;
        // 25m 外按来源放宽：左雷达单独使用更低门限，后/右使用通用门限，
        // 前雷达始终保持正常的地面高度门限。
        const float point_min_z =
            front_fixed_ground_plane_enabled && point.source_id == 0
                ? front_ground_min_height
                : (range > source_z_split_distance_m && point.source_id != 0
                       ? (point.source_id == 1 ? left_min_z : side_rear_min_z)
                       : min_z);
        float filter_z = point.z;
        if (front_fixed_ground_plane_enabled && point.source_id == 0) {
            const float ground_z =
                front_ground_a * point.x + front_ground_b * point.y + front_ground_c;
            filter_z = point.z - ground_z;
        }
        if (filter_z < point_min_z || filter_z > max_z) {
            if (front_fixed_ground_plane_enabled && point.source_id == 0 &&
                filter_z < point_min_z) {
                ++front_ground_plane_rejected_points;
            }
            continue;
        }
        if (near_voxel_leaf > 0.f || far_voxel_leaf > 0.f) {
            const float voxel_leaf = voxelLeafForRange(range);
            const int vx = static_cast<int>(std::floor(point.x / voxel_leaf));
            const int vy = static_cast<int>(std::floor(point.y / voxel_leaf));
            const int vz = static_cast<int>(std::floor(point.z / voxel_leaf));
            if (!occupied_voxels.insert(mergeVoxelKey(vx, vy, vz)).second) continue;
        }

        const int idx = static_cast<int>(seeds.size());
        seeds.push_back(LidarSeed{&point, -1});
        const int ix = static_cast<int>(std::floor(point.x / grid_cell));
        const int iy = static_cast<int>(std::floor(point.y / grid_cell));
        const int iz = static_cast<int>(std::floor(point.z / grid_cell));
        grid[mergeVoxelKey(ix, iy, iz)].push_back(idx);
    }

    std::vector<LidarClusterCandidate> clusters;
    std::vector<int> queue;
    queue.reserve(512);
    for (int seed = 0; seed < static_cast<int>(seeds.size()); ++seed) {
        if (seeds[seed].cluster_id >= 0) continue;
        const int cluster_id = static_cast<int>(clusters.size());
        LidarClusterCandidate cluster;
        queue.clear();
        queue.push_back(seed);
        seeds[seed].cluster_id = cluster_id;

        for (size_t q = 0; q < queue.size(); ++q) {
            const int cur = queue[q];
            cluster.indices.push_back(cur);
            const auto& cp = *seeds[cur].point;
            const int ix = static_cast<int>(std::floor(cp.x / grid_cell));
            const int iy = static_cast<int>(std::floor(cp.y / grid_cell));
            const int iz = static_cast<int>(std::floor(cp.z / grid_cell));
            for (int dx = -1; dx <= 1; ++dx) {
                for (int dy = -1; dy <= 1; ++dy) {
                    for (int dz = -1; dz <= 1; ++dz) {
                        auto it = grid.find(mergeVoxelKey(ix + dx, iy + dy, iz + dz));
                        if (it == grid.end()) continue;
                        for (int nb : it->second) {
                            if (seeds[nb].cluster_id >= 0) continue;
                            const auto& np = *seeds[nb].point;
                            const float ddx = cp.x - np.x;
                            const float ddy = cp.y - np.y;
                            const float ddz = cp.z - np.z;
                            const float neighbor_range = std::hypot(np.x, np.y);
                            const float pair_tol = std::max(
                                clusterToleranceForRange(std::hypot(cp.x, cp.y)),
                                clusterToleranceForRange(neighbor_range));
                            if (ddx * ddx + ddy * ddy + ddz * ddz > pair_tol * pair_tol) continue;
                            seeds[nb].cluster_id = cluster_id;
                            queue.push_back(nb);
                        }
                    }
                }
            }
        }

        const int normal_min_points = std::max(1, cfg_.orsm_lidar_primary_min_cluster_points);
        const float mid_min_distance_m = std::max(
            0.0f, cfg_.orsm_lidar_primary_mid_min_distance_m);
        const int mid_min_points = std::max(
            1, cfg_.orsm_lidar_primary_mid_min_cluster_points);
        const float far_min_distance_m = std::max(0.0f, cfg_.orsm_lidar_primary_far_min_distance_m);
        const int far_min_points = std::max(1, cfg_.orsm_lidar_primary_far_min_cluster_points);
        const int anchor_min_points = std::max(
            2, std::min(normal_min_points,
                        cfg_.orsm_lidar_primary_anchor_min_cluster_points));
        // 【侧向稀疏放宽】补盲雷达覆盖的侧向远距区域（|y| 占距离比例高），行人
        // 走出前向主雷达 FOV 后每帧可能只剩 1~2 点；用簇点粗算均值位置判定
        // 是否处于侧向远距带，命中则允许低于 anchor_min_points 的簇继续参与
        // 分带门限判定（后续仍有 blindside 半径/高度约束与发布层门控）。
        bool blindside_candidate = false;
        if (cfg_.orsm_lidar_primary_blindside_sparse_enabled) {
            const int pts = static_cast<int>(cluster.indices.size());
            const int blindside_min_pts = std::max(
                2, cfg_.orsm_lidar_primary_blindside_min_cluster_points);
            if (pts >= blindside_min_pts && pts < anchor_min_points) {
                float sum_x = 0.f, sum_y = 0.f;
                for (int idx : cluster.indices) {
                    sum_x += seeds[idx].point->x;
                    sum_y += seeds[idx].point->y;
                }
                const float mean_d =
                    std::sqrt(sum_x * sum_x + sum_y * sum_y) /
                    static_cast<float>(pts);
                if (mean_d > far_min_distance_m &&
                    std::fabs(sum_y / static_cast<float>(pts)) >=
                        cfg_.orsm_lidar_primary_blindside_min_lateral_ratio *
                            mean_d) {
                    blindside_candidate = true;
                }
            }
        }
        if (static_cast<int>(cluster.indices.size()) < anchor_min_points &&
            !blindside_candidate) {
            for (int idx : cluster.indices) seeds[idx].cluster_id = -2;
            continue;
        }

        std::vector<float> xs, ys, zs;
        xs.reserve(cluster.indices.size());
        ys.reserve(cluster.indices.size());
        zs.reserve(cluster.indices.size());
        for (int idx : cluster.indices) {
            const auto& p = *seeds[idx].point;
            xs.push_back(p.x);
            ys.push_back(p.y);
            zs.push_back(p.z);
        }
        cluster.center = cv::Point3f(medianFloat(xs), medianFloat(ys), medianFloat(zs));
        cluster.distance_m = pointNorm2D(cluster.center);
        const int cluster_min_points = cluster.distance_m > far_min_distance_m
            ? far_min_points
            : (cluster.distance_m > mid_min_distance_m
                   ? mid_min_points
                   : normal_min_points);
        float radius = 0.f;
        for (int idx : cluster.indices) {
            const auto& p = *seeds[idx].point;
            const float dx = p.x - cluster.center.x;
            const float dy = p.y - cluster.center.y;
            const float dz = p.z - cluster.center.z;
            radius = std::max(radius, std::sqrt(dx * dx + dy * dy + dz * dz));
        }
        cluster.radius_m = radius;
        if (cluster.distance_m <= 0.f || cluster.radius_m > 4.0f) continue;
        // 侧向稀疏簇自身的紧凑/离地约束：不满足则回退到原分带门控。
        if (blindside_candidate &&
            (cluster.radius_m >
                 cfg_.orsm_lidar_primary_blindside_max_radius_m ||
             cluster.center.z < 0.05f)) {
            blindside_candidate = false;
        }

        // Blind-spot lidars have far fewer returns than the front lidar.  Keep
        // the normal threshold nearby, but use the configured sparse threshold
        // beyond the far-distance boundary.  Any small component still needs
        // compact, non-flat geometry or a trusted semantic/vehicle-corner gate;
        // occlusion shadows remain empty and no points are synthesized here.
        if (static_cast<int>(cluster.indices.size()) < cluster_min_points &&
            !blindside_candidate) {
            const auto [min_z_it, max_z_it] = std::minmax_element(zs.begin(), zs.end());
            const float height_m = *max_z_it - *min_z_it;
            const float anchor_gate_m = std::max(
                base_semantic_gate, 0.12f * cluster.distance_m);
            const bool near_trusted_anchor = std::any_of(
                anchors.begin(), anchors.end(), [&](const SemanticAnchor& anchor) {
                    if (!(orsmBiologicalLabel(anchor.label) || anchor.far_image_motion) ||
                        pointDistance2D(cluster.center, anchor.position) >
                            std::max(anchor.history_anchor
                                         ? cfg_.orsm_lidar_history_anchor_gate_m
                                         : anchor_gate_m,
                                     0.1f)) {
                        return false;
                    }
                    if (anchor.history_anchor) return true;
                    return clusterTouchesVisualRoi(cluster, anchor) &&
                           visualRangeConsistent(cluster, anchor);
                });
            const bool near_vehicle_corner =
                cluster.distance_m <= 8.0f &&
                std::abs(cluster.center.x) >= 0.8f &&
                std::abs(cluster.center.y) >= 0.8f;
            const float allowed_radius = near_trusted_anchor
                ? std::max(0.2f, cfg_.orsm_lidar_primary_anchor_max_radius_m)
                : 1.0f;
            if ((!near_trusted_anchor && !near_vehicle_corner) ||
                cluster.radius_m > allowed_radius ||
                height_m < std::max(0.05f, cfg_.orsm_lidar_primary_anchor_min_height_m)) {
                continue;
            }
        }
        clusters.push_back(std::move(cluster));
    }

    std::vector<TrackedObject> raw_output;
    raw_output.reserve(clusters.size());
    int local_id = 1;
    for (const auto& cluster : clusters) {
        TrackedObject obj;
        obj.track_id = 800000000 + local_id++;
        obj.class_id = -1;
        obj.label = "unknown";
        obj.confidence = 0.35f;
        obj.age = std::max(1, cfg_.orsm_visual_confirm_min_age);
        obj.lost = 0;
        obj.has_lidar_fusion = true;
        obj.position_3d = cluster.center;
        obj.distance_m = cluster.distance_m;
        obj.lidar_point_count = static_cast<int>(cluster.indices.size());
        obj.lidar_cluster_radius_m = cluster.radius_m;
        obj.fusion_timestamp_ns = timestamp_ns > 0 ? timestamp_ns : lidar.timestamp_ns;
        obj.source_camera = "lidar_primary";
        obj.origin = ObservationOrigin::OriginLidarCluster;

        const SemanticAnchor* best_anchor = nullptr;
        float best_anchor_dist = std::numeric_limits<float>::max();
        for (const auto& anchor : anchors) {
            // Far image-motion anchors are associated below by projecting the
            // LiDAR cluster into the single difference box (in_box).  They
            // must not also use the ordinary centre-distance gate.
            // History anchors are intentionally not allowed to paste an old
            // class onto a new cluster; they only influence sparse-cluster
            // admission above and temporal L# continuity.
            if (anchor.far_image_motion || anchor.history_anchor) continue;
            const float d = pointDistance2D(cluster.center, anchor.position);
            float gate = std::max(base_semantic_gate,
                                  0.12f * std::max(cluster.distance_m, anchor.distance_m));
            if (anchor.fallback_vision) {
                gate = std::max(gate, cfg_.orsm_lidar_primary_fallback_semantic_gate_m);
            }
            if (d > gate) continue;
            if (!clusterTouchesVisualRoi(cluster, anchor) ||
                !visualRangeConsistent(cluster, anchor)) {
                if (cfg_.orsm_debug_track) {
                    std::cout << "[ORSMVisualRoiReject] camera=" << anchor.source_camera
                              << " cluster_dist=" << cluster.distance_m
                              << " visual_dist=" << anchor.visual_ground_distance_m
                              << " bbox=(" << anchor.bbox.x << ',' << anchor.bbox.y << ','
                              << anchor.bbox.width << ',' << anchor.bbox.height << ')' << std::endl;
                }
                continue;
            }
            if (d < best_anchor_dist) {
                best_anchor_dist = d;
                best_anchor = &anchor;
            }
        }
        if (best_anchor) {
            obj.class_id = best_anchor->class_id;
            obj.label = best_anchor->label;
            obj.confidence = std::max(best_anchor->confidence, obj.confidence);
            obj.bbox = best_anchor->bbox;
            obj.lidar_claim = LidarClaim::LidarClaimYolo;
            obj.orsm_semantic_anchor_id = best_anchor->anchor_id;
            obj.orsm_visual_track_id = best_anchor->visual_track_id;
            obj.has_visual_ground_position = best_anchor->has_visual_ground_position;
            obj.visual_ground_position_3d = best_anchor->visual_ground_position;
            obj.visual_ground_distance_m = best_anchor->visual_ground_distance_m;
            obj.source_camera = best_anchor->source_camera.empty()
                                        ? "lidar_primary+vision"
                                        : best_anchor->source_camera;
        }

        raw_output.push_back(std::move(obj));
    }


    const float unclassed_motion_min_distance_m = std::max(
        0.0f, cfg_.orsm_far_image_motion_unclassed_min_distance_m);
    const float near_filter_ratio =
        std::clamp(cfg_.orsm_far_image_motion_near_filter_ratio, 0.0f, 1.0f);
    const float far_filter_ratio =
        std::max(0.0f, cfg_.orsm_far_image_motion_far_filter_ratio);
    const float max_assoc_range_gap_m =
        std::max(0.0f, cfg_.orsm_far_image_motion_lidar_assoc_max_range_gap_m);
    const float max_box_area_ratio =
        std::max(0.0f, cfg_.orsm_far_image_motion_max_box_area_ratio);
    const float max_visual_range_front_m =
        std::max(0.0f, cfg_.orsm_far_image_motion_max_visual_range_front_m);
    const float max_visual_range_other_m =
        std::max(0.0f, cfg_.orsm_far_image_motion_max_visual_range_other_m);
    // 目标离开画面后，框高兜底可能产生异常远距框，此类框不参与轨迹处理。
    auto farMotionMaxVisualRangeM = [&](const std::string& camera_name) {
        return camera_name.find("front") != std::string::npos
            ? max_visual_range_front_m
            : max_visual_range_other_m;
    };

    constexpr float kInboxLeaseBaseGateM = 1.35f;
    constexpr float kInboxLeaseMaxSpeedMps = 3.0f;
    struct InboxLeaseClaim {
        int temporal_id{0};
        cv::Point3f predicted{0.f, 0.f, 0.f};
        float dt{0.f};
    };
    std::vector<InboxLeaseClaim> inbox_lease_claims;
    if (cfg_.orsm_far_image_motion_inbox_lease_priority_enabled) {
        for (const auto& [id, track] : orsm_lidar_temporal_tracks_) {
            const bool lease_active =
                track.lidar_publication_lease_until_ts_ns > 0 &&
                timestamp_ns <= track.lidar_publication_lease_until_ts_ns;
            if (!lease_active && track.bound_global_track_id == 0) continue;
            if (!finitePoint2D(track.position) || timestamp_ns < track.last_seen_ts_ns) {
                continue;
            }
            const float dt = static_cast<float>(timestamp_ns - track.last_seen_ts_ns) * 1.e-9f;
            if (dt > 3.f) continue;
            InboxLeaseClaim claim;
            claim.temporal_id = id;
            claim.dt = dt;
            claim.predicted = track.position;
            if (pointNorm2D(track.velocity) <= kInboxLeaseMaxSpeedMps) {
                claim.predicted.x += track.velocity.x * std::min(dt, 1.f);
                claim.predicted.y += track.velocity.y * std::min(dt, 1.f);
            }
            inbox_lease_claims.push_back(claim);
        }
    }

    // 仅根据已有 L# 历史判断当前簇是否属于长期静态目标。
    auto clusterIsLongStatic = [&](const cv::Point3f& center) {
        if (!cfg_.orsm_static_cluster_semantic_lockout_enabled) return false;
        const float static_max_disp = std::max(
            0.05f, cfg_.orsm_static_cluster_semantic_lockout_max_displacement_m);
        const int static_min_hits = std::max(
            2, cfg_.orsm_static_cluster_semantic_lockout_min_hits);
        for (const auto& entry : orsm_lidar_temporal_tracks_) {
            const auto& ttrack = entry.second;
            if (ttrack.hits < static_min_hits) continue;
            if (ttrack.max_displacement_m > static_max_disp) continue;
            const float dx = ttrack.position.x - center.x;
            const float dy = ttrack.position.y - center.y;
            if (dx * dx + dy * dy > 1.44f) continue;  // 1.2m radius
            return true;
        }
        return false;
    };

    for (int anchor_idx = 0; anchor_idx < static_cast<int>(anchors.size()); ++anchor_idx) {
        const auto& anchor = anchors[static_cast<size_t>(anchor_idx)];
        if (!anchor.far_image_motion ||
            anchor.bbox.width <= 0 || anchor.bbox.height <= 0 ||
            anchor.source_camera.empty() || !inbox_fisheye_mgr_) {
            continue;
        }
        // The difference box's own bottom-edge ground estimate.  A negative/zero
        // value means the estimate is unusable; disarm the 0.8 near filter then.
        const float box_range_m = anchor.distance_m > 0.f
            ? anchor.distance_m
            : anchor.visual_ground_distance_m;
        const float max_visual_range_m = farMotionMaxVisualRangeM(anchor.source_camera);
        if (box_range_m > max_visual_range_m) {
            if (cfg_.orsm_debug_track) {
                std::cout << "[ORSMFarMotionInBoxSkip] reason=unreliable_box_range"
                          << " camera=" << anchor.source_camera
                          << " box_range=" << box_range_m
                          << " max=" << max_visual_range_m << std::endl;
            }
            continue;
        }
        const auto* calib = inbox_fisheye_mgr_->getCalib(anchor.source_camera);
        if (!calib) continue;
        const float image_area = static_cast<float>(calib->img_w) *
                                 static_cast<float>(calib->img_h);
        const float box_area_ratio = image_area > 0.f
            ? static_cast<float>(anchor.bbox.area()) / image_area
            : 0.f;
        if (max_box_area_ratio > 0.f && box_area_ratio > max_box_area_ratio) {
            if (cfg_.orsm_debug_track ||
                std::getenv("ORSM_LIDAR_CLUSTER_DEBUG") != nullptr) {
                std::cout << "[ORSMFarMotionInBoxSkip] reason=large_box"
                          << " camera=" << anchor.source_camera
                          << " box_area_ratio=" << box_area_ratio
                          << " max=" << max_box_area_ratio
                          << " bbox=(" << anchor.bbox.x << ',' << anchor.bbox.y << ','
                          << anchor.bbox.width << ',' << anchor.bbox.height << ')'
                          << std::endl;
            }
            continue;
        }

        constexpr int kFarMotionAssocPadPx = 8;
        cv::Rect assoc_box(anchor.bbox.x - kFarMotionAssocPadPx,
                           anchor.bbox.y - kFarMotionAssocPadPx,
                           anchor.bbox.width + 2 * kFarMotionAssocPadPx,
                           anchor.bbox.height + 2 * kFarMotionAssocPadPx);
        assoc_box &= cv::Rect(0, 0, calib->img_w, calib->img_h);
        if (assoc_box.width <= 0 || assoc_box.height <= 0) continue;

        auto projectPointToImage = [&](const cv::Point3f& point, cv::Point2f& uv) {
            if (calib->extrinsic.empty() || calib->K_new.empty() ||
                calib->extrinsic.rows != 4 || calib->extrinsic.cols != 4) {
                return false;
            }
            cv::Mat transform;
            calib->extrinsic.convertTo(transform, CV_64F);
            const cv::Mat inverse = transform.inv();
            const cv::Mat point_car =
                (cv::Mat_<double>(4, 1) << point.x, point.y, point.z, 1.0);
            const cv::Mat point_cam = inverse * point_car;
            const double x = point_cam.at<double>(0, 0);
            const double y = point_cam.at<double>(1, 0);
            const double z = point_cam.at<double>(2, 0);
            if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) || z <= 0.05) {
                return false;
            }
            const double u = calib->K_new.at<double>(0, 0) * x / z + calib->K_new.at<double>(0, 2);
            const double v = calib->K_new.at<double>(1, 1) * y / z + calib->K_new.at<double>(1, 2);
            if (!std::isfinite(u) || !std::isfinite(v) ||
                u < 0 || v < 0 || u >= calib->img_w || v >= calib->img_h) {
                return false;
            }
            uv = cv::Point2f(static_cast<float>(u), static_cast<float>(v));
            return true;
        };
        auto pointInAssocBox = [&](const cv::Point3f& point) {
            cv::Point2f uv;
            if (!projectPointToImage(point, uv)) return false;
            return assoc_box.contains(cv::Point(static_cast<int>(std::lround(uv.x)),
                                                static_cast<int>(std::lround(uv.y))));
        };

        // 仅使用预测位置落入当前框的活跃租约。
        std::vector<const InboxLeaseClaim*> box_lease_claims;
        for (const auto& claim : inbox_lease_claims) {
            if (pointInAssocBox(claim.predicted)) box_lease_claims.push_back(&claim);
        }

        int best_cluster_idx = -1;
        float best_range_error = std::numeric_limits<float>::max();
        // 先按轨迹连续性匹配，再按距离残差排序。
        int priority_cluster_idx = -1;
        int priority_lease_id = 0;
        float priority_lease_distance = std::numeric_limits<float>::max();
        int eligible_clusters = 0;
        int box_hits = 0;
        int nearer_rejected = 0;
        int farther_rejected = 0;
        // 带类别差值框不受近远分界限制；无类别差值框保留原分界规则。
        const bool classed_motion_box = orsmBiologicalLabel(anchor.label);
        for (int cluster_idx = 0; cluster_idx < static_cast<int>(clusters.size()); ++cluster_idx) {
            const auto& cluster = clusters[static_cast<size_t>(cluster_idx)];
            if (!classed_motion_box && cluster.distance_m < unclassed_motion_min_distance_m) {
                continue;
            }
            if (clusterIsLongStatic(cluster.center)) {
                if (cfg_.orsm_debug_track) {
                    std::cout << "[ORSMFarMotionStaticSkip] camera="
                              << anchor.source_camera
                              << " anchor=" << anchor.anchor_id
                              << " cluster_dist=" << cluster.distance_m
                              << " center=(" << cluster.center.x << ','
                              << cluster.center.y << ')' << std::endl;
                }
                continue;
            }
            ++eligible_clusters;
            bool hits_box = pointInAssocBox(cluster.center);
            if (!hits_box) {
                for (int seed_idx : cluster.indices) {
                    const auto& point = *seeds[static_cast<size_t>(seed_idx)].point;
                    if (pointInAssocBox(cv::Point3f(point.x, point.y, point.z))) {
                        hits_box = true;
                        break;
                    }
                }
            }
            if (!hits_box) continue;
            ++box_hits;

            // 命中活跃 L# 预测时优先保持轨迹连续性，并跳过常规距离门限。
            const InboxLeaseClaim* matched_lease_claim = nullptr;
            float matched_lease_distance = std::numeric_limits<float>::max();
            for (const InboxLeaseClaim* claim : box_lease_claims) {
                const float gate = std::min(
                    1.8f, kInboxLeaseBaseGateM + kInboxLeaseMaxSpeedMps * claim->dt);
                const float distance = pointDistance2D(cluster.center, claim->predicted);
                if (distance > gate) continue;
                if (distance < matched_lease_distance) {
                    matched_lease_distance = distance;
                    matched_lease_claim = claim;
                }
            }
            if (matched_lease_claim != nullptr) {
                if (matched_lease_distance < priority_lease_distance) {
                    priority_lease_distance = matched_lease_distance;
                    priority_cluster_idx = cluster_idx;
                    priority_lease_id = matched_lease_claim->temporal_id;
                }
                continue;
            }

            // One 2D box can contain both a near distractor and the far target on
            // the same ray.  Drop clusters clearly nearer than the box's own range
            // estimate, then rank the survivors by that estimate below.
            const bool drop_near =
                box_range_m > 0.f && near_filter_ratio > 0.f &&
                cluster.distance_m < near_filter_ratio * box_range_m;
            if (drop_near) {
                ++nearer_rejected;
                continue;
            }
            const bool drop_far =
                box_range_m > 0.f && far_filter_ratio > 0.f &&
                cluster.distance_m > far_filter_ratio * box_range_m;
            if (drop_far) {
                ++farther_rejected;
                continue;
            }

            // Match on the absolute range difference between the cluster and the
            // difference box's own bottom-edge ground estimate: pick the cluster
            // whose range is closest to the visual estimate, not merely the one
            // nearest to the ego vehicle.
            const float range_error = std::fabs(cluster.distance_m - anchor.distance_m);
            if (range_error < best_range_error) {
                best_range_error = range_error;
                best_cluster_idx = cluster_idx;
            }

        }
        // 活跃租约在框内匹配到合格簇时保持原身份，并跳过下方距离残差门限。
        const bool lease_priority_match = priority_cluster_idx >= 0;
        if (lease_priority_match) {
            std::cout << "[ORSMFarMotionInBoxLeasePriority] camera=" << anchor.source_camera
                      << " anchor=" << anchor.anchor_id
                      << " L=" << priority_lease_id
                      << " cluster_dist="
                      << clusters[static_cast<size_t>(priority_cluster_idx)].distance_m
                      << " points="
                      << clusters[static_cast<size_t>(priority_cluster_idx)].indices.size()
                      << " lease_distance=" << priority_lease_distance
                      << " visual_box_range=" << anchor.distance_m
                      << " residual_cluster_dist="
                      << (best_cluster_idx >= 0
                              ? clusters[static_cast<size_t>(best_cluster_idx)].distance_m
                              : 0.f)
                      << " residual_range_error="
                      << (best_cluster_idx >= 0 ? best_range_error : 0.f)
                      << std::endl;
            best_cluster_idx = priority_cluster_idx;
            best_range_error = std::fabs(
                clusters[static_cast<size_t>(best_cluster_idx)].distance_m - anchor.distance_m);
        }
        // 绝对距离残差门限与比例门限同时生效，用于排除同射线上的其他目标。
        if (!lease_priority_match && best_cluster_idx >= 0 && max_assoc_range_gap_m > 0.f &&
            best_range_error > max_assoc_range_gap_m) {
            if (cfg_.orsm_debug_track) {
                std::cout << "[ORSMFarMotionInBoxSkip] reason=range_residual_too_large"
                          << " camera=" << anchor.source_camera
                          << " anchor=" << anchor.anchor_id
                          << " visual_box_range=" << anchor.distance_m
                          << " cluster_dist="
                          << clusters[static_cast<size_t>(best_cluster_idx)].distance_m
                          << " range_error=" << best_range_error
                          << " max=" << max_assoc_range_gap_m
                          << " label=" << anchor.label << std::endl;
            }
            continue;
        }
        if (best_cluster_idx < 0) {
            if (cfg_.orsm_debug_track) {
                std::cout << "[ORSMFarMotionInBoxNoMatch] camera=" << anchor.source_camera
                          << " visual_box_range=" << anchor.distance_m
                          << " eligible_clusters=" << eligible_clusters
                          << " box_hits=" << box_hits
                          << " nearer_rejected=" << nearer_rejected
                          << " farther_rejected=" << farther_rejected << std::endl;
            }
            continue;
        }

        auto& matched = raw_output[static_cast<size_t>(best_cluster_idx)];
        matched.bbox = anchor.bbox;
        matched.lidar_claim = LidarClaim::LidarClaimDiffbox;
        matched.orsm_far_image_motion_lidar_confirmed = true;
        matched.orsm_semantic_anchor_id = anchor.anchor_id;
        matched.orsm_visual_track_id = anchor.visual_track_id;
        matched.has_visual_ground_position = anchor.has_visual_ground_position;
        matched.visual_ground_position_3d = anchor.visual_ground_position;
        matched.visual_ground_distance_m = anchor.visual_ground_distance_m;
        matched.source_camera = anchor.source_camera;
        // 重叠差值框继承检测框类别，但目标存在性和距离仍由差值框提供。
        if (orsmBiologicalLabel(anchor.label)) {
            matched.class_id = anchor.class_id;
            matched.label = anchor.label;
            matched.confidence = std::max(matched.confidence, anchor.confidence);
        }
        std::cout << "[ORSMFarMotionInBoxMatched] camera=" << anchor.source_camera
                  << " anchor=" << anchor.anchor_id
                  << " visual_box_range=" << anchor.distance_m
                  << " cluster_dist=" << matched.distance_m
                  << " points=" << matched.lidar_point_count
                  << " range_error=" << best_range_error
                  << " class_source=" << (orsmBiologicalLabel(anchor.label) ? "transferred" : "none")
                  << " label=" << matched.label << std::endl;
    }

    // 过滤超大聚类，避免历史语义将墙面或地面连通区域识别为生物目标。
    raw_output.erase(std::remove_if(raw_output.begin(), raw_output.end(),
        [&](const TrackedObject& candidate) {
            return candidate.lidar_cluster_radius_m > 2.0f ||
                   (orsmBiologicalLabel(candidate.label) &&
                    (candidate.lidar_cluster_radius_m >
                         std::max(0.2f, cfg_.orsm_lidar_primary_anchor_max_radius_m) ||
                     candidate.lidar_point_count > 120));
        }), raw_output.end());

    std::vector<TrackedObject> output;
    output.reserve(raw_output.size());

    // A detector observation describes one physical target.  Before this
    // guard, every cluster inside its ground-projection gate could inherit the
    // same YOLO label and independently become a final L#/global
    // track.  Keep only the nearest LiDAR cluster for each YOLO anchor; all
    // other clusters remain diagnostic LiDAR-only candidates and cannot renew
    // a semantic-output lease.
    std::unordered_map<int, cv::Point3f> yolo_anchor_positions;
    for (const auto& anchor : anchors) {
        yolo_anchor_positions.emplace(anchor.anchor_id, anchor.position);
    }
    std::unordered_map<int, std::pair<size_t, float>> yolo_anchor_best;
    for (size_t index = 0; index < raw_output.size(); ++index) {
        const auto& candidate = raw_output[index];
        if (candidate.lidar_claim != LidarClaim::LidarClaimYolo) continue;
        const auto anchor_it = yolo_anchor_positions.find(
            candidate.orsm_semantic_anchor_id);
        if (anchor_it == yolo_anchor_positions.end()) continue;
        const float distance = pointDistance2D(candidate.position_3d,
                                               anchor_it->second);
        const auto best_it = yolo_anchor_best.find(candidate.orsm_semantic_anchor_id);
        if (best_it == yolo_anchor_best.end() || distance < best_it->second.second) {
            yolo_anchor_best[candidate.orsm_semantic_anchor_id] = {index, distance};
        }
    }
    for (size_t index = 0; index < raw_output.size(); ++index) {
        auto& candidate = raw_output[index];
        if (candidate.lidar_claim != LidarClaim::LidarClaimYolo) continue;
        const auto best_it = yolo_anchor_best.find(candidate.orsm_semantic_anchor_id);
        if (best_it == yolo_anchor_best.end() || best_it->second.first == index) continue;
        candidate.lidar_claim = LidarClaim::LidarClaimNone;
        candidate.class_id = -1;
        candidate.label = "unknown";
        candidate.orsm_semantic_anchor_id = -1;
        candidate.orsm_visual_track_id = -1;
        candidate.bbox = cv::Rect{};
    }

    std::unordered_map<int, size_t> anchor_to_output;
    int merged_by_anchor = 0;
    for (const auto& obj : raw_output) {
        const int anchor_id = obj.orsm_semantic_anchor_id;
        auto it = anchor_to_output.find(anchor_id);
        if (anchor_id < 0 || it == anchor_to_output.end()) {
            if (anchor_id >= 0) anchor_to_output[anchor_id] = output.size();
            output.push_back(obj);
            continue;
        }

        TrackedObject& keep = output[it->second];
        const int keep_points = std::max(1, keep.lidar_point_count);
        const int obj_points = std::max(1, obj.lidar_point_count);
        const int total_points = keep_points + obj_points;
        const cv::Point3f old_pos = keep.position_3d;
        keep.position_3d.x = (old_pos.x * keep_points + obj.position_3d.x * obj_points) /
                             static_cast<float>(total_points);
        keep.position_3d.y = (old_pos.y * keep_points + obj.position_3d.y * obj_points) /
                             static_cast<float>(total_points);
        keep.position_3d.z = (old_pos.z * keep_points + obj.position_3d.z * obj_points) /
                             static_cast<float>(total_points);
        keep.distance_m = pointNorm2D(keep.position_3d);
        const float old_offset = pointDistance2D(old_pos, keep.position_3d);
        const float obj_offset = pointDistance2D(obj.position_3d, keep.position_3d);
        keep.lidar_cluster_radius_m = std::max(keep.lidar_cluster_radius_m + old_offset,
                                               obj.lidar_cluster_radius_m + obj_offset);
        keep.lidar_point_count = total_points;
        keep.confidence = std::max(keep.confidence, obj.confidence);
        if (obj.orsm_semantic_anchor_id >= 0 && keep.orsm_semantic_anchor_id < 0) {
            keep.bbox = obj.bbox;
            keep.source_camera = obj.source_camera;
            keep.orsm_visual_track_id = obj.orsm_visual_track_id;
        }
        if (obj.has_visual_ground_position && !keep.has_visual_ground_position) {
            keep.has_visual_ground_position = true;
            keep.visual_ground_position_3d = obj.visual_ground_position_3d;
            keep.visual_ground_distance_m = obj.visual_ground_distance_m;
        }
        if (keep.orsm_visual_track_id < 0) keep.orsm_visual_track_id = obj.orsm_visual_track_id;
        merged_by_anchor++;
    }

    local_id = 1;
    for (auto& obj : output) {
        obj.track_id = 800000000 + local_id++;
    }

    if (cfg_.orsm_debug_track || !output.empty()) {
        const int semantic_count = static_cast<int>(std::count_if(
            output.begin(), output.end(),
            [](const TrackedObject& obj) { return orsmBiologicalLabel(obj.label); }));
        std::cout << "[ORSMLidarPrimary] raw_points=" << lidar.points.size()
                  << " seeds=" << seeds.size()
                  << " front_ground_rejected=" << front_ground_plane_rejected_points
                  << " clusters=" << clusters.size()
                  << " raw_candidates=" << raw_output.size()
                  << " candidates=" << output.size()
                  << " semantic=" << semantic_count
                  << " anchor_merged=" << merged_by_anchor
                  << std::endl;
    }
    return output;
}

std::vector<TrackedObject> SentinelNode::dedupeSentryGroupCandidates(
    const std::vector<TrackedObject>& candidates) const {
    auto merged = dedupeOrsmGroupCandidates(candidates, "SentryGroupDedupe");
    if (!merged.empty() || candidates.empty()) return merged;

    // If a visual candidate has no usable vehicle-frame geometry, keep the best stable
    // per-camera tracks instead of publishing nothing.  These remain internal AlertManager
    // inputs; externally published status still happens only when AlertManager confirms a warning.
    std::vector<TrackedObject> fallback;
    fallback.reserve(candidates.size());
    for (const auto& obj : candidates) {
        if (obj.track_id < 0) continue;
        fallback.push_back(obj);
    }
    std::sort(fallback.begin(), fallback.end(), [](const TrackedObject& a, const TrackedObject& b) {
        if (a.age != b.age) return a.age > b.age;
        return a.confidence > b.confidence;
    });
    if (fallback.size() > 4) fallback.resize(4);
    return fallback;
}

std::vector<TrackedObject> SentinelNode::updateOrsmGlobalTracks(const std::vector<TrackedObject>& camera_tracks,
                                                                 const LidarFrame& lidar,
                                                                 const std::string& camera_name,
                                                                 uint64_t timestamp_ns) {
    // ORSM 输出必须是车体坐标系下的全局入侵物列表，而不是当前相机局部 tracks。
    // 这里把四路相机轮流到达的目标聚合为稳定 intruder_id，并对位置/速度做轻量平滑：
    // 1) 优先保留 LiDAR 融合坐标；LiDAR 短暂缺失时不立刻回退视觉粗估，避免位置跳变。
    // 2) 用车体坐标空间门限做跨相机关联，吸收不同 camera tracker_id 不一致。
    // 3) 输出前保留短时目标，连续丢失后再删除，避免 intruders_num 逐帧闪变。
    const uint64_t kTrackHoldNs =
        static_cast<uint64_t>(std::max(100, cfg_.orsm_track_lost_timeout_ms)) *
        1000000ULL;
    const uint64_t kTrackPruneNs =
        static_cast<uint64_t>(
            std::max(cfg_.orsm_track_lost_timeout_ms + 100,
                     cfg_.orsm_track_prune_timeout_ms)) *
        1000000ULL;
    constexpr uint64_t kRecentLidarNs = 2500ULL * 1000000ULL;
    constexpr uint64_t kRecentSemanticNs = 1500ULL * 1000000ULL;
    constexpr uint64_t kDynamicMotionWindowNs = 1500ULL * 1000000ULL;
    constexpr uint64_t kDynamicMotionMinDurationNs = 600ULL * 1000000ULL;
    constexpr float kDynamicSemanticHoldMinDisplacementM = 0.75f;
    constexpr float kDynamicSemanticHoldMinStraightness = 0.55f;
    constexpr float kMaxReasonableSpeedMps = 12.0f;
    const int kMinOutputHits = std::max(1, cfg_.orsm_track_stable_min_observations);
    const int kMinLidarOutputHits =
        std::max(1, cfg_.orsm_track_stable_lidar_min_observations);
    const uint64_t kMinOutputAgeNs =
        static_cast<uint64_t>(std::max(0, cfg_.orsm_track_stable_min_age_ms)) *
        1000000ULL;
    const float kStableMaxUncertaintyM =
        std::max(0.25f, cfg_.orsm_track_stable_max_uncertainty_m);
    const uint64_t lidar_continuation_max_age_ns =
        static_cast<uint64_t>(
            std::max(0, cfg_.orsm_lidar_continuation_max_age_ms)) *
        1000000ULL;
    const uint64_t near_static_visual_refresh_ns =
        static_cast<uint64_t>(std::max(0, cfg_.orsm_near_static_visual_refresh_ms)) *
        1000000ULL;
    const float far_dynamic_min_distance_m =
        std::max(0.0f, cfg_.orsm_far_dynamic_min_distance_m);
    const bool debug_track = cfg_.orsm_debug_track || std::getenv("ORSM_DEBUG_TRACK") != nullptr;
    // Narrow offline diagnostic: unlike ORSM_DEBUG_PUBLISH_GATE this emits
    // only one algorithm-owned LiDAR track and therefore does not perturb the
    // real-time replay schedule with thousands of log lines.
    int debug_lidar_track_id = -1;
    if (const char* value = std::getenv("ORSM_DEBUG_LIDAR_TRACK_ID")) {
        debug_lidar_track_id = std::atoi(value);
    }

    if (timestamp_ns == 0) return camera_tracks;

    auto tracePersistentBinding = [&](int temporal_id, int global_id,
                                      const char* event, const char* reason) {
        std::ostringstream line;
        line << "[ORSMOfflineTraceBinding] ts=" << timestamp_ns
             << " L=" << temporal_id << " global=" << global_id
             << " event=" << event << " reason=" << reason;
        std::lock_guard<std::mutex> lock(orsm_binding_trace_mutex_);
        pending_orsm_binding_trace_events_.push_back(line.str());
    };

    std::vector<TrackedObject> lidar_primary_tracks;
    // 相机 YOLO 候选不再是轨迹主体：它们只在 buildLidarPrimary 内部作为锚点
    // 给簇递标签，递完即消。候选队列里只剩带 L# 编号的雷达候选。
    // 无 LiDAR 帧时队列为空，轨迹的剪枝与发布流程照常运行。
    static const std::vector<TrackedObject> kEmptyCandidates;
    const std::vector<TrackedObject>* source_tracks = &kEmptyCandidates;
    if (lidar.valid()) {
        lidar_primary_tracks = buildLidarPrimaryOrsmCandidates(lidar, camera_tracks, timestamp_ns);
        associateOrsmLidarTemporalTracks(lidar_primary_tracks, timestamp_ns);
        source_tracks = &lidar_primary_tracks;

        // Write the algorithm-owned temporal LiDAR identity separately from
        // the per-scan diagnostic cluster index.  Offline renderers consume
        // this mapping only; it does not affect fusion, lifecycle or alerts.
        if (std::getenv("ORSM_LIDAR_CLUSTER_DEBUG") != nullptr) {
            std::cout << "[ORSMLidarClusterDiag] ts=" << timestamp_ns
                      << " candidates=" << lidar_primary_tracks.size() << std::endl;
            for (size_t index = 0; index < lidar_primary_tracks.size(); ++index) {
                const auto& candidate = lidar_primary_tracks[index];
                std::cout << "[ORSMLidarClusterDiag] ts=" << timestamp_ns
                          << " cluster=" << index
                          << " center=(" << candidate.position_3d.x << ','
                          << candidate.position_3d.y << ',' << candidate.position_3d.z << ')'
                          << " radius=" << candidate.lidar_cluster_radius_m
                          << " points=" << candidate.lidar_point_count
                          << " semantic=" << (orsmBiologicalLabel(candidate.label) ? 1 : 0)
                          << " label=" << candidate.label
                          << std::endl;
            }
            for (const auto& candidate : lidar_primary_tracks) {
                if (candidate.orsm_lidar_temporal_track_id < 0) continue;
                std::cout << "[ORSMLidarTemporalMap] ts=" << timestamp_ns
                          << " L=" << candidate.orsm_lidar_temporal_track_id
                          << " center=(" << candidate.position_3d.x << ','
                          << candidate.position_3d.y << ")"
                          << std::endl;
            }
        }
    }
    const bool using_lidar_primary = !lidar_primary_tracks.empty();

    std::unordered_set<int> updated_track_ids;

    auto candidatePosition = [](const TrackedObject& obj) {
        // Keep the candidate position and distance on the same measurement
        // source.  In particular, a far LiDAR cluster that happens to project
        // into a near visual/difference box must stay at its LiDAR position;
        // replacing only its position with visual_ground_position_3d creates a
        // contradictory near position + far range and spawns ghost globals.
        return obj.position_3d;
    };
    auto sourceTrackKey = [&](const TrackedObject& obj) {
        // The local cluster index is regenerated every scan.  Prefer the
        // temporal LiDAR identity so a semantically confirmed physical cluster
        // retains one global ORSM owner across camera seams and visual misses.
        if (obj.orsm_lidar_temporal_track_id >= 0) {
            return 700000000 + obj.orsm_lidar_temporal_track_id;
        }
        if (obj.orsm_visual_track_id >= 0) return obj.orsm_visual_track_id;
        return obj.track_id;
    };
    auto refreshTrackLifecycle = [&](OrsmGlobalTrack& track) {
        if (timestamp_ns == 0 || track.first_seen_ts_ns == 0) return;
        if (track.last_seen_ts_ns > 0 &&
            timestamp_ns > track.last_seen_ts_ns &&
            timestamp_ns - track.last_seen_ts_ns > kTrackHoldNs) {
            track.lifecycle = OrsmTrackLifecycle::Lost;
            return;
        }
        if (track.lifecycle == OrsmTrackLifecycle::Stable) return;
        const uint64_t age_ns =
            timestamp_ns > track.first_seen_ts_ns
                ? timestamp_ns - track.first_seen_ts_ns
                : 0;
        const bool stable_by_observation =
            track.hits >= kMinOutputHits ||
            track.lidar_hits >= kMinLidarOutputHits;
        if (age_ns >= kMinOutputAgeNs &&
            stable_by_observation &&
            track.position_uncertainty_m <= kStableMaxUncertaintyM &&
            track.distance_m <= cfg_.orsm_zone_a_radius) {
            track.lifecycle = OrsmTrackLifecycle::Stable;
            if (track.stable_since_ts_ns == 0) track.stable_since_ts_ns = timestamp_ns;
            if (debug_track) {
                std::cout << "[ORSMTrackLifecycle] action=stable"
                          << " id=" << track.stable_id
                          << " hits=" << track.hits
                          << " lidar_hits=" << track.lidar_hits
                          << " age_ms=" << age_ns / 1000000ULL
                          << " uncertainty=" << track.position_uncertainty_m
                          << std::endl;
            }
        }
    };
    // YOLO semantic confirmation and Inbox/difference-box confirmation renew
    // the same LiDAR publication lease.  Distance eligibility belongs to the
    // grant path (the difference-box path already enforces its far-distance
    // threshold); applying that threshold again here incorrectly disables a
    // valid YOLO lease for near targets.
    auto hasLidarPublicationLease = [&](const OrsmGlobalTrack& track) {
        return track.lidar_publication_lease_until_ts_ns > 0 &&
               timestamp_ns <= track.lidar_publication_lease_until_ts_ns;
    };
    // lidar-only 发布资格：无语义标签、LiDAR 观测新鲜、累计观测足够、平滑速度
    // 合理，且运动窗口内相对最新观测的净位移达到阈值（真实运动证据）。距离带
    // 分两段：近场(≤near_output_max_distance)由新开关控制，远场(≥far_output
    // min_distance)沿用既有开关。静止背景由静态背景滤除与位移门共同排除。
    auto lidarOnlyOutputEligible = [&](int temporal_id, float distance_m) -> bool {
        if (temporal_id < 0) return false;
        const bool near_enabled = cfg_.orsm_lidar_only_near_output_enabled;
        const bool far_enabled = cfg_.orsm_lidar_only_far_output_enabled;
        if (!near_enabled && !far_enabled) return false;
        const auto it = orsm_lidar_temporal_tracks_.find(temporal_id);
        if (it == orsm_lidar_temporal_tracks_.end()) return false;
        const auto& temporal = it->second;
        if (orsmBiologicalLabel(temporal.label)) return false;
        if (timestamp_ns > temporal.last_seen_ts_ns &&
            timestamp_ns - temporal.last_seen_ts_ns > 350ULL * 1000000ULL) {
            return false; // LiDAR 观测已失效，防止冻结位置持续发布
        }
        if (temporal.hits < std::max(1, cfg_.orsm_lidar_only_output_min_observations)) {
            return false;
        }
        if (pointNorm2D(temporal.velocity) >
            std::max(0.1f, cfg_.orsm_lidar_only_output_max_speed_mps)) {
            return false;
        }
        const uint64_t window_ns =
            static_cast<uint64_t>(std::max(1000, cfg_.orsm_lidar_only_output_motion_window_ms)) *
            1000000ULL;
        // 用“窗口内最旧样本→最新样本”的净位移，而非任意样本最大位移：
        // 远距离簇中心偶发单帧跳变会被最大位移口径误判为真实运动。
        float net_displacement_m = 0.f;
        uint64_t window_span_ns = 0;
        if (!temporal.output_motion_history.empty()) {
            const auto& newest = temporal.output_motion_history.back();
            // 历史按时间戳升序：首个早于 newest 的样本即窗口内最旧样本。
            for (const auto& sample : temporal.output_motion_history) {
                if (newest.first > sample.first) {
                    net_displacement_m =
                        pointDistance2D(sample.second, newest.second);
                    window_span_ns = newest.first - sample.first;
                    break;
                }
            }
        }
        // 至少半个窗口的持续历史才参与判定，避免两三帧的短历史侥幸过关。
        if (window_span_ns < window_ns / 2) {
            return false;
        }
        // 位移门按距离缩放：远距离簇中心抖动与聚类不稳定随距离放大
        //（与语义关联门 0.12*distance 的抖动启发式一致），固定 0.6m 门在
        // 25m 外会被停车场的抖动“假运动”击穿。
        const float scaled_min_motion_m = std::max(
            std::max(0.05f, cfg_.orsm_lidar_only_output_min_motion_m),
            0.12f * std::max(0.f, distance_m));
        if (net_displacement_m < scaled_min_motion_m) {
            return false;
        }
        const bool near_ok = near_enabled &&
            distance_m > 0.f &&
            distance_m <= std::max(0.f, cfg_.orsm_lidar_only_near_output_max_distance_m);
        const float far_max_m = std::max(0.f, cfg_.orsm_lidar_only_far_output_max_distance_m);
        const bool far_ok = far_enabled &&
            distance_m >= std::max(0.f, cfg_.orsm_lidar_only_far_output_min_distance_m) &&
            (far_max_m <= 0.f || distance_m <= far_max_m);
        return near_ok || far_ok;
    };
    auto isPersonVehiclePair = [](int lhs, int rhs) {
        return (lhs == 0 && rhs == 2) || (lhs == 2 && rhs == 0);
    };

   auto hasActivePublicationLease = [&](const OrsmGlobalTrack& track) {
        return track.lidar_publication_lease_until_ts_ns > 0 &&
               timestamp_ns <= track.lidar_publication_lease_until_ts_ns;
    };
    auto isPublishingLifecycleTrack = [&](const OrsmGlobalTrack& track) {
        if (track.lifecycle != OrsmTrackLifecycle::Stable) return false;
        if (track.last_seen_ts_ns == 0 || timestamp_ns < track.last_seen_ts_ns) return true;
        return timestamp_ns - track.last_seen_ts_ns <= kTrackHoldNs;
    };

    for (size_t i = 0; i < source_tracks->size(); ++i) {
        const auto& obj = (*source_tracks)[i];
        if (obj.track_id < 0 || obj.distance_m <= 0.f) continue;
        const cv::Point3f pos = candidatePosition(obj);
        if (!finitePoint2D(pos)) continue;
        const bool yolo_claimed_this_frame =
            orsmBiologicalLabel(obj.label) &&
            obj.origin == ObservationOrigin::OriginLidarCluster &&
            obj.lidar_claim == LidarClaim::LidarClaimYolo;
        const int source_track_key = sourceTrackKey(obj);
        const std::string candidate_camera = obj.source_camera.empty()
            ? camera_name
            : obj.source_camera;
        const auto motion_temporal_it = orsm_lidar_temporal_tracks_.find(
            obj.orsm_lidar_temporal_track_id);
        const uint64_t publication_lease_until =
            motion_temporal_it != orsm_lidar_temporal_tracks_.end()
            ? motion_temporal_it->second.lidar_publication_lease_until_ts_ns : 0;
        const bool lidar_publication_lease_active =
            publication_lease_until > 0 && timestamp_ns <= publication_lease_until;
        // lidar-only 合格轨迹与持租约轨迹同样具备“出生/认领”资格：否则合格
        // 裸簇会在 L# 层内空转（bare_cluster_waits_at_lidar），永远到不了发布闸门。
        const bool lidar_only_output_candidate =
            lidarOnlyOutputEligible(obj.orsm_lidar_temporal_track_id, obj.distance_m);
        const bool lease_bound_lidar_candidate =
            obj.orsm_lidar_temporal_track_id >= 0 &&
            (lidar_publication_lease_active || lidar_only_output_candidate);
        const bool lidar_owner_candidate = lease_bound_lidar_candidate;
        if (lease_bound_lidar_candidate &&
            cfg_.orsm_lidar_semantic_latch_max_active_targets > 0) {
            int other_active_owners = 0;
            for (const auto& [global_id, existing] : orsm_global_tracks_) {
                (void)global_id;
                if (existing.lidar_temporal_id < 0 ||
                    existing.lidar_temporal_id ==
                        obj.orsm_lidar_temporal_track_id ||
                    existing.lidar_publication_lease_until_ts_ns == 0 ||
                    timestamp_ns > existing.lidar_publication_lease_until_ts_ns) {
                    continue;
                }
                ++other_active_owners;
            }
            if (other_active_owners >=
                cfg_.orsm_lidar_semantic_latch_max_active_targets) {
                tracePersistentBinding(obj.orsm_lidar_temporal_track_id, 0,
                                       "reject", "physical_conflict");
                continue;
            }
        }
        int bound_global_id = 0;
        if (lidar_owner_candidate) {
            const auto temporal_it = orsm_lidar_temporal_tracks_.find(
                obj.orsm_lidar_temporal_track_id);
            if (temporal_it != orsm_lidar_temporal_tracks_.end()) {
                bound_global_id = temporal_it->second.bound_global_track_id;
                if (bound_global_id != 0 &&
                    orsm_global_tracks_.count(bound_global_id) == 0) {
                    tracePersistentBinding(obj.orsm_lidar_temporal_track_id,
                                           bound_global_id, "unbind",
                                           "temporal_track_lost");
                    temporal_it->second.bound_global_track_id = 0;
                    bound_global_id = 0;
                }
            }
        }

        int best_id = 0;
        // 优先复用 L# 已绑定的全局轨迹。
        if (bound_global_id != 0) {
            best_id = bound_global_id;
        }

        // 新建全局轨迹需要活跃租约。无租约 LiDAR 簇仅在 L# 层跟踪；满足稳定性、
        // 运动和距离条件的 lidar-only 轨迹可通过配置允许的通道直接新建。
        if (best_id == 0 && !lidar_publication_lease_active &&
            !lidar_only_output_candidate) {
            if (debug_lidar_track_id >= 0 &&
                obj.orsm_lidar_temporal_track_id == debug_lidar_track_id) {
                const auto dbg_it = orsm_lidar_temporal_tracks_.find(
                    obj.orsm_lidar_temporal_track_id);
                if (dbg_it != orsm_lidar_temporal_tracks_.end()) {
                    const auto& tt = dbg_it->second;
                    uint64_t dbg_span = 0; float dbg_net = 0.f;
                    if (!tt.output_motion_history.empty()) {
                        const auto& nw = tt.output_motion_history.back();
                        for (const auto& s : tt.output_motion_history) {
                            if (nw.first > s.first) {
                                dbg_net = pointDistance2D(s.second, nw.second);
                                dbg_span = nw.first - s.first;
                                break;
                            }
                        }
                    }
                    std::cout << "[ORSMTargetLidarOnly] ts=" << timestamp_ns
                              << " L=" << tt.stable_id
                              << " hits=" << tt.hits
                              << " vel=" << pointNorm2D(tt.velocity)
                              << " hist_n=" << tt.output_motion_history.size()
                              << " span_ms=" << dbg_span / 1000000ULL
                              << " net=" << dbg_net
                              << " dist=" << obj.distance_m
                              << " near_en=" << cfg_.orsm_lidar_only_near_output_enabled
                              << " last_seen_dt_ms="
                              << (timestamp_ns > tt.last_seen_ts_ns
                                      ? (timestamp_ns - tt.last_seen_ts_ns) / 1000000ULL : 0)
                              << std::endl;
                }
            }
            if (debug_track) {
                std::cout << "[ORSMTrackDiag] action=bare_cluster_waits_at_lidar"
                          << " L=" << obj.orsm_lidar_temporal_track_id
                          << " camera=" << candidate_camera
                          << " dist=" << obj.distance_m
                          << std::endl;
            }
            continue;
        }

        if (best_id == 0) {
            OrsmGlobalTrack track;
            track.stable_id = next_orsm_global_track_id_++;
            // A real class may be adopted at birth; a different one can replace
            // it later only with a strictly higher confidence, so starting with
            // a weak label is self-correcting rather than permanent.
            const bool born_with_class =
                obj.class_id >= 0 && !isUnknownOrsmLabel(obj.label, obj.class_id);
            track.label = born_with_class ? obj.label : "unknown";
            track.class_id = born_with_class ? obj.class_id : -1;
            track.class_confidence = born_with_class ? obj.confidence : 0.f;
            track.bbox = obj.bbox;
            track.confidence = obj.confidence;
            track.position = pos;
            track.distance_m = obj.distance_m > 0.f ? obj.distance_m : pointNorm2D(pos);
            track.velocity = obj.velocity_3d;
            track.speed_mps = pointNorm2D(track.velocity);
            if (!std::isfinite(track.speed_mps) || track.speed_mps > kMaxReasonableSpeedMps) {
                track.velocity = cv::Point3f{0.f, 0.f, 0.f};
                track.speed_mps = 0.f;
            }
            track.position_uncertainty_m = obj.has_lidar_fusion ? 0.35f : 0.60f;
            track.has_lidar_recent = obj.has_lidar_fusion;
            track.has_lidar_tracking_anchor = obj.has_lidar_fusion;
            if (track.has_lidar_tracking_anchor) {
                track.lidar_tracking_anchor = pos;
            }
            if (lease_bound_lidar_candidate) {
                track.lidar_temporal_id =
                    obj.orsm_lidar_temporal_track_id;
            }
            if (lidar_publication_lease_active) {
                track.lidar_publication_lease_until_ts_ns =
                    publication_lease_until;
            }
            track.last_lidar_ts_ns = obj.has_lidar_fusion ? timestamp_ns : 0;
            track.last_lidar_point_count = obj.has_lidar_fusion ? obj.lidar_point_count : 0;
            track.last_lidar_cluster_radius_m = obj.has_lidar_fusion
                ? obj.lidar_cluster_radius_m : 0.f;
            track.first_seen_ts_ns = timestamp_ns;
            track.last_seen_ts_ns = timestamp_ns;
            track.last_camera_seen_ts_ns = yolo_claimed_this_frame ? timestamp_ns : 0;
            track.hits = 1;
            track.lidar_hits = obj.has_lidar_fusion ? 1 : 0;
            if (lease_bound_lidar_candidate) {
                // An active lease is already the publication credential; do not
                // require a second global-layer warm-up.
                track.lifecycle = OrsmTrackLifecycle::Stable;
                track.stable_since_ts_ns = timestamp_ns;
            }
            track.last_source_track_id = source_track_key;
            track.last_camera = candidate_camera;
            best_id = track.stable_id;
            orsm_global_tracks_[track.stable_id] = std::move(track);
            if (lidar_owner_candidate) {
                auto& temporal = orsm_lidar_temporal_tracks_.at(
                    obj.orsm_lidar_temporal_track_id);
                temporal.bound_global_track_id = best_id;
                tracePersistentBinding(obj.orsm_lidar_temporal_track_id,
                                       best_id, "bind", "lease_created");
            }
            refreshTrackLifecycle(orsm_global_tracks_[best_id]);
            if (debug_track) {
                std::cout << "[ORSMTrackDiag] action=new"
                          << " camera=" << camera_name
                          << " obj_track=" << obj.track_id
                          << " new_id=" << best_id
                          << " pos=(" << pos.x << "," << pos.y << ")"
                          << " dist=" << obj.distance_m
                          << " lidar=" << obj.has_lidar_fusion
                          << " class_id=" << obj.class_id
                          << std::endl;
            }
        } else {
            auto& track = orsm_global_tracks_[best_id];
            if (lidar_publication_lease_active) {
                track.lidar_publication_lease_until_ts_ns =
                    publication_lease_until;
            }
            if (lease_bound_lidar_candidate) {
                if (track.lidar_temporal_id >= 0 &&
                    track.lidar_temporal_id !=
                        obj.orsm_lidar_temporal_track_id) {
                    tracePersistentBinding(obj.orsm_lidar_temporal_track_id,
                                           best_id, "reject", "physical_conflict");
                    continue;
                }
                if (track.lidar_temporal_id < 0) {
                    track.lidar_temporal_id =
                        obj.orsm_lidar_temporal_track_id;
                    auto& temporal = orsm_lidar_temporal_tracks_.at(
                        obj.orsm_lidar_temporal_track_id);
                    temporal.bound_global_track_id = best_id;
                    tracePersistentBinding(obj.orsm_lidar_temporal_track_id,
                                           best_id, "bind", "claimed");
                }
            }
            if (lease_bound_lidar_candidate) {
                auto& temporal = orsm_lidar_temporal_tracks_.at(
                    obj.orsm_lidar_temporal_track_id);
                temporal.bound_global_track_id = best_id;
                tracePersistentBinding(obj.orsm_lidar_temporal_track_id,
                                       best_id, "bind", "lease_claimed");
            }
            if (debug_track) {
                std::cout << "[ORSMTrackDiag] action=match"
                          << " camera=" << camera_name
                          << " obj_track=" << obj.track_id
                          << " id=" << best_id
                          << " obj_pos=(" << pos.x << "," << pos.y << ")"
                          << " track_pos=(" << track.position.x << "," << track.position.y << ")"
                          << " lidar=" << obj.has_lidar_fusion
                          << " class_id=" << obj.class_id
                          << " track_class_id=" << track.class_id
                          << std::endl;
            }
            // 对齐重写（R2）：全局轨迹不自行融合——位置/速度/类别全部由其
            // 绑定的 L# 投影。雷达候选的测量值写入 L#（时序关联层已完成），
            // 这里只做单向拷贝。发布闸门读取的全局状态与 L# 永远一致。
            if (obj.orsm_lidar_temporal_track_id >= 0) {
                auto& temporal =
                    orsm_lidar_temporal_tracks_.at(obj.orsm_lidar_temporal_track_id);
                if (finitePoint2D(temporal.position)) {
                    track.position = temporal.position;
                    track.distance_m = pointNorm2D(track.position);
                }
                if (finitePoint2D(temporal.velocity) &&
                    pointNorm2D(temporal.velocity) <= kMaxReasonableSpeedMps) {
                    track.velocity = temporal.velocity;
                    track.speed_mps = pointNorm2D(track.velocity);
                }
                if (temporal.class_id >= 0 &&
                    !isUnknownOrsmLabel(temporal.label, temporal.class_id)) {
                    track.class_id = temporal.class_id;
                    track.label = temporal.label;
                    track.class_confidence = temporal.class_confidence;
                }
                if (temporal.last_seen_ts_ns == timestamp_ns) {
                    track.last_lidar_ts_ns = timestamp_ns;
                    track.last_lidar_point_count = obj.lidar_point_count;
                    track.last_lidar_cluster_radius_m = obj.lidar_cluster_radius_m;
                    track.has_lidar_recent = true;
                    track.has_lidar_tracking_anchor = true;
                    track.lidar_tracking_anchor = track.position;
                    track.lidar_hits++;
                }
            }
            if (yolo_claimed_this_frame) {
                track.last_camera_seen_ts_ns = timestamp_ns;
                track.bbox = obj.bbox;
            }
            track.confidence = std::max(track.confidence * 0.9f, obj.confidence);
            track.last_seen_ts_ns = timestamp_ns;
            track.hits++;
            if (lease_bound_lidar_candidate) {
                track.lifecycle = OrsmTrackLifecycle::Stable;
                if (track.stable_since_ts_ns == 0) track.stable_since_ts_ns = timestamp_ns;
            }
            refreshTrackLifecycle(track);
        }

        updated_track_ids.insert(best_id);
        if (obj.orsm_lidar_temporal_track_id == debug_lidar_track_id) {
            std::cout << "[ORSMTargetLidarTrack] ts=" << timestamp_ns
                      << " L=" << obj.orsm_lidar_temporal_track_id
                      << " global=" << best_id
                      << " source=" << source_track_key
                      << " pos=(" << pos.x << ',' << pos.y << ')'
                      << " semantic=" << (yolo_claimed_this_frame ? 1 : 0)
                      << " lidar=" << (obj.has_lidar_fusion ? 1 : 0)
                      << std::endl;
        }
    }

    // YOLO/差值框仅授予 L# 发布资格，发布位置仍由 LiDAR 身份维护。
    // 租约有效时，每帧 L# 观测均同步到已绑定的全局轨迹。
    for (const auto& [temporal_id, temporal] : orsm_lidar_temporal_tracks_) {
        if (temporal.bound_global_track_id == 0 ||
            temporal.lidar_publication_lease_until_ts_ns == 0 ||
            timestamp_ns > temporal.lidar_publication_lease_until_ts_ns ||
            temporal.last_seen_ts_ns != timestamp_ns ||
            !finitePoint2D(temporal.position)) {
            continue;
        }

        const auto global_it = orsm_global_tracks_.find(
            temporal.bound_global_track_id);
        if (global_it == orsm_global_tracks_.end()) continue;

        auto& global = global_it->second;
        const bool owns_global_position =
            global.lidar_temporal_id == temporal_id ||
            (global.lidar_temporal_id < 0 &&
             global.last_source_track_id == 700000000 + temporal_id);
        if (!owns_global_position) continue;

        // 类别权威在 L#：L# 上以单调规则确立的类别直接投影到全局轨迹，
        // 全局层不再对有 L# 绑定的轨迹做独立的类别替换。
        if (temporal.class_id >= 0 &&
            !isUnknownOrsmLabel(temporal.label, temporal.class_id)) {
            global.class_id = temporal.class_id;
            global.label = temporal.label;
            global.class_confidence = temporal.class_confidence;
        }
        global.position = temporal.position;
        global.distance_m = pointNorm2D(global.position);
        if (finitePoint2D(temporal.velocity)) {
            const float speed_mps = pointNorm2D(temporal.velocity);
            if (std::isfinite(speed_mps) &&
                speed_mps <= kMaxReasonableSpeedMps) {
                global.velocity = temporal.velocity;
                global.speed_mps = speed_mps;
            }
        }
        global.last_lidar_ts_ns = temporal.last_seen_ts_ns;
        global.has_lidar_recent = true;
        global.has_lidar_tracking_anchor = true;
        global.lidar_tracking_anchor = temporal.position;
        global.lidar_publication_lease_until_ts_ns = std::max(
            global.lidar_publication_lease_until_ts_ns,
            temporal.lidar_publication_lease_until_ts_ns);
        updated_track_ids.insert(global.stable_id);

        if (debug_track) {
            std::cout << "[ORSMLeasePositionSync] ts=" << timestamp_ns
                      << " L=" << temporal_id
                      << " global=" << global.stable_id
                      << " pos=(" << global.position.x << ','
                      << global.position.y << ')'
                      << " lease_until="
                      << global.lidar_publication_lease_until_ts_ns
                      << std::endl;
        }
    }


    std::vector<int> to_remove;
    for (const auto& [id, track] : orsm_global_tracks_) {
        if (track.distance_m > cfg_.orsm_zone_a_radius + 0.5f) {
            to_remove.push_back(id);
            continue;
        }
        if (timestamp_ns > track.last_seen_ts_ns) {
            const uint64_t stale_ns = timestamp_ns - track.last_seen_ts_ns;
            const bool bound_lidar_temporal_alive =
                track.lidar_temporal_id >= 0 &&
                orsm_lidar_temporal_tracks_.count(
                    track.lidar_temporal_id) != 0;
            if (stale_ns > kTrackPruneNs) {
                if (bound_lidar_temporal_alive) {
                    auto it = orsm_global_tracks_.find(id);
                    if (it != orsm_global_tracks_.end()) {
                        it->second.lifecycle = OrsmTrackLifecycle::Lost;
                    }
                } else {
                    if (track.lidar_temporal_id >= 0) {
                        tracePersistentBinding(track.lidar_temporal_id,
                                               id, "stop", "temporal_track_lost");
                    }
                    to_remove.push_back(id);
                }
            } else if (stale_ns > kTrackHoldNs) {
                auto it = orsm_global_tracks_.find(id);
                if (it != orsm_global_tracks_.end()) {
                    it->second.lifecycle = OrsmTrackLifecycle::Lost;
                }
            }
        }
    }
    for (int id : to_remove) orsm_global_tracks_.erase(id);

    // 发布轨迹必须具有 L# 绑定或活跃租约，因此不再执行全局轨迹合并。
    // 物理目标重复由输出层和 single_orsm_manager 去重，其余轨迹由剪枝清理。

    std::vector<TrackedObject> fused;
    fused.reserve(orsm_global_tracks_.size());
    const bool debug_publish_gate = std::getenv("ORSM_DEBUG_PUBLISH_GATE") != nullptr;
    for (auto& [id, track] : orsm_global_tracks_) {
        auto reject = [&](const char* reason) {
            const bool target_lidar_track = debug_lidar_track_id >= 0 &&
                track.last_source_track_id == 700000000 + debug_lidar_track_id;
            if (!debug_publish_gate && !target_lidar_track) return;
            std::cout << (target_lidar_track ? "[ORSMTargetLidarGate]" : "[ORSMGlobalPublishGate]")
                      << " ts=" << timestamp_ns
                      << " global=" << track.stable_id
                      << " source=" << track.last_source_track_id
                      << " reason=" << reason
                      << " label=" << track.label
                      << " hits=" << track.hits << " lidar_hits=" << track.lidar_hits
                      << " dist=" << track.distance_m << std::endl;
        };
        if (timestamp_ns > 0 && finitePoint2D(track.position)) {
            track.motion_history.emplace_back(timestamp_ns, track.position);
            while (!track.motion_history.empty() &&
                   timestamp_ns >= track.motion_history.front().first &&
                   timestamp_ns - track.motion_history.front().first > kDynamicMotionWindowNs) {
                track.motion_history.pop_front();
            }
        }

        bool dynamic_motion_confirmed = false;
        bool motion_history_observed = false;
        float motion_net_displacement_m = 0.f;
        float motion_path_length_m = 0.f;
        float motion_straightness = 0.f;
        if (track.motion_history.size() >= 2 && timestamp_ns >= track.motion_history.front().first) {
            const uint64_t history_duration_ns = timestamp_ns - track.motion_history.front().first;
            for (size_t i = 1; i < track.motion_history.size(); ++i) {
                motion_path_length_m += pointDistance2D(track.motion_history[i - 1].second,
                                                       track.motion_history[i].second);
            }
            motion_net_displacement_m = pointDistance2D(track.motion_history.front().second,
                                                        track.motion_history.back().second);
            motion_straightness = motion_path_length_m > 1e-3f
                ? motion_net_displacement_m / motion_path_length_m
                : 0.f;
            motion_history_observed = history_duration_ns >= kDynamicMotionMinDurationNs;
            dynamic_motion_confirmed =
                history_duration_ns >= kDynamicMotionMinDurationNs &&
                motion_net_displacement_m >= kDynamicSemanticHoldMinDisplacementM &&
                motion_straightness >= kDynamicSemanticHoldMinStraightness;
            if (dynamic_motion_confirmed) {
                track.last_dynamic_motion_confirmed_ts_ns = timestamp_ns;
            }
        }
        const bool recently_confirmed_dynamic =
            track.last_dynamic_motion_confirmed_ts_ns > 0 &&
            timestamp_ns >= track.last_dynamic_motion_confirmed_ts_ns &&
            (lidar_continuation_max_age_ns == 0 ||
             timestamp_ns - track.last_dynamic_motion_confirmed_ts_ns <=
                 lidar_continuation_max_age_ns);

        const bool lidar_publication_lease_active =
            hasLidarPublicationLease(track);
        // A valid Inbox/LiDAR lease is already an explicit publication grant.
        // Do not add another global-track warm-up or recent-sensor gate here.
        if (track.lifecycle != OrsmTrackLifecycle::Stable &&
            !lidar_publication_lease_active) {
            reject("not_stable");
            continue;
        }
        if (!lidar_publication_lease_active && timestamp_ns > track.last_seen_ts_ns &&
            timestamp_ns - track.last_seen_ts_ns > kTrackHoldNs) {
            reject("track_hold_expired");
            continue;
        }
        // The offline single-target guard must apply at the final publication
        // boundary as well.  Otherwise an unrelated visual-only/global track
        // can bypass the L# admission guard and be published beside the active
        // semantic-latched LiDAR owner.
        if (cfg_.orsm_lidar_semantic_latch_max_active_targets > 0 &&
            track.lidar_temporal_id < 0) {
            int active_lease_owners = 0;
            for (const auto& [other_id, other] : orsm_global_tracks_) {
                (void)other_id;
                if (other.lidar_temporal_id >= 0 &&
                    other.lidar_publication_lease_until_ts_ns > 0 &&
                    timestamp_ns <= other.lidar_publication_lease_until_ts_ns) {
                    ++active_lease_owners;
                }
            }
            if (active_lease_owners >=
                cfg_.orsm_lidar_semantic_latch_max_active_targets) {
                reject("active_lidar_owner");
                continue;
            }
        }
        const bool recent_lidar = track.last_lidar_ts_ns > 0 && timestamp_ns >= track.last_lidar_ts_ns &&
            timestamp_ns - track.last_lidar_ts_ns <= kRecentLidarNs;
        // 可选的首次发布类别门槛。当前 build-bev 配置关闭该门槛，因此 15m 外
        // 的 unknown 差值框不会仅因缺少类别被拒绝；类别继承仍可作为增强信息。
        if (cfg_.orsm_alarm_first_publish_class_required &&
            !track.class_established_publication) {
            const float first_publish_max_m =
                cfg_.orsm_alarm_first_publish_max_distance_m > 0.f
                    ? cfg_.orsm_alarm_first_publish_max_distance_m
                    : std::numeric_limits<float>::max();
            const bool in_first_publish_band =
                track.distance_m >= cfg_.orsm_alarm_first_publish_min_distance_m &&
                track.distance_m <= first_publish_max_m;
            if (in_first_publish_band) {
                if (!orsmBiologicalLabel(track.label)) {
                    reject("first_publish_needs_class");
                    continue;
                }
                track.class_established_publication = true;
                if (debug_track) {
                    std::cout << "[ORSMAlarmFirstPublish] id=" << track.stable_id
                              << " dist=" << track.distance_m
                              << " label=" << track.label
                              << " evidence="
                              << (lidar_publication_lease_active ? "lease" : "other")
                              << std::endl;
                }
            }
        }
        // 语义年龄从绑定的 L# 读取：类别权威在 L#，全局不再自持语义账本。
        OrsmLidarTemporalTrack* bound_temporal = nullptr;
        if (track.lidar_temporal_id >= 0) {
            const auto temporal_it =
                orsm_lidar_temporal_tracks_.find(track.lidar_temporal_id);
            if (temporal_it != orsm_lidar_temporal_tracks_.end()) {
                bound_temporal = &temporal_it->second;
            }
        }
        const bool has_lidar_semantic_memory =
            bound_temporal != nullptr && bound_temporal->semantic_hits > 0 &&
            orsmBiologicalLabel(bound_temporal->label);
        const uint64_t semantic_age_ns =
            (has_lidar_semantic_memory && timestamp_ns >= bound_temporal->last_semantic_ts_ns)
                ? timestamp_ns - bound_temporal->last_semantic_ts_ns
                : 0;
        const uint64_t static_release_ns =
            static_cast<uint64_t>(std::max(0, cfg_.orsm_lidar_primary_static_semantic_release_ms)) *
            1000000ULL;
        const uint64_t effective_static_release_ns =
            track.distance_m < far_dynamic_min_distance_m
                ? near_static_visual_refresh_ns
                : static_release_ns;
        const float static_max_displacement_m =
            std::max(0.0f, cfg_.orsm_lidar_primary_static_semantic_max_displacement_m);
        if (recent_lidar && finitePoint2D(track.position)) {
            if (track.lidar_stationary_since_ts_ns == 0 ||
                !finitePoint2D(track.lidar_stationary_anchor) ||
                pointDistance2D(track.lidar_stationary_anchor, track.position) > static_max_displacement_m) {
                track.lidar_stationary_anchor = track.position;
                track.lidar_stationary_since_ts_ns = timestamp_ns;
            }
        } else {
            track.lidar_stationary_since_ts_ns = 0;
            track.lidar_stationary_anchor = track.position;
        }
        const bool lidar_smoothed_stationary_timeout =
            effective_static_release_ns > 0 &&
            recent_lidar &&
            track.lidar_stationary_since_ts_ns > 0 &&
            timestamp_ns >= track.lidar_stationary_since_ts_ns &&
            timestamp_ns - track.lidar_stationary_since_ts_ns >= effective_static_release_ns;
        const bool stale_static_lidar_semantic =
            using_lidar_primary &&
            effective_static_release_ns > 0 &&
            has_lidar_semantic_memory &&
            semantic_age_ns >= effective_static_release_ns &&
            recent_lidar &&
            (!recently_confirmed_dynamic &&
             !dynamic_motion_confirmed &&
             (!motion_history_observed ||
              motion_net_displacement_m <= static_max_displacement_m) ||
             lidar_smoothed_stationary_timeout);
        // 静态背景释放是轨迹维护：语义陈旧且 LiDAR 证明目标静止时撤销语义，
        // 防止静止目标凭旧类别档案继续参与下游判定。
        if (stale_static_lidar_semantic) {
            reject("semantic_stale_static");
            if (debug_track) {
                std::cout << "[ORSMTrackDiag] action=release_static_semantic"
                          << " id=" << track.stable_id
                          << " semantic_age_ms=" << semantic_age_ns / 1000000ULL
                          << " motion_net_disp=" << motion_net_displacement_m
                          << " motion_path=" << motion_path_length_m
                          << " motion_straightness=" << motion_straightness
                          << " stationary_age_ms="
                          << (track.lidar_stationary_since_ts_ns > 0 &&
                              timestamp_ns >= track.lidar_stationary_since_ts_ns
                                  ? (timestamp_ns - track.lidar_stationary_since_ts_ns) / 1000000ULL
                                  : 0)
                          << " lidar_hits=" << track.lidar_hits
                          << " dist=" << track.distance_m
                          << std::endl;
            }
            // 类别由 L# 管理；释放时清除类别记忆和语义命中数，后续需重新关联 YOLO。
            if (bound_temporal != nullptr) {
                auto& temporal = *bound_temporal;
                if (!cfg_.orsm_class_label_lock_enabled ||
                    isUnknownOrsmLabel(temporal.label, temporal.class_id)) {
                    temporal.class_id = -1;
                    temporal.label = "unknown";
                    temporal.class_confidence = 0.f;
                }
                temporal.semantic_hits = 0;
            }
            // 全局侧只清展示用类别，保证已发布标签不会残留。
            if (!cfg_.orsm_class_label_lock_enabled ||
                isUnknownOrsmLabel(track.label, track.class_id)) {
                track.class_id = -1;
                track.label = "unknown";
                track.class_confidence = 0.f;
            }
            track.last_dynamic_motion_confirmed_ts_ns = 0;
            track.lidar_stationary_since_ts_ns = 0;
            continue;
        }
        // 发布需要 L# 租约；YOLO 和差值框共享截止时间并取较晚值。
        // 满足稳定性、运动和距离条件的 lidar-only 轨迹可按配置绕过租约。
        const bool lidar_only_output_eligible =
            lidarOnlyOutputEligible(track.lidar_temporal_id, track.distance_m);
        if (!lidar_publication_lease_active && !lidar_only_output_eligible) {
            reject("no_publication_lease");
            continue;
        }
        TrackedObject obj;
        obj.track_id = track.stable_id;
        obj.class_id = track.class_id;
        obj.label = track.label;
        obj.bbox = track.bbox;
        obj.confidence = track.confidence;
        obj.age = track.hits;
        obj.lost = timestamp_ns > track.last_seen_ts_ns ? 1 : 0;
        obj.has_lidar_fusion = recent_lidar || lidar_publication_lease_active ||
                               lidar_only_output_eligible;
        obj.position_3d = track.position;
        obj.velocity_3d = track.velocity;
        obj.distance_m = track.distance_m;
        obj.speed_mps = track.speed_mps;
        obj.fusion_timestamp_ns = track.last_seen_ts_ns;
        obj.source_camera = track.last_camera;
        obj.orsm_far_image_motion_lidar_confirmed =
            lidar_publication_lease_active || lidar_only_output_eligible;
        fused.push_back(obj);
    }

    fused = dedupeOrsmGroupCandidates(fused, "ORSMOutputDedupe");

    std::sort(fused.begin(), fused.end(), [](const TrackedObject& a, const TrackedObject& b) {
        return a.distance_m < b.distance_m;
    });

    static std::atomic<uint64_t> log_count{0};
    const uint64_t log_idx = ++log_count;
    if (!fused.empty() || log_idx <= 20 || log_idx % 120 == 0) {
        const int lidar_count = static_cast<int>(std::count_if(
            fused.begin(), fused.end(), [](const TrackedObject& obj) { return obj.has_lidar_fusion; }));
        int candidate_count = 0;
        int stable_count = 0;
        int lost_count = 0;
        for (const auto& [id, track] : orsm_global_tracks_) {
            switch (track.lifecycle) {
                case OrsmTrackLifecycle::Candidate:
                    candidate_count++;
                    break;
                case OrsmTrackLifecycle::Stable:
                    stable_count++;
                    break;
                case OrsmTrackLifecycle::Lost:
                    lost_count++;
                    break;
            }
        }
        std::cout << "[ORSMGlobalTrack] camera_candidates=" << camera_tracks.size()
                  << " tracks=" << orsm_global_tracks_.size()
                  << " candidate=" << candidate_count
                  << " stable=" << stable_count
                  << " lost=" << lost_count
                  << " output=" << fused.size()
                  << " lidar_output=" << lidar_count
                  << std::endl;
        if (debug_track) {
            for (const auto& obj : fused) {
                std::cout << "[ORSMTrackDiag] output"
                          << " id=" << obj.track_id
                          << " label=" << obj.label
                          << " class_id=" << obj.class_id
                          << " dist=" << obj.distance_m
                          << " pos=(" << obj.position_3d.x << "," << obj.position_3d.y << ")"
                          << " age=" << obj.age
                          << " lidar=" << obj.has_lidar_fusion
                          << " conf=" << obj.confidence
                          << " origin=" << static_cast<int>(obj.origin)
                          << " claim=" << static_cast<int>(obj.lidar_claim)
                          << " ts=" << timestamp_ns
                          << " temporal_source=" << orsm_global_tracks_.at(obj.track_id).last_source_track_id
                          << " publication_lease_until="
                          << orsm_global_tracks_.at(obj.track_id).lidar_publication_lease_until_ts_ns
                          << std::endl;
            }
        }
    }

    return fused;
}

bool SentinelNode::vehiclePowerOffLocked(const VehicleOddSnapshot& vehicle) const {
    const bool power_off = cfg_.override_power_off
        ? cfg_.default_power_off
        : vehicle.power_off.value_or(cfg_.default_power_off);
    const bool vehicle_locked = cfg_.override_vehicle_locked
        ? cfg_.default_vehicle_locked
        : vehicle.vehicle_locked.value_or(cfg_.default_vehicle_locked);
    return power_off && vehicle_locked;
}

OddConditions SentinelNode::oddIgnoringDeferredEntryConditions(OddConditions odd) {
    odd.power_off = true;
    odd.vehicle_locked = true;
    odd.no_occupant = true;
    return odd;
}

std::optional<std::time_t> SentinelNode::scheduledStartEpoch(uint8_t day, uint32_t hour,
                                                             uint32_t minute,
                                                             long long request_time_s) const {
    if (hour > 23 || minute > 59) return std::nullopt;
    if (request_time_s <= 0) return std::nullopt;

    const std::time_t req_t = static_cast<std::time_t>(request_time_s);
    std::tm target_tm{};
    localtime_r(&req_t, &target_tm);
    target_tm.tm_hour = static_cast<int>(hour);
    target_tm.tm_min = static_cast<int>(minute);
    target_tm.tm_sec = 0;
    target_tm.tm_isdst = -1;

    // Cap'n Proto 枚举：1=Monday ... 7=Sunday, 8=TODAY, 9=TOMORROW。
    if (day >= 1 && day <= 7) {
        const int req_day = target_tm.tm_wday == 0 ? 7 : target_tm.tm_wday;
        const int days_ahead = (static_cast<int>(day) - req_day + 7) % 7;
        target_tm.tm_mday += days_ahead;
    } else if (day == 8) {
        // TODAY: keep request date.
    } else if (day == 9) {
        target_tm.tm_mday += 1;
    } else {
        return std::nullopt;
    }

    const std::time_t target_t = std::mktime(&target_tm);
    if (target_t == static_cast<std::time_t>(-1) || target_t <= req_t) return std::nullopt;
    return target_t;
}

bool SentinelNode::scheduledStartIsFuture(uint8_t day, uint32_t hour, uint32_t minute,
                                          long long request_time_s) const {
    return scheduledStartEpoch(day, hour, minute, request_time_s).has_value();
}

bool SentinelNode::scheduledStartDue(const VehicleOddSnapshot& vehicle) const {
    if (!vehicle.scheduled_start_pending) return true;
    const auto target = scheduledStartEpoch(vehicle.scheduled_start_day.value_or(0),
                                            vehicle.scheduled_start_hour.value_or(24),
                                            vehicle.scheduled_start_minute.value_or(60),
                                            vehicle.scheduled_request_time_s.value_or(0));
    if (!target) return false;
    return std::time(nullptr) >= *target;
}

void SentinelNode::evaluateSentryRequestTriggers(uint64_t timestamp_ns) {
    VehicleOddSnapshot vehicle;
    {
        std::lock_guard<std::mutex> lk(vehicle_mutex_);
        vehicle = latest_vehicle_;
    }

    if (timestamp_ns == 0) timestamp_ns = currentSteadyTimeNs();
    const bool power_off_locked = vehiclePowerOffLocked(vehicle);

    // Full ODD evaluation at the actual activation point. user_enabled/user_shutdown_requested
    // do not affect oddStatus(), so this is safe even before the request latch flips on.
    const OddConditions odd = buildOdd();
    struct PendingSentryResponse {
        uint16_t req_value{0};
        OddConditions odd;
        std::optional<SentinelEnableFailed> forced_failed;
        bool approve{false};
    };
    std::vector<PendingSentryResponse> responses;

    {
        std::lock_guard<std::mutex> lk(sentry_request_mutex_);

        if (sentry_request_control_.sentry_enabled_by_request &&
            odd.oddStatus() != SentinelODDStatus::SATISFY_ALL_ODD) {
            // 哨兵已开启后，车辆状态变为不满足 ODD（例如 P 挡切到非 P 挡）时，
            // 主动反馈关闭，并携带当前 ODD/失败原因，而不是继续发 0 未知状态。
            // 若当前会话由预约开启触发，则按 APP 协议反馈 6=关闭预约开启；
            // failed reason 必须来自当前 ODD，避免把所有退出都误报为 P 挡不满足。
            const bool scheduled_session = sentry_request_control_.sentry_enabled_by_scheduled_request;
            sentry_request_control_.sentry_enabled_by_request = false;
            sentry_request_control_.sentry_enabled_by_scheduled_request = false;
            sentry_request_control_.sentry_enabled_by_automatic_request = false;
            responses.push_back({
                static_cast<uint16_t>(scheduled_session
                    ? SentinelReqMsgType::DISABLE_SENTINEL_SCHEDULED_START
                    : SentinelReqMsgType::DISABLE_SENTINEL_IMMEDIATELY),
                odd,
                std::nullopt,
                false
            });
            std::cout << "[SentinelNode][SentryRequestTrigger] active sentry closed by odd="
                      << static_cast<int>(odd.oddStatus())
                      << " scheduled_session=" << (scheduled_session ? 1 : 0) << std::endl;
        } else if (!power_off_locked) {
            // Leaving the down-power+locked condition closes the request-driven active window.
            sentry_request_control_.sentry_enabled_by_request = false;
            sentry_request_control_.sentry_enabled_by_scheduled_request = false;
            sentry_request_control_.sentry_enabled_by_automatic_request = false;
        }

        const bool automatic_rising_edge =
            power_off_locked && !sentry_request_control_.last_power_off_locked;
        const bool automatic_request_trigger =
            sentry_request_control_.automatic_enable_pending_once;

        if (sentry_request_control_.immediate_enable_pending_once && power_off_locked) {
            if (odd.oddStatus() == SentinelODDStatus::SATISFY_ALL_ODD) {
                sentry_request_control_.sentry_enabled_by_request = true;
                sentry_request_control_.sentry_enabled_by_scheduled_request = false;
                sentry_request_control_.sentry_enabled_by_automatic_request = false;
                std::cout << "[SentinelNode][SentryRequestTrigger] immediate enable activated" << std::endl;
            } else {
                responses.push_back({static_cast<uint16_t>(SentinelReqMsgType::ENABLE_SENTINEL_IMMEDIATELY), odd, std::nullopt, false});
                std::cout << "[SentinelNode][SentryRequestTrigger] immediate enable declined odd="
                          << static_cast<int>(odd.oddStatus()) << std::endl;
            }
            sentry_request_control_.immediate_enable_pending_once = false;
        }

        if (sentry_request_control_.automatic_enable_enabled &&
            (automatic_rising_edge || automatic_request_trigger)) {
            const SentinelODDStatus automatic_odd_status = odd.oddStatus();
            OddConditions odd_with_deferred_entry_ready = odd;
            odd_with_deferred_entry_ready.power_off = true;
            odd_with_deferred_entry_ready.vehicle_locked = true;
            odd_with_deferred_entry_ready.key_far = true;
            const bool only_deferred_entry_not_ready =
                (!odd.power_off || !odd.vehicle_locked || !odd.key_far) &&
                odd_with_deferred_entry_ready.oddStatus() == SentinelODDStatus::SATISFY_ALL_ODD;

            if (automatic_odd_status == SentinelODDStatus::SATISFY_ALL_ODD) {
                sentry_request_control_.sentry_enabled_by_request = true;
                sentry_request_control_.sentry_enabled_by_scheduled_request = false;
                sentry_request_control_.sentry_enabled_by_automatic_request = true;
                std::cout << "[SentinelNode][SentryRequestTrigger] automatic enable activated" << std::endl;
                sentry_request_control_.automatic_enable_pending_once = false;
            } else if (only_deferred_entry_not_ready) {
                // Automatic enable is a future trigger. If the only missing conditions are the
                // deferred entry signals, keep waiting instead of replying DECLINE immediately.
                std::cout << "[SentinelNode][SentryRequestTrigger] automatic enable waits for power_off/lock/key_far"
                          << std::endl;
            } else {
                responses.push_back({static_cast<uint16_t>(SentinelReqMsgType::ENABLE_SENTINEL_AUTOMATICALLY), odd, std::nullopt, false});
                std::cout << "[SentinelNode][SentryRequestTrigger] automatic enable declined odd="
                          << static_cast<int>(odd.oddStatus()) << std::endl;
                sentry_request_control_.automatic_enable_pending_once = false;
            }
        }

        if (sentry_request_control_.scheduled_enable_pending) {
            VehicleOddSnapshot schedule_vehicle = vehicle;
            schedule_vehicle.scheduled_start_pending = true;
            schedule_vehicle.scheduled_start_day = sentry_request_control_.scheduled_start_day;
            schedule_vehicle.scheduled_start_hour = sentry_request_control_.scheduled_start_hour;
            schedule_vehicle.scheduled_start_minute = sentry_request_control_.scheduled_start_minute;
            schedule_vehicle.scheduled_request_time_s = sentry_request_control_.scheduled_request_time_s;
            if (scheduledStartDue(schedule_vehicle)) {
                if (odd.oddStatus() == SentinelODDStatus::SATISFY_ALL_ODD) {
                    sentry_request_control_.sentry_enabled_by_request = true;
                    sentry_request_control_.sentry_enabled_by_scheduled_request = true;
                    sentry_request_control_.sentry_enabled_by_automatic_request = false;
                    responses.push_back({
                        static_cast<uint16_t>(SentinelReqMsgType::ENABLE_SENTINEL_SCHEDULED_START),
                        odd,
                        std::nullopt,
                        true
                    });
                    std::cout << "[SentinelNode][SentryRequestTrigger] scheduled enable activated" << std::endl;
                } else {
                    responses.push_back({static_cast<uint16_t>(SentinelReqMsgType::ENABLE_SENTINEL_SCHEDULED_START), odd, std::nullopt, false});
                    std::cout << "[SentinelNode][SentryRequestTrigger] scheduled enable declined odd="
                              << static_cast<int>(odd.oddStatus()) << std::endl;
                }
                sentry_request_control_.scheduled_enable_pending = false;
            }
        }

        sentry_request_control_.last_power_off_locked = power_off_locked;
    }

    for (const auto& response : responses) {
        publishSentryRequestResponse(response.req_value, timestamp_ns, response.approve,
                                     response.odd, response.forced_failed);
    }
}

void SentinelNode::onUssFusedStaticMap(
    const std::shared_ptr<ReceivedMsg<senseAD::msg::avp_perception::UssFusedStaticMap>>& msg)
{
    if (!msg) return;
    const auto& map = msg->Msg();
    if (!map.hasFusedPoints()) return;
    const auto fused_points = map.getFusedPoints();
    const auto list_size = fused_points.size();
    static std::atomic<uint64_t> uss_fused_cb_count{0};
    const uint64_t cb_idx = ++uss_fused_cb_count;
    if (cb_idx <= 10 || cb_idx % 100 == 0) {
        std::cout << "[SentinelNode][USS-FusedRaw] cb=" << cb_idx
                  << " ts=" << map.getTimestamp()
                  << " seq=" << map.getSeq()
                  << " fusedPointsSize=" << map.getFusedPointsSize()
                  << " listSize=" << list_size
                  << " hasFusedPoints=" << map.hasFusedPoints()
                  << std::endl;
        int shown = 0;
        for (const auto& raw_pt : fused_points) {
            if (shown++ >= 3) break;
            std::cout << "[SentinelNode][USS-FusedRawPoint] x=" << raw_pt.getX()
                      << " y=" << raw_pt.getY()
                      << " age=" << raw_pt.getAge()
                      << " exist=" << static_cast<int>(raw_pt.getExistProb())
                      << " high=" << static_cast<int>(raw_pt.getHighProb())
                      << " low=" << static_cast<int>(raw_pt.getLowProb())
                      << " free=" << static_cast<int>(raw_pt.getFreeProb())
                      << std::endl;
        }
    }
    if (list_size == 0) return;

    std::vector<UssPoint> points;
    points.reserve(list_size);
    const size_t declared_size = static_cast<size_t>(map.getFusedPointsSize());
    const size_t scan_size = declared_size > 0
        ? std::min(declared_size, static_cast<size_t>(list_size))
        : static_cast<size_t>(list_size);
    const int min_exist_prob = std::clamp(cfg_.uss_fused_min_exist_prob, 1, 100);
    const float coordinate_scale_m = cfg_.uss_fused_coordinate_scale_m > 0.f
        ? cfg_.uss_fused_coordinate_scale_m
        : 0.001f;
    for (size_t i = 0; i < scan_size; ++i) {
        const auto pt = fused_points[i];
        // Some producers expose a fixed-capacity list while leaving fusedPointsSize at zero.
        // In that compatibility mode, existence probability distinguishes populated entries
        // from all-zero and (-2000,-2000) padding slots.
        if (static_cast<int>(pt.getExistProb()) < min_exist_prob) continue;
        const float x_m = static_cast<float>(pt.getX()) * coordinate_scale_m;
        const float y_m = static_cast<float>(pt.getY()) * coordinate_scale_m;
        const float dist_m = std::sqrt(x_m * x_m + y_m * y_m);
        if (!std::isfinite(dist_m) || dist_m <= 0.f) continue;
        points.push_back(UssPoint{x_m, y_m, dist_m, map.getTimestamp()});
    }

    if (!points.empty() && (cb_idx <= 10 || cb_idx % 100 == 0)) {
        std::cout << "[SentinelNode][USS-FusedParsed] cb=" << cb_idx
                  << " valid_points=" << points.size()
                  << " scale_mm_per_unit=" << coordinate_scale_m * 1000.f
                  << " min_exist_prob=" << min_exist_prob
                  << " first_xy_m=(" << points.front().x_m << ',' << points.front().y_m << ')'
                  << " first_distance_m=" << points.front().distance_m
                  << std::endl;
    }

    if (uss_) {
        uss_->updateFromFusedPoints(map.getTimestamp(), points);
    }
}

OddConditions SentinelNode::buildOdd() const {
    VehicleOddSnapshot vehicle;
    {
        std::lock_guard<std::mutex> lk(vehicle_mutex_);
        vehicle = latest_vehicle_;
    }

    const uint64_t now_ns = currentSteadyTimeNs();
    const bool vehicle_fresh = vehicle.timestamp_ns > 0 &&
        now_ns >= vehicle.timestamp_ns &&
        (now_ns - vehicle.timestamp_ns) <=
            static_cast<uint64_t>(cfg_.vehicle_report_timeout_sec * 1e9f);

    bool request_enabled = false;
    {
        std::lock_guard<std::mutex> req_lk(sentry_request_mutex_);
        request_enabled = sentry_request_control_.sentry_enabled_by_request;
    }

    OddConditions odd;
    if (vehicle_fresh) {
        const bool soc_ok = !vehicle.soc_percent ||
            *vehicle.soc_percent >= cfg_.soc_min_sentry;
        const bool speed_zero = vehicle.speed_kmh
                                ? std::abs(*vehicle.speed_kmh) <= cfg_.speed_zero_threshold_kmh
                                : cfg_.default_speed_zero;
        odd.power_ok        = cfg_.override_power_ok ? cfg_.default_power_ok : soc_ok;
        odd.speed_zero      = cfg_.override_speed_zero ? cfg_.default_speed_zero : speed_zero;
        odd.gear_park       = cfg_.override_gear_park ? cfg_.default_gear_park : vehicle.gear_park.value_or(cfg_.default_gear_park);
        odd.handbrake_on    = cfg_.override_handbrake_on ? cfg_.default_handbrake_on : vehicle.handbrake_on.value_or(cfg_.default_handbrake_on);
        odd.power_off       = cfg_.override_power_off ? cfg_.default_power_off : vehicle.power_off.value_or(cfg_.default_power_off);
        odd.no_sensor_fault = cfg_.override_no_sensor_fault ? cfg_.default_no_sensor_fault : vehicle.no_sensor_fault.value_or(cfg_.default_no_sensor_fault);
        odd.no_occupant     = cfg_.override_no_occupant ? cfg_.default_no_occupant : vehicle.no_occupant.value_or(cfg_.default_no_occupant);
        odd.vehicle_locked  = cfg_.override_vehicle_locked ? cfg_.default_vehicle_locked : vehicle.vehicle_locked.value_or(cfg_.default_vehicle_locked);
        odd.temp_ok         = true;
        odd.key_far         = cfg_.override_key_far ? cfg_.default_key_far : vehicle.key_far.value_or(cfg_.default_key_far);
        odd.user_enabled    = vehicle.user_enabled.value_or(cfg_.user_enabled) || request_enabled;
        odd.user_shutdown_requested = vehicle.user_shutdown_requested.value_or(cfg_.user_shutdown_requested);
    } else {
        // 未收到车辆状态或消息超时：使用配置默认值，避免单节点在台架无 CAN 数据时无法调试。
        odd.power_ok        = cfg_.default_power_ok;
        odd.speed_zero      = cfg_.default_speed_zero;
        odd.gear_park       = cfg_.default_gear_park;
        odd.handbrake_on    = cfg_.default_handbrake_on;
        odd.power_off       = cfg_.default_power_off;
        odd.no_sensor_fault = cfg_.default_no_sensor_fault;
        odd.no_occupant     = cfg_.default_no_occupant;
        odd.vehicle_locked  = cfg_.default_vehicle_locked;
        odd.temp_ok         = true;
        odd.key_far         = cfg_.default_key_far;
        odd.user_enabled    = cfg_.user_enabled || request_enabled;
        odd.user_shutdown_requested = cfg_.user_shutdown_requested;
    }

    odd.storage_ok = checkStorageOk();
    if (sentrySuppressedByHigherPriorityMode()) {
        // 全局模式互斥：单车/多车堡垒优先于哨兵。堡垒开启时，哨兵不激活。
        odd.power_ok = false;
    }
    return odd;
}

// ---- 报警处理 ----

void SentinelNode::onAlert(const AlertEvent& evt) {
    std::cout << "[SentinelNode][ALERT]"
              << " level=" << static_cast<int>(evt.level)
              << " state=" << static_cast<int>(evt.sentry_state)
              << " reason=" << evt.reason
              << " targets=" << evt.target_count
              << " light=" << evt.light_req
              << " horn=" << evt.horn_req
              << " hazard=" << evt.hazard_lamp_req
              << std::endl;

    AD_LINFO(Sentinel) << "[SentinelNode][ALERT] level=" << static_cast<int>(evt.level)
               << " " << evt.reason;

    // LOW 仅表示“监测范围内出现目标，持续观察”，按设计不属于对外告警。
    // 不写正式事件，避免调试 bag 中每帧/每个 track 都刷 /sentry/event。
    if (evt.level < ThreatLevel::MEDIUM) {
        return;
    }

    bool should_publish = false;
    bool should_record = false;
    int stable_session_id = 0;
    const uint64_t event_ts_ns = evt.timestamp_ns > 0 ? evt.timestamp_ns : currentSteadyTimeNs();
    {
        std::lock_guard<std::mutex> lk(threat_session_mutex_);
        const bool expired = active_threat_session_.active &&
            elapsedAtLeastNs(active_threat_session_.last_seen_ns, evt.timestamp_ns, kThreatSessionMergeNs);

        if (!active_threat_session_.active || expired) {
            active_threat_session_ = ThreatSession{};
            active_threat_session_.active = true;
            active_threat_session_.session_id = next_threat_session_id_++;
            active_threat_session_.first_state = evt.sentry_state;
            active_threat_session_.first_seen_ns = evt.timestamp_ns;
            active_threat_session_.max_level = evt.level;
        }

        active_threat_session_.last_seen_ns = evt.timestamp_ns;
        active_threat_session_.last_raw_threat_id = evt.threat_id;
        if (evt.level > active_threat_session_.max_level) {
            active_threat_session_.max_level = evt.level;
        }
        stable_session_id = active_threat_session_.session_id;

        // 方案 C：以 ThreatSession 生命周期作为“同一危险”边界。
        // MEDIUM 只在进入警戒窗口时发布一次；WARNING 内 track_id 抖动不重复发布。
        if (evt.level == ThreatLevel::MEDIUM && !active_threat_session_.warning_event_published) {
            should_publish = true;
            active_threat_session_.warning_event_published = true;
        }

        // HIGH 只在同一危险从警戒升级到报警时发布一次。
        if (evt.level == ThreatLevel::HIGH && !active_threat_session_.alarm_event_published) {
            should_publish = true;
            active_threat_session_.alarm_event_published = true;
        }

        const bool video_window_available = last_video_record_trigger_ns_ == 0 ||
            elapsedAtLeastNs(last_video_record_trigger_ns_,
                             event_ts_ns,
                             kVideoRecordSuppressWindowNs);

        // 录像按 3 分钟全局时间窗口限流：窗口内第一次 MEDIUM/HIGH 报警触发一次录像，
        // 后续报警只更新状态/告警，不再重复触发录像；若首次就是 HIGH，也覆盖严重告警窗口。
        if (cfg_.enable_event_recording && video_recorder_ && video_window_available &&
            ((evt.sentry_state == SentryState::ACTIVE && evt.level >= ThreatLevel::MEDIUM) ||
             (evt.sentry_state == SentryState::WARNING && evt.level == ThreatLevel::HIGH))) {
            should_record = true;
            active_threat_session_.video_triggered = true;
            last_video_record_trigger_ns_ = event_ts_ns;
        }
    }

    if (!should_publish && !should_record) {
        std::cout << "[SentinelNode][ThreatSession] suppress duplicate alert"
                  << " session=" << stable_session_id
                  << " raw_threat_id=" << evt.threat_id
                  << " level=" << static_cast<int>(evt.level)
                  << std::endl;
        return;
    }

    const int manifest_threat_level = sentryManifestThreatLevel(evt.level);
    const std::string record_reason = sentryRecordingReason(evt);
    if (cfg_.enable_event_recording && video_recorder_ && manifest_threat_level > 0) {
        // manifest 的 threat_level/reason 使用产品口径：当前录像窗口内多个告警产生时的最大值及其对应原因。
        // 1=alert(一般威胁警戒)，2=alarm(严重威胁报警)。
        video_recorder_->updateMaxThreatLevel(manifest_threat_level, record_reason);
    }

    const uint64_t event_id = next_event_id_++;
    std::cout << "[SentinelNode][RecordingTrace] alert_decision"
              << " event_id=" << event_id
              << " session=" << stable_session_id
              << " should_publish=" << (should_publish ? 1 : 0)
              << " should_record=" << (should_record ? 1 : 0)
              << " alert_sensor_ts=" << evt.timestamp_ns
              << " event_ts=" << event_ts_ns
              << " wall_ns=" << currentSteadyTimeNs()
              << " level=" << static_cast<int>(evt.level)
              << " state=" << static_cast<int>(evt.sentry_state)
              << " threat_id=" << evt.threat_id
              << std::endl;

    if (should_record) {
        const uint64_t trigger_call_wall_ns = currentSteadyTimeNs();
        bool accepted = video_recorder_->triggerRecord(
            event_id,
            record_reason,
            manifest_threat_level);
        std::cout << "[SentinelNode][RecordingTrace] trigger_return"
                  << " event_id=" << event_id
                  << " accepted=" << (accepted ? 1 : 0)
                  << " trigger_call_wall_ns=" << trigger_call_wall_ns
                  << " trigger_return_wall_ns=" << currentSteadyTimeNs()
                  << std::endl;
        if (accepted) {
            AlertEvent record_evt = evt;
            record_evt.reason = record_reason;
            std::lock_guard<std::mutex> lk(pending_event_mutex_);
            pending_record_events_[event_id] = record_evt;
            pending_record_trigger_wall_ns_[event_id] = trigger_call_wall_ns;
        }
    }
    if (should_publish) {
        // The immediate event is the only place that may issue a body-control
        // request.  Recording completion later republishes metadata but must not
        // repeat horn/hazard actions.
        publishSentryOutputs(event_id, evt, {}, true);
    }
}

int SentinelNode::stableThreatIdForStateMachine(ThreatLevel level, int raw_threat_id, uint64_t timestamp_ns) {
    std::lock_guard<std::mutex> lk(threat_session_mutex_);
    if (level < ThreatLevel::MEDIUM) {
        return 0;
    }

    const bool expired = active_threat_session_.active &&
        elapsedAtLeastNs(active_threat_session_.last_seen_ns, timestamp_ns, kThreatSessionMergeNs);
    if (!active_threat_session_.active || expired) {
        active_threat_session_ = ThreatSession{};
        active_threat_session_.active = true;
        active_threat_session_.session_id = next_threat_session_id_++;
        active_threat_session_.first_seen_ns = timestamp_ns;
        active_threat_session_.last_seen_ns = timestamp_ns;
        active_threat_session_.last_raw_threat_id = raw_threat_id;
        active_threat_session_.max_level = level;
    } else {
        active_threat_session_.last_seen_ns = timestamp_ns;
        active_threat_session_.last_raw_threat_id = raw_threat_id;
        if (level > active_threat_session_.max_level) {
            active_threat_session_.max_level = level;
        }
    }
    return active_threat_session_.session_id;
}



void SentinelNode::handleRecordedVideoDone(uint64_t event_id, const std::vector<std::string>& paths) {
    const uint64_t done_wall_ns = currentSteadyTimeNs();
    if (!paths.empty()) {
        std::cout << "[SentinelNode] Video recorded: event_" << event_id
                  << " videos=" << paths.size() << std::endl;
        for (const auto& path : paths) {
            std::cout << "  → " << path << std::endl;
        }
    }

    AlertEvent evt;
    bool found = false;
    uint64_t trigger_wall_ns = 0;
    {
        std::lock_guard<std::mutex> lk(pending_event_mutex_);
        auto it = pending_record_events_.find(event_id);
        if (it != pending_record_events_.end()) {
            evt = it->second;
            pending_record_events_.erase(it);
            found = true;
        }
        auto wall_it = pending_record_trigger_wall_ns_.find(event_id);
        if (wall_it != pending_record_trigger_wall_ns_.end()) {
            trigger_wall_ns = wall_it->second;
            pending_record_trigger_wall_ns_.erase(wall_it);
        }
    }

    const double trigger_to_done_s = trigger_wall_ns > 0 && done_wall_ns >= trigger_wall_ns
        ? static_cast<double>(done_wall_ns - trigger_wall_ns) / 1.0e9
        : -1.0;
    std::cout << "[SentinelNode][RecordingTrace] done_received"
              << " event_id=" << event_id
              << " found_pending=" << (found ? 1 : 0)
              << " videos=" << paths.size()
              << " trigger_to_done_wall_s=" << trigger_to_done_s
              << " alert_sensor_ts=" << (found ? evt.timestamp_ns : 0)
              << " done_wall_ns=" << done_wall_ns
              << std::endl;

    if (found && !paths.empty()) {
        publishSentryOutputs(event_id, evt, paths, false);
    }
    cleanupEventRecordingStorage(paths);
}

void SentinelNode::stateTickLoop() {
    const int tick_ms = std::max(20, cfg_.sentry_state_tick_ms);
    while (state_tick_running_.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(tick_ms));
        std::lock_guard<std::mutex> lk(processing_mutex_);
        if (!state_machine_) continue;

        const uint64_t sensor_ts = last_sensor_time_ns_.load(std::memory_order_relaxed);
        const uint64_t state_ts = sensor_ts > 0 ? sensor_ts : currentSteadyTimeNs();
        evaluateSentryRequestTriggers(state_ts);

        OddConditions odd = buildOdd();
        latest_odd_ = odd;

        const bool suppress_sentry = sentrySuppressedByHigherPriorityMode();
        const ThreatLevel level = suppress_sentry || !alert_manager_
            ? ThreatLevel::NONE
            : alert_manager_->currentLevel();
        const int threat_id = suppress_sentry || !alert_manager_
            ? 0
            : alert_manager_->currentThreatId();
        // Bag 回放使用传感器时间戳驱动帧处理；tick 线程也必须使用同一时间基准。
        // 否则在状态机里用 bag timestamp 记录 threat_start，再用 steady_clock tick
        // 做超时判断，会发生 uint64 下溢，30s bag 误触发 “同一威胁>3min”。
        const int stable_threat_id = stableThreatIdForStateMachine(level, threat_id, state_ts);
        state_machine_->update(odd, level, stable_threat_id,
                               state_ts);

        updateCachedSentryStatus();
        publishSentryStateBroadcastTick();
    }
}

std::string SentinelNode::sentryRecordingReason(const AlertEvent& evt) const {
    if (!evt.reason.empty()) return evt.reason;
    if (evt.level == ThreatLevel::HIGH) {
        return "严重威胁：大幅震动/碰撞 → 声光报警 + 录像";
    }
    if (evt.imu_triggered) {
        return "一般威胁：小幅震动/门把手震动 → 中控警戒 + 录像";
    }
    return "一般威胁：移动目标距离<3m且速度≥0.1m/s → 中控警戒 + 录像";
}

int SentinelNode::sentryManifestThreatLevel(ThreatLevel level) const {
    if (level >= ThreatLevel::HIGH) return 2;    // alarm：严重威胁报警
    if (level >= ThreatLevel::MEDIUM) return 1;  // alert：一般威胁警戒
    return 0;
}

void SentinelNode::pushFrameForEventRecording(const DecodedFrame& frame) {
    if (!video_recorder_ || frame.image.empty()) return;
    const std::string camera = frame.camera_name.empty() ? "camera" : frame.camera_name;
    video_recorder_->pushFrame(camera, frame.image, frame.timestamp_ns);
}

bool SentinelNode::statusPublishFixed10Hz() const {
    return cfg_.status_publish_mode == "fixed_10hz";
}

bool SentinelNode::statusPublishPerFrame() const {
    return cfg_.status_publish_mode == "per_frame";
}

void SentinelNode::publishFrameStatusOutputs(uint64_t frame_timestamp_ns) {
    // 保留兼容入口，但 ORSM 状态已改为独立固定10Hz发送，避免 per_frame/fixed_10hz 下重复发送。
    publishSentryStatusTick(frame_timestamp_ns);
}

void SentinelNode::statusPublishLoop() {
    using clock = std::chrono::steady_clock;
    auto next_tick = clock::now();
    while (status_publish_running_.load()) {
        next_tick += std::chrono::milliseconds(100);
        // 固定频率线程只负责哨兵状态；ORSM 状态由独立线程固定10Hz发送。
        publishSentryStatusTick();
        publishSentryStateBroadcastTick();
        std::this_thread::sleep_until(next_tick);
        if (clock::now() > next_tick + std::chrono::seconds(1)) {
            // If the process was paused or heavily scheduled out, restart cadence from now.
            next_tick = clock::now();
        }
    }
}

void SentinelNode::orsmStatusPublishLoop() {
    using clock = std::chrono::steady_clock;
    auto next_tick = clock::now();
    while (orsm_status_publish_running_.load()) {
        next_tick += std::chrono::milliseconds(100);
        publishOrsmStatusTick();
        std::this_thread::sleep_until(next_tick);
        if (clock::now() > next_tick + std::chrono::seconds(1)) {
            next_tick = clock::now();
        }
    }
}

void SentinelNode::logSentryModeLifecycle(const SentryStateEvent& evt) const {
    std::cout << "[模式切换][哨兵] " << sentryLifecycleActionZh(evt.from, evt.to)
              << " from=" << static_cast<int>(evt.from) << "(" << sentryStateNameZh(evt.from) << ")"
              << " to=" << static_cast<int>(evt.to) << "(" << sentryStateNameZh(evt.to) << ")"
              << " reason=" << evt.reason
              << " ts=" << evt.timestamp_ns
              << std::endl;
}

void SentinelNode::logOrsmModeLifecycle(const OrsmStateEvent& evt) const {
    std::cout << "[模式切换][堡垒] " << orsmLifecycleActionZh(evt.from, evt.to)
              << " from=" << static_cast<int>(evt.from) << "(" << orsmStateNameZh(evt.from) << ")"
              << " to=" << static_cast<int>(evt.to) << "(" << orsmStateNameZh(evt.to) << ")"
              << " reason=" << evt.reason
              << " ts=" << evt.timestamp_ns
              << std::endl;
}

void SentinelNode::logOrsmOddIfNeeded(const OrsmOdd& odd, OrsmState state, uint64_t timestamp_ns) const {
    const bool has_fault = odd.had_fault || odd.sensor_fault;
    const bool sw_on = odd.sw_enable && orsmEnabled(odd.sw_mode);
    const bool can_enter = odd.power_on &&
                           odd.gear_park &&
                           odd.soc >= odd.soc_activate_threshold &&
                           odd.handbrake_on &&
                           !has_fault &&
                           sw_on;
    const bool exit_condition = !sw_on ||
                                !odd.gear_park ||
                                odd.soc < odd.soc_deactivate_threshold ||
                                odd.speed_kmh > odd.speed_exit_threshold_kmh;

    const char* block_reason = "满足进入条件";
    if (!sw_on) {
        block_reason = "软开关未开启";
    } else if (!odd.power_on) {
        block_reason = "电源未ON";
    } else if (!odd.gear_park) {
        block_reason = "非P挡";
    } else if (odd.soc < odd.soc_activate_threshold) {
        block_reason = "SOC不足";
    } else if (!odd.handbrake_on) {
        block_reason = "手刹未拉起";
    } else if (has_fault) {
        block_reason = "系统或传感器故障";
    }

    std::ostringstream sig;
    sig << static_cast<int>(state) << '|'
        << odd.power_on << '|'
        << odd.gear_park << '|'
        << odd.handbrake_on << '|'
        << odd.had_fault << '|'
        << odd.sensor_fault << '|'
        << odd.sw_enable << '|'
        << static_cast<int>(odd.sw_mode) << '|'
        << std::fixed << std::setprecision(1)
        << odd.soc << '|'
        << odd.speed_kmh << '|'
        << odd.soc_activate_threshold << '|'
        << odd.soc_deactivate_threshold << '|'
        << odd.speed_exit_threshold_kmh;

    static std::atomic<uint64_t> log_count{0};
    static std::string last_signature;
    const uint64_t idx = ++log_count;
    const std::string signature = sig.str();
    const bool changed = signature != last_signature;
    if (!(idx <= 5 || idx % 100 == 0 || changed)) return;
    last_signature = signature;

    std::cout << "[ORSM_ODD][堡垒进入条件] state=" << static_cast<int>(state)
              << "(" << orsmStateNameZh(state) << ")"
              << " can_enter=" << (can_enter ? 1 : 0)
              << " exit_condition=" << (exit_condition ? 1 : 0)
              << " block_reason=" << block_reason
              << " power_on=" << (odd.power_on ? 1 : 0)
              << " gear_park=" << (odd.gear_park ? 1 : 0)
              << " soc=" << std::fixed << std::setprecision(1) << odd.soc
              << " soc_activate_threshold=" << odd.soc_activate_threshold
              << " soc_deactivate_threshold=" << odd.soc_deactivate_threshold
              << " speed_kmh=" << odd.speed_kmh
              << " speed_exit_threshold_kmh=" << odd.speed_exit_threshold_kmh
              << " handbrake_on=" << (odd.handbrake_on ? 1 : 0)
              << " had_fault=" << (odd.had_fault ? 1 : 0)
              << " sensor_fault=" << (odd.sensor_fault ? 1 : 0)
              << " sw_enable=" << (odd.sw_enable ? 1 : 0)
              << " sw_mode=" << static_cast<int>(odd.sw_mode)
              << " ts=" << timestamp_ns
              << std::endl;
}

void SentinelNode::updateCachedSentryStatus() {
    const bool suppress = sentrySuppressedByHigherPriorityMode();
    const ThreatLevel raw_level = suppress ? ThreatLevel::NONE :
        (alert_manager_ ? alert_manager_->currentLevel() : ThreatLevel::NONE);
    const SentryState state = state_machine_ ? state_machine_->currentState() : SentryState::UNAVAILABLE;

    // Periodic SentinelResStatus must follow the debounced sentry state machine, not only the
    // instantaneous detector result. Otherwise a short missed detection inside WARNING/ALARM keeps
    // the state machine latched but toggles sentinelGeneralThreat/AudioVisualAlarm OFF for one tick,
    // which appears to IVI as the alarm switch jumping back and forth.
    ThreatLevel output_level = raw_level;
    if (!suppress && state == SentryState::WARNING && output_level < ThreatLevel::MEDIUM) {
        output_level = ThreatLevel::MEDIUM;
    } else if (!suppress && state == SentryState::ALARM && output_level < ThreatLevel::HIGH) {
        output_level = ThreatLevel::HIGH;
    }

    AlertEvent evt;
    evt.timestamp_ns = last_sensor_time_ns_.load(std::memory_order_relaxed);
    if (!suppress && output_level >= ThreatLevel::MEDIUM) {
        evt.level = output_level;
        evt.sentry_state = state;
        evt.threat_id = alert_manager_ ? alert_manager_->currentThreatId() : 0;
        if (evt.threat_id == 0) {
            std::lock_guard<std::mutex> lk(threat_session_mutex_);
            if (active_threat_session_.active) evt.threat_id = active_threat_session_.session_id;
        }
        if (output_level == ThreatLevel::HIGH) {
            // 与 AlertManager 告警事件输出保持一致：严重威胁只置 severe/alarm ACTIVE。
            evt.sentinel_general_threat = SentinelGeneralThreat::OFF;
            evt.sentinel_severe_threat = SentinelGeneralThreat::ACTIVE;
            evt.sentinel_audio_visual_alarm = SentinelGeneralThreat::ACTIVE;
            evt.light_req = true;
            evt.horn_req = true;
            evt.hazard_lamp_req = true;
            evt.reason = "严重威胁";
        } else {
            // 与 AlertManager 告警事件输出保持一致：一般威胁置 general ACTIVE，不置 severe/alarm。
            evt.sentinel_general_threat = SentinelGeneralThreat::ACTIVE;
            evt.sentinel_severe_threat = SentinelGeneralThreat::OFF;
            evt.sentinel_audio_visual_alarm = SentinelGeneralThreat::OFF;
            evt.light_req = false;
            evt.horn_req = false;
            evt.hazard_lamp_req = false;
            evt.reason = "一般威胁";
        }
    } else {
        evt.level = ThreatLevel::NONE;
        evt.sentry_state = state;
        evt.threat_id = 0;
        evt.sentinel_general_threat = SentinelGeneralThreat::OFF;
        evt.sentinel_severe_threat = SentinelGeneralThreat::OFF;
        evt.sentinel_audio_visual_alarm = SentinelGeneralThreat::OFF;
        evt.sentinel_obstacle_type = SentinelObstacleType::UNKNOWN_OBSTACLE;
        evt.light_req = false;
        evt.horn_req = false;
        evt.hazard_lamp_req = false;
        evt.reason = "";
    }

    const auto res = makeSentinelResStatus(latest_odd_, evt);
    {
        std::lock_guard<std::mutex> lk(latest_sentry_status_mutex_);
        latest_sentry_status_event_ = evt;
        latest_sentry_status_res_ = res;
    }
}

void SentinelNode::rememberFrameForRecording(const DecodedFrame& frame, const cv::Mat& vis) {
    if (!cfg_.enable_event_recording || vis.empty()) return;
    if (cfg_.save_interval > 1 && frame.seq % cfg_.save_interval != 0) return;
    if (!fs::exists(cfg_.event_record_dir)) fs::create_directories(cfg_.event_record_dir);

    std::ostringstream name;
    name << cfg_.event_record_dir << "/ring_seq_" << frame.seq << ".jpg";
    cv::imwrite(name.str(), vis, {cv::IMWRITE_JPEG_QUALITY, 85});

    std::lock_guard<std::mutex> lk(record_mutex_);
    recent_frames_.push_back({frame.seq, frame.timestamp_ns, name.str()});
    const uint64_t keep_ns = static_cast<uint64_t>(std::max(1, cfg_.event_pre_seconds) * 1000000000ULL);
    while (!recent_frames_.empty() && frame.timestamp_ns > recent_frames_.front().timestamp_ns &&
           frame.timestamp_ns - recent_frames_.front().timestamp_ns > keep_ns) {
        recent_frames_.pop_front();
    }
}

bool SentinelNode::checkStorageOk() const {
    if (cfg_.override_storage_ok) return cfg_.default_storage_ok;
    if (!cfg_.default_storage_ok) return false;
    try {
        if (!cfg_.save_dir.empty() && !fs::exists(cfg_.save_dir)) fs::create_directories(cfg_.save_dir);
        const auto save_space = fs::space(cfg_.save_dir.empty() ? "." : cfg_.save_dir);
        const bool temp_ok = save_space.available >= cfg_.min_temp_storage_bytes;
        bool persistent_ok = true;
        if (!cfg_.persistent_storage_dir.empty() && fs::exists(cfg_.persistent_storage_dir)) {
            persistent_ok = fs::space(cfg_.persistent_storage_dir).available >= cfg_.min_persistent_storage_bytes;
        }
        return temp_ok && persistent_ok;
    } catch (const std::exception& e) {
        std::cerr << "[SentinelNode][Storage] check failed: " << e.what() << std::endl;
        return cfg_.default_storage_ok;
    }
}

void SentinelNode::cleanupEventRecordingStorage(const std::vector<std::string>& protected_paths) const {
    if (cfg_.event_record_max_bytes == 0 || cfg_.event_record_dir.empty()) return;

    try {
        const fs::path record_dir(cfg_.event_record_dir);
        if (!fs::exists(record_dir) || !fs::is_directory(record_dir)) return;

        std::unordered_set<std::string> protected_dirs;
        for (const auto& path_str : protected_paths) {
            if (path_str.empty()) continue;
            protected_dirs.insert(canonicalPathString(fs::path(path_str).parent_path()));
        }

        const uint64_t max_bytes = cfg_.event_record_max_bytes;
        const uint64_t target_bytes = max_bytes / 2;
        uint64_t total_bytes = pathSizeBytes(record_dir);
        if (total_bytes <= max_bytes) {
            std::cout << "[SentinelNode][RecordingCleanup] skip dir=" << record_dir.string()
                      << " total_bytes=" << total_bytes
                      << " max_bytes=" << max_bytes << std::endl;
            return;
        }

        struct Candidate {
            fs::path path;
            fs::file_time_type mtime;
            uint64_t size_bytes{0};
        };

        std::vector<Candidate> candidates;
        std::error_code ec;
        for (const auto& entry : fs::directory_iterator(record_dir, fs::directory_options::skip_permission_denied, ec)) {
            if (ec) break;
            const fs::path path = entry.path();
            if (!isEventRecordDirectory(path)) continue;
            if (protected_dirs.count(canonicalPathString(path)) > 0) continue;

            std::error_code time_ec;
            const auto mtime = fs::last_write_time(path, time_ec);
            candidates.push_back({path, time_ec ? fs::file_time_type::min() : mtime, pathSizeBytes(path)});
        }

        std::sort(candidates.begin(), candidates.end(), [](const Candidate& a, const Candidate& b) {
            if (a.mtime == b.mtime) return a.path.string() < b.path.string();
            return a.mtime < b.mtime;
        });

        uint64_t deleted_count = 0;
        uint64_t freed_bytes = 0;
        for (const auto& candidate : candidates) {
            if (total_bytes <= target_bytes) break;
            std::error_code remove_ec;
            const auto removed = fs::remove_all(candidate.path, remove_ec);
            if (remove_ec) {
                std::cerr << "[SentinelNode][RecordingCleanup] remove_failed path="
                          << candidate.path.string() << " error=" << remove_ec.message() << std::endl;
                continue;
            }
            ++deleted_count;
            freed_bytes += candidate.size_bytes;
            total_bytes = total_bytes > candidate.size_bytes ? total_bytes - candidate.size_bytes : 0;
            std::cout << "[SentinelNode][RecordingCleanup] removed path=" << candidate.path.string()
                      << " files=" << removed
                      << " size_bytes=" << candidate.size_bytes
                      << " remaining_bytes=" << total_bytes << std::endl;
        }

        std::cout << "[SentinelNode][RecordingCleanup] done dir=" << record_dir.string()
                  << " max_bytes=" << max_bytes
                  << " target_bytes=" << target_bytes
                  << " final_bytes=" << total_bytes
                  << " deleted_dirs=" << deleted_count
                  << " freed_bytes=" << freed_bytes << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "[SentinelNode][RecordingCleanup] failed: " << e.what() << std::endl;
    }
}

void SentinelNode::rotateEventLogIfNeeded() const {
    if (cfg_.event_log_max_bytes == 0 || cfg_.event_log_path.empty()) return;

    try {
        const fs::path log_path(cfg_.event_log_path);
        std::error_code ec;
        if (!fs::exists(log_path, ec) || !fs::is_regular_file(log_path, ec)) return;

        const auto size = fs::file_size(log_path, ec);
        if (ec || size < cfg_.event_log_max_bytes) return;

        const fs::path rotated_path(log_path.string() + ".1");
        fs::remove(rotated_path, ec);
        ec.clear();
        fs::rename(log_path, rotated_path, ec);
        if (ec) {
            std::cerr << "[SentinelNode][EventLogRotate] rotate_failed path="
                      << log_path.string() << " error=" << ec.message() << std::endl;
            return;
        }

        std::cout << "[SentinelNode][EventLogRotate] rotated path=" << log_path.string()
                  << " rotated_path=" << rotated_path.string()
                  << " size_bytes=" << size
                  << " max_bytes=" << cfg_.event_log_max_bytes << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "[SentinelNode][EventLogRotate] failed: " << e.what() << std::endl;
    }
}

std::vector<std::string> SentinelNode::recordEventClip(const AlertEvent& evt) {
    if (!cfg_.enable_event_recording) return {};

    const uint64_t event_id = next_event_id_++;
    std::ostringstream dir;
    dir << cfg_.event_record_dir << "/event_" << event_id;
    fs::create_directories(dir.str());

    std::vector<FrameRecord> frames;
    {
        std::lock_guard<std::mutex> lk(record_mutex_);
        frames.assign(recent_frames_.begin(), recent_frames_.end());
    }

    std::vector<std::string> copied;
    for (const auto& f : frames) {
        if (!fs::exists(f.path)) continue;
        std::ostringstream dst;
        dst << dir.str() << "/pre_seq_" << f.seq << ".jpg";
        std::error_code ec;
        fs::copy_file(f.path, dst.str(), fs::copy_options::overwrite_existing, ec);
        if (!ec) copied.push_back(dst.str());
    }

    const std::string manifest = dir.str() + "/manifest.json";
    std::ofstream os(manifest);
    os << "{\n"
       << "  \"event_id\": " << event_id << ",\n"
       << "  \"timestamp_ns\": " << evt.timestamp_ns << ",\n"
       << "  \"level\": " << static_cast<int>(evt.level) << ",\n"
       << "  \"threat_level\": " << sentryManifestThreatLevel(evt.level) << ",\n"
       << "  \"pre_seconds\": " << cfg_.event_pre_seconds << ",\n"
       << "  \"post_seconds\": " << cfg_.event_post_seconds << ",\n"
       << "  \"frames\": [";
    for (size_t i = 0; i < copied.size(); ++i) {
        os << (i ? ", " : "") << "\"" << copied[i] << "\"";
    }
    os << "]\n}\n";
    copied.push_back(manifest);
    return copied;
}

void SentinelNode::stateBroadcastLoop() {
    using clock = std::chrono::steady_clock;
    auto next_tick = clock::now();
    while (state_broadcast_running_.load()) {
        next_tick += std::chrono::milliseconds(100);
        publishSentryStateBroadcastTick();
        std::this_thread::sleep_until(next_tick);
        if (clock::now() > next_tick + std::chrono::seconds(1)) {
            next_tick = clock::now();
        }
    }
}

void SentinelNode::publishSentryStateBroadcastTick(uint64_t output_timestamp_ns) {
    if (!rscl_publisher_ || !rscl_publisher_->enabled()) return;

    const SentryState sentry_state = state_machine_ ? state_machine_->currentState()
                                                    : SentryState::UNAVAILABLE;
    const ThreatLevel live_level = alert_manager_ ? alert_manager_->currentLevel()
                                                  : ThreatLevel::NONE;
    const int raw_threat_id = alert_manager_ ? alert_manager_->currentThreatId() : 0;
    const int sm_threat_id = state_machine_ ? state_machine_->activeThreatId() : 0;
    const int target_count = alert_manager_ ? alert_manager_->currentTargetCount() : 0;
    const SentinelObstacleType obstacle =
        alert_manager_ ? alert_manager_->currentObstacleType()
                       : SentinelObstacleType::UNKNOWN_OBSTACLE;
    const std::string reason = alert_manager_ ? alert_manager_->currentReason() : std::string();
    const std::string reason_code = threatReasonFrameToken(reason);

    const int id_for_camera = raw_threat_id != 0 ? raw_threat_id : sm_threat_id;
    const std::string threat_camera = cameraNameFromScopedThreatId(id_for_camera, cfg_.camera_names);
    const int local_track_id = id_for_camera > 0 ? (id_for_camera % 100000) : id_for_camera;

    uint64_t ts = output_timestamp_ns;
    if (ts == 0) ts = last_sensor_time_ns_.load(std::memory_order_relaxed);

    StateBroadcastView msg;
    msg.timestamp_ns = ts;
    msg.sentry_state = static_cast<int>(sentry_state);
    msg.sentry_state_name = sentryStateNameEn(sentry_state);
    msg.threat_level = static_cast<int>(live_level);
    msg.threat_level_name = threatLevelNameEn(live_level);
    msg.raw_threat_id = raw_threat_id;
    msg.sm_threat_id = sm_threat_id;
    msg.local_track_id = local_track_id;
    msg.threat_camera = threat_camera;
    msg.target_count = target_count;
    msg.obstacle_type = static_cast<int>(obstacle);
    msg.obstacle_type_name = obstacleTypeNameEn(obstacle);
    msg.reason_code = reason_code;
    msg.reason = reason;

    std::ostringstream summary;
    summary << sentryStateFrameToken(sentry_state)
            << '|' << threatLevelFrameToken(live_level)
            << '|' << obstacleFrameToken(obstacle)
            << '|' << reason_code
            << "|cam=" << shortCameraToken(threat_camera)
            << "|raw=" << raw_threat_id
            << "|sm=" << sm_threat_id
            << "|n=" << target_count;
    msg.summary = summary.str();

    static std::atomic<uint64_t> broadcast_log_count{0};
    const uint64_t log_idx = ++broadcast_log_count;
    if (log_idx <= 5 || log_idx % 100 == 0 || live_level >= ThreatLevel::MEDIUM) {
        std::cout << "[SentinelNode][StateBroadcast] topic=" << cfg_.sentry_state_topic
                  << " summary=" << msg.summary
                  << " reason=" << reason
                  << std::endl;
    }

    rscl_publisher_->publishSentryStateBroadcast(msg);
}

void SentinelNode::publishSentryStatusTick(uint64_t output_timestamp_ns) {
    AlertEvent evt;
    SentinelResStatusView res;
    {
        std::lock_guard<std::mutex> lk(latest_sentry_status_mutex_);
        evt = latest_sentry_status_event_;
        res = latest_sentry_status_res_;
    }
    if (output_timestamp_ns > 0) {
        evt.timestamp_ns = output_timestamp_ns;
    }

    const SentinelResMsgType cached_res_type = res.sentinel_res_msg_type;
    const bool normalized_res_type =
        cached_res_type != SentinelResMsgType::UNKNOWN_SENTINEL_RES;
    // Status frames share the same RSCL topic as request responses, but they are
    // not request responses. Never let a cached request response type leak into
    // periodic/event-driven status output.
    res.sentinel_res_msg_type = SentinelResMsgType::UNKNOWN_SENTINEL_RES;

    const bool has_warning_status =
        evt.level >= ThreatLevel::MEDIUM ||
        res.sentinel_general_threat == SentinelGeneralThreat::ACTIVE ||
        res.sentinel_severe_threat == SentinelGeneralThreat::ACTIVE ||
        res.sentinel_audio_visual_alarm == SentinelGeneralThreat::ACTIVE;
    if (!has_warning_status) {
        return;
    }

    // 限频日志（固定 10Hz tick 下每 100 次约 10 秒一次，避免刷屏）
    static std::atomic<uint64_t> status_output_log_count{0};
    const uint64_t log_idx = ++status_output_log_count;
    if (log_idx <= 5 || log_idx % 100 == 0 || evt.level >= ThreatLevel::MEDIUM ||
        normalized_res_type) {
        std::cout << "[SentinelNode][StatusOutput][Sentry] mode=" << cfg_.status_publish_mode
                  << " res=" << static_cast<int>(res.sentinel_res_msg_type)
                  << " cached_res=" << static_cast<int>(cached_res_type)
                  << " normalized_res=" << (normalized_res_type ? 1 : 0)
                  << " level=" << static_cast<int>(evt.level)
                  << " state=" << static_cast<int>(evt.sentry_state)
                  << " odd=" << static_cast<int>(res.sentinel_odd_status)
                  << " general=" << static_cast<int>(res.sentinel_general_threat)
                  << " severe=" << static_cast<int>(res.sentinel_severe_threat)
                  << " alarm=" << static_cast<int>(res.sentinel_audio_visual_alarm)
                  << " light=" << (evt.light_req ? 1 : 0)
                  << " horn=" << (evt.horn_req ? 1 : 0)
                  << " hazard=" << (evt.hazard_lamp_req ? 1 : 0)
                  << std::endl;
    }

    if (rscl_publisher_) {
        rscl_publisher_->publishSentryEvent(evt, res, 0);
    }
}

void SentinelNode::publishOrsmStatusTick(uint64_t output_timestamp_ns) {
    if (!rscl_publisher_) return;
    const uint64_t publish_steady_ms = currentSteadyTimeMs();
    const uint64_t publish_system_ms = currentSystemTimeMs();

    ORSMAlertOutput output;
    uint64_t ts_ns = output_timestamp_ns > 0
        ? output_timestamp_ns
        : last_sensor_time_ns_.load(std::memory_order_relaxed);
    {
        std::lock_guard<std::mutex> lk(latest_orsm_output_mutex_);
        output = latest_orsm_output_;
        if (output_timestamp_ns == 0 && latest_orsm_output_ts_ns_ > 0) ts_ns = latest_orsm_output_ts_ns_;
    }

    static std::atomic<uint64_t> status_output_log_count{0};
    const uint64_t log_idx = ++status_output_log_count;
    const int orsm_state_value = orsm_manager_ ? static_cast<int>(orsm_manager_->currentState()) : 0;
    const int orsm_setting_fb_value = static_cast<int>(output.orsm_setting_st_fb);
    const int orsm_warning_value = static_cast<int>(output.orsm_warning_st);
    const int orsm_warn_level_value = static_cast<int>(output.orsm_warn_level);
    const int orsm_msg_rem_value = static_cast<int>(output.orsm_message_rem);
    const size_t orsm_intruder_count = output.intruders_info.size();

    // Bag replay/export may request a lossless ORSM trace.  The normal console
    // log below is intentionally rate-limited, so it cannot be used to rebuild
    // a moving trajectory offline.  Keep this opt-in and separate from stdout
    // to avoid changing production log volume or the published message.
    if (const char* trace_path = std::getenv("ORSM_OFFLINE_TRACE_PATH");
        trace_path != nullptr && *trace_path != '\0') {
        static std::mutex trace_mutex;
        static std::string opened_trace_path;
        static std::ofstream trace;
        static uint64_t last_trace_ts = 0;
        std::lock_guard<std::mutex> trace_lock(trace_mutex);
        if (opened_trace_path != trace_path) {
            trace.close();
            const fs::path path(trace_path);
            if (!path.parent_path().empty()) {
                std::error_code ec;
                fs::create_directories(path.parent_path(), ec);
            }
            trace.open(path, std::ios::out | std::ios::trunc);
            opened_trace_path = trace_path;
            last_trace_ts = 0;
        }
        if (trace) {
            std::vector<std::string> binding_events;
            {
                std::lock_guard<std::mutex> binding_lock(orsm_binding_trace_mutex_);
                binding_events.swap(pending_orsm_binding_trace_events_);
            }
            for (const auto& event : binding_events) trace << event << '\n';
            // last_sensor_time_ns advances with the replay clock even while a
            // held ORSM output has an older observation timestamp.  It is the
            // correct time axis for the 10 Hz status publication.
            uint64_t trace_ts = last_sensor_time_ns_.load(std::memory_order_relaxed) > 0
                ? last_sensor_time_ns_.load(std::memory_order_relaxed)
                : ts_ns;
            // Multiple replay topics can arrive slightly out of timestamp
            // order.  The published 10 Hz trace is consumed sequentially by
            // the offline renderer, so never allow its time axis to go back.
            trace_ts = std::max(trace_ts, last_trace_ts);
            last_trace_ts = trace_ts;
            trace << "[ORSMOfflineTrace] ts=" << trace_ts
                  << " seq=" << log_idx
                  << " state=" << orsm_state_value
                  << " setting_fb=" << orsm_setting_fb_value
                  << " warning_st=" << orsm_warning_value
                  << " warn_level=" << orsm_warn_level_value
                  << " intruders=" << orsm_intruder_count
                  << " msg_rem=" << orsm_msg_rem_value
                  << " intruder_details=" << summarizeOrsmIntruders(output.intruders_info)
                  << '\n';
            trace.flush();
        }
    }
    static int last_orsm_state_value = -1;
    static int last_orsm_setting_fb_value = -1;
    static int last_orsm_warning_value = -1;
    static int last_orsm_warn_level_value = -1;
    static int last_orsm_msg_rem_value = -1;
    const bool orsm_changed =
        orsm_state_value != last_orsm_state_value ||
        orsm_setting_fb_value != last_orsm_setting_fb_value ||
        orsm_warning_value != last_orsm_warning_value ||
        orsm_warn_level_value != last_orsm_warn_level_value ||
        orsm_msg_rem_value != last_orsm_msg_rem_value;
    if (cfg_.orsm_debug_track || log_idx <= 5 || log_idx % 100 == 0 || orsm_changed) {
        std::cout << "[SentinelNode][StatusOutput][ORSM] publish=fixed_10hz"
                  << " publish_steady_ms=" << publish_steady_ms
                  << " publish_system_ms=" << publish_system_ms
                  << " output_ts_ns=" << ts_ns
                  << " seq=" << log_idx
                  << " state=" << orsm_state_value
                  << " setting_fb=" << orsm_setting_fb_value
                  << " warning_st=" << orsm_warning_value
                  << " warn_level=" << orsm_warn_level_value
                  << " intruders=" << orsm_intruder_count
                  << " msg_rem=" << orsm_msg_rem_value
                  << " intruder_details=" << summarizeOrsmIntruders(output.intruders_info)
                  << std::endl;
        last_orsm_state_value = orsm_state_value;
        last_orsm_setting_fb_value = orsm_setting_fb_value;
        last_orsm_warning_value = orsm_warning_value;
        last_orsm_warn_level_value = orsm_warn_level_value;
        last_orsm_msg_rem_value = orsm_msg_rem_value;
    }

    rscl_publisher_->publishOrsmState(output, ts_ns);
}

void SentinelNode::publishSentryOutputs(uint64_t event_id,
                                        const AlertEvent& evt,
                                        const std::vector<std::string>& video_paths,
                                        bool publish_action) {
    const fs::path log_path(cfg_.event_log_path);
    const fs::path log_dir = log_path.parent_path();
    if (!log_dir.empty() && !fs::exists(log_dir)) {
        fs::create_directories(log_dir);
    }
    rotateEventLogIfNeeded();
    const auto res = makeSentinelResStatus(latest_odd_, evt);
    const bool should_publish_action = publish_action && evt.level == ThreatLevel::HIGH;
    const bool action_published = should_publish_action && rscl_publisher_ &&
                                  rscl_publisher_->enabled();
    if (should_publish_action && rscl_publisher_) {
        // RsclPublisher remains a no-op while enable_rscl_publish=false.  On a
        // real vehicle with publishing enabled this sends the severe-threat
        // AlarmCtrl request exactly once per threat session escalation.
        rscl_publisher_->publishSentryAction(evt, res);
    }

    std::ofstream os(cfg_.event_log_path, std::ios::app);
    os << "{\"event_id\":" << event_id
       << ",\"topic_event\":\"" << cfg_.sentry_event_topic
       << "\",\"alarmctrl_publish\":" << (action_published ? "true" : "false")
       << ",\"topic_media_cmd\":\"" << cfg_.sentry_media_cmd_topic
       << "\",\"formal_msg\":\"SentinelResStatus\""
       << ",\"sentinel_res_msg_type\":" << static_cast<int>(res.sentinel_res_msg_type)
       << ",\"sentinel_odd_status\":" << static_cast<int>(res.sentinel_odd_status)
       << ",\"sentinel_enable_failed\":" << static_cast<int>(res.sentinel_enable_failed)
       << ",\"sentinel_general_threat\":" << static_cast<int>(res.sentinel_general_threat)
       << ",\"sentinel_severe_threat\":" << static_cast<int>(res.sentinel_severe_threat)
       << ",\"sentinel_audio_visual_alarm\":" << static_cast<int>(res.sentinel_audio_visual_alarm)
       << ",\"sentinel_obstacle_type\":" << static_cast<int>(res.sentinel_obstacle_type)
       << ",\"can_id\":\"0x3D4\""
       << ",\"level\":" << static_cast<int>(evt.level)
       << ",\"light_req\":" << (evt.light_req ? 2 : 0)
       << ",\"hazard_req\":" << (evt.hazard_lamp_req ? 2 : 0)
       << ",\"horn_req\":" << (evt.horn_req ? 1 : 0)
       << ",\"threat_id\":" << evt.threat_id
       << ",\"wake_screen\":" << (evt.level >= ThreatLevel::MEDIUM ? "true" : "false")
       << ",\"video_paths\":[";
    for (size_t i = 0; i < video_paths.size(); ++i) {
        os << (i ? "," : "") << "\"" << video_paths[i] << "\"";
    }
    os << "]"
       << ",\"reason\":\"" << evt.reason << "\"}" << std::endl;

    std::cout << "[SentinelNode][RecordingTrace] output_write"
              << " event_id=" << event_id
              << " video_paths=" << video_paths.size()
              << " alert_sensor_ts=" << evt.timestamp_ns
              << " output_wall_ns=" << currentSteadyTimeNs()
              << std::endl;

    std::cout << "[SentinelNode][Output] event_id=" << event_id
              << " event=" << cfg_.sentry_event_topic
              << " action=" << (action_published ? "PUBLISHED" : "NOT_PUBLISHED")
              << " media_cmd=" << cfg_.sentry_media_cmd_topic
              << " SentinelResStatus(odd=" << static_cast<int>(res.sentinel_odd_status)
              << ", general=" << static_cast<int>(res.sentinel_general_threat)
              << ", severe=" << static_cast<int>(res.sentinel_severe_threat)
              << ", alarm=" << static_cast<int>(res.sentinel_audio_visual_alarm)
              << ", obstacle=" << static_cast<int>(res.sentinel_obstacle_type) << ")"
              << " CAN(0x3D4): Light=" << (evt.light_req ? 2 : 0)
              << " Hazard=" << (evt.hazard_lamp_req ? 2 : 0)
              << " Horn=" << (evt.horn_req ? 1 : 0)
              << std::endl;

    // RSCL Status 由 status_publish_mode 控制：event_driven 模式下由状态机转换/请求响应发布；
    // 严重威胁的 AlarmCtrl 已在本次即时事件中单独发送，录像完成回调不重复动作。
}

// ---- 可视化 ----

void SentinelNode::visualize(cv::Mat& image,
                              const DecodedFrame& frame,
                              const std::vector<TrackedObject>& tracked,
                              ThreatLevel level,
                              SentryState state)
{
    const auto belongs_to_frame_camera = [&frame](const TrackedObject& obj) {
        return frame.camera_name.empty() || obj.source_camera.empty() ||
               obj.source_camera == frame.camera_name;
    };
    const int local_target_count = static_cast<int>(std::count_if(
        tracked.begin(), tracked.end(), belongs_to_frame_camera));

    // 状态颜色映射
    cv::Scalar level_color;
    std::string level_str;
    switch (level) {
        case ThreatLevel::HIGH:   level_color = {0,0,255};   level_str = "HIGH";   break;
        case ThreatLevel::MEDIUM: level_color = {0,165,255}; level_str = "MEDIUM"; break;
        case ThreatLevel::LOW:    level_color = {0,255,255}; level_str = "LOW";    break;
        default:                  level_color = {0,255,0};   level_str = "CLEAR";  break;
    }

    static const char* kStateNames[] = {
        "UNAVAILABLE", "STANDBY", "ACTIVE", "WARNING", "ALARM"
    };
    const char* state_name = kStateNames[static_cast<int>(state)];

    // 威胁时显示边框
    if (level != ThreatLevel::NONE)
        cv::rectangle(image, cv::Point(0,0),
                      cv::Point(image.cols-1, image.rows-1), level_color, 10);

    // 帧信息
    char buf[256];
    std::snprintf(buf, sizeof(buf), "seq=%lu  %s  [%s]  state=%s  targets=%d",
                  frame.seq,
                  frame.frame_type == 2 ? "I" : "P",
                  level_str.c_str(), state_name,
                  local_target_count);
    cv::putText(image, buf, cv::Point(10, 50),
                cv::FONT_HERSHEY_SIMPLEX, 1.0, level_color, 2);

    // IMU 数据
    std::snprintf(buf, sizeof(buf),
                  "IMU accel=(%.2f,%.2f,%.2f) gyro=(%.2f,%.2f,%.2f)%s",
                  frame.imu.accel_x, frame.imu.accel_y, frame.imu.accel_z,
                  frame.imu.gyro_x,  frame.imu.gyro_y,  frame.imu.gyro_z,
                  frame.imu.vibration_alert ? " [VIBRATION]" : "");
    cv::putText(image, buf, cv::Point(10, 100),
                cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 200, 0), 2);

    // 跟踪目标框 + 轨迹
    for (const auto& obj : tracked) {
        // A completed synchronized group contains targets from all four cameras.
        // Bboxes are camera-local pixel coordinates, so drawing another camera's
        // target on this frame creates convincing but false detections.
        if (!belongs_to_frame_camera(obj)) continue;
        cv::rectangle(image, obj.bbox, level_color, 2);
        std::snprintf(buf, sizeof(buf), "%s #%d %.0f%%",
                      obj.label.c_str(), obj.track_id, obj.confidence * 100);
        cv::putText(image, buf,
                    cv::Point(obj.bbox.x, obj.bbox.y - 8),
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, level_color, 2);
        for (size_t i = 1; i < obj.trajectory.size(); ++i)
            cv::line(image,
                     cv::Point(obj.trajectory[i-1]),
                     cv::Point(obj.trajectory[i]),
                     cv::Scalar(255, 100, 100), 2);
    }
}

}  // namespace sentinel
