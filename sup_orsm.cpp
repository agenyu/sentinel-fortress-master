#ifndef SUP_ORSM_OFFLINE_LOG
#include <ad_rscl/ad_rscl.h>
#include <ad_msg_idl/l3_msgs/orsm_info.capnp.h>
#include "rscl_access/rscl_access.h"
#endif

#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cmath>
#include <cstdint>
#include <csignal>
#include <cstdlib>
#include <ctime>
#include <deque>
#include <iomanip>
#include <fstream>
#include <iostream>
#include <mutex>
#include <regex>
#include <sstream>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

namespace {

// 本地枚举值与 l3_msgs/orsm_info 保持一致，使离线日志模式无需车端L3 SDK。
enum class DetecRangeSetting : uint8_t { NONE_RANGE = 0, RANGE_A = 1, RANGE_B = 2, RANGE_C = 3 };
enum class IntruderType : uint8_t { INTRUDER_UNKNOWN = 0, INTRUDER_ANIMAL = 1, INTRUDER_HUMANKIND = 2 };
enum class IntrudersNumType : uint8_t { NO_INTRUDER = 0, SINGLE_INTRUDER = 1, MULTIPLE_INTRUDERS = 2 };
enum class OrsmWarningStatus : uint8_t { WARNING_OFF = 0, WARNING_ON = 1 };
enum class OrsmWarnLevel : uint8_t { LEVEL_NONE = 0, WARN_LEVEL1 = 1, WARN_LEVEL2 = 2, WARN_LEVEL3 = 3 };
enum class OrsmSettingType : uint8_t {
    OFF = 0, SINGLE_AUTO_SECURITY_ON = 1, MULTI_VEHICLE_JOINT_SECURITY_ON = 2,
    FUNCTION_LIMITED_ENABLE_FAILED = 3
};
enum class OrsmMessageReminder : uint8_t {
    ORSM_NONE = 0, SECURITY_MODE_ON = 1, LOW_BATTERY = 2,
    NOT_IN_ON_READY_STATE_FUNCTION_ENTRY_FAILED = 3,
    GEAR_NOT_IN_P_POSITION_FUNCTION_ENTRY_FAILED = 4,
    LOW_BATTERY_FUNCTION_ENTRY_FAILED = 5,
    INTELLIGENT_DRIVING_SYSTEM_MALFUNCTION_FUNCTION_ENTRY_FAILED = 6,
    RADAR_CAMERA_MALFUNCTION_FUNCTION_ENTRY_FAILED = 7,
    SECURITY_FUNCTION_EXITED = 8, LOW_BATTERY_FUNCTION_EXITED = 9,
    MULTI_VEHICLE_JOINT_SECURITY_FUNCTION_EXITED = 10,
    VEHICLE_SPEED_TOO_HIGH_FUNCTION_CANNOT_BE_ACTIVATED = 11,
    VEHICLE_SPEED_TOO_HIGH_JOINT_SECURITY_FUNCTION_EXITED = 12,
    NETWORK_CONNECTION_DISCONNECTED_JOINT_SECURITY_FUNCTION_EXITED = 13
};

constexpr int kCanvasWidth = 1280;
constexpr int kCanvasHeight = 880;
constexpr int kMarginLeft = 440;
constexpr int kMarginRight = 40;
constexpr int kMarginTop = 55;
constexpr int kMarginBottom = 70;
constexpr int kLabelPaddingX = 7;
constexpr int kLabelPaddingY = 5;
constexpr float kZeroVelocityThreshold = 0.05F;
constexpr float kArrowSeconds = 1.5F;
constexpr double kOutputFps = 10.0;
constexpr int kFrameBudgetMs = 100;
constexpr int kStopPollIntervalMs = 100;
constexpr size_t kMaxHistoryPoints = 2000;
constexpr size_t kVideoQueueLimit = 30;
constexpr const char* kOutputPathEnv = "ORSM_VIDEO_PATH";

volatile std::sig_atomic_t g_stop_requested = 0;

void HandleStopSignal(int) {
    g_stop_requested = 1;
}

bool StopRequestedBySignal() {
    return g_stop_requested != 0;
}

void InstallSignalHandlers() {
    std::signal(SIGINT, HandleStopSignal);
    std::signal(SIGTERM, HandleStopSignal);
}

struct IntruderPoint {
    uint32_t id = 0;
    IntruderType type = IntruderType::INTRUDER_UNKNOWN;
    float x = 0.0F;
    float y = 0.0F;
    float vx = 0.0F;
    float vy = 0.0F;
    uint64_t timestamp_nsec = 0;
};

struct OrsmState {
    OrsmWarnLevel warn_level = OrsmWarnLevel::LEVEL_NONE;
    IntrudersNumType intruders_num = IntrudersNumType::NO_INTRUDER;
    OrsmWarningStatus warning_status = OrsmWarningStatus::WARNING_OFF;
    DetecRangeSetting range_setting = DetecRangeSetting::NONE_RANGE;
    OrsmMessageReminder message_reminder = OrsmMessageReminder::ORSM_NONE;
    OrsmSettingType setting_type = OrsmSettingType::OFF;
    size_t intruders_info_count = 0;
    uint64_t frame_index = 0;
    bool has_frame = false;
};

struct BevRange {
    float x_min = -30.0F;
    float x_max = 50.0F;
    float y_min = -30.0F;
    float y_max = 30.0F;
    float reference_radius = 30.0F;
    std::string label = "A";
};

std::string ToString(OrsmWarnLevel value) {
    switch (value) {
        case OrsmWarnLevel::LEVEL_NONE:
            return "levelNone";
        case OrsmWarnLevel::WARN_LEVEL1:
            return "warnLevel1";
        case OrsmWarnLevel::WARN_LEVEL2:
            return "warnLevel2";
        case OrsmWarnLevel::WARN_LEVEL3:
            return "warnLevel3";
        default:
            return "unknown(" + std::to_string(static_cast<int>(value)) + ")";
    }
}

std::string ToString(IntrudersNumType value) {
    switch (value) {
        case IntrudersNumType::NO_INTRUDER:
            return "noIntruder";
        case IntrudersNumType::SINGLE_INTRUDER:
            return "singleIntruder";
        case IntrudersNumType::MULTIPLE_INTRUDERS:
            return "multipleIntruders";
        default:
            return "unknown(" + std::to_string(static_cast<int>(value)) + ")";
    }
}

std::string ToString(OrsmWarningStatus value) {
    switch (value) {
        case OrsmWarningStatus::WARNING_OFF:
            return "warningOff";
        case OrsmWarningStatus::WARNING_ON:
            return "warningOn";
        default:
            return "unknown(" + std::to_string(static_cast<int>(value)) + ")";
    }
}

std::string ToString(DetecRangeSetting value) {
    switch (value) {
        case DetecRangeSetting::NONE_RANGE:
            return "noneRange";
        case DetecRangeSetting::RANGE_A:
            return "rangeA";
        case DetecRangeSetting::RANGE_B:
            return "rangeB";
        case DetecRangeSetting::RANGE_C:
            return "rangeC";
        default:
            return "unknown(" + std::to_string(static_cast<int>(value)) + ")";
    }
}

std::string ToString(OrsmMessageReminder value) {
    switch (value) {
        case OrsmMessageReminder::ORSM_NONE:
            return "orsmNone";
        case OrsmMessageReminder::SECURITY_MODE_ON:
            return "securityModeOn";
        case OrsmMessageReminder::LOW_BATTERY:
            return "lowBattery";
        case OrsmMessageReminder::NOT_IN_ON_READY_STATE_FUNCTION_ENTRY_FAILED:
            return "notInOnReady";
        case OrsmMessageReminder::GEAR_NOT_IN_P_POSITION_FUNCTION_ENTRY_FAILED:
            return "gearNotInP";
        case OrsmMessageReminder::LOW_BATTERY_FUNCTION_ENTRY_FAILED:
            return "lowBatteryEntryFailed";
        case OrsmMessageReminder::INTELLIGENT_DRIVING_SYSTEM_MALFUNCTION_FUNCTION_ENTRY_FAILED:
            return "idsMalfunction";
        case OrsmMessageReminder::RADAR_CAMERA_MALFUNCTION_FUNCTION_ENTRY_FAILED:
            return "radarCameraMalfunction";
        case OrsmMessageReminder::SECURITY_FUNCTION_EXITED:
            return "securityExited";
        case OrsmMessageReminder::LOW_BATTERY_FUNCTION_EXITED:
            return "lowBatteryExited";
        case OrsmMessageReminder::MULTI_VEHICLE_JOINT_SECURITY_FUNCTION_EXITED:
            return "jointSecurityExited";
        case OrsmMessageReminder::VEHICLE_SPEED_TOO_HIGH_FUNCTION_CANNOT_BE_ACTIVATED:
            return "speedTooHighCannotActivate";
        case OrsmMessageReminder::VEHICLE_SPEED_TOO_HIGH_JOINT_SECURITY_FUNCTION_EXITED:
            return "speedTooHighJointExited";
        case OrsmMessageReminder::NETWORK_CONNECTION_DISCONNECTED_JOINT_SECURITY_FUNCTION_EXITED:
            return "networkDisconnectedJointExited";
        default:
            return "unknown(" + std::to_string(static_cast<int>(value)) + ")";
    }
}

std::string ToString(OrsmSettingType value) {
    switch (value) {
        case OrsmSettingType::OFF:
            return "off";
        case OrsmSettingType::SINGLE_AUTO_SECURITY_ON:
            return "singleAutoSecurityOn";
        case OrsmSettingType::MULTI_VEHICLE_JOINT_SECURITY_ON:
            return "multiVehicleJointSecurityOn";
        case OrsmSettingType::FUNCTION_LIMITED_ENABLE_FAILED:
            return "functionLimitedEnableFailed";
        default:
            return "unknown(" + std::to_string(static_cast<int>(value)) + ")";
    }
}

std::string ToString(IntruderType value) {
    switch (value) {
        case IntruderType::INTRUDER_UNKNOWN:
            return "unknown";
        case IntruderType::INTRUDER_ANIMAL:
            return "animal";
        case IntruderType::INTRUDER_HUMANKIND:
            return "human";
        default:
            return "unknown(" + std::to_string(static_cast<int>(value)) + ")";
    }
}

BevRange DisplayRange() {
    return {-30.0F, 50.0F, -30.0F, 30.0F, 30.0F, "A"};
}

float Speed(const IntruderPoint& point) {
    return std::sqrt(point.vx * point.vx + point.vy * point.vy);
}

std::string FormatFloat(float value, int precision) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(precision) << value;
    return oss.str();
}

cv::Rect PlotRect(const BevRange& range) {
    const cv::Rect full(kMarginLeft,
                        kMarginTop,
                        kCanvasWidth - kMarginLeft - kMarginRight,
                        kCanvasHeight - kMarginTop - kMarginBottom);
    const float x_span = range.x_max - range.x_min;
    const float y_span = range.y_max - range.y_min;
    const float scale = std::min(static_cast<float>(full.width) / y_span,
                                 static_cast<float>(full.height) / x_span);
    const int width = std::max(1, static_cast<int>(std::round(y_span * scale)));
    const int height = std::max(1, static_cast<int>(std::round(x_span * scale)));
    return cv::Rect(full.x + (full.width - width) / 2,
                    full.y + (full.height - height) / 2,
                    width,
                    height);
}

cv::Point WorldToPixel(float x, float y, const BevRange& range, const cv::Rect& plot) {
    const float x_span = range.x_max - range.x_min;
    const float y_span = range.y_max - range.y_min;
    const float u = (range.y_max - y) / y_span;
    const float v = (range.x_max - x) / x_span;
    return cv::Point(plot.x + static_cast<int>(std::round(u * plot.width)),
                     plot.y + static_cast<int>(std::round(v * plot.height)));
}

bool IsInsidePlot(const cv::Point& point, const cv::Rect& plot) {
    return point.x >= plot.x && point.x <= plot.x + plot.width &&
           point.y >= plot.y && point.y <= plot.y + plot.height;
}

cv::Point ClampToPlot(cv::Point point, const cv::Rect& plot, int margin = 0) {
    point.x = std::max(plot.x + margin, std::min(point.x, plot.x + plot.width - margin));
    point.y = std::max(plot.y + margin, std::min(point.y, plot.y + plot.height - margin));
    return point;
}

cv::Rect TextBox(const std::string& text, cv::Point origin, double scale, int thickness) {
    int baseline = 0;
    const cv::Size size = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, scale, thickness, &baseline);
    return cv::Rect(origin.x - kLabelPaddingX,
                    origin.y - size.height - kLabelPaddingY,
                    size.width + kLabelPaddingX * 2,
                    size.height + baseline + kLabelPaddingY * 2);
}

void PutText(cv::Mat& image,
             const std::string& text,
             cv::Point origin,
             double scale,
             const cv::Scalar& color,
             int thickness = 1) {
    cv::putText(image, text, origin, cv::FONT_HERSHEY_SIMPLEX, scale,
                cv::Scalar(25, 25, 25), thickness + 2, cv::LINE_AA);
    cv::putText(image, text, origin, cv::FONT_HERSHEY_SIMPLEX, scale,
                color, thickness, cv::LINE_AA);
}

void PutTextBox(cv::Mat& image,
                const std::string& text,
                cv::Point origin,
                double scale,
                const cv::Scalar& text_color,
                const cv::Scalar& fill_color,
                const cv::Scalar& border_color,
                int thickness = 1) {
    const cv::Rect box = TextBox(text, origin, scale, thickness);
    cv::rectangle(image, box, fill_color, -1, cv::LINE_AA);
    cv::rectangle(image, box, border_color, 1, cv::LINE_AA);
    cv::putText(image, text, origin, cv::FONT_HERSHEY_SIMPLEX, scale,
                text_color, thickness, cv::LINE_AA);
}

void DrawRadarBackground(cv::Mat& image, const BevRange& range, const cv::Rect& plot) {
    cv::rectangle(image, plot, cv::Scalar(12, 24, 24), -1, cv::LINE_AA);
    cv::rectangle(image, plot, cv::Scalar(45, 95, 95), 2, cv::LINE_AA);

    const cv::Point center = WorldToPixel(0.0F, 0.0F, range, plot);

    const cv::Scalar grid_color(42, 82, 82);
    for (float x = -30.0F; x <= 50.0F; x += 5.0F) {
        const cv::Point left = WorldToPixel(x, range.y_min, range, plot);
        const cv::Point right = WorldToPixel(x, range.y_max, range, plot);
        const cv::Scalar color = std::abs(x) < 1e-4F ? cv::Scalar(70, 135, 135) : grid_color;
        cv::line(image, left, right, color, 1, cv::LINE_AA);
    }
    for (float y = -30.0F; y <= 30.0F; y += 5.0F) {
        const cv::Point bottom = WorldToPixel(range.x_min, y, range, plot);
        const cv::Point top = WorldToPixel(range.x_max, y, range, plot);
        const cv::Scalar color = std::abs(y) < 1e-4F ? cv::Scalar(70, 135, 135) : grid_color;
        cv::line(image, bottom, top, color, 1, cv::LINE_AA);
    }

    for (float radius = 5.0F; radius <= 30.0F; radius += 5.0F) {
        const cv::Point label_pos = WorldToPixel(0.0F, radius, range, plot);
        PutText(image, FormatFloat(radius, 0) + "m",
                cv::Point(label_pos.x + 6, label_pos.y - 5),
                0.38, cv::Scalar(145, 190, 190));
    }

    for (int deg = 0; deg < 360; deg += 30) {
        const double rad = deg * CV_PI / 180.0;
        const float x = 30.0F * static_cast<float>(std::cos(rad));
        const float y = 30.0F * static_cast<float>(std::sin(rad));
        const cv::Point end = WorldToPixel(x, y, range, plot);
        if (IsInsidePlot(end, plot)) {
            cv::line(image, center, end, cv::Scalar(28, 75, 75), 1, cv::LINE_AA);
        }
    }

    std::vector<cv::Point> forward_sector = {
        center,
        WorldToPixel(50.0F, -8.0F, range, plot),
        WorldToPixel(50.0F, 8.0F, range, plot),
    };
    cv::Mat overlay = image.clone();
    cv::fillConvexPoly(overlay, forward_sector, cv::Scalar(22, 70, 55), cv::LINE_AA);
    cv::addWeighted(overlay, 0.22, image, 0.78, 0.0, image);

    const cv::Point front = WorldToPixel(50.0F, 0.0F, range, plot);
    cv::arrowedLine(image, center, front, cv::Scalar(75, 190, 190), 2, cv::LINE_AA, 0, 0.04);
    cv::line(image, WorldToPixel(0.0F, range.y_min, range, plot),
             WorldToPixel(0.0F, range.y_max, range, plot), cv::Scalar(45, 125, 125), 1, cv::LINE_AA);

    PutText(image, "X forward", cv::Point(plot.x + plot.width / 2 - 46, plot.y - 18),
            0.52, cv::Scalar(175, 230, 230));
    PutText(image, "Y left", cv::Point(plot.x - 64, plot.y + plot.height / 2),
            0.52, cv::Scalar(175, 230, 230));
}

void DrawSingleRangeCircle(cv::Mat& image,
                           const BevRange& range,
                           const cv::Rect& plot,
                           float radius,
                           const cv::Scalar& color,
                           bool active) {
    const cv::Point center = WorldToPixel(0.0F, 0.0F, range, plot);
    const cv::Point radius_x = WorldToPixel(radius, 0.0F, range, plot);
    const cv::Point radius_y = WorldToPixel(0.0F, radius, range, plot);
    const int rx = std::max(1, std::abs(center.y - radius_x.y));
    const int ry = std::max(1, std::abs(center.x - radius_y.x));
    const int thickness = active ? 3 : 1;
    cv::ellipse(image, center, cv::Size(ry, rx), 0.0, 0.0, 360.0,
                color, thickness, cv::LINE_AA);
}

void DrawRangeReferences(cv::Mat& image,
                         const BevRange& range,
                         const cv::Rect& plot,
                         DetecRangeSetting active_range) {
    DrawSingleRangeCircle(image, range, plot, 30.0F,
                          cv::Scalar(80, 160, 255),
                          active_range == DetecRangeSetting::RANGE_A ||
                              active_range == DetecRangeSetting::NONE_RANGE);
    DrawSingleRangeCircle(image, range, plot, 15.0F,
                          cv::Scalar(80, 220, 120),
                          active_range == DetecRangeSetting::RANGE_B);
    DrawSingleRangeCircle(image, range, plot, 4.5F,
                          cv::Scalar(240, 190, 70),
                          active_range == DetecRangeSetting::RANGE_C);

    const cv::Point center = WorldToPixel(0.0F, 0.0F, range, plot);
    const cv::Point forward = WorldToPixel(50.0F, 0.0F, range, plot);
    cv::line(image, center, forward, cv::Scalar(80, 160, 255), 2, cv::LINE_AA);
}

void DrawEgoVehicle(cv::Mat& image, const BevRange& range, const cv::Rect& plot) {
    const cv::Point center = WorldToPixel(0.0F, 0.0F, range, plot);
    std::vector<cv::Point> car = {
        cv::Point(center.x, center.y - 16),
        cv::Point(center.x - 10, center.y + 12),
        cv::Point(center.x + 10, center.y + 12),
    };
    cv::fillConvexPoly(image, car, cv::Scalar(235, 235, 235), cv::LINE_AA);
    cv::polylines(image, car, true, cv::Scalar(20, 20, 20), 1, cv::LINE_AA);
    PutText(image, "ego", cv::Point(center.x + 12, center.y + 5), 0.45, cv::Scalar(230, 230, 230));
}

void DrawPoint(cv::Mat& image,
               const IntruderPoint& point,
               const BevRange& range,
               const cv::Rect& plot,
               const cv::Scalar& color,
               int radius,
               bool draw_arrow) {
    const cv::Point center = WorldToPixel(point.x, point.y, range, plot);
    if (!IsInsidePlot(center, plot)) {
        return;
    }

    cv::circle(image, center, radius + 2, cv::Scalar(20, 20, 20), -1, cv::LINE_AA);
    cv::circle(image, center, radius, color, -1, cv::LINE_AA);

    if (draw_arrow && Speed(point) > kZeroVelocityThreshold) {
        const float arrow_x = point.x + point.vx * kArrowSeconds;
        const float arrow_y = point.y + point.vy * kArrowSeconds;
        const cv::Point arrow_end = WorldToPixel(arrow_x, arrow_y, range, plot);
        cv::arrowedLine(image, center, arrow_end, color, 2, cv::LINE_AA, 0, 0.25);
    }
}

cv::Scalar TrackTrailColor(uint32_t id) {
    // 稳定ID映射到稳定颜色；避免不同目标的历史点看起来像同一条轨迹。
    const uint32_t hash = id * 2654435761U;
    return cv::Scalar(80 + (hash & 0x7f),
                      80 + ((hash >> 8) & 0x7f),
                      80 + ((hash >> 16) & 0x7f));
}

void DrawTrackTrails(cv::Mat& image,
                     const std::vector<IntruderPoint>& history_points,
                     const std::vector<IntruderPoint>& previous_points,
                     const std::vector<IntruderPoint>& current_points,
                     const BevRange& range,
                     const cv::Rect& plot) {
    std::unordered_map<uint32_t, std::vector<IntruderPoint>> tracks;
    for (const auto& point : history_points) tracks[point.id].push_back(point);
    for (const auto& point : previous_points) tracks[point.id].push_back(point);
    for (const auto& point : current_points) tracks[point.id].push_back(point);

    constexpr size_t kMaxTrailSamplesPerTrack = 60;
    constexpr float kMaxConnectedStepM = 2.0F;
    for (const auto& [id, points] : tracks) {
        const size_t begin = points.size() > kMaxTrailSamplesPerTrack
            ? points.size() - kMaxTrailSamplesPerTrack : 0;
        const cv::Scalar color = TrackTrailColor(id);
        for (size_t i = begin; i < points.size(); ++i) {
            const cv::Point pixel = WorldToPixel(points[i].x, points[i].y, range, plot);
            if (!IsInsidePlot(pixel, plot)) continue;
            cv::circle(image, pixel, 3, color, -1, cv::LINE_AA);
            if (i == begin) continue;
            const float dx = points[i].x - points[i - 1].x;
            const float dy = points[i].y - points[i - 1].y;
            if (std::hypot(dx, dy) > kMaxConnectedStepM) continue;
            const cv::Point previous = WorldToPixel(points[i - 1].x, points[i - 1].y, range, plot);
            if (IsInsidePlot(previous, plot)) {
                cv::line(image, previous, pixel, color, 2, cv::LINE_AA);
            }
        }
    }
}

std::string LabelForPoint(const IntruderPoint& point) {
    return "ID:" + std::to_string(point.id) + " " + ToString(point.type) +
           " v:" + FormatFloat(Speed(point), 2) + "m/s";
}

std::string IntruderTypesSummary(const std::vector<IntruderPoint>& points) {
    int unknown = 0;
    int animal = 0;
    int human = 0;
    for (const auto& point : points) {
        switch (point.type) {
            case IntruderType::INTRUDER_ANIMAL:
                ++animal;
                break;
            case IntruderType::INTRUDER_HUMANKIND:
                ++human;
                break;
            case IntruderType::INTRUDER_UNKNOWN:
            default:
                ++unknown;
                break;
        }
    }

    std::ostringstream oss;
    oss << "unknown:" << unknown << " animal:" << animal << " human:" << human;
    return oss.str();
}

bool IntersectsAny(const cv::Rect& rect, const std::vector<cv::Rect>& occupied) {
    for (const auto& item : occupied) {
        if ((rect & item).area() > 0) {
            return true;
        }
    }
    return false;
}

cv::Point PickLabelOrigin(const std::string& label,
                          const cv::Point& anchor,
                          const cv::Rect& plot,
                          const std::vector<cv::Rect>& occupied) {
    const double scale = 0.46;
    const int thickness = 1;
    const std::vector<cv::Point> offsets = {
        {12, -12}, {12, 24}, {-210, -12}, {-210, 24},
        {18, -42}, {18, 54}, {-210, -42}, {-210, 54},
        {36, -72}, {36, 84}, {-230, -72}, {-230, 84},
    };

    for (const auto& offset : offsets) {
        cv::Point origin(anchor.x + offset.x, anchor.y + offset.y);
        cv::Rect box = TextBox(label, origin, scale, thickness);
        if (box.x < plot.x || box.y < plot.y ||
            box.x + box.width > plot.x + plot.width ||
            box.y + box.height > plot.y + plot.height) {
            continue;
        }
        if (!IntersectsAny(box, occupied)) {
            return origin;
        }
    }

    cv::Point fallback(anchor.x + 12, anchor.y - 12);
    cv::Rect box = TextBox(label, fallback, scale, thickness);
    if (box.x + box.width > plot.x + plot.width) {
        fallback.x -= box.x + box.width - (plot.x + plot.width);
    }
    if (box.x < plot.x) {
        fallback.x += plot.x - box.x;
    }
    box = TextBox(label, fallback, scale, thickness);
    if (box.y < plot.y) {
        fallback.y += plot.y - box.y;
    }
    if (box.y + box.height > plot.y + plot.height) {
        fallback.y -= box.y + box.height - (plot.y + plot.height);
    }
    return fallback;
}

void DrawCurrentLabels(cv::Mat& image,
                       const std::vector<IntruderPoint>& history_points,
                       const std::vector<IntruderPoint>& previous_points,
                       const std::vector<IntruderPoint>& points,
                       const BevRange& range,
                       const cv::Rect& plot) {
    std::vector<cv::Rect> occupied;
    for (const auto& point : history_points) {
        const cv::Point center = WorldToPixel(point.x, point.y, range, plot);
        if (IsInsidePlot(center, plot)) {
            occupied.emplace_back(center.x - 9, center.y - 9, 18, 18);
        }
    }
    for (const auto& point : previous_points) {
        const cv::Point center = WorldToPixel(point.x, point.y, range, plot);
        if (IsInsidePlot(center, plot)) {
            occupied.emplace_back(center.x - 11, center.y - 11, 22, 22);
        }
    }
    for (const auto& point : points) {
        const cv::Point center = WorldToPixel(point.x, point.y, range, plot);
        if (!IsInsidePlot(center, plot)) {
            continue;
        }
        occupied.emplace_back(center.x - 13, center.y - 13, 26, 26);
    }

    for (const auto& point : points) {
        const cv::Point center = WorldToPixel(point.x, point.y, range, plot);
        if (!IsInsidePlot(center, plot)) {
            continue;
        }
        const std::string label = LabelForPoint(point);
        const cv::Point origin = PickLabelOrigin(label, center, plot, occupied);
        const cv::Rect box = TextBox(label, origin, 0.46, 1);
        cv::line(image, center, cv::Point(box.x, box.y + box.height / 2),
                 cv::Scalar(180, 180, 180), 1, cv::LINE_AA);
        PutTextBox(image, label, origin, 0.46, cv::Scalar(245, 245, 245),
                   cv::Scalar(28, 28, 28), cv::Scalar(90, 90, 90));
        occupied.push_back(box);
    }
}

void DrawLegend(cv::Mat& image, const cv::Rect& plot) {
    const int x = plot.x + plot.width - 185;
    const int y = plot.y + plot.height - 74;
    cv::rectangle(image, cv::Rect(x - 12, y - 28, 185, 72), cv::Scalar(35, 35, 35), -1, cv::LINE_AA);
    cv::rectangle(image, cv::Rect(x - 12, y - 28, 185, 72), cv::Scalar(80, 80, 80), 1, cv::LINE_AA);
    cv::circle(image, cv::Point(x, y - 8), 5, cv::Scalar(0, 0, 255), -1, cv::LINE_AA);
    PutText(image, "current frame", cv::Point(x + 14, y - 3), 0.44, cv::Scalar(225, 225, 225));
    cv::circle(image, cv::Point(x, y + 14), 5, cv::Scalar(0, 200, 0), -1, cv::LINE_AA);
    PutText(image, "previous frame", cv::Point(x + 14, y + 19), 0.44, cv::Scalar(225, 225, 225));
    cv::circle(image, cv::Point(x, y + 36), 5, cv::Scalar(210, 80, 230), -1, cv::LINE_AA);
    PutText(image, "older history", cv::Point(x + 14, y + 41), 0.44, cv::Scalar(225, 225, 225));
}

void DrawStatusPanel(cv::Mat& image,
                     const OrsmState& state,
                     const std::string& intruder_types,
                     size_t current_count,
                     size_t previous_count,
                     size_t history_count) {
    const cv::Rect panel(16, 14, 405, 284);
    cv::rectangle(image, panel, cv::Scalar(32, 32, 32), -1, cv::LINE_AA);
    cv::rectangle(image, panel, cv::Scalar(85, 85, 85), 1, cv::LINE_AA);

    int y = panel.y + 26;
    PutText(image, "ORSM Status", cv::Point(panel.x + 12, y), 0.58, cv::Scalar(245, 245, 245), 1);
    y += 24;
    PutText(image, "OrsmWarnLevel: " + ToString(state.warn_level),
            cv::Point(panel.x + 12, y), 0.48, cv::Scalar(225, 225, 225));
    y += 22;
    PutText(image, "IntrudersNumType: " + ToString(state.intruders_num),
            cv::Point(panel.x + 12, y), 0.48, cv::Scalar(225, 225, 225));
    y += 22;
    PutText(image, "OrsmWarningStatus: " + ToString(state.warning_status),
            cv::Point(panel.x + 12, y), 0.48, cv::Scalar(225, 225, 225));
    y += 22;
    PutText(image, "DetecRangeSetting: " + ToString(state.range_setting),
            cv::Point(panel.x + 12, y), 0.48, cv::Scalar(225, 225, 225));
    y += 22;
    PutText(image, "OrsmMessageReminder: " + ToString(state.message_reminder),
            cv::Point(panel.x + 12, y), 0.48, cv::Scalar(225, 225, 225));
    y += 22;
    PutText(image, "OrsmSettingType: " + ToString(state.setting_type),
            cv::Point(panel.x + 12, y), 0.48, cv::Scalar(225, 225, 225));
    y += 22;
    PutText(image, "IntruderType: " + intruder_types,
            cv::Point(panel.x + 12, y), 0.48, cv::Scalar(225, 225, 225));
    y += 22;
    PutText(image, "intrudersInfo list size: " + std::to_string(state.intruders_info_count),
            cv::Point(panel.x + 12, y), 0.48, cv::Scalar(225, 225, 225));
    y += 22;
    PutText(image, "Frame: " + std::to_string(state.frame_index) +
                       "  Current: " + std::to_string(current_count),
            cv::Point(panel.x + 12, y), 0.48, cv::Scalar(225, 225, 225));
    y += 22;
    PutText(image, "Previous: " + std::to_string(previous_count) +
                       "  OlderHistory: " + std::to_string(history_count),
            cv::Point(panel.x + 12, y), 0.48, cv::Scalar(225, 225, 225));
}

std::string BuildDefaultVideoPath() {
    const std::time_t now = std::time(nullptr);
    std::tm local_time {};
    localtime_r(&now, &local_time);

    std::ostringstream oss;
    oss << "orsm_bev_" << std::put_time(&local_time, "%Y%m%d_%H%M%S") << ".mp4";
    return oss.str();
}

std::string ResolveVideoPath() {
    const char* env_path = std::getenv(kOutputPathEnv);
    if (env_path != nullptr && env_path[0] != '\0') {
        return env_path;
    }
    return BuildDefaultVideoPath();
}

class AsyncVideoWriter {
public:
    AsyncVideoWriter() = default;
    AsyncVideoWriter(const AsyncVideoWriter&) = delete;
    AsyncVideoWriter& operator=(const AsyncVideoWriter&) = delete;

    ~AsyncVideoWriter() {
        Close();
    }

    bool Open(const std::string& path, double fps, const cv::Size& size) {
        const std::vector<std::pair<std::string, int>> codec_candidates = {
            {"mp4v", cv::VideoWriter::fourcc('m', 'p', '4', 'v')},
            {"avc1", cv::VideoWriter::fourcc('a', 'v', 'c', '1')},
            {"H264", cv::VideoWriter::fourcc('H', '2', '6', '4')},
        };

        for (const auto& codec : codec_candidates) {
            writer_.release();
            if (writer_.open(path, codec.second, fps, size, true)) {
                codec_name_ = codec.first;
                break;
            }
        }

        if (!writer_.isOpened()) {
            std::cerr << "Failed to open mp4 writer: " << path << std::endl;
            return false;
        }

        output_path_ = path;
        stop_requested_ = false;
        writer_thread_ = std::thread(&AsyncVideoWriter::WriterLoop, this);
        return true;
    }

    void Enqueue(cv::Mat&& frame) {
        if (frame.empty()) {
            return;
        }

        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (frames_.size() >= kVideoQueueLimit) {
                frames_.pop_front();
                ++dropped_frames_;
            }
            frames_.emplace_back(std::move(frame));
        }
        condition_.notify_one();
    }

    void Close() {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (closed_) {
                return;
            }
            stop_requested_ = true;
        }
        condition_.notify_one();

        if (writer_thread_.joinable()) {
            writer_thread_.join();
        }

        if (writer_.isOpened()) {
            writer_.release();
        }
        closed_ = true;
    }

    const std::string& OutputPath() const {
        return output_path_;
    }

    const std::string& CodecName() const {
        return codec_name_;
    }

    size_t WrittenFrames() const {
        return written_frames_;
    }

    size_t DroppedFrames() const {
        return dropped_frames_;
    }

private:
    void WriterLoop() {
        while (true) {
            cv::Mat frame;
            {
                std::unique_lock<std::mutex> lock(mutex_);
                condition_.wait(lock, [this] {
                    return stop_requested_ || !frames_.empty();
                });

                if (frames_.empty()) {
                    if (stop_requested_) {
                        break;
                    }
                    continue;
                }

                frame = std::move(frames_.front());
                frames_.pop_front();
            }

            try {
                writer_.write(frame);
                ++written_frames_;
            } catch (const cv::Exception& ex) {
                std::cerr << "VideoWriter write failed: " << ex.what() << std::endl;
            }
        }
    }

    cv::VideoWriter writer_;
    std::thread writer_thread_;
    std::mutex mutex_;
    std::condition_variable condition_;
    std::deque<cv::Mat> frames_;
    std::string output_path_;
    std::string codec_name_;
    size_t written_frames_ = 0;
    size_t dropped_frames_ = 0;
    bool stop_requested_ = false;
    bool closed_ = false;
};

}  // namespace

class SubTest_orsm {
public:
#ifndef SUP_ORSM_OFFLINE_LOG
    void Test(int argc, char* argv[]) {
        RsclAccess::Instance()->init(argc, argv, "test_sub_orsm");
        if (!RsclAccess::Instance()->subscribe("/network/fortress_resp",
                                               &SubTest_orsm::Callback, this)) {
            std::cerr << "Failed to subscribe /network/fortress_resp" << std::endl;
        }
    }

    void Callback(const std::shared_ptr<ReceivedMsg<senseAD::msg::l3::OrsmStatus>>& msg) {
        if (!msg) {
            return;
        }

        const auto received_time = std::chrono::steady_clock::now();
        auto msg_content = msg->Msg();
        std::vector<IntruderPoint> next_points;
        if (msg_content.hasIntrudersInfo()) {
            auto intruders = msg_content.getIntrudersInfo();
            next_points.reserve(intruders.size());
            for (auto intruder : intruders) {
                IntruderPoint point;
                point.id = intruder.getIntruderId();
                point.type = static_cast<IntruderType>(static_cast<uint8_t>(intruder.getIntruderType()));
                point.x = intruder.getIntruderPoseX();
                point.y = intruder.getIntruderPoseY();
                point.vx = intruder.getIntruderVelocityX();
                point.vy = intruder.getIntruderVelocityY();
                point.timestamp_nsec = intruder.getTimestampNsec();
                next_points.push_back(point);
            }
        }

        {
            std::lock_guard<std::mutex> lock(mutex_);
            history_points_.insert(history_points_.end(), previous_points_.begin(), previous_points_.end());
            if (history_points_.size() > kMaxHistoryPoints) {
                history_points_.erase(history_points_.begin(),
                                      history_points_.begin() +
                                          static_cast<std::ptrdiff_t>(history_points_.size() -
                                                                      kMaxHistoryPoints));
            }
            previous_points_ = current_points_;
            current_points_ = std::move(next_points);
            state_.warn_level = static_cast<OrsmWarnLevel>(static_cast<uint8_t>(msg_content.getOrsmWarnLevel()));
            state_.intruders_num = static_cast<IntrudersNumType>(static_cast<uint8_t>(msg_content.getIntrudersNum()));
            state_.warning_status = static_cast<OrsmWarningStatus>(static_cast<uint8_t>(msg_content.getOrsmWarningSt()));
            state_.range_setting = static_cast<DetecRangeSetting>(static_cast<uint8_t>(msg_content.getDetecRangeSettingFb()));
            state_.message_reminder = static_cast<OrsmMessageReminder>(static_cast<uint8_t>(msg_content.getOrsmMessageRem()));
            state_.setting_type = static_cast<OrsmSettingType>(static_cast<uint8_t>(msg_content.getOrsmSettingStFb()));
            state_.intruders_info_count = current_points_.size();
            state_.frame_index++;
            state_.has_frame = true;
            last_update_time_ = received_time;
        }
        frame_condition_.notify_one();
    }
#else
    void SetOfflineLogPath(const std::string& path) {
        offline_log_path_ = path;
    }

    // 使用 rsclbag 时间范围补全稀疏或延迟开始的离线日志时间轴。
    void SetOfflineTimeline(uint64_t start_ns, uint64_t duration_ms) {
        offline_timeline_start_ns_ = start_ns;
        offline_timeline_duration_ms_ = duration_ms;
    }
#endif

    bool RenderLoop(const std::string& output_path) {
        const BevRange range = DisplayRange();
        const cv::Rect plot = PlotRect(range);

        cv::Mat base_canvas(kCanvasHeight, kCanvasWidth, CV_8UC3, cv::Scalar(22, 22, 22));
        DrawRadarBackground(base_canvas, range, plot);

        AsyncVideoWriter video_writer;
        if (!video_writer.Open(output_path, kOutputFps, cv::Size(kCanvasWidth, kCanvasHeight))) {
            return false;
        }

        std::cout << "Recording ORSM BEV mp4: " << video_writer.OutputPath()
                  << " (" << kOutputFps << " FPS, codec " << video_writer.CodecName() << ")"
                  << std::endl;
        std::cout << "Press Ctrl+C to stop and finalize the mp4." << std::endl;

#ifdef SUP_ORSM_OFFLINE_LOG
        if (!offline_log_path_.empty()) {
            offline_thread_ = std::thread(&SubTest_orsm::ReplayOfflineLog, this);
        }
#endif

        auto last_budget_warning = std::chrono::steady_clock::now() - std::chrono::seconds(2);
        uint64_t rendered_frame_index = 0;

        while (!stop_requested_.load() && !StopRequestedBySignal()) {
            std::vector<IntruderPoint> history_points;
            std::vector<IntruderPoint> previous_points;
            std::vector<IntruderPoint> current_points;
            OrsmState state;
            std::chrono::steady_clock::time_point received_time;

            {
                std::unique_lock<std::mutex> lock(mutex_);
                frame_condition_.wait_for(lock, std::chrono::milliseconds(kStopPollIntervalMs),
                                          [this, rendered_frame_index] {
                                              return stop_requested_.load() || StopRequestedBySignal() ||
                                                     state_.frame_index > rendered_frame_index;
                                          });

                if (stop_requested_.load() || StopRequestedBySignal()) {
                    break;
                }
                if (state_.frame_index <= rendered_frame_index) {
                    continue;
                }

                history_points = history_points_;
                previous_points = previous_points_;
                current_points = current_points_;
                state = state_;
                received_time = last_update_time_;
            }
            rendered_frame_index = state.frame_index;

            const auto frame_start = std::chrono::steady_clock::now();
            cv::Mat canvas = base_canvas.clone();

            DrawRangeReferences(canvas, range, plot, state.range_setting);
            DrawEgoVehicle(canvas, range, plot);

            DrawTrackTrails(canvas, history_points, previous_points, current_points, range, plot);
            for (const auto& point : previous_points) {
                DrawPoint(canvas, point, range, plot, cv::Scalar(0, 200, 0), 6, false);
            }
            for (const auto& point : current_points) {
                DrawPoint(canvas, point, range, plot, cv::Scalar(0, 0, 255), 7, true);
            }
            DrawCurrentLabels(canvas, history_points, previous_points, current_points, range, plot);

            if (!state.has_frame) {
                PutText(canvas, "Waiting for /network/fortress_resp ...",
                        cv::Point(plot.x + 18, plot.y + 32), 0.62, cv::Scalar(230, 230, 230));
            }

            DrawLegend(canvas, plot);
            DrawStatusPanel(canvas, state, IntruderTypesSummary(current_points),
                            current_points.size(), previous_points.size(), history_points.size());

            video_writer.Enqueue(std::move(canvas));

            const auto frame_finish = std::chrono::steady_clock::now();
            const auto render_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                frame_finish - frame_start);
            const auto data_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                frame_finish - received_time);
            if (data_elapsed.count() > kFrameBudgetMs &&
                std::chrono::steady_clock::now() - last_budget_warning > std::chrono::seconds(1)) {
                std::cerr << "ORSM data-to-frame processing exceeded " << kFrameBudgetMs
                          << " ms: " << data_elapsed.count()
                          << " ms, render: " << render_elapsed.count() << " ms" << std::endl;
                last_budget_warning = std::chrono::steady_clock::now();
            }
        }

        video_writer.Close();
#ifdef SUP_ORSM_OFFLINE_LOG
        if (offline_thread_.joinable()) offline_thread_.join();
#endif
        std::cout << "Finalized mp4: " << video_writer.OutputPath()
                  << ", written frames: " << video_writer.WrittenFrames()
                  << ", dropped frames: " << video_writer.DroppedFrames() << std::endl;
        return true;
    }

private:
#ifdef SUP_ORSM_OFFLINE_LOG
    void ReplayOfflineLog() {
        std::ifstream input(offline_log_path_);
        if (!input) {
            std::cerr << "Failed to open ORSM log: " << offline_log_path_ << std::endl;
            stop_requested_ = true;
            frame_condition_.notify_one();
            return;
        }

        const std::regex state_pattern(
            R"(state=([0-9]+).*warning_st=([0-9]+) warn_level=([0-9]+) intruders=([0-9]+) msg_rem=([0-9]+))");
        const std::regex point_pattern(
            R"(#([0-9]+) type=([0-9]+) zone=([0-9]+) pos=\((-?[0-9.]+),(-?[0-9.]+)\) vel=\((-?[0-9.]+),(-?[0-9.]+)\))");

        struct OfflineEvent {
            uint64_t timestamp_ns{0};
            int warning_status{0};
            int warn_level{0};
            int intruders_num{0};
            int message_reminder{0};
            std::vector<IntruderPoint> points;
        };
        const std::regex group_timestamp_pattern(
            R"(\[PerceptionStageDiag\]\[GroupPreLiDAR\].*ts=([0-9]+))");
        const std::regex trace_timestamp_pattern(
            R"(\[ORSMOfflineTrace\] ts=([0-9]+))");

        std::vector<OfflineEvent> events;
        std::string line;
        uint64_t latest_group_timestamp_ns = 0;
        while (std::getline(input, line)) {
            std::smatch group_match;
            if (std::regex_search(line, group_match, group_timestamp_pattern)) {
                latest_group_timestamp_ns = std::stoull(group_match[1].str());
            }
            const bool is_legacy_status =
                line.find("[SentinelNode][StatusOutput][ORSM]") != std::string::npos;
            const bool is_lossless_trace =
                line.find("[ORSMOfflineTrace]") != std::string::npos;
            if (!is_legacy_status && !is_lossless_trace) continue;
            std::smatch state_match;
            if (!std::regex_search(line, state_match, state_pattern)) continue;

            OfflineEvent event;
            event.timestamp_ns = latest_group_timestamp_ns;
            if (is_lossless_trace) {
                std::smatch trace_match;
                if (std::regex_search(line, trace_match, trace_timestamp_pattern)) {
                    event.timestamp_ns = std::stoull(trace_match[1].str());
                }
            }
            event.warning_status = std::stoi(state_match[2].str());
            event.warn_level = std::stoi(state_match[3].str());
            event.intruders_num = std::stoi(state_match[4].str());
            event.message_reminder = std::stoi(state_match[5].str());
            for (std::sregex_iterator it(line.begin(), line.end(), point_pattern), end; it != end; ++it) {
                IntruderPoint point;
                point.id = static_cast<uint32_t>(std::stoul((*it)[1].str()));
                point.type = static_cast<IntruderType>(std::stoi((*it)[2].str()));
                point.x = std::stof((*it)[4].str());
                point.y = std::stof((*it)[5].str());
                point.vx = std::stof((*it)[6].str());
                point.vy = std::stof((*it)[7].str());
                event.points.push_back(point);
            }
            events.push_back(std::move(event));
        }

        // 旧日志缺少 GroupPreLiDAR 时间戳时，将有序 StatusOutput 均匀映射到
        // 已知 bag 时间范围，避免所有事件集中在首个视频时刻。
        const bool has_synchronized_timestamp = std::any_of(
            events.begin(), events.end(), [](const OfflineEvent& event) {
                return event.timestamp_ns != 0;
            });
        if (!has_synchronized_timestamp && !events.empty() &&
            offline_timeline_start_ns_ > 0 && offline_timeline_duration_ms_ > 0) {
            const uint64_t duration_ns = offline_timeline_duration_ms_ * 1000000ULL;
            for (size_t i = 0; i < events.size(); ++i) {
                events[i].timestamp_ns = offline_timeline_start_ns_ +
                    (duration_ns * i) / events.size();
            }
            std::cout << "Offline ORSM timeline fallback: " << events.size()
                      << " ordered StatusOutput events stretched over "
                      << offline_timeline_duration_ms_ << " ms (no GroupPreLiDAR timestamps)"
                      << std::endl;
        }

        auto publish_event = [&](const OfflineEvent* event, bool data_changed) {
            std::lock_guard<std::mutex> lock(mutex_);
            if (event && data_changed) {
                history_points_.insert(history_points_.end(), previous_points_.begin(), previous_points_.end());
                if (history_points_.size() > kMaxHistoryPoints) {
                    history_points_.erase(history_points_.begin(),
                        history_points_.begin() + static_cast<std::ptrdiff_t>(history_points_.size() - kMaxHistoryPoints));
                }
                previous_points_ = current_points_;
                current_points_ = event->points;
                state_.warning_status = static_cast<OrsmWarningStatus>(event->warning_status);
                state_.warn_level = static_cast<OrsmWarnLevel>(event->warn_level);
                state_.intruders_num = static_cast<IntrudersNumType>(event->intruders_num);
                state_.message_reminder = static_cast<OrsmMessageReminder>(event->message_reminder);
                state_.has_frame = true;
            }
            state_.setting_type = OrsmSettingType::SINGLE_AUTO_SECURITY_ON;
            state_.range_setting = DetecRangeSetting::RANGE_A;
            state_.intruders_info_count = current_points_.size();
            state_.frame_index++;
            last_update_time_ = std::chrono::steady_clock::now();
        };

        size_t emitted_frames = 0;
        if (offline_timeline_start_ns_ > 0 && offline_timeline_duration_ms_ > 0) {
            const uint64_t frame_period_ns = 1000000000ULL /
                static_cast<uint64_t>(std::lround(kOutputFps));
            const uint64_t end_ns = offline_timeline_start_ns_ +
                offline_timeline_duration_ms_ * 1000000ULL;
            size_t event_index = 0;
            const OfflineEvent* active_event = nullptr;
            for (uint64_t tick_ns = offline_timeline_start_ns_;
                 tick_ns < end_ns && !StopRequestedBySignal();
                 tick_ns += frame_period_ns) {
                bool data_changed = false;
                // Startup status messages precede the first synchronized
                // camera/LiDAR group and therefore have no group timestamp.
                // Apply them at the initial tick instead of letting one zero
                // timestamp block every later, timestamped event.
                while (event_index < events.size() &&
                       (events[event_index].timestamp_ns == 0 ||
                        events[event_index].timestamp_ns <= tick_ns)) {
                    active_event = &events[event_index++];
                    data_changed = true;
                }
                publish_event(active_event, data_changed);
                frame_condition_.notify_one();
                ++emitted_frames;
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        } else {
            for (const auto& event : events) {
                if (StopRequestedBySignal()) break;
                publish_event(&event, true);
                frame_condition_.notify_one();
                ++emitted_frames;
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        }
        std::cout << "Offline ORSM events parsed: " << events.size()
                  << ", timeline frames emitted: " << emitted_frames << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
        stop_requested_ = true;
        frame_condition_.notify_one();
    }

    std::string offline_log_path_;
    uint64_t offline_timeline_start_ns_{0};
    uint64_t offline_timeline_duration_ms_{0};
    std::thread offline_thread_;
#endif
    std::mutex mutex_;
    std::condition_variable frame_condition_;
    std::vector<IntruderPoint> history_points_;
    std::vector<IntruderPoint> previous_points_;
    std::vector<IntruderPoint> current_points_;
    OrsmState state_;
    std::chrono::steady_clock::time_point last_update_time_ {};
    std::atomic<bool> stop_requested_{false};
};

int main(int argc, char* argv[]) {
    InstallSignalHandlers();
    const std::string output_path = ResolveVideoPath();

    SubTest_orsm test;
#ifdef SUP_ORSM_OFFLINE_LOG
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0]
                  << " /path/to/sentinel.log [timeline_start_ns duration_ms]" << std::endl;
        return 2;
    }
    test.SetOfflineLogPath(argv[1]);
    if (argc >= 4) {
        test.SetOfflineTimeline(std::stoull(argv[2]), std::stoull(argv[3]));
    }
#else
    test.Test(argc, argv);
#endif
    const bool ok = test.RenderLoop(output_path);
#ifndef SUP_ORSM_OFFLINE_LOG
    RsclAccess::Instance()->release();
#endif
    return ok ? 0 : 1;
}
