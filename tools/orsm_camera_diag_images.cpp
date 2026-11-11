#include "camera/camera_subscriber.h"
#include "camera/fisheye_undistort.h"
#include "detection/detector.h"
#include "rscl_access/rscl_access.h"

#include <yaml-cpp/yaml.h>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <atomic>
#include <csignal>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>
#include <unistd.h>

namespace fs = std::filesystem;

namespace {

std::atomic<bool> g_running{true};

void SignalHandler(int) {
    g_running.store(false);
}

struct ToolConfig {
    std::vector<std::string> camera_topics;
    std::vector<std::string> camera_names;
    std::string calib_base_dir;
    std::string inbox_calib_dir;
    std::string engine_path;
    std::string detector_model_type{"yolo"};
    std::string camera_input_mode{"encoded_hevc"};
    std::string camera_decoder_backend{"ffmpeg_nvdec"};
    int camera_width{1920};
    int camera_height{1536};
    int gpu_id{0};
    float conf_threshold{0.35f};
    float nms_threshold{0.45f};
    int motion_min_streak{2};
    int motion_max_age_ms{1500};
    float motion_min_changed_ratio{0.0015f};
    float motion_min_mean_delta{0.6f};
    float motion_min_component_ratio{0.00025f};
    float motion_max_box_area_ratio{0.05f};
    float class_transfer_min_overlap{0.50f};
};

struct Options {
    std::string config_path;
    fs::path output_dir;
    std::string camera_filter;
    int max_images{0};
    int save_every{1};
    bool include_all_yolo{false};
    bool require_ground_projection{false};
};

struct MotionState {
    cv::Mat previous_gray;
    uint64_t last_timestamp_ns{0};
    int streak{0};
};

struct MotionResult {
    cv::Rect bbox;
    int streak{0};
    double changed_ratio{0.0};
    double mean_delta{0.0};
    double component_ratio{0.0};
};

std::string ResolvePath(const std::string& config_path, const std::string& value) {
    fs::path path(value);
    if (path.is_relative()) {
        path = fs::absolute(fs::path(config_path)).parent_path() / path;
    }
    return path.lexically_normal().string();
}

ToolConfig LoadConfig(const std::string& path) {
    const YAML::Node root = YAML::LoadFile(path);
    ToolConfig cfg;
    if (root["camera_topics"]) {
        for (const auto& value : root["camera_topics"]) {
            cfg.camera_topics.push_back(value.as<std::string>());
        }
    }
    if (root["camera_names"]) {
        for (const auto& value : root["camera_names"]) {
            cfg.camera_names.push_back(value.as<std::string>());
        }
    }
    if (root["calib_base_dir"]) {
        cfg.calib_base_dir = ResolvePath(path, root["calib_base_dir"].as<std::string>());
    }
    if (root["inbox_calib_dir"]) {
        cfg.inbox_calib_dir = ResolvePath(path, root["inbox_calib_dir"].as<std::string>());
    }
    if (root["engine_path"]) {
        cfg.engine_path = ResolvePath(path, root["engine_path"].as<std::string>());
    }
    if (root["detector_model_type"]) cfg.detector_model_type = root["detector_model_type"].as<std::string>();
    if (root["camera_input_mode"]) cfg.camera_input_mode = root["camera_input_mode"].as<std::string>();
    if (root["camera_decoder_backend"]) cfg.camera_decoder_backend = root["camera_decoder_backend"].as<std::string>();
    if (root["camera_width"]) cfg.camera_width = root["camera_width"].as<int>();
    if (root["camera_height"]) cfg.camera_height = root["camera_height"].as<int>();
    if (root["gpu_id"]) cfg.gpu_id = root["gpu_id"].as<int>();
    if (root["conf_threshold"]) cfg.conf_threshold = root["conf_threshold"].as<float>();
    if (root["nms_threshold"]) cfg.nms_threshold = root["nms_threshold"].as<float>();

    if (const auto orsm = root["orsm"]) {
        if (orsm["far_image_motion_min_streak"]) cfg.motion_min_streak = orsm["far_image_motion_min_streak"].as<int>();
        if (orsm["near_image_motion_max_age_ms"]) cfg.motion_max_age_ms = orsm["near_image_motion_max_age_ms"].as<int>();
        if (orsm["far_image_motion_min_changed_ratio"]) cfg.motion_min_changed_ratio = orsm["far_image_motion_min_changed_ratio"].as<float>();
        if (orsm["far_image_motion_min_mean_delta"]) cfg.motion_min_mean_delta = orsm["far_image_motion_min_mean_delta"].as<float>();
        if (orsm["far_image_motion_min_component_ratio"]) cfg.motion_min_component_ratio = orsm["far_image_motion_min_component_ratio"].as<float>();
        if (orsm["far_image_motion_max_box_area_ratio"]) cfg.motion_max_box_area_ratio = orsm["far_image_motion_max_box_area_ratio"].as<float>();
        if (orsm["far_image_motion_class_transfer_min_overlap"]) cfg.class_transfer_min_overlap = orsm["far_image_motion_class_transfer_min_overlap"].as<float>();
    }
    if (cfg.camera_topics.empty() || cfg.camera_topics.size() != cfg.camera_names.size()) {
        throw std::runtime_error("camera_topics and camera_names must be present and have equal sizes");
    }
    if (cfg.engine_path.empty()) throw std::runtime_error("engine_path is missing");
    return cfg;
}

bool IsBiological(const sentinel::Detection& detection) {
    return detection.class_id == 0 || detection.class_id == 1 ||
           detection.label == "person" || detection.label == "animal";
}

double BoxCoverage(const cv::Rect& first, const cv::Rect& second) {
    const double intersection = static_cast<double>((first & second).area());
    const double smaller = static_cast<double>(std::min(first.area(), second.area()));
    return smaller > 0.0 ? intersection / smaller : 0.0;
}

struct DistanceEstimate {
    double distance_m{0.0};
    const char* method{"invalid"};
};

float NominalObjectHeightM(const std::string& label) {
    if (label == "person" || label == "human" || label == "pedestrian") return 1.70f;
    if (label == "bicycle" || label == "motorcycle") return 1.40f;
    if (label == "vehicle" || label == "car" || label == "sedan") return 1.50f;
    if (label == "truck" || label == "bus" || label == "suv") return 2.20f;
    if (label == "animal" || label == "dog" || label == "cat") return 0.60f;
    if (label == "horse" || label == "cow" || label == "sheep") return 1.40f;
    return 1.0f;
}

// 与 SentinelNode::enrichVisualGeometry 保持一致：优先使用标定地面交点，
// 失败时按目标名义高度和框高估距。
DistanceEstimate EstimateBoxDistance(const cv::Rect& box,
                                     const std::string& label,
                                     const sentinel::FisheyeCalib* calib,
                                     int image_width,
                                     int image_height) {
    if (box.width <= 0 || box.height <= 0 || image_width <= 0 || image_height <= 0) {
        return {};
    }

    cv::Vec2d ray_xy(0.0, 0.0);
    cv::Vec2d origin_xy(0.0, 0.0);
    bool has_calibrated_ray = false;
    if (calib && !calib->K_new.empty() && !calib->extrinsic.empty() &&
        calib->K_new.rows == 3 && calib->K_new.cols == 3 &&
        calib->extrinsic.rows == 4 && calib->extrinsic.cols == 4) {
        cv::Mat K_new, transform;
        calib->K_new.convertTo(K_new, CV_64F);
        calib->extrinsic.convertTo(transform, CV_64F);
        const double fx = K_new.at<double>(0, 0);
        const double fy = K_new.at<double>(1, 1);
        const double cx = K_new.at<double>(0, 2);
        const double cy = K_new.at<double>(1, 2);
        const double u = std::clamp(static_cast<double>(box.x) + box.width * 0.5,
                                    0.0, static_cast<double>(image_width - 1));
        const double v = std::clamp(static_cast<double>(box.y + box.height),
                                    0.0, static_cast<double>(image_height - 1));
        if (fx > 1e-6 && fy > 1e-6) {
            const cv::Vec3d ray_cam((u - cx) / fx, (v - cy) / fy, 1.0);
            const cv::Matx33d rotation(
                transform.at<double>(0, 0), transform.at<double>(0, 1), transform.at<double>(0, 2),
                transform.at<double>(1, 0), transform.at<double>(1, 1), transform.at<double>(1, 2),
                transform.at<double>(2, 0), transform.at<double>(2, 1), transform.at<double>(2, 2));
            const cv::Vec3d origin(transform.at<double>(0, 3),
                                   transform.at<double>(1, 3),
                                   transform.at<double>(2, 3));
            const cv::Vec3d ray_car = rotation * ray_cam;
            const double ray_xy_norm = std::hypot(ray_car[0], ray_car[1]);
            if (ray_xy_norm > 1e-6) {
                ray_xy = cv::Vec2d(ray_car[0] / ray_xy_norm, ray_car[1] / ray_xy_norm);
                origin_xy = cv::Vec2d(origin[0], origin[1]);
                has_calibrated_ray = true;
            }
            if (ray_car[2] < -1e-4 && origin[2] > 0.05) {
                const double scale = -origin[2] / ray_car[2];
                const cv::Vec3d ground = origin + ray_car * scale;
                const double range = std::hypot(ground[0], ground[1]);
                if (scale > 0.0 && std::isfinite(range) && range >= 0.3 && range <= 30.0) {
                    return {range, "bottom_ground"};
                }
            }
        }
    }

    const double focal_px = static_cast<double>(image_height) * 0.45;
    const double distance = static_cast<double>(NominalObjectHeightM(label)) * focal_px /
                            static_cast<double>(box.height);
    if (!std::isfinite(distance) || distance <= 0.0) return {};

    // The application still places this fallback along the calibrated camera ray
    // when possible, but the range itself remains a bbox-height estimate.
    (void)has_calibrated_ray;
    (void)ray_xy;
    (void)origin_xy;
    return {distance, "bbox_height_fallback"};
}

std::string DistanceText(const std::string& source,
                         const DistanceEstimate& estimate,
                         const cv::Rect& box) {
    std::ostringstream text;
    text << source << " d=";
    if (estimate.distance_m > 0.0) {
        text << std::fixed << std::setprecision(2) << estimate.distance_m << "m";
    } else {
        text << "invalid";
    }
    text << " src=" << estimate.method
         << " bottom=" << (box.y + box.height)
         << " h=" << box.height;
    return text.str();
}

std::optional<MotionResult> UpdateMotion(const cv::Mat& image,
                                         uint64_t timestamp_ns,
                                         const std::string& camera_name,
                                         const ToolConfig& cfg,
                                         MotionState& state,
                                         const sentinel::FisheyeCalib* diff_calib,
                                         bool require_ground_projection) {
    cv::Mat gray;
    if (image.channels() == 1) gray = image;
    else cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);

    cv::Mat scaled;
    cv::GaussianBlur(gray, scaled, cv::Size(5, 5), 0.0);
    cv::GaussianBlur(scaled, scaled, cv::Size(5, 5), 0.0);
    if (state.previous_gray.empty()) {
        state.previous_gray = scaled.clone();
        state.last_timestamp_ns = timestamp_ns;
        return std::nullopt;
    }

    const uint64_t max_age_ns = static_cast<uint64_t>(std::max(0, cfg.motion_max_age_ms)) * 1000000ULL;
    if (max_age_ns > 0 && timestamp_ns > state.last_timestamp_ns &&
        timestamp_ns - state.last_timestamp_ns > max_age_ns) {
        state.previous_gray = scaled.clone();
        state.last_timestamp_ns = timestamp_ns;
        state.streak = 0;
        return std::nullopt;
    }

    cv::Mat diff;
    cv::absdiff(scaled, state.previous_gray, diff);
    cv::Mat mask;
    cv::threshold(diff, mask, 18.0, 255.0, cv::THRESH_BINARY);
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, cv::Mat(), cv::Point(-1, -1), 1);
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, cv::Mat(), cv::Point(-1, -1), 1);

    const int mask_area = std::max(1, mask.rows * mask.cols);
    const double changed_ratio = static_cast<double>(cv::countNonZero(mask)) / mask_area;
    const double mean_delta = cv::mean(diff)[0];
    cv::Mat labels;
    cv::Mat stats;
    cv::Mat centroids;
    const int components = cv::connectedComponentsWithStats(mask, labels, stats, centroids, 8);
    int best_label = -1;
    int best_area = 0;
    int largest_unfiltered_area = 0;
    const double max_box_area_ratio = std::max(0.0f, cfg.motion_max_box_area_ratio);
    int min_bottom_y = 0;
    if (camera_name == "rear_camera_fov195") {
        min_bottom_y = 512;
    } else if (camera_name == "left_camera_fov195") {
        min_bottom_y = 510;
    }
    std::vector<int> eligible_labels;
    eligible_labels.reserve(std::max(0, components - 1));
    for (int label = 1; label < components; ++label) {
        const int area = stats.at<int>(label, cv::CC_STAT_AREA);
        largest_unfiltered_area = std::max(largest_unfiltered_area, area);
        const int top = stats.at<int>(label, cv::CC_STAT_TOP);
        const int width = stats.at<int>(label, cv::CC_STAT_WIDTH);
        const int height = stats.at<int>(label, cv::CC_STAT_HEIGHT);
        const double box_area_ratio =
            static_cast<double>(width) * static_cast<double>(height) /
            static_cast<double>(mask_area);
        if (max_box_area_ratio > 0.0 && box_area_ratio > max_box_area_ratio) {
            std::cout << "[DiagImages] skip_diff_component reason=box_area_ratio"
                      << " camera=" << camera_name
                      << " component=" << label
                      << " size_percent=" << (box_area_ratio * 100.0)
                      << " max_percent=" << (max_box_area_ratio * 100.0)
                      << std::endl;
            continue;
        }
        if (min_bottom_y > 0 && top + height < min_bottom_y) continue;
        eligible_labels.push_back(label);
    }
    std::sort(eligible_labels.begin(), eligible_labels.end(),
              [&](int lhs, int rhs) {
                  return stats.at<int>(lhs, cv::CC_STAT_AREA) >
                         stats.at<int>(rhs, cv::CC_STAT_AREA);
              });
    for (const int label : eligible_labels) {
        if (require_ground_projection) {
            cv::Rect probe_box(
                stats.at<int>(label, cv::CC_STAT_LEFT),
                stats.at<int>(label, cv::CC_STAT_TOP),
                stats.at<int>(label, cv::CC_STAT_WIDTH),
                stats.at<int>(label, cv::CC_STAT_HEIGHT));
            probe_box &= cv::Rect(0, 0, image.cols, image.rows);
            const DistanceEstimate estimate = EstimateBoxDistance(
                probe_box, "unknown", diff_calib, image.cols, image.rows);
            if (std::string(estimate.method) != "bottom_ground") {
                std::cout << "[DiagImages] skip_diff_component reason=bbox_height_fallback"
                          << " camera=" << camera_name
                          << " component=" << label
                          << " bbox=(" << probe_box.x << "," << probe_box.y << ","
                          << probe_box.width << "," << probe_box.height << ")"
                          << " estimate=" << estimate.distance_m << std::endl;
                continue;
            }
        }
        best_label = label;
        best_area = stats.at<int>(best_label, cv::CC_STAT_AREA);
        break;
    }
    const double component_ratio = static_cast<double>(best_area) / mask_area;
    const double largest_unfiltered_component_ratio =
        static_cast<double>(largest_unfiltered_area) / mask_area;
    const bool global_flash =
        changed_ratio > 0.60 && largest_unfiltered_component_ratio > 0.40;
    const bool motion_hit = best_label > 0 && !global_flash &&
        changed_ratio >= std::max(0.0f, cfg.motion_min_changed_ratio) &&
        mean_delta >= std::max(0.0f, cfg.motion_min_mean_delta) &&
        component_ratio >= std::max(0.0f, cfg.motion_min_component_ratio);
    state.streak = motion_hit ? state.streak + 1 : 0;

    std::optional<MotionResult> result;
    if (state.streak >= std::max(1, cfg.motion_min_streak) && best_label > 0) {
        cv::Rect box(stats.at<int>(best_label, cv::CC_STAT_LEFT),
                     stats.at<int>(best_label, cv::CC_STAT_TOP),
                     stats.at<int>(best_label, cv::CC_STAT_WIDTH),
                     stats.at<int>(best_label, cv::CC_STAT_HEIGHT));
        box &= cv::Rect(0, 0, image.cols, image.rows);
        result = MotionResult{box, state.streak, changed_ratio, mean_delta, component_ratio};
    }
    state.previous_gray = scaled.clone();
    state.last_timestamp_ns = timestamp_ns;
    return result;
}

void DrawLabel(cv::Mat& image, const cv::Rect& box, const std::string& text,
               const cv::Scalar& colour) {
    cv::rectangle(image, box, colour, 3, cv::LINE_AA);
    int baseline = 0;
    const cv::Size size = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, 0.55, 2, &baseline);
    const int x = std::clamp(box.x, 0, std::max(0, image.cols - size.width - 8));
    const int y = std::max(size.height + 8, box.y);
    cv::rectangle(image, cv::Rect(x, y - size.height - 7, size.width + 8, size.height + 10),
                  cv::Scalar(20, 20, 20), cv::FILLED);
    cv::putText(image, text, cv::Point(x + 4, y), cv::FONT_HERSHEY_SIMPLEX,
                0.55, colour, 2, cv::LINE_AA);
}

class DiagnosticImageWriter {
public:
    DiagnosticImageWriter(ToolConfig cfg, Options options)
        : cfg_(std::move(cfg)), options_(std::move(options)) {}

    bool Start(int argc, char** argv) {
        fs::create_directories(options_.output_dir);
        if (!fisheye_.loadAll(cfg_.calib_base_dir, cfg_.camera_names)) {
            std::cerr << "[DiagImages] warning: some calibration files could not be loaded from "
                      << cfg_.calib_base_dir << std::endl;
        }
        if (!cfg_.inbox_calib_dir.empty() &&
            !inbox_fisheye_.loadAll(cfg_.inbox_calib_dir, cfg_.camera_names)) {
            std::cerr << "[DiagImages] warning: some Inbox calibration files could not be loaded from "
                      << cfg_.inbox_calib_dir << std::endl;
        }
        if (!detector_.init(cfg_.engine_path, cfg_.conf_threshold, cfg_.nms_threshold,
                            cfg_.detector_model_type)) {
            std::cerr << "[DiagImages] detector initialization failed: " << cfg_.engine_path << std::endl;
            return false;
        }

        RsclAccess::Instance()->init(argc, argv, "orsm_camera_diag_images");
        const auto input_mode = sentinel::parseCameraInputMode(cfg_.camera_input_mode);
        const auto decoder_backend = parseHevcDecoderBackend(cfg_.camera_decoder_backend);
        for (size_t i = 0; i < cfg_.camera_topics.size(); ++i) {
            if (!options_.camera_filter.empty() && cfg_.camera_names[i] != options_.camera_filter) continue;
            auto camera = std::make_shared<sentinel::CameraSubscriber>(
                nullptr, cfg_.camera_width, cfg_.camera_height, cfg_.gpu_id,
                input_mode, decoder_backend);
            camera->setFrameCallback([this](const sentinel::DecodedFrame& frame) { OnFrame(frame); });
            camera->subscribe(cfg_.camera_topics[i], cfg_.camera_names[i]);
            cameras_.push_back(std::move(camera));
        }
        if (cameras_.empty()) {
            std::cerr << "[DiagImages] no camera matched --camera=" << options_.camera_filter << std::endl;
            return false;
        }
        std::cout << "[DiagImages] ready output=" << options_.output_dir
                  << " cameras=" << cameras_.size()
                  << " max_images=" << options_.max_images << std::endl;
        return true;
    }

    void Stop() {
        cameras_.clear();
        RsclAccess::Instance()->release();
        std::cout << "[DiagImages] stopped received=" << received_.load()
                  << " saved=" << saved_.load() << std::endl;
    }

private:
    void OnFrame(const sentinel::DecodedFrame& frame) {
        if (!g_running.load()) return;
        std::lock_guard<std::mutex> lock(process_mutex_);
        ++received_;
        cv::Mat rectified = frame.image;
        if (fisheye_.hasCamera(frame.camera_name)) {
            rectified = fisheye_.undistort(frame.camera_name, frame.image);
        }
        if (rectified.empty()) return;

        const auto* camera_calib = fisheye_.getCalib(frame.camera_name);
        const auto* diff_calib = inbox_fisheye_.getCalib(frame.camera_name);
        if (!diff_calib) diff_calib = camera_calib;
        const auto detections = detector_.detect(rectified);
        auto motion = UpdateMotion(rectified, frame.timestamp_ns, frame.camera_name,
                                   cfg_, motion_states_[frame.camera_name], diff_calib,
                                   options_.require_ground_projection);
        std::vector<sentinel::Detection> selected;
        for (const auto& detection : detections) {
            if (options_.include_all_yolo || IsBiological(detection)) selected.push_back(detection);
        }
        if (!motion && selected.empty()) return;
        const uint64_t interesting_index = ++interesting_;
        if (options_.save_every > 1 && (interesting_index - 1) % options_.save_every != 0) return;
        if (options_.max_images > 0 && saved_.load() >= static_cast<uint64_t>(options_.max_images)) return;

        cv::Mat annotated = rectified.clone();
        for (const auto& detection : selected) {
            // YOLO 与差值框使用同一组 Inbox 标定，避免外参差异影响诊断对比。
            const DistanceEstimate distance = EstimateBoxDistance(
                detection.bbox, detection.label, diff_calib, rectified.cols, rectified.rows);
            std::ostringstream label;
            label << DistanceText("YOLO " + detection.label, distance, detection.bbox)
                  << " conf=" << std::fixed << std::setprecision(2) << detection.confidence;
            DrawLabel(annotated, detection.bbox, label.str(), cv::Scalar(40, 230, 40));
        }

        if (motion) {
            const sentinel::Detection* transferred = nullptr;
            double best_overlap = std::clamp(
                static_cast<double>(cfg_.class_transfer_min_overlap), 0.01, 1.0);
            for (const auto& detection : detections) {
                if (!IsBiological(detection)) continue;
                const double overlap = BoxCoverage(motion->bbox, detection.bbox);
                if (overlap > best_overlap) {
                    best_overlap = overlap;
                    transferred = &detection;
                }
            }
            // 主链路中的差值框类别为 unknown，框高兜底固定使用 1.0m 名义高度。
            const DistanceEstimate distance = EstimateBoxDistance(
                motion->bbox, "unknown", diff_calib, rectified.cols, rectified.rows);
            std::ostringstream label;
            const double box_area_ratio =
                static_cast<double>(motion->bbox.area()) /
                static_cast<double>(std::max(1, rectified.cols * rectified.rows));
            label << DistanceText("DIFF", distance, motion->bbox)
                  << " streak=" << motion->streak
                  << " size=" << std::fixed << std::setprecision(2)
                  << (box_area_ratio * 100.0) << "%/"
                  << (cfg_.motion_max_box_area_ratio * 100.0) << "%";
            cv::Scalar colour(0, 220, 255);
            if (transferred != nullptr) {
                label << " -> " << transferred->label << " overlap="
                      << std::fixed << std::setprecision(2) << best_overlap;
                colour = cv::Scalar(255, 220, 40);
            }
            DrawLabel(annotated, motion->bbox, label.str(), colour);
        }

        std::ostringstream header;
        header << frame.camera_name << " ts=" << frame.timestamp_ns
               << " yolo=" << selected.size() << " diff=" << (motion ? 1 : 0);
        cv::rectangle(annotated, cv::Rect(0, 0, annotated.cols, 42), cv::Scalar(15, 15, 15), cv::FILLED);
        cv::putText(annotated, header.str(), cv::Point(12, 29), cv::FONT_HERSHEY_SIMPLEX,
                    0.70, cv::Scalar(245, 245, 245), 2, cv::LINE_AA);

        const fs::path camera_dir = options_.output_dir / frame.camera_name;
        fs::create_directories(camera_dir);
        const std::string tags = std::string(motion ? "diff" : "nodiff") +
                                 (selected.empty() ? "_noyolo" : "_yolo");
        const fs::path output = camera_dir /
            (std::to_string(frame.timestamp_ns) + "_" + tags + ".jpg");
        if (cv::imwrite(output.string(), annotated, {cv::IMWRITE_JPEG_QUALITY, 92})) {
            const uint64_t count = ++saved_;
            std::cout << "[DiagImages] saved=" << output << " count=" << count << std::endl;
            if (options_.max_images > 0 && count >= static_cast<uint64_t>(options_.max_images)) {
                std::cout << "[DiagImages] max image count reached" << std::endl;
            }
        }
    }

    ToolConfig cfg_;
    Options options_;
    sentinel::FisheyeUndistortManager fisheye_;
    sentinel::FisheyeUndistortManager inbox_fisheye_;
    sentinel::Detector detector_;
    std::vector<std::shared_ptr<sentinel::CameraSubscriber>> cameras_;
    std::unordered_map<std::string, MotionState> motion_states_;
    std::mutex process_mutex_;
    std::atomic<uint64_t> received_{0};
    std::atomic<uint64_t> interesting_{0};
    std::atomic<uint64_t> saved_{0};
};

void PrintUsage(const char* program) {
    std::cerr << "Usage: " << program
              << " CONFIG.yaml OUTPUT_DIR [--camera NAME] [--max-images N]"
                 " [--save-every N] [--all-yolo]"
                 " [--require-ground-projection]\n";
}

Options ParseOptions(int argc, char** argv) {
    if (argc < 3) throw std::runtime_error("missing CONFIG.yaml or OUTPUT_DIR");
    Options options;
    options.config_path = argv[1];
    options.output_dir = argv[2];
    for (int i = 3; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--camera" && i + 1 < argc) options.camera_filter = argv[++i];
        else if (arg == "--max-images" && i + 1 < argc) options.max_images = std::stoi(argv[++i]);
        else if (arg == "--save-every" && i + 1 < argc) options.save_every = std::max(1, std::stoi(argv[++i]));
        else if (arg == "--all-yolo") options.include_all_yolo = true;
        else if (arg == "--require-ground-projection") options.require_ground_projection = true;
        else throw std::runtime_error("unknown or incomplete argument: " + arg);
    }
    return options;
}

}  // namespace

int main(int argc, char** argv) {
    std::signal(SIGINT, SignalHandler);
    std::signal(SIGTERM, SignalHandler);
    try {
        Options options = ParseOptions(argc, argv);
        ToolConfig cfg = LoadConfig(options.config_path);
        DiagnosticImageWriter writer(std::move(cfg), std::move(options));
        if (!writer.Start(argc, argv)) return 2;
        while (g_running.load()) sleep(1);
        writer.Stop();
        return 0;
    } catch (const std::exception& error) {
        PrintUsage(argv[0]);
        std::cerr << "[DiagImages] error: " << error.what() << std::endl;
        return 1;
    }
}
