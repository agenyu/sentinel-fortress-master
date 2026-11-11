#include "video_recorder.h"

#include <iostream>
#include <sstream>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <ctime>
#include <iomanip>
#include <future>

#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>

#ifdef SENTINEL_ENABLE_FFMPEG_DECODER
extern "C" {
#include <libavformat/avformat.h>
#include <libavcodec/avcodec.h>
#include <libavutil/opt.h>
#include <libavutil/imgutils.h>
#include <libavutil/mathematics.h>
#include <libswscale/swscale.h>
}
#endif

#include <fstream>
#include <limits>
#include <experimental/filesystem>
namespace fs = std::experimental::filesystem;

namespace sentinel {

namespace {

std::string safeCameraName(const std::string& camera_name) {
    if (camera_name.empty()) return "camera";
    std::string out = camera_name;
    for (char& c : out) {
        const bool ok = (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
                        (c >= '0' && c <= '9') || c == '_' || c == '-';
        if (!ok) c = '_';
    }
    return out;
}

std::string sentryCameraSuffix(const std::string& camera_name) {
    const std::string name = safeCameraName(camera_name);
    if (name.find("front") != std::string::npos) return "front";
    if (name.find("rear") != std::string::npos || name.find("back") != std::string::npos) return "rear";
    if (name.find("left") != std::string::npos) return "left";
    if (name.find("right") != std::string::npos) return "right";
    return name.empty() ? "camera" : name;
}

double elapsedSeconds(std::chrono::steady_clock::time_point start,
                      std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now()) {
    return std::chrono::duration<double>(end - start).count();
}

std::string sentryTimestampString(uint64_t timestamp_ns) {
    // 车端/rsclbag 时间戳通常为 Unix epoch ns。若遇到非 epoch/无效时间戳，
    // 用当前墙钟兜底，避免生成 1970 年目录名。
    constexpr uint64_t kMinReasonableEpochNs = 946684800ULL * 1000000000ULL;  // 2000-01-01
    std::time_t seconds = 0;
    if (timestamp_ns >= kMinReasonableEpochNs) {
        seconds = static_cast<std::time_t>(timestamp_ns / 1000000000ULL);
    } else {
        seconds = std::time(nullptr);
    }

    std::tm tm_value{};
    localtime_r(&seconds, &tm_value);
    std::ostringstream ss;
    ss << "Sentry-" << std::put_time(&tm_value, "%Y-%m-%d_%H-%M-%S");
    return ss.str();
}

}  // namespace

VideoRecorder::VideoRecorder(const VideoRecorderConfig& cfg)
    : cfg_(cfg)
{}

VideoRecorder::~VideoRecorder() {
    stop();
}

void VideoRecorder::start() {
    if (running_.load()) return;
    running_ = true;
    cache_worker_ = std::thread(&VideoRecorder::cacheWorkerLoop, this);
    worker_ = std::thread(&VideoRecorder::workerLoop, this);
    std::cout << "[VideoRecorder] Worker started. pre=" << cfg_.pre_seconds
              << "s post=" << cfg_.post_seconds << "s fps=" << cfg_.fps
              << " bitrate=" << cfg_.bitrate_kbps << "kbps"
              << " max_frame_width=" << cfg_.max_frame_width
              << " jpeg_quality=" << cfg_.jpeg_quality << std::endl;
}

void VideoRecorder::stop() {
    if (!running_.load()) return;
    running_ = false;
    frame_cache_cv_.notify_all();
    task_cv_.notify_all();
    if (cache_worker_.joinable()) cache_worker_.join();
    if (worker_.joinable()) worker_.join();
    std::cout << "[VideoRecorder] Worker stopped." << std::endl;
}

void VideoRecorder::pushFrame(const cv::Mat& frame, uint64_t timestamp_ns) {
    pushFrame("camera", frame, timestamp_ns);
}

bool VideoRecorder::ingestFrameAtTargetFps(const std::string& camera_name,
                                           const cv::Mat& frame,
                                           uint64_t timestamp_ns) {
    const int fps = std::max(1, cfg_.fps);
    if (timestamp_ns == 0) return true;

    const uint64_t period_ns = 1000000000ULL / static_cast<uint64_t>(fps);
    const uint64_t slot = timestamp_ns / period_ns;
    const uint64_t center = slot * period_ns + period_ns / 2ULL;
    const uint64_t dist = timestamp_ns > center ? timestamp_ns - center : center - timestamp_ns;

    auto distToCenter = [period_ns](uint64_t ts) -> uint64_t {
        const uint64_t s = ts / period_ns;
        const uint64_t c = s * period_ns + period_ns / 2ULL;
        return ts > c ? ts - c : c - ts;
    };

    for (auto it = pending_frame_cache_.rbegin(); it != pending_frame_cache_.rend(); ++it) {
        if (it->camera_name != camera_name || it->timestamp_ns == 0) continue;
        if (it->timestamp_ns / period_ns != slot) break;
        if (dist < distToCenter(it->timestamp_ns)) {
            it->frame = frame.clone();
            it->timestamp_ns = timestamp_ns;
        }
        return false;
    }

    auto last_slot = last_ingest_slot_by_camera_.find(camera_name);
    if (last_slot != last_ingest_slot_by_camera_.end() && last_slot->second == slot) {
        const uint64_t dropped = ++rate_limited_frames_;
        if (dropped <= 10 || dropped % 500 == 0) {
            std::cout << "[VideoRecorder] rate-limit drop camera=" << camera_name
                      << " slot=" << slot
                      << " target_fps=" << fps
                      << " dropped=" << dropped << std::endl;
        }
        return false;
    }

    last_ingest_slot_by_camera_[camera_name] = slot;
    return true;
}

void VideoRecorder::pushFrame(const std::string& camera_name, const cv::Mat& frame, uint64_t timestamp_ns) {
    if (frame.empty()) return;

    // 相机回调线程只做一次 Mat clone 和入队；缩放/JPEG 编码交给录像器缓存线程，
    // 避免 cv::imencode 阻塞解码/感知主链路。抽帧必须在 clone 之前，降低 JPEG 队列压力。
    const size_t max_pending = static_cast<size_t>(std::max(16, cfg_.fps * 4));
    {
        std::lock_guard<std::mutex> lk(frame_cache_mutex_);
        const std::string camera = safeCameraName(camera_name);
        if (!ingestFrameAtTargetFps(camera, frame, timestamp_ns)) return;

        while (pending_frame_cache_.size() >= max_pending) {
            pending_frame_cache_.pop_front();
            const uint64_t dropped = ++dropped_cache_frames_;
            if (dropped <= 10 || dropped % 100 == 0) {
                std::cerr << "[VideoRecorder] Drop raw cache frame, pending="
                          << pending_frame_cache_.size()
                          << " dropped=" << dropped << std::endl;
            }
        }
        pending_frame_cache_.push_back({camera, frame.clone(), timestamp_ns});
    }
    frame_cache_cv_.notify_one();
}

void VideoRecorder::cacheWorkerLoop() {
    while (running_.load()) {
        RawFrameTask task;
        {
            std::unique_lock<std::mutex> lk(frame_cache_mutex_);
            frame_cache_cv_.wait(lk, [this] {
                return !pending_frame_cache_.empty() || !running_.load();
            });
            if (!running_.load() && pending_frame_cache_.empty()) break;
            if (pending_frame_cache_.empty()) continue;
            task = std::move(pending_frame_cache_.front());
            pending_frame_cache_.pop_front();
        }
        encodeAndStoreFrame(task.camera_name, task.frame, task.timestamp_ns);
    }
}

void VideoRecorder::drainPendingFrameCache() {
    // 告警触发前把已入队但尚未 JPEG 化的帧刷入环形缓存，避免异步缓存线程
    // 尚未追上时 pre-event 快照漏掉触发前最后几帧。该操作只在低频触发点执行。
    std::deque<RawFrameTask> tasks;
    {
        std::lock_guard<std::mutex> lk(frame_cache_mutex_);
        tasks.swap(pending_frame_cache_);
    }
    for (auto& task : tasks) {
        encodeAndStoreFrame(task.camera_name, task.frame, task.timestamp_ns);
    }
}

void VideoRecorder::encodeAndStoreFrame(const std::string& camera_name, const cv::Mat& frame, uint64_t timestamp_ns) {
    if (frame.empty()) return;
    const size_t configured_capacity = static_cast<size_t>(std::max(1, cfg_.queue_capacity));
    const size_t time_capacity = static_cast<size_t>((std::max(1, cfg_.pre_seconds) + 1) * std::max(1, cfg_.fps));
    const size_t max_size = std::max<size_t>(1, std::min(configured_capacity, time_capacity));

    cv::Mat frame_for_cache;
    if (cfg_.max_frame_width > 0 && frame.cols > cfg_.max_frame_width) {
        const double scale = static_cast<double>(cfg_.max_frame_width) / static_cast<double>(frame.cols);
        const int target_h = std::max(1, static_cast<int>(std::lround(frame.rows * scale)));
        cv::resize(frame, frame_for_cache, cv::Size(cfg_.max_frame_width, target_h), 0.0, 0.0, cv::INTER_AREA);
    } else {
        frame_for_cache = frame;
    }

    std::vector<uchar> jpeg_data;
    const int quality = std::max(1, std::min(100, cfg_.jpeg_quality));
    if (!cv::imencode(".jpg", frame_for_cache, jpeg_data, {cv::IMWRITE_JPEG_QUALITY, quality}) || jpeg_data.empty()) {
        std::cerr << "[VideoRecorder] JPEG encode failed for camera=" << camera_name
                  << " size=" << frame.cols << "x" << frame.rows << std::endl;
        return;
    }

    std::lock_guard<std::mutex> lk(ring_mutex_);
    auto& ring_buffer = ring_buffers_by_camera_[safeCameraName(camera_name)];
    ring_buffer.push_back({std::move(jpeg_data), timestamp_ns});
    while (ring_buffer.size() > max_size) {
        ring_buffer.pop_front();
    }
}

bool VideoRecorder::triggerRecord(uint64_t event_id, const std::string& reason, int threat_level) {
    {
        std::lock_guard<std::mutex> lk(task_mutex_);
        if (recording_.load() || !pending_tasks_.empty()) {
            int cur = current_max_threat_level_.load();
            while (threat_level > cur &&
                   !current_max_threat_level_.compare_exchange_weak(cur, threat_level)) {
            }
            if (!reason.empty() && threat_level >= current_max_threat_level_.load()) {
                std::lock_guard<std::mutex> manifest_lk(current_manifest_mutex_);
                current_manifest_reason_ = reason;
            }
            if (!pending_tasks_.empty() && threat_level >= pending_tasks_.back().threat_level) {
                pending_tasks_.back().threat_level = threat_level;
                if (!reason.empty()) pending_tasks_.back().reason = reason;
            }
            std::cout << "[VideoRecorder] Skip event_" << event_id
                      << ": active/pending recording already covers this alert window"
                      << " max_threat_level=" << current_max_threat_level_.load()
                      << std::endl;
            return false;
        }
    }

    // 快照当前环形缓冲作为 pre-event 帧。先同步刷入缓存队列中已接收的帧，
    // 兼顾异步缓存降阻塞和触发瞬间 pre-frame 完整性。
    drainPendingFrameCache();

    RecordTask task;
    task.event_id = event_id;
    task.reason = reason;
    task.threat_level = threat_level;
    current_max_threat_level_.store(threat_level);
    {
        std::lock_guard<std::mutex> lk(current_manifest_mutex_);
        current_manifest_reason_ = reason;
    }
    std::map<std::string, size_t> pre_counts_by_camera;
    size_t total_pre_frames = 0;
    {
        std::lock_guard<std::mutex> lk(ring_mutex_);
        // 只保留最近 pre_seconds 秒，并多留 1 个抽帧槽，方便编码时对齐窗口起点。
        const uint64_t period_ns = 1000000000ULL / static_cast<uint64_t>(std::max(1, cfg_.fps));
        const uint64_t keep_ns = static_cast<uint64_t>(std::max(0, cfg_.pre_seconds)) * 1000000000ULL
                                 + period_ns;
        uint64_t latest_ts = 0;
        for (const auto& kv : ring_buffers_by_camera_) {
            if (!kv.second.empty()) latest_ts = std::max(latest_ts, kv.second.back().timestamp_ns);
        }
        for (const auto& kv : ring_buffers_by_camera_) {
            auto& dst = task.pre_frames_by_camera[kv.first];
            for (const auto& f : kv.second) {
                if (latest_ts > 0 && f.timestamp_ns > 0 &&
                    latest_ts - f.timestamp_ns <= keep_ns) {
                    dst.push_back(f);
                }
            }
            pre_counts_by_camera[kv.first] = dst.size();
            total_pre_frames += dst.size();
        }
        task.trigger_timestamp_ns = latest_ts;
    }

    std::ostringstream pre_ss;
    pre_ss << "{";
    bool first = true;
    for (const auto& kv : pre_counts_by_camera) {
        pre_ss << (first ? "" : ",") << kv.first << ":" << kv.second;
        first = false;
    }
    pre_ss << "}";

    std::cout << "[VideoRecorder][Trace] trigger_accept"
              << " event_id=" << event_id
              << " reason=\"" << reason << "\""
              << " threat_level=" << threat_level
              << " trigger_sensor_ts=" << task.trigger_timestamp_ns
              << " pre_total=" << total_pre_frames
              << " pre_by_camera=" << pre_ss.str()
              << " expected_pre_s=" << cfg_.pre_seconds
              << " expected_post_s=" << cfg_.post_seconds
              << std::endl;

    if (total_pre_frames == 0) {
        std::cerr << "[VideoRecorder] Warning event_" << event_id
                  << ": no pre-event frames in ring buffer at trigger time"
                  << std::endl;
    }

    {
        std::lock_guard<std::mutex> lk(task_mutex_);
        pending_tasks_.push_back(std::move(task));
    }
    task_cv_.notify_one();
    return true;
}

void VideoRecorder::updateMaxThreatLevel(int threat_level, const std::string& reason) {
    if (threat_level <= 0) return;

    bool promoted = false;
    int cur = current_max_threat_level_.load();
    while (threat_level > cur) {
        if (current_max_threat_level_.compare_exchange_weak(cur, threat_level)) {
            promoted = true;
            break;
        }
    }

    if (!reason.empty() && (promoted || threat_level >= current_max_threat_level_.load())) {
        std::lock_guard<std::mutex> lk(current_manifest_mutex_);
        current_manifest_reason_ = reason;
    }

    std::lock_guard<std::mutex> lk(task_mutex_);
    if (!pending_tasks_.empty()) {
        if (threat_level >= pending_tasks_.back().threat_level) {
            pending_tasks_.back().threat_level = threat_level;
            if (!reason.empty()) pending_tasks_.back().reason = reason;
        }
    }
}

void VideoRecorder::workerLoop() {
    while (running_.load()) {
        RecordTask task;
        {
            std::unique_lock<std::mutex> lk(task_mutex_);
            task_cv_.wait(lk, [this] {
                return !pending_tasks_.empty() || !running_.load();
            });
            if (!running_.load() && pending_tasks_.empty()) break;
            if (pending_tasks_.empty()) continue;
            task = std::move(pending_tasks_.front());
            pending_tasks_.pop_front();
        }

        recording_ = true;
        const auto task_wall_start = std::chrono::steady_clock::now();
        std::cout << "[VideoRecorder][Trace] worker_start"
                  << " event_id=" << task.event_id
                  << " trigger_sensor_ts=" << task.trigger_timestamp_ns
                  << " pre_cameras=" << task.pre_frames_by_camera.size()
                  << " expected_pre_s=" << cfg_.pre_seconds
                  << " expected_post_s=" << cfg_.post_seconds
                  << std::endl;

        // 收集 post-event 帧：按帧时间戳持续采集 trigger 后 post_seconds 秒。
        // 不能按真实时钟简单 sleep post_seconds：bag 慢放/调度抖动时会导致视频时长不足。
        const uint64_t post_ns = static_cast<uint64_t>(cfg_.post_seconds) * 1000000000ULL;
        const auto post_start = std::chrono::steady_clock::now();
        std::map<std::string, std::vector<TimestampedFrame>> post_frames_by_camera;
        // post-event 只采集触发点之后的新帧。旧实现从 0 开始，会把 ring buffer
        // 中已有 pre-frame 再次 clone 到 post_frames，30s+30s 配置下会显著放大
        // 内存占用，并造成重复帧。
        std::map<std::string, uint64_t> last_captured_ts_by_camera;
        for (const auto& kv : task.pre_frames_by_camera) {
            last_captured_ts_by_camera[kv.first] = task.trigger_timestamp_ns;
        }
        const uint64_t post_end_ts = task.trigger_timestamp_ns > 0
            ? task.trigger_timestamp_ns + post_ns
            : 0;

        while (running_.load()) {
            // 以各路环形缓冲里最新传感器时间为准；任意一路到达 post_end 即认为
            // 时间窗走完，缺帧的路在编码时用最近帧补齐到固定时长。
            uint64_t newest_ts = 0;
            {
                std::lock_guard<std::mutex> lk(ring_mutex_);
                for (const auto& kv : ring_buffers_by_camera_) {
                    auto& dst = post_frames_by_camera[kv.first];
                    auto& last_captured_ts = last_captured_ts_by_camera[kv.first];
                    if (last_captured_ts == 0) last_captured_ts = task.trigger_timestamp_ns;
                    for (const auto& f : kv.second) {
                        if (f.timestamp_ns > last_captured_ts) {
                            if (post_end_ts == 0 || f.timestamp_ns <= post_end_ts) {
                                dst.push_back(f);
                            }
                            last_captured_ts = f.timestamp_ns;
                        }
                    }
                    if (!kv.second.empty()) {
                        newest_ts = std::max(newest_ts, kv.second.back().timestamp_ns);
                    }
                }
            }
            if (post_end_ts > 0 && newest_ts >= post_end_ts) {
                break;
            }
            // 摄像头停止或节点退出时避免无限等待；正常车端帧流会按上面的 sensor-time 条件退出。
            const auto elapsed = std::chrono::steady_clock::now() - post_start;
            if (std::chrono::duration_cast<std::chrono::seconds>(elapsed).count()
                >= std::max(1, cfg_.post_seconds) * 3) {
                std::cerr << "[VideoRecorder][Trace] post_collect_timeout"
                          << " event_id=" << task.event_id
                          << " waited_wall_s=" << elapsedSeconds(post_start)
                          << " expected_post_s=" << cfg_.post_seconds
                          << " trigger_sensor_ts=" << task.trigger_timestamp_ns
                          << " post_end_sensor_ts=" << post_end_ts
                          << std::endl;
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }

        const auto post_done = std::chrono::steady_clock::now();
        std::ostringstream post_counts_ss;
        post_counts_ss << "{";
        bool post_first = true;
        for (const auto& kv : post_frames_by_camera) {
            post_counts_ss << (post_first ? "" : ",") << kv.first << ":" << kv.second.size();
            post_first = false;
        }
        post_counts_ss << "}";
        std::cout << "[VideoRecorder][Trace] post_collect_done"
                  << " event_id=" << task.event_id
                  << " waited_wall_s=" << elapsedSeconds(post_start, post_done)
                  << " trigger_sensor_ts=" << task.trigger_timestamp_ns
                  << " post_end_sensor_ts=" << post_end_ts
                  << " post_by_camera=" << post_counts_ss.str()
                  << std::endl;

        const int output_fps = std::max(1, cfg_.fps);
        const int expected_seconds = std::max(0, cfg_.pre_seconds) + std::max(0, cfg_.post_seconds);
        const int expected_frames = expected_seconds * output_fps;
        const uint64_t pre_ns = static_cast<uint64_t>(std::max(0, cfg_.pre_seconds)) * 1000000000ULL;
        const uint64_t window_start_ns = (task.trigger_timestamp_ns > pre_ns)
            ? task.trigger_timestamp_ns - pre_ns
            : task.trigger_timestamp_ns;

        std::map<std::string, std::vector<TimestampedFrame>> all_frames_by_camera;
        std::map<std::string, size_t> pre_frames_count_by_camera;
        std::map<std::string, size_t> post_frames_count_by_camera;
        std::map<std::string, int> camera_names;
        for (const auto& kv : task.pre_frames_by_camera) camera_names[kv.first] = 1;
        for (const auto& kv : post_frames_by_camera) camera_names[kv.first] = 1;
        for (const auto& name_kv : camera_names) {
            const std::string& camera = name_kv.first;
            auto& pre_frames = task.pre_frames_by_camera[camera];
            auto& post_frames = post_frames_by_camera[camera];

            std::vector<TimestampedFrame> all_frames;
            all_frames.reserve(pre_frames.size() + post_frames.size());
            all_frames.insert(all_frames.end(),
                              std::make_move_iterator(pre_frames.begin()),
                              std::make_move_iterator(pre_frames.end()));
            all_frames.insert(all_frames.end(),
                              std::make_move_iterator(post_frames.begin()),
                              std::make_move_iterator(post_frames.end()));
            pre_frames_count_by_camera[camera] = pre_frames.size();
            post_frames_count_by_camera[camera] = post_frames.size();
            all_frames_by_camera[camera] = std::move(all_frames);
        }

        // 输出路径：同一次报警使用统一事件窗口起点命名。
        const std::string event_start_time = sentryTimestampString(window_start_ns);
        std::ostringstream dir_ss;
        dir_ss << cfg_.output_dir << "/" << event_start_time << "_" << task.event_id;
        fs::create_directories(dir_ss.str());

        struct EncodeResult {
            std::string camera;
            std::string video_path;
            size_t frames{0};
            bool ok{false};
            double encode_wall_s{0.0};
        };

        std::vector<std::string> video_paths;
        std::map<std::string, size_t> total_frames_by_camera;
        std::vector<std::future<EncodeResult>> encode_futures;
        bool any_ok = false;
        for (auto& kv : all_frames_by_camera) {
            const std::string camera = kv.first;
            auto& all_frames = kv.second;
            if (all_frames.empty()) {
                total_frames_by_camera[camera] = 0;
                continue;
            }

            std::vector<TimestampedFrame> cfr_frames = resampleFramesToFixedWindow(
                all_frames, window_start_ns, expected_frames, output_fps);
            if (cfr_frames.empty()) {
                total_frames_by_camera[camera] = 0;
                continue;
            }
            total_frames_by_camera[camera] = cfr_frames.size();
            const double sensor_duration_s = all_frames.back().timestamp_ns > all_frames.front().timestamp_ns
                ? static_cast<double>(all_frames.back().timestamp_ns - all_frames.front().timestamp_ns) / 1.0e9
                : 0.0;
            const double pre_duration_s = task.trigger_timestamp_ns > all_frames.front().timestamp_ns
                ? static_cast<double>(task.trigger_timestamp_ns - all_frames.front().timestamp_ns) / 1.0e9
                : 0.0;
            const double post_duration_s = all_frames.back().timestamp_ns > task.trigger_timestamp_ns
                ? static_cast<double>(all_frames.back().timestamp_ns - task.trigger_timestamp_ns) / 1.0e9
                : 0.0;
            std::ostringstream path_ss;
            path_ss << dir_ss.str() << "/" << event_start_time << "_"
                    << sentryCameraSuffix(camera) << ".mp4";
            const std::string video_path = path_ss.str();

            std::cout << "[VideoRecorder] Encoding event_" << task.event_id
                      << " camera=" << camera
                      << " src_frames=" << all_frames.size()
                      << " cfr_frames=" << cfr_frames.size()
                      << " pre=" << pre_frames_count_by_camera[camera]
                      << " post=" << post_frames_count_by_camera[camera]
                      << " start_ts=" << all_frames.front().timestamp_ns
                      << " end_ts=" << all_frames.back().timestamp_ns
                      << " window_start_ts=" << window_start_ns
                      << " sensor_duration_s=" << sensor_duration_s
                      << " pre_duration_s=" << pre_duration_s
                      << " post_duration_s=" << post_duration_s
                      << " expected_seconds=" << expected_seconds
                      << " expected_frames=" << expected_frames
                      << " pre_cache_complete=" << (pre_duration_s + 0.1 >= static_cast<double>(cfg_.pre_seconds) ? 1 : 0)
                      << " → " << video_path << std::endl;

            encode_futures.push_back(std::async(std::launch::async,
                [this, camera, video_path, frames = std::move(cfr_frames)]() mutable -> EncodeResult {
                    const auto encode_start = std::chrono::steady_clock::now();
                    const bool encoded_ok = encodeToFile(video_path, frames);
                    return EncodeResult{camera, video_path, frames.size(), encoded_ok, elapsedSeconds(encode_start)};
                }));
        }

        for (auto& fut : encode_futures) {
            const EncodeResult result = fut.get();
            std::cout << "[VideoRecorder][Trace] encode_done"
                      << " event_id=" << task.event_id
                      << " camera=" << result.camera
                      << " ok=" << (result.ok ? 1 : 0)
                      << " encode_wall_s=" << result.encode_wall_s
                      << " frames=" << result.frames
                      << " path=" << result.video_path
                      << std::endl;

            if (result.ok) {
                any_ok = true;
                video_paths.push_back(result.video_path);
            } else {
                std::cerr << "[VideoRecorder] Event " << task.event_id
                          << " camera=" << result.camera << " encoding FAILED" << std::endl;
            }
        }

        if (any_ok) {
            const int manifest_threat_level = std::max(task.threat_level,
                                                       current_max_threat_level_.load());
            std::string manifest_reason = task.reason;
            {
                std::lock_guard<std::mutex> lk(current_manifest_mutex_);
                if (!current_manifest_reason_.empty()) manifest_reason = current_manifest_reason_;
            }
            std::ostringstream manifest_ss;
            manifest_ss << dir_ss.str() << "/manifest.json";
            std::ofstream mf(manifest_ss.str());
            mf << "{\n"
               << "  \"event_id\": " << task.event_id << ",\n"
               << "  \"reason\": \"" << manifest_reason << "\",\n"
               << "  \"threat_level\": " << manifest_threat_level << ",\n"
               << "  \"pre_seconds\": " << cfg_.pre_seconds << ",\n"
               << "  \"post_seconds\": " << cfg_.post_seconds << ",\n"
               << "  \"videos\": [";
            for (size_t i = 0; i < video_paths.size(); ++i) {
                mf << (i ? ", " : "") << "\"" << video_paths[i] << "\"";
            }
            mf << "],\n  \"frames_by_camera\": {";
            size_t idx = 0;
            for (const auto& kv : total_frames_by_camera) {
                mf << (idx++ ? ", " : "") << "\"" << kv.first << "\": " << kv.second;
            }
            mf << "}\n}\n";

            std::cout << "[VideoRecorder][Trace] manifest_written"
                      << " event_id=" << task.event_id
                      << " manifest=" << manifest_ss.str()
                      << " videos=" << video_paths.size()
                      << " total_wall_s=" << elapsedSeconds(task_wall_start)
                      << std::endl;
        } else {
            std::cerr << "[VideoRecorder] Event " << task.event_id
                      << " encoding FAILED for all cameras" << std::endl;
        }

        std::cout << "[VideoRecorder][Trace] done_callback"
                  << " event_id=" << task.event_id
                  << " any_ok=" << (any_ok ? 1 : 0)
                  << " videos=" << video_paths.size()
                  << " total_wall_s=" << elapsedSeconds(task_wall_start)
                  << std::endl;
        if (done_cb_) done_cb_(task.event_id, any_ok ? video_paths : std::vector<std::string>{});
        recording_ = false;
    }
}

std::vector<VideoRecorder::TimestampedFrame> VideoRecorder::resampleFramesToFixedWindow(
    const std::vector<TimestampedFrame>& input,
    uint64_t window_start_ns,
    int frame_count,
    int fps) const {
    if (input.empty() || fps <= 0 || frame_count <= 0) return {};

    std::vector<TimestampedFrame> sorted;
    sorted.reserve(input.size());
    for (const auto& f : input) {
        if (f.jpeg_data.empty() || f.timestamp_ns == 0) continue;
        sorted.push_back(f);
    }
    if (sorted.empty()) return {};

    std::sort(sorted.begin(), sorted.end(),
              [](const TimestampedFrame& a, const TimestampedFrame& b) {
                  return a.timestamp_ns < b.timestamp_ns;
              });

    std::vector<TimestampedFrame> unique;
    unique.reserve(sorted.size());
    for (auto& f : sorted) {
        if (!unique.empty() && f.timestamp_ns == unique.back().timestamp_ns) continue;
        unique.push_back(std::move(f));
    }
    if (unique.empty()) return {};

    const uint64_t period_ns = 1000000000ULL / static_cast<uint64_t>(fps);
    uint64_t start_ts = window_start_ns;
    if (start_ts == 0) start_ts = unique.front().timestamp_ns;

    auto nearestIndex = [&](uint64_t target) -> size_t {
        auto it = std::lower_bound(
            unique.begin(), unique.end(), target,
            [](const TimestampedFrame& f, uint64_t ts) { return f.timestamp_ns < ts; });
        if (it == unique.begin()) return 0;
        if (it == unique.end()) return unique.size() - 1;
        const size_t hi = static_cast<size_t>(it - unique.begin());
        const size_t lo = hi - 1;
        const uint64_t d_lo = target - unique[lo].timestamp_ns;
        const uint64_t d_hi = unique[hi].timestamp_ns - target;
        return (d_lo <= d_hi) ? lo : hi;
    };

    std::vector<TimestampedFrame> out;
    out.reserve(static_cast<size_t>(frame_count));
    for (int i = 0; i < frame_count; ++i) {
        const uint64_t offset_ns = static_cast<uint64_t>(i) * period_ns;
        const uint64_t target = (offset_ns > std::numeric_limits<uint64_t>::max() - start_ts)
            ? std::numeric_limits<uint64_t>::max()
            : start_ts + offset_ns;
        out.push_back(unique[nearestIndex(target)]);
    }
    return out;
}

bool VideoRecorder::encodeToFile(const std::string& path,
                                  const std::vector<TimestampedFrame>& frames) {
#ifndef SENTINEL_ENABLE_FFMPEG_DECODER
    (void)path;
    (void)frames;
    static std::atomic<bool> warned{false};
    bool expected = false;
    if (warned.compare_exchange_strong(expected, true)) {
        std::cerr << "[VideoRecorder] FFmpeg encoder disabled at build time; "
                  << "event video recording is unavailable." << std::endl;
    }
    return false;
#else
    if (frames.empty()) return false;

    const int output_fps = std::max(1, cfg_.fps);
    const std::vector<TimestampedFrame>& cfr_frames = frames;

    cv::Mat first_image = cv::imdecode(cfr_frames.front().jpeg_data, cv::IMREAD_COLOR);
    if (first_image.empty()) {
        std::cerr << "[VideoRecorder] Failed to decode first JPEG frame for " << path << std::endl;
        return false;
    }
    const int width = first_image.cols;
    const int height = first_image.rows;
    if (width <= 0 || height <= 0) return false;

    std::cout << "[VideoRecorder] encode " << path
              << " frames=" << cfr_frames.size()
              << " fps=" << output_fps
              << " duration_s=" << (static_cast<double>(cfr_frames.size()) / output_fps)
              << std::endl;

    // 查找编码器
    AVCodec* codec = avcodec_find_encoder_by_name(cfg_.codec_name.c_str());
    if (!codec) {
        // 回退到 libx264
        codec = avcodec_find_encoder_by_name("libx264");
    }
    if (!codec) {
        codec = avcodec_find_encoder(AV_CODEC_ID_H264);
    }
    if (!codec) {
        std::cerr << "[VideoRecorder] No suitable encoder found" << std::endl;
        return false;
    }

    AVCodecContext* enc_ctx = avcodec_alloc_context3(codec);
    if (!enc_ctx) return false;

    enc_ctx->width = width;
    enc_ctx->height = height;
    enc_ctx->time_base = {1, output_fps};
    enc_ctx->framerate = {output_fps, 1};
    enc_ctx->pix_fmt = AV_PIX_FMT_YUV420P;
    enc_ctx->bit_rate = static_cast<int64_t>(cfg_.bitrate_kbps) * 1000;
    enc_ctx->gop_size = output_fps * 2;  // 每 2 秒一个关键帧
    enc_ctx->max_b_frames = 0;

    // 低延迟预设
    if (codec->id == AV_CODEC_ID_H265 || codec->id == AV_CODEC_ID_HEVC) {
        av_opt_set(enc_ctx->priv_data, "preset", "fast", 0);
        av_opt_set(enc_ctx->priv_data, "tune", "zerolatency", 0);
    } else {
        av_opt_set(enc_ctx->priv_data, "preset", "fast", 0);
        av_opt_set(enc_ctx->priv_data, "tune", "zerolatency", 0);
    }

    AVFormatContext* fmt_ctx = nullptr;
    int ret = avformat_alloc_output_context2(&fmt_ctx, nullptr, "mp4", path.c_str());
    if (ret < 0 || !fmt_ctx) {
        avcodec_free_context(&enc_ctx);
        return false;
    }

    // 某些 muxer 需要 GLOBAL_HEADER
    if (fmt_ctx->oformat->flags & AVFMT_GLOBALHEADER) {
        enc_ctx->flags |= AV_CODEC_FLAG_GLOBAL_HEADER;
    }

    ret = avcodec_open2(enc_ctx, codec, nullptr);
    if (ret < 0) {
        std::cerr << "[VideoRecorder] avcodec_open2 failed: " << ret << std::endl;
        avcodec_free_context(&enc_ctx);
        avformat_free_context(fmt_ctx);
        return false;
    }

    AVStream* stream = avformat_new_stream(fmt_ctx, nullptr);
    if (!stream) {
        avcodec_free_context(&enc_ctx);
        avformat_free_context(fmt_ctx);
        return false;
    }
    avcodec_parameters_from_context(stream->codecpar, enc_ctx);
    stream->time_base = enc_ctx->time_base;
    stream->avg_frame_rate = {output_fps, 1};
    stream->r_frame_rate = {output_fps, 1};

    if (!(fmt_ctx->oformat->flags & AVFMT_NOFILE)) {
        ret = avio_open(&fmt_ctx->pb, path.c_str(), AVIO_FLAG_WRITE);
        if (ret < 0) {
            avcodec_free_context(&enc_ctx);
            avformat_free_context(fmt_ctx);
            return false;
        }
    }

    ret = avformat_write_header(fmt_ctx, nullptr);
    if (ret < 0) {
        avcodec_free_context(&enc_ctx);
        if (!(fmt_ctx->oformat->flags & AVFMT_NOFILE)) avio_closep(&fmt_ctx->pb);
        avformat_free_context(fmt_ctx);
        return false;
    }

    // BGR → YUV420P 转换
    SwsContext* sws = sws_getContext(width, height, AV_PIX_FMT_BGR24,
                                     width, height, AV_PIX_FMT_YUV420P,
                                     SWS_FAST_BILINEAR, nullptr, nullptr, nullptr);

    AVFrame* av_frame = av_frame_alloc();
    av_frame->format = AV_PIX_FMT_YUV420P;
    av_frame->width = width;
    av_frame->height = height;
    av_image_alloc(av_frame->data, av_frame->linesize, width, height, AV_PIX_FMT_YUV420P, 32);

    AVPacket* pkt = av_packet_alloc();
    bool success = true;
    int64_t pts = 0;
    cv::Mat last_ok;

    auto write_packet = [&]() {
        if (pkt->duration <= 0) pkt->duration = 1;
        av_packet_rescale_ts(pkt, enc_ctx->time_base, stream->time_base);
        pkt->stream_index = stream->index;
        av_interleaved_write_frame(fmt_ctx, pkt);
        av_packet_unref(pkt);
    };

    for (const auto& f : cfr_frames) {
        cv::Mat image;
        if (!f.jpeg_data.empty()) {
            image = cv::imdecode(f.jpeg_data, cv::IMREAD_COLOR);
        }
        if (image.empty() || image.cols != width || image.rows != height) {
            if (last_ok.empty()) continue;
            image = last_ok;
        } else {
            last_ok = image;
        }

        const uint8_t* src_data[1] = { image.data };
        int src_linesize[1] = { static_cast<int>(image.step[0]) };
        sws_scale(sws, src_data, src_linesize, 0, height,
                  av_frame->data, av_frame->linesize);

        av_frame->pts = pts++;
        av_frame->pkt_duration = 1;

        ret = avcodec_send_frame(enc_ctx, av_frame);
        if (ret < 0) { success = false; break; }

        while (ret >= 0) {
            ret = avcodec_receive_packet(enc_ctx, pkt);
            if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF) break;
            if (ret < 0) { success = false; break; }
            write_packet();
        }
    }

    // Flush encoder
    avcodec_send_frame(enc_ctx, nullptr);
    while (true) {
        ret = avcodec_receive_packet(enc_ctx, pkt);
        if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF) break;
        if (ret < 0) break;
        write_packet();
    }

    if (pts > 0) {
        stream->duration = pts;
        fmt_ctx->duration = av_rescale_q(pts, enc_ctx->time_base, AV_TIME_BASE_Q);
    }

    av_write_trailer(fmt_ctx);

    // Cleanup
    av_packet_free(&pkt);
    av_freep(&av_frame->data[0]);
    av_frame_free(&av_frame);
    sws_freeContext(sws);
    avcodec_free_context(&enc_ctx);
    if (!(fmt_ctx->oformat->flags & AVFMT_NOFILE)) avio_closep(&fmt_ctx->pb);
    avformat_free_context(fmt_ctx);

    return success;
#endif
}

}  // namespace sentinel
