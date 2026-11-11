#pragma once
/**
 * @brief 告警视频录制器（独立线程）
 *
 * 功能：
 *   - 持续接收各路相机原始鱼眼 BGR 帧，压缩为 JPEG 后写入环形缓冲（保留最近 pre_seconds 秒余量）
 *   - 告警触发时，把各路相机环形缓冲中的 JPEG 帧解码后编码为 H.265 视频（pre-event），
 *     并继续录制 post_seconds 秒（post-event），最终每路相机输出一个完整 .mp4 文件。
 *   - 编码和 I/O 在独立工作线程完成，不阻塞感知主线程。
 *
 * 依赖：FFmpeg libavcodec/libavformat/libavutil/libswscale（已在项目 CMake 中链接）。
 */

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <string>
#include <vector>
#include <deque>
#include <mutex>
#include <condition_variable>
#include <thread>
#include <atomic>
#include <functional>
#include <cstdint>
#include <map>
#include <utility>

// Forward-declare FFmpeg opaque types to avoid leaking C headers into every TU.
struct AVFormatContext;
struct AVCodecContext;
struct AVStream;
struct AVFrame;
struct AVPacket;
struct SwsContext;

namespace sentinel {

struct VideoRecorderConfig {
    int    pre_seconds{30};           // 告警前保留秒数
    int    post_seconds{30};          // 告警后继续录制秒数
    int    fps{10};                   // 四路统一目标帧率；入队按此时钟抽帧，编码输出恒定帧率
    int    bitrate_kbps{2000};        // 编码码率 kbps
    std::string output_dir{"./events"};  // 输出目录
    std::string codec_name{"libx265"}; // 编码器名称（可选 libx264）
    int    queue_capacity{1200};      // 每路相机环形缓冲最大帧数上限
    int    max_frame_width{640};      // JPEG缓存前的最大宽度；0 表示保留原始分辨率
    int    jpeg_quality{85};          // 环形缓冲JPEG质量，降低BGR原图缓存内存占用
};

// 录制完成回调：(event_id, video_paths)
using RecordDoneCallback = std::function<void(uint64_t event_id, const std::vector<std::string>& paths)>;

class VideoRecorder {
public:
    explicit VideoRecorder(const VideoRecorderConfig& cfg = {});
    ~VideoRecorder();

    VideoRecorder(const VideoRecorder&) = delete;
    VideoRecorder& operator=(const VideoRecorder&) = delete;

    /// 启动工作线程
    void start();

    /// 停止工作线程（会等待当前录制完成）
    void stop();

    /// 每帧调用，把原始鱼眼帧 JPEG 压缩后送入环形缓冲（线程安全）
    void pushFrame(const cv::Mat& frame, uint64_t timestamp_ns);
    void pushFrame(const std::string& camera_name, const cv::Mat& frame, uint64_t timestamp_ns);

    /// 告警触发时调用，启动一次录制任务。
    /// 同一时间只保留一个录制任务：若已有任务正在录制/等待，则认为当前告警
    /// 已被正在覆盖的 pre+post 窗口包含，直接丢弃，避免告警风暴下重复 clone
    /// 大量 pre-frame 导致内存膨胀。
    /// @return true 表示已接受本次录制任务；false 表示已被合并/限流。
    bool triggerRecord(uint64_t event_id, const std::string& reason = {}, int threat_level = 0);

    /// 更新当前/待处理录像窗口覆盖到的最大告警等级和对应原因。
    /// manifest 中 threat_level 使用产品口径：1=alert(一般威胁警戒), 2=alarm(严重威胁报警)。
    /// reason 必须与最终最大 threat_level 保持一致。
    void updateMaxThreatLevel(int threat_level, const std::string& reason = {});

    /// 注册录制完成回调
    void setDoneCallback(RecordDoneCallback cb) { done_cb_ = std::move(cb); }

    bool isRecording() const { return recording_.load(); }

private:
    struct TimestampedFrame {
        std::vector<uchar> jpeg_data;
        uint64_t timestamp_ns{0};
    };

    // 工作线程主循环
    void workerLoop();

    // 原始帧缓存线程：把相机回调线程送入的 BGR 帧异步压缩为 JPEG 环形缓存。
    void cacheWorkerLoop();
    void encodeAndStoreFrame(const std::string& camera_name, const cv::Mat& frame, uint64_t timestamp_ns);
    void drainPendingFrameCache();

    // 入队前把各路抽到 cfg_.fps。同 100ms 槽只保留更靠近槽中心的一帧。
    // 调用方必须已持有 frame_cache_mutex_。
    bool ingestFrameAtTargetFps(const std::string& camera_name,
                                const cv::Mat& frame,
                                uint64_t timestamp_ns);

    // 把各路帧对齐到统一时间窗，输出固定帧数 (pre+post)*fps；缺帧用最近帧补齐。
    std::vector<TimestampedFrame> resampleFramesToFixedWindow(
        const std::vector<TimestampedFrame>& input,
        uint64_t window_start_ns,
        int frame_count,
        int fps) const;

    // 解码一组 JPEG 缓存帧并编码到 mp4
    bool encodeToFile(const std::string& path,
                      const std::vector<TimestampedFrame>& frames);

    VideoRecorderConfig cfg_;
    RecordDoneCallback  done_cb_;

    // 环形缓冲（缓存线程写，录制工作线程读快照）
    mutable std::mutex ring_mutex_;
    std::map<std::string, std::deque<TimestampedFrame>> ring_buffers_by_camera_;

    struct RawFrameTask {
        std::string camera_name;
        cv::Mat frame;
        uint64_t timestamp_ns{0};
    };
    std::mutex frame_cache_mutex_;
    std::condition_variable frame_cache_cv_;
    std::deque<RawFrameTask> pending_frame_cache_;
    std::thread cache_worker_;
    std::atomic<uint64_t> dropped_cache_frames_{0};
    std::map<std::string, uint64_t> last_ingest_slot_by_camera_;
    std::atomic<uint64_t> rate_limited_frames_{0};

    // 录制任务队列
    struct RecordTask {
        uint64_t event_id{0};
        std::string reason;
        int threat_level{0};
        std::map<std::string, std::vector<TimestampedFrame>> pre_frames_by_camera;  // 快照
        uint64_t trigger_timestamp_ns{0};
    };

    std::mutex task_mutex_;
    std::condition_variable task_cv_;
    std::deque<RecordTask> pending_tasks_;

    std::thread worker_;
    std::atomic<bool> running_{false};
    std::atomic<bool> recording_{false};
    std::atomic<int> current_max_threat_level_{0};
    mutable std::mutex current_manifest_mutex_;
    std::string current_manifest_reason_;
    std::atomic<uint64_t> next_event_id_{1};
};

}  // namespace sentinel
