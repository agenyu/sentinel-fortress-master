#pragma once

#include "../hevc_decoder_factory.h"
#include "../imu/imu_subscriber.h"
#include "rscl_access/rscl_access.h"
#include "ad_msg_idl/std_msgs/raw_data.capnp.h"
#include "ad_common/data_type/base.hpp"

#include <opencv2/opencv.hpp>
#include <functional>
#include <thread>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <vector>
#include <memory>
#include <string>
#include <chrono>

namespace sentinel {

enum class CameraInputMode {
    ENCODED_HEVC,  // RSCL 普通订阅 CameraEncodedStruct，再走 NVDEC 解码
    CUDA_FRAME,    // RSCL CudaSubscriber 订阅 ImageHeader_t + BGR/NV12 raw frame
};

CameraInputMode parseCameraInputMode(const std::string& mode);
const char* cameraInputModeName(CameraInputMode mode);

struct CameraDecodeDebugConfig {
    bool enabled{false};
    std::string output_dir{"./nvmedia_decode_debug"};
    int max_frames_per_camera{5};
    bool save_encoded_hevc{true};
    bool save_decoded_bgr{true};
};

// 解码后的帧数据，向下游传递
struct DecodedFrame {
    uint64_t    seq{0};
    uint64_t    timestamp_ns{0};
    uint64_t    processing_enqueue_steady_ns{0};  // SentinelNode 入处理队列的 steady clock 时间，用于 queue wait 统计
    uint64_t    recording_group_id{0};             // 多相机同步组 ID；用于 4 路录像 mosaic 合成
    uint64_t    recording_group_timestamp_ns{0};   // 同步组时间戳
    size_t      recording_group_size{1};           // 同步组路数；单路回退为 1
    int         frame_type{0};   // 编码流: 2=I帧, 1=P帧；CUDA raw frame: 0
    cv::Mat     image;           // BGR（鱼眼 1920x1536 或其他分辨率）
    ImuData     imu;             // 与该帧时间对齐的 IMU 快照
    std::string camera_name;     // 来源相机名称，如 "front_camera_fov195"
};

// 帧就绪回调
using FrameCallback = std::function<void(const DecodedFrame&)>;

/**
 * @brief 摄像头订阅器
 *
 * 支持两种输入：
 *  1. encoded_hevc：订阅 HEVC 编码流 → 异步 NVDEC 解码 → BGR 帧。
 *  2. cuda_frame：订阅车端 CUDA Memblock → 解析 ImageHeader_t + BGR/NV12 raw frame → BGR 帧。
 * 每帧携带同步的 ImuData 快照。
 */
class CameraSubscriber {
public:
    explicit CameraSubscriber(std::shared_ptr<ImuSubscriber> imu,
                              int width = 1920, int height = 1536,
                              int gpu_id = 0,
                              CameraInputMode input_mode = CameraInputMode::ENCODED_HEVC,
                              HevcDecoderBackend decoder_backend = HevcDecoderBackend::FfmpegNvdec,
                              CameraDecodeDebugConfig debug_config = {});
    ~CameraSubscriber();

    // 注册到 RSCL，camera_name 用于标识来源（如 "front_camera_fov195"）
    void subscribe(const std::string& topic, const std::string& camera_name = {});

    // 注册帧就绪回调（在解码线程调用，勿做耗时操作）
    void setFrameCallback(FrameCallback cb) { frame_cb_ = std::move(cb); }

    uint64_t successCount() const { return success_count_.load(); }
    uint64_t failCount()    const { return fail_count_.load(); }

private:
    using CameraEncodedMsg = senseAD::msg::std_msgs::CameraEncodedStruct;
    using MemblockPtr = RsclAccess::MemblockPtr;

    // 解码/转码任务
    struct DecodeTask {
        uint64_t             seq{0};
        uint64_t             timestamp_ns{0};
        int                  frame_type{0};
        bool                 is_keyframe{false};
        bool                 already_decoded{false};
        std::vector<uint8_t> encoded_data;
        cv::Mat              decoded_bgr;
        ImuData              imu;
    };

    void onCamera(const std::shared_ptr<ReceivedMsg<CameraEncodedMsg>>& msg);
    void onCudaFrame(const MemblockPtr& msg);
    void submitTask(uint64_t seq, uint64_t ts, int ftype, bool is_key,
                    const uint8_t* data, size_t size, const ImuData& imu);
    void submitDecodedTask(uint64_t seq, uint64_t ts, cv::Mat bgr, const ImuData& imu);
    void workerLoop();
    void processTask(const DecodeTask& task);
    void saveDecodeDebugArtifacts(const DecodeTask& task, const cv::Mat& decoded);

    std::shared_ptr<ImuSubscriber> imu_;
    std::unique_ptr<IHevcDecoder> decoder_;
    FrameCallback                  frame_cb_;
    CameraInputMode                input_mode_{CameraInputMode::ENCODED_HEVC};
    CameraDecodeDebugConfig        debug_config_;

    // 当前订阅的相机名称
    std::string camera_name_;

    // GOP 状态（仅编码流回调线程访问）
    bool     has_keyframe_{false};
    uint64_t last_seq_{0};
    uint64_t last_ts_{0};
    int      gop_frame_count_{0};
    // 解码队列曾丢过编码帧：HEVC 参考链已断，必须等到下一 I 帧再送解，否则会出马赛克。
    std::atomic<bool> hevc_resync_required_{false};

    // 车端视频链路诊断：定位 HEVC 参考帧缺失/POC、解码前丢包、队列丢帧和慢解码。
    uint64_t encoded_count_{0};
    uint64_t wait_keyframe_drop_count_{0};
    uint64_t seq_gap_count_{0};
    uint64_t queue_drop_count_{0};
    uint64_t decode_fail_log_count_{0};
    uint64_t slow_decode_log_count_{0};
    uint64_t debug_saved_count_{0};
    std::chrono::steady_clock::time_point last_encoded_diag_log_{};

    // 解码队列
    static constexpr size_t kMaxQueueSize = 10;
    std::queue<DecodeTask>  queue_;
    std::mutex              queue_mutex_;
    std::condition_variable queue_cv_;
    std::thread             worker_;
    std::atomic<bool>       running_{true};

    std::atomic<uint64_t> success_count_{0};
    std::atomic<uint64_t> fail_count_{0};
    std::atomic<uint64_t> cuda_seq_{0};
};

}  // namespace sentinel
