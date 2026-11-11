#include "camera_subscriber.h"
#include <ad_log/ad_log.hpp>
#include <cuda_runtime.h>
#include <iostream>
#include <chrono>
#include <algorithm>
#include <cctype>
#include <limits>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>

namespace sentinel {

namespace {

std::string normalizeMode(std::string mode) {
    std::transform(mode.begin(), mode.end(), mode.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    std::replace(mode.begin(), mode.end(), '-', '_');
    return mode;
}

constexpr int kMaxCudaFrameWidth = 4096;
constexpr int kMaxCudaFrameHeight = 4096;
constexpr size_t kMaxCudaFrameBytes = 128ULL * 1024ULL * 1024ULL;
constexpr uint64_t kDiagEveryEncodedFrames = 300;
constexpr int64_t kSlowDecodeWarnMs = 120;

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

std::string frameTypeToken(int frame_type) {
    if (frame_type == 2) return "I";
    if (frame_type == 1) return "P";
    return "raw";
}

size_t expectedImageBytes(const senseAD::ImageHeader_t& header) {
    if (header.width <= 0 || header.height <= 0) return 0;
    if (header.width > kMaxCudaFrameWidth || header.height > kMaxCudaFrameHeight) return 0;

    const size_t width = static_cast<size_t>(header.width);
    const size_t height = static_cast<size_t>(header.height);
    if (width > (std::numeric_limits<size_t>::max() / height)) return 0;
    const size_t pixels = width * height;

    size_t bytes = 0;
    if (header.format == senseAD::ImageFormat_t::BGR) {
        if (pixels > (std::numeric_limits<size_t>::max() / 3U)) return 0;
        bytes = pixels * 3U;
    } else if (header.format == senseAD::ImageFormat_t::NV12) {
        if (pixels > (std::numeric_limits<size_t>::max() / 3U)) return 0;
        bytes = pixels * 3U / 2U;
    } else {
        return 0;
    }

    return bytes <= kMaxCudaFrameBytes ? bytes : 0;
}

}  // namespace

CameraInputMode parseCameraInputMode(const std::string& mode) {
    const auto m = normalizeMode(mode);
    if (m == "cuda_frame" || m == "cuda" || m == "memblock" || m == "raw_cuda") {
        return CameraInputMode::CUDA_FRAME;
    }
    return CameraInputMode::ENCODED_HEVC;
}

const char* cameraInputModeName(CameraInputMode mode) {
    switch (mode) {
    case CameraInputMode::CUDA_FRAME: return "cuda_frame";
    case CameraInputMode::ENCODED_HEVC:
    default: return "encoded_hevc";
    }
}

CameraSubscriber::CameraSubscriber(std::shared_ptr<ImuSubscriber> imu,
                                   int width, int height, int gpu_id,
                                   CameraInputMode input_mode,
                                   HevcDecoderBackend decoder_backend,
                                   CameraDecodeDebugConfig debug_config)
    : imu_(std::move(imu))
    , input_mode_(input_mode)
    , debug_config_(std::move(debug_config))
{
    if (input_mode_ == CameraInputMode::ENCODED_HEVC) {
        decoder_ = createHevcDecoder(decoder_backend);
        HevcDecoderConfig decoder_config;
        decoder_config.width = width;
        decoder_config.height = height;
        decoder_config.gpu_id = gpu_id;
        decoder_config.verbose = true;
        if (!decoder_->init(decoder_config)) {
            if (decoder_backend == HevcDecoderBackend::Auto) {
                std::cerr << "[CameraSubscriber] NvMedia decoder unavailable in auto mode, "
                          << "falling back to ffmpeg_nvdec." << std::endl;
                decoder_ = createHevcDecoder(HevcDecoderBackend::FfmpegNvdec);
                if (!decoder_->init(decoder_config)) {
                    std::cerr << "[CameraSubscriber] HEVC decoder init failed backend=ffmpeg_nvdec, "
                              << "frames will fail until configuration is fixed." << std::endl;
                }
            } else {
                std::cerr << "[CameraSubscriber] HEVC decoder init failed backend="
                          << hevcDecoderBackendName(decoder_backend)
                          << ", frames will fail until configuration is fixed." << std::endl;
            }
        } else {
            std::cout << "[CameraSubscriber] HEVC decoder backend="
                      << decoder_->backendName() << std::endl;
        }
    } else {
        std::cout << "[CameraSubscriber] CUDA raw frame mode, NVDEC disabled." << std::endl;
    }
    if (debug_config_.enabled) {
        std::cout << "[CameraSubscriber][DecodeDebug] enabled output_dir="
                  << debug_config_.output_dir
                  << " max_frames_per_camera=" << debug_config_.max_frames_per_camera
                  << " save_hevc=" << (debug_config_.save_encoded_hevc ? 1 : 0)
                  << " save_bgr=" << (debug_config_.save_decoded_bgr ? 1 : 0)
                  << std::endl;
    }
    worker_ = std::thread(&CameraSubscriber::workerLoop, this);
}

CameraSubscriber::~CameraSubscriber() {
    {
        std::lock_guard<std::mutex> lk(queue_mutex_);
        running_ = false;
    }
    queue_cv_.notify_all();
    if (worker_.joinable()) worker_.join();
}

void CameraSubscriber::subscribe(const std::string& topic, const std::string& camera_name) {
    camera_name_ = camera_name;
    if (input_mode_ == CameraInputMode::CUDA_FRAME) {
        RsclAccess::Instance()->CudaSubscriber(topic, &CameraSubscriber::onCudaFrame, this);
    } else {
        RsclAccess::Instance()->subscribe(topic, &CameraSubscriber::onCamera, this);
    }
    std::cout << "[CameraSubscriber] Subscribed to " << topic
              << " camera=" << (camera_name_.empty() ? "(default)" : camera_name_)
              << " mode=" << cameraInputModeName(input_mode_) << std::endl;
}

// ---- 回调（尽量快速返回：只做必要拷贝/入队） ----

void CameraSubscriber::onCamera(
    const std::shared_ptr<ReceivedMsg<CameraEncodedMsg>>& msg)
{
    auto mc = msg->Msg();
    const uint64_t seq       = mc.getHeader().getSeq();
    const uint64_t ts        = mc.getHeader().getTime().getNanoSec();
    const uint32_t data_size = mc.getData().size();
    const int      ftype     = static_cast<int>(mc.getFrameType());

    if (data_size == 0) return;

    const auto now = std::chrono::steady_clock::now();
    const uint64_t encoded_idx = ++encoded_count_;
    const int64_t delta_ms = last_ts_ > 0 && ts > last_ts_
        ? static_cast<int64_t>((ts - last_ts_) / 1000000ULL)
        : -1;
    const bool periodic_diag = encoded_idx <= 5 || encoded_idx % kDiagEveryEncodedFrames == 0 ||
        last_encoded_diag_log_.time_since_epoch().count() == 0 ||
        std::chrono::duration_cast<std::chrono::seconds>(now - last_encoded_diag_log_).count() >= 10;
    if (periodic_diag || ftype == 2) {
        size_t queue_size = 0;
        {
            std::lock_guard<std::mutex> lk(queue_mutex_);
            queue_size = queue_.size();
        }
        std::cout << "[CameraSubscriber][Encoded] camera=" << (camera_name_.empty() ? "camera" : camera_name_)
                  << " count=" << encoded_idx
                  << " seq=" << seq
                  << " ts=" << ts
                  << " delta_ms=" << delta_ms
                  << " frame_type=" << ftype
                  << " key=" << (ftype == 2 ? 1 : 0)
                  << " bytes=" << data_size
                  << " has_keyframe=" << (has_keyframe_ ? 1 : 0)
                  << " gop_count=" << gop_frame_count_
                  << " queue=" << queue_size
                  << " queue_drop=" << queue_drop_count_
                  << " wait_keyframe_drop=" << wait_keyframe_drop_count_
                  << " seq_gap=" << seq_gap_count_
                  << std::endl;
        last_encoded_diag_log_ = now;
    }

    ImuData imu_snap = imu_ ? imu_->latest() : ImuData{};

    if (ftype == 2) {  // I 帧
        if (!has_keyframe_ || hevc_resync_required_.load()) {
            std::cout << "[CameraSubscriber][DecoderWarmup] camera=" << (camera_name_.empty() ? "camera" : camera_name_)
                      << (has_keyframe_ ? " resync_keyframe" : " first_keyframe")
                      << " seq=" << seq
                      << " ts=" << ts
                      << " wait_keyframe_drop=" << wait_keyframe_drop_count_
                      << " queue_drop=" << queue_drop_count_
                      << std::endl;
        }
        has_keyframe_  = true;
        hevc_resync_required_.store(false);
        last_seq_      = seq;
        last_ts_       = ts;
        gop_frame_count_ = 0;
        submitTask(seq, ts, ftype, true, mc.getData().begin(), data_size, imu_snap);
        return;
    }

    if (!has_keyframe_ || hevc_resync_required_.load()) {
        ++wait_keyframe_drop_count_;
        if (wait_keyframe_drop_count_ <= 10 || wait_keyframe_drop_count_ % 100 == 0) {
            std::cout << "[CameraSubscriber][DecoderWarmup] camera=" << (camera_name_.empty() ? "camera" : camera_name_)
                      << (has_keyframe_ ? " drop_non_key_after_queue_drop" : " drop_non_key_before_first_I")
                      << " seq=" << seq
                      << " frame_type=" << ftype
                      << " ts=" << ts
                      << " drops=" << wait_keyframe_drop_count_
                      << std::endl;
        }
        last_ts_ = ts;
        return;
    }

    const uint64_t gap = seq >= last_seq_ ? seq - last_seq_ : 0;
    const uint64_t prev_seq = last_seq_;
    last_seq_ = seq;
    last_ts_ = ts;
    if (seq < prev_seq || gap > 10) {
        ++seq_gap_count_;
        std::cout << "[CameraSubscriber][EncodedGap] camera=" << (camera_name_.empty() ? "camera" : camera_name_)
                  << " prev_seq=" << prev_seq
                  << " seq=" << seq
                  << " gap=" << gap
                  << " reset_decoder_wait_I=1"
                  << " total_seq_gap=" << seq_gap_count_
                  << std::endl;
        has_keyframe_    = false;
        hevc_resync_required_.store(true);
        gop_frame_count_ = 0;
        return;
    }

    gop_frame_count_++;
    submitTask(seq, ts, ftype, false, mc.getData().begin(), data_size, imu_snap);
}

void CameraSubscriber::onCudaFrame(const MemblockPtr& msg) {
    if (!msg || !msg->ptr) {
        fail_count_++;
        std::cerr << "[CameraSubscriber] Empty CUDA Memblock." << std::endl;
        return;
    }
    if (msg->size < sizeof(senseAD::ImageHeader_t)) {
        fail_count_++;
        std::cerr << "[CameraSubscriber] CUDA Memblock too small for ImageHeader_t: "
                  << msg->size << std::endl;
        return;
    }

    senseAD::ImageHeader_t header{};
    cudaError_t err = cudaMemcpy(&header, msg->ptr, sizeof(header), cudaMemcpyDefault);
    if (err != cudaSuccess) {
        fail_count_++;
        std::cerr << "[CameraSubscriber] Failed to copy CUDA image header: "
                  << cudaGetErrorString(err) << std::endl;
        return;
    }

    const size_t image_bytes = expectedImageBytes(header);
    const size_t payload_bytes = msg->size - sizeof(senseAD::ImageHeader_t);
    if (image_bytes == 0 || payload_bytes < image_bytes) {
        fail_count_++;
        std::cerr << "[CameraSubscriber] Unsupported/incomplete CUDA image: "
                  << header.width << "x" << header.height
                  << " format=" << static_cast<int>(header.format)
                  << " expected=" << image_bytes
                  << " payload=" << payload_bytes << std::endl;
        return;
    }

    std::vector<uint8_t> host(image_bytes);
    void* device_image = static_cast<uint8_t*>(msg->ptr) + sizeof(senseAD::ImageHeader_t);
    err = cudaMemcpy(host.data(), device_image, image_bytes, cudaMemcpyDefault);
    if (err != cudaSuccess) {
        fail_count_++;
        std::cerr << "[CameraSubscriber] Failed to copy CUDA image payload: "
                  << cudaGetErrorString(err) << std::endl;
        return;
    }

    cv::Mat bgr;
    if (header.format == senseAD::ImageFormat_t::BGR) {
        cv::Mat wrapped(static_cast<int>(header.height), static_cast<int>(header.width), CV_8UC3, host.data());
        bgr = wrapped.clone();
    } else if (header.format == senseAD::ImageFormat_t::NV12) {
        cv::Mat nv12(static_cast<int>(header.height * 3 / 2), static_cast<int>(header.width), CV_8UC1, host.data());
        cv::cvtColor(nv12, bgr, cv::COLOR_YUV2BGR_NV12);
    }

    if (bgr.empty()) {
        fail_count_++;
        std::cerr << "[CameraSubscriber] CUDA frame converted to empty BGR image." << std::endl;
        return;
    }

    ImuData imu_snap = imu_ ? imu_->latest() : ImuData{};
    submitDecodedTask(++cuda_seq_, header.timestamp, std::move(bgr), imu_snap);
}

void CameraSubscriber::submitTask(uint64_t seq, uint64_t ts, int ftype, bool is_key,
                                  const uint8_t* data, size_t size, const ImuData& imu)
{
    std::lock_guard<std::mutex> lk(queue_mutex_);
    if (queue_.size() >= kMaxQueueSize) {
        // 任意编码帧入队丢弃都会打断 HEVC 参考链；清空队列并强制等到下一 I 帧，
        // 避免继续解残缺 GOP 产出马赛克花屏。
        const auto dropped = queue_.front();
        const size_t flushed = queue_.size();
        while (!queue_.empty()) queue_.pop();
        queue_drop_count_ += flushed;
        hevc_resync_required_.store(true);
        std::cout << "[CameraSubscriber][DecodeQueueDrop] camera=" << (camera_name_.empty() ? "camera" : camera_name_)
                  << " drop_seq=" << dropped.seq
                  << " drop_ts=" << dropped.timestamp_ns
                  << " drop_frame_type=" << dropped.frame_type
                  << " flushed=" << flushed
                  << " new_seq=" << seq
                  << " new_frame_type=" << ftype
                  << " queue_capacity=" << kMaxQueueSize
                  << " total_queue_drop=" << queue_drop_count_
                  << " risk=decoder_input_drop_may_break_hevc_refs"
                  << " action=flush_wait_next_I"
                  << std::endl;
        if (!is_key) {
            return;
        }
    }
    if (hevc_resync_required_.load() && !is_key) {
        return;
    }
    if (is_key) {
        hevc_resync_required_.store(false);
    }
    DecodeTask task;
    task.seq          = seq;
    task.timestamp_ns = ts;
    task.frame_type   = ftype;
    task.is_keyframe  = is_key;
    task.imu          = imu;
    task.encoded_data.assign(data, data + size);
    queue_.push(std::move(task));
    queue_cv_.notify_one();
}

void CameraSubscriber::submitDecodedTask(uint64_t seq, uint64_t ts, cv::Mat bgr, const ImuData& imu)
{
    std::lock_guard<std::mutex> lk(queue_mutex_);
    if (queue_.size() >= kMaxQueueSize) {
        const auto dropped = queue_.front();
        queue_.pop();  // 丢最旧帧，保实时性
        ++queue_drop_count_;
        std::cout << "[CameraSubscriber][DecodeQueueDrop] camera=" << (camera_name_.empty() ? "camera" : camera_name_)
                  << " drop_seq=" << dropped.seq
                  << " drop_ts=" << dropped.timestamp_ns
                  << " new_seq=" << seq
                  << " queue_capacity=" << kMaxQueueSize
                  << " total_queue_drop=" << queue_drop_count_
                  << std::endl;
    }
    DecodeTask task;
    task.seq             = seq;
    task.timestamp_ns    = ts;
    task.frame_type      = 0;
    task.is_keyframe     = true;
    task.already_decoded = true;
    task.imu             = imu;
    task.decoded_bgr     = std::move(bgr);
    queue_.push(std::move(task));
    queue_cv_.notify_one();
}

// ---- 解码/转码工作线程 ----

void CameraSubscriber::workerLoop() {
    while (true) {
        DecodeTask task;
        {
            std::unique_lock<std::mutex> lk(queue_mutex_);
            queue_cv_.wait(lk, [this] { return !queue_.empty() || !running_; });
            if (!running_ && queue_.empty()) break;
            task = std::move(queue_.front());
            queue_.pop();
        }
        processTask(task);
    }
    std::cout << "[CameraSubscriber] Worker exiting." << std::endl;
}

void CameraSubscriber::processTask(const DecodeTask& task) {
    auto t0 = std::chrono::high_resolution_clock::now();

    cv::Mat image;
    HevcPacketMetadata emitted_metadata{task.seq, task.timestamp_ns, task.frame_type};
    bool has_emitted_metadata = task.already_decoded;
    bool ok = false;
    if (task.already_decoded) {
        image = task.decoded_bgr;
        ok = !image.empty();
    } else {
        HevcDecodedOutput decoded;
        ok = decoder_ && decoder_->isInitialized() &&
             decoder_->decodeWithMetadata(
                 task.encoded_data,
                 HevcPacketMetadata{task.seq, task.timestamp_ns, task.frame_type},
                 decoded);
        if (ok) {
            image = std::move(decoded.image);
            emitted_metadata = decoded.metadata;
            has_emitted_metadata = decoded.has_metadata;
        }
    }

    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                  std::chrono::high_resolution_clock::now() - t0).count();

    if (ok && !image.empty()) {
        DecodeTask emitted_task = task;
        if (has_emitted_metadata) {
            emitted_task.seq = emitted_metadata.seq;
            emitted_task.timestamp_ns = emitted_metadata.timestamp_ns;
            emitted_task.frame_type = emitted_metadata.frame_type;
        }
        saveDecodeDebugArtifacts(emitted_task, image);

        success_count_++;
        if (ms >= kSlowDecodeWarnMs) {
            ++slow_decode_log_count_;
            std::cout << "[CameraSubscriber][SlowDecode] camera=" << (camera_name_.empty() ? "camera" : camera_name_)
                      << " seq=" << emitted_task.seq
                      << " ts=" << emitted_task.timestamp_ns
                      << " frame_type=" << emitted_task.frame_type
                      << " key=" << (task.is_keyframe ? 1 : 0)
                      << " decode_ms=" << ms
                      << " slow_count=" << slow_decode_log_count_
                      << " risk=camera_sync_timeout_or_stale"
                      << std::endl;
        }
        std::cout << "[Camera] camera=" << (camera_name_.empty() ? "camera" : camera_name_)
                  << " seq=" << emitted_task.seq
                  << " ts=" << emitted_task.timestamp_ns;
        if (input_mode_ == CameraInputMode::ENCODED_HEVC) {
            std::cout << (emitted_task.frame_type == 2 ? " [I]" : " [P]")
                      << " decode=" << ms << "ms";
        } else {
            std::cout << " [CUDA_RAW] convert=" << ms << "ms";
        }
        std::cout << " imu_alert=" << task.imu.vibration_alert << std::endl;

        if (frame_cb_) {
            DecodedFrame frame;
            frame.seq          = emitted_task.seq;
            frame.timestamp_ns = emitted_task.timestamp_ns;
            frame.frame_type   = emitted_task.frame_type;
            frame.image        = std::move(image);
            frame.imu          = task.imu;
            frame.camera_name  = camera_name_;
            frame_cb_(frame);
        }
    } else {
        fail_count_++;
        ++decode_fail_log_count_;
        std::cerr << "[CameraSubscriber][DecodeFail] camera=" << (camera_name_.empty() ? "camera" : camera_name_)
                  << " seq=" << task.seq
                  << " ts=" << task.timestamp_ns
                  << " frame_type=" << task.frame_type
                  << " key=" << (task.is_keyframe ? 1 : 0)
                  << " decode_ms=" << ms
                  << " fail_count=" << fail_count_.load()
                  << " log_count=" << decode_fail_log_count_
                  << " hint=check_for_HEVC_ref_POC_or_input_drop"
                  << std::endl;
    }
}

void CameraSubscriber::saveDecodeDebugArtifacts(const DecodeTask& task, const cv::Mat& decoded) {
    if (!debug_config_.enabled || task.already_decoded) {
        return;
    }
    if (debug_config_.max_frames_per_camera <= 0 ||
        debug_saved_count_ >= static_cast<uint64_t>(debug_config_.max_frames_per_camera)) {
        return;
    }
    if (!debug_config_.save_encoded_hevc && !debug_config_.save_decoded_bgr) {
        return;
    }

    const uint64_t index = ++debug_saved_count_;
    const std::string cam = safeFileToken(camera_name_);
    const std::filesystem::path cam_dir = std::filesystem::path(debug_config_.output_dir) / cam;
    std::error_code ec;
    std::filesystem::create_directories(cam_dir, ec);
    if (ec) {
        std::cerr << "[CameraSubscriber][DecodeDebug] create_dir_failed dir="
                  << cam_dir.string() << " error=" << ec.message() << std::endl;
        return;
    }

    std::ostringstream prefix;
    prefix << std::setw(3) << std::setfill('0') << index
           << "_seq_" << task.seq
           << "_ts_" << task.timestamp_ns
           << "_" << frameTypeToken(task.frame_type);

    if (debug_config_.save_encoded_hevc && !task.encoded_data.empty()) {
        const auto path = cam_dir / (prefix.str() + ".h265");
        std::ofstream out(path, std::ios::binary);
        if (out.is_open()) {
            out.write(reinterpret_cast<const char*>(task.encoded_data.data()),
                      static_cast<std::streamsize>(task.encoded_data.size()));
        } else {
            std::cerr << "[CameraSubscriber][DecodeDebug] write_hevc_failed path="
                      << path.string() << std::endl;
        }
    }

    if (debug_config_.save_decoded_bgr && !decoded.empty()) {
        const auto path = cam_dir / (prefix.str() + "_decoded_bgr.png");
        if (!cv::imwrite(path.string(), decoded)) {
            std::cerr << "[CameraSubscriber][DecodeDebug] write_bgr_failed path="
                      << path.string() << std::endl;
        }
    }

    std::cout << "[CameraSubscriber][DecodeDebug] saved camera=" << cam
              << " index=" << index
              << " seq=" << task.seq
              << " ts=" << task.timestamp_ns
              << " frame_type=" << task.frame_type
              << " bytes=" << task.encoded_data.size()
              << " decoded=" << decoded.cols << "x" << decoded.rows
              << " dir=" << cam_dir.string()
              << std::endl;
}

}  // namespace sentinel
