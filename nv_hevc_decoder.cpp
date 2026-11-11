#include "nv_hevc_decoder.h"
#include <opencv2/opencv.hpp>
#include <chrono>
#include <iostream>
#include <cstdio>
#include <inttypes.h>  // for PRId64

// FFmpeg 头文件
extern "C" {
#include <libavformat/avformat.h>
#include <libavcodec/avcodec.h>
#include <libavutil/hwcontext.h>
#include <libavutil/opt.h>
#include <libavutil/imgutils.h>
#include <libswscale/swscale.h>
}

#define LOGI(fmt, ...) do { \
    std::printf("[NVDEC] " fmt "\n", ##__VA_ARGS__); \
} while(0)
#define LOGW(fmt, ...) do { \
    std::fprintf(stderr, "[NVDEC WARN] " fmt "\n", ##__VA_ARGS__); \
} while(0)
#define LOGE(fmt, ...) do { \
    std::fprintf(stderr, "[NVDEC ERROR] " fmt "\n", ##__VA_ARGS__); \
} while(0)

NVHevcDecoder::NVHevcDecoder(bool force_software)
    : fmt_ctx_(nullptr)
    , codec_ctx_(nullptr)
    , hw_device_ctx_(nullptr)
    , bsf_ctx_(nullptr)
    , frame_(nullptr)
    , sw_frame_(nullptr)
    , pkt_(nullptr)
    , sws_ctx_(nullptr)
    , sws_src_width_(0)
    , sws_src_height_(0)
    , sws_src_format_(0)
    , initialized_(false)
    , hw_available_(false)
    , width_(0)
    , height_(0)
    , gpu_id_(0)
    , last_frame_index_(-1)
    , current_frame_index_(0)
    , verbose_(false)
    , force_software_(force_software)
    , total_frames_decoded_(0)
    , total_decode_time_ms_(0)
{
}

NVHevcDecoder::~NVHevcDecoder() {
    reset();
}

bool NVHevcDecoder::init(int width, int height, int gpu_id, bool verbose) {
    std::lock_guard<std::mutex> lock(mutex_);

    if (initialized_) {
        if (verbose) LOGI("Already initialized, resetting...");
        reset();
    }

    width_ = width;
    height_ = height;
    gpu_id_ = gpu_id;
    verbose_ = verbose;
    current_frame_index_ = 0;
    total_frames_decoded_ = 0;
    total_decode_time_ms_ = 0;
    next_packet_token_ = 1;
    pending_metadata_.clear();
    ready_frames_.clear();

    // 初始化硬件设备上下文。CUDA/NVDEC 不可用时不再失败，直接降级到 CPU HEVC 解码。
    hw_available_ = !force_software_ && initHWDeviceCtx(gpu_id);
    if (!hw_available_) {
        if (verbose_) {
            LOGI("CUDA/NVDEC unavailable, falling back to software HEVC decoder");
        }
    }

    // 初始化解码器
    if (!initCodecContext()) {
        LOGE("Failed to initialize codec context");
        return false;
    }

    // 初始化比特流过滤器 (HEVC -> annexb)
    if (!initBSF()) {
        LOGE("Failed to initialize BSF");
        return false;
    }

    // 分配帧和包
    frame_ = av_frame_alloc();
    sw_frame_ = av_frame_alloc();
    pkt_ = av_packet_alloc();

    if (!frame_ || !sw_frame_ || !pkt_) {
        LOGE("Failed to allocate frame/packet");
        return false;
    }

    initialized_ = true;
    if (verbose_) {
        LOGI("Initialized %s HEVC decoder (GPU=%d, %dx%d)",
             hw_available_ ? "NVDEC" : "software",
             gpu_id,
             width_ > 0 ? width_ : 3840, height_ > 0 ? height_ : 2160);
    }
    return true;
}

bool NVHevcDecoder::initHWDeviceCtx(int gpu_id) {
    AVBufferRef* hw_ctx = nullptr;
    char gpu_device_str[32];
    std::snprintf(gpu_device_str, sizeof(gpu_device_str), "cuda:%d", gpu_id);

    int ret = av_hwdevice_ctx_create(&hw_ctx, AV_HWDEVICE_TYPE_CUDA,
                                     gpu_device_str, nullptr, 0);
    if (ret < 0) {
        char err_buf[128];
        av_strerror(ret, err_buf, sizeof(err_buf));
        LOGW("Failed to create CUDA device: %s", err_buf);
        return false;
    }
    hw_device_ctx_ = hw_ctx;
    return true;
}

bool NVHevcDecoder::initCodecContext() {
    // 查找 HEVC 解码器
    const AVCodec* codec = nullptr;
    bool use_hw = false;

    if (hw_available_) {
        codec = avcodec_find_decoder_by_name("hevc_cuvid");
        use_hw = (codec != nullptr);
        if (!codec && verbose_) {
            LOGI("HEVC CUVID decoder not found, using software decoder");
        }
    }

    if (!codec) {
        codec = avcodec_find_decoder(AV_CODEC_ID_HEVC);
        if (!codec) {
            LOGE("HEVC decoder not found at all");
            return false;
        }
    }

    // 创建解码器上下文
    codec_ctx_ = avcodec_alloc_context3(codec);
    if (!codec_ctx_) {
        LOGE("Failed to allocate codec context");
        return false;
    }

    // 设置硬件设备（仅硬件解码时）
    if (use_hw) {
        codec_ctx_->hw_device_ctx = av_buffer_ref(hw_device_ctx_);
    }

    // 设置解码参数
    if (width_ > 0 && height_ > 0) {
        codec_ctx_->width = width_;
        codec_ctx_->height = height_;
    }

    // 重要：启用纠错模式，但不要完全忽略错误
    codec_ctx_->err_recognition = AV_EF_CRCCHECK;
    codec_ctx_->skip_frame = AVDISCARD_DEFAULT;
    codec_ctx_->skip_loop_filter = AVDISCARD_DEFAULT;

    // CUVID 特定选项
    if (use_hw) {
        av_opt_set(codec_ctx_->priv_data, "deint", "0", 0);  // 不去隔行
        av_opt_set(codec_ctx_->priv_data, "drop_second_field", "1", 0);
        // 尝试设置参考帧数量
        av_opt_set(codec_ctx_->priv_data, "refcheck_delay", "0", 0);
    }

    // 打开解码器
    int ret = avcodec_open2(codec_ctx_, codec, nullptr);
    if (ret < 0) {
        char err_buf[128];
        av_strerror(ret, err_buf, sizeof(err_buf));

        // 硬件解码失败，尝试降级到软件解码
        if (use_hw) {
            LOGI("Hardware decoder failed (%s), falling back to software", err_buf);
            avcodec_free_context(&codec_ctx_);

            codec = avcodec_find_decoder(AV_CODEC_ID_HEVC);
            if (!codec) {
                LOGE("Software HEVC decoder not found");
                return false;
            }

            codec_ctx_ = avcodec_alloc_context3(codec);
            if (!codec_ctx_) {
                LOGE("Failed to allocate software codec context");
                return false;
            }

            if (width_ > 0 && height_ > 0) {
                codec_ctx_->width = width_;
                codec_ctx_->height = height_;
            }

            codec_ctx_->err_recognition = AV_EF_CRCCHECK;

            ret = avcodec_open2(codec_ctx_, codec, nullptr);
            if (ret < 0) {
                av_strerror(ret, err_buf, sizeof(err_buf));
                LOGE("Software decoder also failed: %s", err_buf);
                return false;
            }

            if (verbose_) LOGI("Using software HEVC decoder");
        } else {
            LOGE("Failed to open codec: %s", err_buf);
            return false;
        }
    } else {
        if (verbose_) LOGI("Opened %s decoder successfully", codec->name);
    }

    return true;
}

bool NVHevcDecoder::initBSF() {
    const AVBitStreamFilter* bsf = av_bsf_get_by_name("hevc_mp4toannexb");
    if (!bsf) {
        if (verbose_) LOGI("HEVC MP4toAnnexB BSF not found, skipping...");
        return true;  // 不是致命错误
    }

    AVBSFContext* ctx = nullptr;
    int ret = av_bsf_alloc(bsf, &ctx);
    if (ret < 0) {
        LOGE("Failed to allocate BSF");
        return false;
    }

    avcodec_parameters_from_context(ctx->par_in, codec_ctx_);

    ret = av_bsf_init(ctx);
    if (ret < 0) {
        av_bsf_free(&ctx);
        LOGE("Failed to init BSF");
        return false;
    }

    bsf_ctx_ = ctx;
    return true;
}

bool NVHevcDecoder::decode(const uint8_t* data, size_t size,
                           cv::Mat& output_frame, int target_frame_index) {
    HevcDecodedOutput output;
    const HevcPacketMetadata metadata{};
    const bool ok = decodeWithMetadata(data, size, metadata, output,
                                       target_frame_index);
    if (ok) output_frame = std::move(output.image);
    return ok;
}

bool NVHevcDecoder::decodeWithMetadata(const uint8_t* data, size_t size,
                                       const HevcPacketMetadata& input_metadata,
                                       HevcDecodedOutput& output,
                                       int target_frame_index) {
    if (!initialized_) {
        LOGE("Decoder not initialized");
        return false;
    }

    output = {};
    const int64_t packet_token = next_packet_token_++;
    pending_metadata_[packet_token] = input_metadata;

    auto start = std::chrono::high_resolution_clock::now();

    // 发送数据到解码器
    if (!sendPacket(data, size, packet_token)) {
        // 即使 sendPacket 失败，也尝试接收帧（错误恢复）
    }

    // 解码器可能一次输出多帧；按显示顺序缓存，每次 API 调用返回一帧。
    drainDecoderFrames(target_frame_index);

    int64_t output_token = AV_NOPTS_VALUE;
    bool success = false;
    if (!ready_frames_.empty()) {
        ReadyFrame ready = std::move(ready_frames_.front());
        ready_frames_.pop_front();
        output.image = std::move(ready.image);
        output_token = ready.token;
        success = true;
    }

    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

    if (success) {
        const auto metadata_it = pending_metadata_.find(output_token);
        if (metadata_it != pending_metadata_.end()) {
            output.metadata = metadata_it->second;
            output.has_metadata = true;
            pending_metadata_.erase(metadata_it);
        } else if (verbose_) {
            LOGW("Decoded frame has unknown token=%" PRId64, output_token);
        }
        total_frames_decoded_++;
        total_decode_time_ms_ += duration.count();
        if (verbose_) {
            LOGI("Decoded frame %d in %" PRId64 " ms (avg: %.1f ms)",
                 last_frame_index_, static_cast<int64_t>(duration.count()),
                 static_cast<float>(total_decode_time_ms_) / total_frames_decoded_);
        }
    } else {
        // 解码失败时打印诊断信息
        if (verbose_) {
            LOGE("Decode failed - codec delay might require more packets");
        }
    }

    return success;
}

bool NVHevcDecoder::decode(const std::vector<uint8_t>& data,
                           cv::Mat& output_frame, int target_frame_index) {
    return decode(data.data(), data.size(), output_frame, target_frame_index);
}

bool NVHevcDecoder::sendPacket(const uint8_t* data, size_t size, int64_t packet_token) {
    // 复制数据到包
    pkt_->data = const_cast<uint8_t*>(data);
    pkt_->size = static_cast<int>(size);
    pkt_->pts = packet_token;
    pkt_->dts = packet_token;

    // 应用 BSF (如果需要)
    if (bsf_ctx_) {
        int ret = av_bsf_send_packet(bsf_ctx_, pkt_);
        if (ret < 0 && ret != AVERROR(EAGAIN)) {
            char err_buf[128];
            av_strerror(ret, err_buf, sizeof(err_buf));
            if (verbose_) LOGE("BSF send failed: %s", err_buf);
            return false;
        }

        ret = av_bsf_receive_packet(bsf_ctx_, pkt_);
        if (ret < 0 && ret != AVERROR(EAGAIN)) {
            char err_buf[128];
            av_strerror(ret, err_buf, sizeof(err_buf));
            if (verbose_) LOGE("BSF receive failed: %s", err_buf);
            return false;
        }
    }

    // 发送到解码器
    int ret = avcodec_send_packet(codec_ctx_, pkt_);
    if (ret == AVERROR(EAGAIN)) {
        // EAGAIN means this packet was *not* consumed.  Drain the decoder and
        // resend this exact AVPacket; treating EAGAIN as success loses an
        // access unit and eventually desynchronizes its metadata token.
        const size_t drained = drainDecoderFrames(-1);
        if (verbose_) {
            LOGI("Decoder send EAGAIN: drained %zu frame(s), retrying packet", drained);
        }
        ret = avcodec_send_packet(codec_ctx_, pkt_);
    }

    if (ret == 0) {
        if (verbose_) LOGI("Packet sent successfully, size=%zu", size);
        return true;
    }

    char err_buf[128];
    av_strerror(ret, err_buf, sizeof(err_buf));
    LOGE("Failed to send packet: %s", err_buf);
    return false;
}

size_t NVHevcDecoder::drainDecoderFrames(int target_frame_index) {
    size_t drained = 0;
    while (true) {
        cv::Mat image;
        int64_t token = AV_NOPTS_VALUE;
        if (!receiveFrame(image, target_frame_index, &token)) break;
        ready_frames_.push_back({std::move(image), token});
        ++drained;
    }
    return drained;
}

bool NVHevcDecoder::receiveFrame(cv::Mat& output_frame, int target_frame_index,
                                 int64_t* output_token) {
    int frames_received = 0;

    while (true) {
        int ret = avcodec_receive_frame(codec_ctx_, frame_);

        if (ret == AVERROR(EAGAIN)) {
            if (verbose_ && frames_received == 0) {
                LOGI("No frame ready yet (EAGAIN), need more packets");
            }
            break;
        }

        if (ret == AVERROR_EOF) {
            if (verbose_) LOGI("Decoder EOF reached");
            break;
        }

        if (ret < 0) {
            char err_buf[128];
            av_strerror(ret, err_buf, sizeof(err_buf));
            LOGE("Failed to receive frame: %s", err_buf);
            break;
        }

        frames_received++;
        current_frame_index_++;

        if (verbose_) {
            LOGI("Received frame #%d, format=%d (%s), key_frame=%d",
                 current_frame_index_, frame_->format,
                 (frame_->format == AV_PIX_FMT_CUDA ? "CUDA" :
                  frame_->format == AV_PIX_FMT_NV12 ? "NV12" :
                  frame_->format == AV_PIX_FMT_YUV420P ? "YUV420P" : "OTHER"),
                 frame_->key_frame);
        }

        // 检查是否是目标帧
        bool is_target = (target_frame_index < 0) ||  // -1 表示任意帧
                         (current_frame_index_ == target_frame_index) ||
                         (target_frame_index == 0);   // 0 表示第一帧

        if (is_target) {
            if (frame_->format == AV_PIX_FMT_CUDA) {
                // GPU 帧，需要传输到 CPU 后转换
                if (gpuMatToBGR(frame_, output_frame)) {
                    last_frame_index_ = current_frame_index_;
                    if (output_token) *output_token = frame_->pts;
                    return true;
                } else {
                    if (verbose_) LOGE("GPU to BGR conversion failed");
                    // 继续尝试下一帧
                }
            } else {
                // CPU 帧（软件解码），直接使用 sws_scale 转换
                SwsContext* sws_ctx = sws_getContext(
                    frame_->width, frame_->height,
                    static_cast<AVPixelFormat>(frame_->format),
                    frame_->width, frame_->height, AV_PIX_FMT_BGR24,
                    SWS_BILINEAR | SWS_FULL_CHR_H_INP, nullptr, nullptr, nullptr);

                if (sws_ctx) {
                    output_frame.create(frame_->height, frame_->width, CV_8UC3);

                    uint8_t* dst_data[1] = { output_frame.data };
                    int dst_linesize[1] = { static_cast<int>(output_frame.step) };

                    sws_scale(sws_ctx, frame_->data, frame_->linesize,
                             0, frame_->height, dst_data, dst_linesize);

                    sws_freeContext(sws_ctx);

                    last_frame_index_ = current_frame_index_;
                    if (output_token) *output_token = frame_->pts;
                    return true;
                }
            }
        }

        // 继续接收下一帧（如果有）
    }

    if (verbose_ && frames_received == 0) {
        LOGI("No frames received from this packet");
    }

    return false;
}

bool NVHevcDecoder::gpuMatToBGR(AVFrame* hw_frame, cv::Mat& output) {
    if (!hw_frame || hw_frame->format != AV_PIX_FMT_CUDA) {
        return false;
    }

    // 确保软件帧已分配并关联到硬件帧上下文
    if (sw_frame_->width != hw_frame->width ||
        sw_frame_->height != hw_frame->height) {

        av_frame_unref(sw_frame_);
        sw_frame_->width = hw_frame->width;
        sw_frame_->height = hw_frame->height;
        sw_frame_->format = AV_PIX_FMT_NONE;  // 将由 transfer_data 设置
    }

    // 从 GPU 传输到 CPU
    int ret = av_hwframe_transfer_data(sw_frame_, hw_frame, 0);
    if (ret < 0) {
        char err_buf[128];
        av_strerror(ret, err_buf, sizeof(err_buf));
        LOGE("Failed to transfer frame: %s", err_buf);
        return false;
    }

    // 获取实际的软件格式
    AVHWFramesContext* hw_frames_ctx =
        (AVHWFramesContext*)hw_frame->hw_frames_ctx->data;
    int actual_format = sw_frame_->format;
    if (actual_format == AV_PIX_FMT_NONE || actual_format == 0) {
        actual_format = hw_frames_ctx->sw_format;
    }

    // 调试信息：打印帧格式
    if (verbose_) {
        LOGI("Frame: %dx%d, sw_format=%d (%s), data[0]=%p, data[1]=%p, data[2]=%p, linesize[0]=%d, linesize[1]=%d",
             sw_frame_->width, sw_frame_->height, actual_format,
             (actual_format == 0 ? "NONE" :
              actual_format == 23 ? "NV12" :
              actual_format == 0 ? "YUV420P" : "OTHER"),
             sw_frame_->data[0], sw_frame_->data[1], sw_frame_->data[2],
             sw_frame_->linesize[0], sw_frame_->linesize[1]);
    }

    // 更新宽高
    if (width_ == 0 || height_ == 0) {
        width_ = sw_frame_->width;
        height_ = sw_frame_->height;
    }

    // 检查是否需要重新创建 SwsContext
    if (!sws_ctx_ ||
        sws_src_width_ != sw_frame_->width ||
        sws_src_height_ != sw_frame_->height ||
        sws_src_format_ != actual_format) {

        if (sws_ctx_) {
            sws_freeContext(sws_ctx_);
            sws_ctx_ = nullptr;
        }

        // 对于 NV12 格式（2平面），使用特定的转换参数
        // SWS_FULL_CHR_H_INT 确保色度水平插值正确
        int sws_flags = SWS_BILINEAR | SWS_FULL_CHR_H_INP;
        if (actual_format == AV_PIX_FMT_NV12) {
            // NV12 使用双线性插值和完整色度处理
            sws_flags = SWS_BILINEAR | SWS_FULL_CHR_H_INP;
        }

        sws_ctx_ = sws_getContext(
            sw_frame_->width, sw_frame_->height,
            static_cast<AVPixelFormat>(actual_format),
            sw_frame_->width, sw_frame_->height, AV_PIX_FMT_BGR24,
            sws_flags, nullptr, nullptr, nullptr);

        if (!sws_ctx_) {
            LOGE("Failed to create SWS context: fmt=%d (%s)", actual_format,
                 (actual_format == 23 ? "NV12" : "OTHER"));
            return false;
        }

        sws_src_width_ = sw_frame_->width;
        sws_src_height_ = sw_frame_->height;
        sws_src_format_ = actual_format;
    }

    // 创建 BGR 输出缓冲区
    output.create(sw_frame_->height, sw_frame_->width, CV_8UC3);

    // 执行转换：直接写入 cv::Mat 的数据
    uint8_t* dst_data[1] = { output.data };
    int dst_linesize[1] = { static_cast<int>(output.step) };

    sws_scale(sws_ctx_, sw_frame_->data, sw_frame_->linesize,
             0, sw_frame_->height, dst_data, dst_linesize);

    return true;
}

void NVHevcDecoder::flush() {
    std::lock_guard<std::mutex> lock(mutex_);

    if (codec_ctx_) {
        avcodec_flush_buffers(codec_ctx_);
    }
    current_frame_index_ = 0;
    pending_metadata_.clear();
    ready_frames_.clear();
}

void NVHevcDecoder::reset() {
    std::lock_guard<std::mutex> lock(mutex_);

    if (sws_ctx_) {
        sws_freeContext(sws_ctx_);
        sws_ctx_ = nullptr;
    }
    if (frame_) av_frame_free(&frame_);
    if (sw_frame_) av_frame_free(&sw_frame_);
    if (pkt_) av_packet_free(&pkt_);
    if (bsf_ctx_) av_bsf_free(&bsf_ctx_);
    if (codec_ctx_) avcodec_free_context(&codec_ctx_);
    if (hw_device_ctx_) av_buffer_unref(&hw_device_ctx_);
    if (fmt_ctx_) avformat_close_input(&fmt_ctx_);

    initialized_ = false;
    hw_available_ = false;
    current_frame_index_ = 0;
    next_packet_token_ = 1;
    pending_metadata_.clear();
    ready_frames_.clear();
    last_frame_index_ = -1;
    sws_src_width_ = 0;
    sws_src_height_ = 0;
    sws_src_format_ = 0;
}
