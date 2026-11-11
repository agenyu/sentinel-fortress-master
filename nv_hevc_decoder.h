#pragma once

#include "hevc_decoder.h"

#include <string>
#include <vector>
#include <memory>
#include <cstdint>
#include <deque>
#include <mutex>
#include <unordered_map>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
// FFmpeg 前置声明
struct AVFormatContext;
struct AVCodecContext;
struct AVFrame;
struct AVPacket;
struct AVBufferRef;
struct AVBSFContext;
struct SwsContext;
struct AVHWFramesContext;

/**
 * @brief NVIDIA NVDEC 硬件加速 HEVC 解码器
 *
 * 特性：
 * - 使用 NVIDIA GPU 硬件解码 (NVDEC/cuvid)
 * - 零拷贝：直接在 GPU 内存中解码，无需经过 CPU
 * - 解码速度：10-20ms (相比软件解码的 100-300ms)
 * - 支持 I 帧和 P 帧解码
 * - 自动降级：硬件解码失败时自动切换到软件解码
 * - 线程安全：内部使用 mutex 保护
 *
 * 使用示例：
 *   NVHevcDecoder decoder;
 *   if (decoder.init(3840, 2160, 0)) {  // width, height, gpu_id
 *       std::vector<uint8_t> data = ...;  // HEVC 数据
 *       cv::Mat frame;
 *       if (decoder.decode(data, frame)) {
 *           cv::imshow("Frame", frame);
 *       }
 *   }
 */
class NVHevcDecoder : public IHevcDecoder {
public:
    explicit NVHevcDecoder(bool force_software = false);
    ~NVHevcDecoder();

    // 禁止拷贝
    NVHevcDecoder(const NVHevcDecoder&) = delete;
    NVHevcDecoder& operator=(const NVHevcDecoder&) = delete;

    /**
     * @brief 初始化解码器
     * @param width 视频宽度
     * @param height 视频高度
     * @param gpu_id GPU ID (默认 0)
     * @param verbose 是否输出详细日志 (默认 false)
     * @return 成功返回 true
     */
    bool init(int width = 0, int height = 0, int gpu_id = 0, bool verbose = false);
    bool init(const HevcDecoderConfig& config) override {
        return init(config.width, config.height, config.gpu_id, config.verbose);
    }

    /**
     * @brief 解码 HEVC 数据
     * @param data HEVC 编码数据
     * @param size 数据大小
     * @param output_frame 输出的 BGR 格式图像
     * @param target_frame_index 目标帧索引 (用于 select 滤镜，-1 表示最后一帧)
     * @return 成功返回 true
     */
    bool decode(const uint8_t* data, size_t size,
                cv::Mat& output_frame, int target_frame_index = -1) override;

    /**
     * @brief 解码 HEVC 数据 (vector 版本)
     */
    bool decode(const std::vector<uint8_t>& data,
                cv::Mat& output_frame, int target_frame_index = -1) override;

    bool decodeWithMetadata(const uint8_t* data, size_t size,
                            const HevcPacketMetadata& input_metadata,
                            HevcDecodedOutput& output,
                            int target_frame_index = -1) override;

    /**
     * @brief 获取最后一次解码的帧信息
     */
    int getLastFrameIndex() const { return last_frame_index_; }
    int getWidth() const { return width_; }
    int getHeight() const { return height_; }

    /**
     * @brief 是否已初始化
     */
    bool isInitialized() const override { return initialized_; }
    const char* backendName() const override { return hw_available_ ? "ffmpeg_nvdec" : "ffmpeg_software"; }

    /**
     * @brief 获取解码统计信息
     */
    uint64_t getTotalFramesDecoded() const { return total_frames_decoded_; }
    uint64_t getTotalDecodeTimeMs() const { return total_decode_time_ms_; }
    float getAverageDecodeTimeMs() const {
        return total_frames_decoded_ > 0
            ? static_cast<float>(total_decode_time_ms_) / total_frames_decoded_
            : 0.0f;
    }

    /**
     * @brief 刷新解码器内部缓冲区（用于 I 帧等关键点）
     * 保留解码器上下文，只清空缓冲区
     */
    void flush() override;

    /**
     * @brief 完全重置解码器（销毁并重新初始化）
     */
    void reset() override;

private:
    bool initHWDeviceCtx(int gpu_id);
    bool initCodecContext();
    bool initBSF();
    bool sendPacket(const uint8_t* data, size_t size, int64_t packet_token);
    // Drain every frame currently available from libavcodec.  Frames drained to
    // make room for a resend must be retained: avcodec_send_packet() has not
    // consumed the packet when it returns EAGAIN.
    size_t drainDecoderFrames(int target_frame_index);
    bool receiveFrame(cv::Mat& output_frame, int target_frame_index,
                      int64_t* output_token);
    bool gpuMatToBGR(AVFrame* hw_frame, cv::Mat& output);

private:
    // FFmpeg 内部结构
    AVFormatContext* fmt_ctx_;
    AVCodecContext* codec_ctx_;
    AVBufferRef* hw_device_ctx_;
    AVBSFContext* bsf_ctx_;

    // 帧和包
    AVFrame* frame_;
    AVFrame* sw_frame_;
    AVPacket* pkt_;

    // 颜色转换上下文缓存（提高性能）
    SwsContext* sws_ctx_;
    int sws_src_width_;
    int sws_src_height_;
    int sws_src_format_;

    // 状态
    bool initialized_;
    bool hw_available_;
    int width_;
    int height_;
    int gpu_id_;
    int last_frame_index_;
    int current_frame_index_;
    bool verbose_;  // 是否输出详细日志
    bool force_software_;

    // 统计
    uint64_t total_frames_decoded_;
    uint64_t total_decode_time_ms_;
    int64_t next_packet_token_{1};
    std::unordered_map<int64_t, HevcPacketMetadata> pending_metadata_;

    struct ReadyFrame {
        cv::Mat image;
        int64_t token{0};
    };
    std::deque<ReadyFrame> ready_frames_;

    // 线程安全
    std::mutex mutex_;
};
