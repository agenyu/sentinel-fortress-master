#pragma once

#include "hevc_decoder.h"
#include "nvmedia_decoder_bridge.h"

#include <cstdint>
#include <string>

class NvMediaHevcDecoder : public IHevcDecoder {
public:
    NvMediaHevcDecoder();
    ~NvMediaHevcDecoder() override;

    bool init(const HevcDecoderConfig& config) override;
    bool decode(const uint8_t* data, size_t size,
                cv::Mat& output_frame, int target_frame_index = -1) override;
    bool isInitialized() const override { return initialized_; }
    const char* backendName() const override { return "nvmedia"; }
    void flush() override;
    void reset() override;

private:
    bool loadBridge();
    void unloadBridge();

private:
    std::string bridge_path_;
    void* bridge_handle_ = nullptr;
    SentinelNvMediaDecoder* decoder_ = nullptr;
    sentinel_nvmedia_create_fn create_ = nullptr;
    sentinel_nvmedia_destroy_fn destroy_ = nullptr;
    sentinel_nvmedia_decode_fn decode_ = nullptr;
    sentinel_nvmedia_release_frame_fn release_frame_ = nullptr;
    sentinel_nvmedia_flush_fn flush_ = nullptr;
    bool initialized_ = false;
    uint64_t decode_attempt_count_ = 0;
    uint64_t decode_no_output_count_ = 0;
    uint64_t decode_fail_count_ = 0;
};
