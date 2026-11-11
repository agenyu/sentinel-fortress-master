#include "nvmedia_hevc_decoder.h"

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <cstdint>
#include <cstring>
#include <dlfcn.h>
#include <iomanip>
#include <iostream>
#include <string>

namespace {

constexpr const char* kDefaultBridgePath = "libsentinel_nvmedia_decoder.so";

bool shouldLogDecodeFailure(uint64_t count) {
    return count <= 10U || count % 300U == 0U;
}

bool shouldLogDecodeAttempt(uint64_t count) {
    return count <= 10U || count % 300U == 0U;
}

int getenvInt(const char* name, int fallback) {
    const char* value = std::getenv(name);
    if (!value || value[0] == '\0') {
        return fallback;
    }
    try {
        return std::stoi(value);
    } catch (...) {
        return fallback;
    }
}

const char* payloadFormatName(const uint8_t* data, size_t size) {
    if (!data || size < 4U) {
        return "too-small";
    }
    if (data[0] == 0U && data[1] == 0U && data[2] == 1U) {
        return "annexb-3byte";
    }
    if (data[0] == 0U && data[1] == 0U && data[2] == 0U && data[3] == 1U) {
        return "annexb-4byte";
    }
    const uint32_t be_len = (static_cast<uint32_t>(data[0]) << 24U) |
                            (static_cast<uint32_t>(data[1]) << 16U) |
                            (static_cast<uint32_t>(data[2]) << 8U) |
                            static_cast<uint32_t>(data[3]);
    if (be_len > 0U && be_len + 4U <= size) {
        return "length-prefixed-be32";
    }
    return "unknown";
}

void logPayloadPrefix(const uint8_t* data, size_t size) {
    const size_t n = std::min<size_t>(size, 16U);
    std::cerr << "[NvMediaDecoder][Payload] first_bytes=";
    for (size_t i = 0; i < n; ++i) {
        std::cerr << (i == 0 ? "" : " ")
                  << std::hex << std::setw(2) << std::setfill('0')
                  << static_cast<unsigned int>(data[i]);
    }
    std::cerr << std::dec << std::setfill(' ') << std::endl;
}

template <typename Fn>
bool loadSymbol(void* handle, const char* name, Fn* out) {
    *out = reinterpret_cast<Fn>(dlsym(handle, name));
    if (!*out) {
        std::cerr << "[NvMediaDecoder] Missing bridge symbol: " << name << std::endl;
        return false;
    }
    return true;
}

}  // namespace

NvMediaHevcDecoder::NvMediaHevcDecoder() {
    const char* override_path = std::getenv("SENTINEL_NVMEDIA_DECODER_LIB");
    bridge_path_ = (override_path && override_path[0] != '\0')
        ? override_path
        : kDefaultBridgePath;
}

NvMediaHevcDecoder::~NvMediaHevcDecoder() {
    reset();
    unloadBridge();
}

bool NvMediaHevcDecoder::init(const HevcDecoderConfig& config) {
    reset();

    if (!loadBridge()) {
        initialized_ = false;
        return false;
    }

    SentinelNvMediaDecoderConfig bridge_config{};
    bridge_config.width = config.width;
    bridge_config.height = config.height;
    bridge_config.device_id = config.gpu_id;
    bridge_config.verbose = config.verbose ? 1 : 0;

    decoder_ = create_(&bridge_config);
    if (!decoder_) {
        std::cerr << "[NvMediaDecoder] Bridge failed to create decoder" << std::endl;
        initialized_ = false;
        return false;
    }

    initialized_ = true;
    if (config.verbose) {
        std::cout << "[NvMediaDecoder] Initialized via bridge: " << bridge_path_
                  << " (" << config.width << "x" << config.height << ")"
                  << std::endl;
    }
    return true;
}

bool NvMediaHevcDecoder::decode(const uint8_t* data, size_t size,
                                cv::Mat& output_frame, int target_frame_index) {
    (void)target_frame_index;
    if (!initialized_ || !decoder_ || !decode_) {
        return false;
    }
    if (!data || size == 0) {
        return false;
    }

    SentinelNvMediaBgrFrame bridge_frame{};
    const uint64_t attempt_count = ++decode_attempt_count_;
    const bool log_attempt = shouldLogDecodeAttempt(attempt_count);
    if (log_attempt) {
        std::cerr << "[NvMediaDecoder][DecodeEnter] count=" << attempt_count
                  << " input_bytes=" << size
                  << " payload=" << payloadFormatName(data, size)
                  << std::endl;
        logPayloadPrefix(data, size);
    }

    const auto start = std::chrono::steady_clock::now();
    const int ok = decode_(decoder_, data, size, &bridge_frame);
    const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - start).count();
    const int slow_threshold_ms = getenvInt("SENTINEL_NVMEDIA_SLOW_DECODE_MS", 200);
    if (log_attempt || elapsed_ms > slow_threshold_ms) {
        std::cerr << "[NvMediaDecoder][DecodeReturn] count=" << attempt_count
                  << " bridge_rc=" << ok
                  << " elapsed_ms=" << elapsed_ms
                  << " frame_data=" << static_cast<void*>(bridge_frame.data)
                  << " frame=" << bridge_frame.width << "x" << bridge_frame.height
                  << " stride=" << bridge_frame.stride
                  << " channels=" << bridge_frame.channels
                  << std::endl;
    }

    if (ok == 0) {
        const uint64_t no_output_count = ++decode_no_output_count_;
        if (shouldLogDecodeAttempt(no_output_count)) {
            std::cerr << "[NvMediaDecoder][NoOutput] count=" << no_output_count
                      << " input_bytes=" << size
                      << " payload=" << payloadFormatName(data, size)
                      << " elapsed_ms=" << elapsed_ms
                      << std::endl;
        }
        return false;
    }

    if (ok < 0 || !bridge_frame.data || bridge_frame.width <= 0 ||
        bridge_frame.height <= 0 || bridge_frame.channels != 3 ||
        bridge_frame.stride < bridge_frame.width * bridge_frame.channels) {
        const uint64_t fail_count = ++decode_fail_count_;
        if (shouldLogDecodeFailure(fail_count)) {
            std::cerr << "[NvMediaDecoder][DecodeFail] count=" << fail_count
                      << " bridge_rc=" << ok
                      << " input_bytes=" << size
                      << " payload=" << payloadFormatName(data, size)
                      << " elapsed_ms=" << elapsed_ms
                      << " frame_data=" << static_cast<void*>(bridge_frame.data)
                      << " frame=" << bridge_frame.width << "x" << bridge_frame.height
                      << " stride=" << bridge_frame.stride
                      << " channels=" << bridge_frame.channels
                      << std::endl;
            logPayloadPrefix(data, size);
        }
        if (bridge_frame.data && release_frame_) {
            release_frame_(decoder_, &bridge_frame);
        }
        return false;
    }

    cv::Mat view(bridge_frame.height, bridge_frame.width, CV_8UC3,
                 bridge_frame.data, static_cast<size_t>(bridge_frame.stride));
    output_frame = view.clone();

    if (release_frame_) {
        release_frame_(decoder_, &bridge_frame);
    }
    return !output_frame.empty();
}

void NvMediaHevcDecoder::flush() {
    if (initialized_ && decoder_ && flush_) {
        flush_(decoder_);
    }
}

void NvMediaHevcDecoder::reset() {
    if (decoder_ && destroy_) {
        destroy_(decoder_);
    }
    decoder_ = nullptr;
    initialized_ = false;
}

bool NvMediaHevcDecoder::loadBridge() {
    if (bridge_handle_) {
        return true;
    }

    bridge_handle_ = dlopen(bridge_path_.c_str(), RTLD_NOW | RTLD_LOCAL);
    if (!bridge_handle_) {
        std::cerr << "[NvMediaDecoder] Failed to load bridge " << bridge_path_
                  << ": " << dlerror() << std::endl;
        return false;
    }

    bool ok = true;
    ok = ok && loadSymbol(bridge_handle_, "sentinel_nvmedia_create", &create_);
    ok = ok && loadSymbol(bridge_handle_, "sentinel_nvmedia_destroy", &destroy_);
    ok = ok && loadSymbol(bridge_handle_, "sentinel_nvmedia_decode", &decode_);
    ok = ok && loadSymbol(bridge_handle_, "sentinel_nvmedia_release_frame", &release_frame_);
    ok = ok && loadSymbol(bridge_handle_, "sentinel_nvmedia_flush", &flush_);
    if (!ok) {
        unloadBridge();
        return false;
    }
    return true;
}

void NvMediaHevcDecoder::unloadBridge() {
    create_ = nullptr;
    destroy_ = nullptr;
    decode_ = nullptr;
    release_frame_ = nullptr;
    flush_ = nullptr;
    if (bridge_handle_) {
        dlclose(bridge_handle_);
        bridge_handle_ = nullptr;
    }
}
