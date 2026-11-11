#include "../../nvmedia_decoder_bridge.h"

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

namespace senseAD {
namespace adHal {

using adHalStatus_t = int32_t;

typedef enum {
    DEC_PAYLOAD_TYPE_NONE = -1,
    DEC_PAYLOAD_TYPE_H264,
    DEC_PAYLOAD_TYPE_H265,
    DEC_PAYLOAD_TYPE_H265_J5,
    DEC_PAYLOAD_TYPE_MAX
} DecPayloadType;

typedef struct {
    uint32_t width;
    uint32_t height;
    DecPayloadType payload_type;
} VdecCfgParams;

class ADVideoDecoder {
public:
    explicit ADVideoDecoder(int mode = 1);
    ~ADVideoDecoder();

    adHalStatus_t CreateVdec(const VdecCfgParams& vdec_params);
    adHalStatus_t SendStream(const uint8_t* data,
                             uint32_t length,
                             uint64_t pts,
                             bool end_of_stream = false,
                             bool only_decode = false);
    adHalStatus_t GetDecodedData(uint8_t** data,
                                 bool* is_empty,
                                 uint64_t* pts = nullptr);
    adHalStatus_t GetDecodedData(uint8_t** data,
                                 uint64_t* phy_ptr,
                                 bool* is_empty,
                                 uint64_t* pts = nullptr);
    adHalStatus_t ReleaseFrame(uint8_t* data);
    adHalStatus_t DestroyVdec();

private:
    void* vdec_ = nullptr;
};

}  // namespace adHal
}  // namespace senseAD

struct SentinelNvMediaDecoder {
    explicit SentinelNvMediaDecoder(const SentinelNvMediaDecoderConfig& cfg)
        : config(cfg), decoder(1) {}

    SentinelNvMediaDecoderConfig config;
    senseAD::adHal::ADVideoDecoder decoder;
    std::vector<uint8_t> bgr;
    bool initialized = false;
    bool saw_decoded_frame = false;
    uint64_t pts = 0;
    int drain_attempts = 8;
    bool use_phy_getter = false;
    bool destroy_vdec = false;
    uint64_t send_count = 0;
    uint64_t send_fail_count = 0;
    uint64_t get_count = 0;
    uint64_t get_fail_count = 0;
    uint64_t empty_count = 0;
    uint64_t output_count = 0;
};

namespace {

constexpr int kOk = 0;

bool shouldLogAttempt(uint64_t count) {
    return count <= 10U || count % 300U == 0U;
}

std::string getenvString(const char* name) {
    const char* value = std::getenv(name);
    return value ? value : "";
}

int getenvInt(const char* name, int fallback) {
    const std::string value = getenvString(name);
    if (value.empty()) {
        return fallback;
    }
    try {
        return std::stoi(value);
    } catch (...) {
        return fallback;
    }
}

bool getenvBool(const char* name, bool fallback) {
    const std::string value = getenvString(name);
    if (value.empty()) {
        return fallback;
    }
    return value == "1" || value == "true" || value == "TRUE" ||
           value == "yes" || value == "YES" || value == "on" || value == "ON";
}

senseAD::adHal::DecPayloadType selectPayloadType() {
    const std::string value = getenvString("SENTINEL_NVMEDIA_PAYLOAD_TYPE");
    if (value == "h265_j5" || value == "H265_J5" || value == "j5" || value == "J5") {
        return senseAD::adHal::DEC_PAYLOAD_TYPE_H265_J5;
    }
    if (value == "h264" || value == "H264") {
        return senseAD::adHal::DEC_PAYLOAD_TYPE_H264;
    }
    return senseAD::adHal::DEC_PAYLOAD_TYPE_H265;
}

const char* payloadTypeName(senseAD::adHal::DecPayloadType payload_type) {
    switch (payload_type) {
        case senseAD::adHal::DEC_PAYLOAD_TYPE_H264:
            return "h264";
        case senseAD::adHal::DEC_PAYLOAD_TYPE_H265:
            return "h265";
        case senseAD::adHal::DEC_PAYLOAD_TYPE_H265_J5:
            return "h265_j5";
        default:
            return "unknown";
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
    std::cerr << "[sentinel_nvmedia_adhal][payload] first_bytes=";
    for (size_t i = 0; i < n; ++i) {
        std::cerr << (i == 0 ? "" : " ")
                  << std::hex << std::setw(2) << std::setfill('0')
                  << static_cast<unsigned int>(data[i]);
    }
    std::cerr << std::dec << std::setfill(' ') << std::endl;
}

uint8_t clampByte(int value) {
    return static_cast<uint8_t>(std::max(0, std::min(255, value)));
}

void nv12ToBgr(const uint8_t* nv12, int width, int height, std::vector<uint8_t>& bgr) {
    bgr.resize(static_cast<size_t>(width) * static_cast<size_t>(height) * 3U);
    const uint8_t* y_plane = nv12;
    const uint8_t* uv_plane = nv12 + static_cast<size_t>(width) * height;

    for (int y = 0; y < height; ++y) {
        const int uv_row = (y / 2) * width;
        for (int x = 0; x < width; ++x) {
            const int y_value = static_cast<int>(y_plane[y * width + x]);
            const int uv_index = uv_row + (x / 2) * 2;
            const int u = static_cast<int>(uv_plane[uv_index]) - 128;
            const int v = static_cast<int>(uv_plane[uv_index + 1]) - 128;

            const int c = y_value - 16;
            const int d = u;
            const int e = v;
            const int r = (298 * c + 409 * e + 128) >> 8;
            const int g = (298 * c - 100 * d - 208 * e + 128) >> 8;
            const int b = (298 * c + 516 * d + 128) >> 8;

            const size_t out = (static_cast<size_t>(y) * width + x) * 3U;
            bgr[out + 0] = clampByte(b);
            bgr[out + 1] = clampByte(g);
            bgr[out + 2] = clampByte(r);
        }
    }
}

int pullDecodedFrame(SentinelNvMediaDecoder* decoder,
                     SentinelNvMediaBgrFrame* output) {
    for (int attempt = 0; attempt < decoder->drain_attempts; ++attempt) {
        uint8_t* decoded = nullptr;
        bool is_empty = true;
        uint64_t pts = 0;
        uint64_t phy_ptr = 0;
        senseAD::adHal::adHalStatus_t get_status = kOk;
        const uint64_t next_get_count = decoder->get_count + 1U;
        const bool log_get = decoder->config.verbose || shouldLogAttempt(next_get_count);
        if (log_get) {
            std::cerr << "[sentinel_nvmedia_adhal] GetDecodedData begin"
                      << " attempt=" << attempt
                      << " get_count=" << next_get_count
                      << " use_phy_getter=" << decoder->use_phy_getter
                      << std::endl;
        }
        const auto get_start = std::chrono::steady_clock::now();
        if (decoder->use_phy_getter) {
            get_status = decoder->decoder.GetDecodedData(
                &decoded, &phy_ptr, &is_empty, &pts);
        } else {
            get_status = decoder->decoder.GetDecodedData(&decoded, &is_empty, &pts);
        }
        const auto get_elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - get_start).count();
        const uint64_t get_count = ++decoder->get_count;

        if (log_get) {
            std::cerr << "[sentinel_nvmedia_adhal] GetDecodedData attempt="
                      << attempt
                      << " get_count=" << get_count
                      << " status=" << get_status
                      << " elapsed_ms=" << get_elapsed_ms
                      << " empty=" << is_empty
                      << " data=" << static_cast<void*>(decoded)
                      << " phy=0x" << std::hex << phy_ptr << std::dec
                      << " pts=" << pts << std::endl;
        }

        if (get_status != kOk) {
            ++decoder->get_fail_count;
            return -1;
        }
        if (is_empty || !decoded) {
            ++decoder->empty_count;
            continue;
        }

        nv12ToBgr(decoded, decoder->config.width, decoder->config.height, decoder->bgr);
        decoder->decoder.ReleaseFrame(decoded);

        output->data = decoder->bgr.data();
        output->width = decoder->config.width;
        output->height = decoder->config.height;
        output->stride = decoder->config.width * 3;
        output->channels = 3;
        decoder->saw_decoded_frame = true;
        ++decoder->output_count;
        if (decoder->config.verbose || shouldLogAttempt(decoder->output_count)) {
            std::cerr << "[sentinel_nvmedia_adhal] decoded output_count="
                      << decoder->output_count
                      << " bgr=" << output->width << "x" << output->height
                      << " stride=" << output->stride
                      << std::endl;
        }
        return 1;
    }
    if (decoder->config.verbose || shouldLogAttempt(decoder->empty_count)) {
        std::cerr << "[sentinel_nvmedia_adhal] no decoded frame"
                  << " empty_count=" << decoder->empty_count
                  << " get_fail_count=" << decoder->get_fail_count
                  << " output_count=" << decoder->output_count
                  << std::endl;
    }
    return 0;
}

}  // namespace

extern "C" SentinelNvMediaDecoder* sentinel_nvmedia_create(
    const SentinelNvMediaDecoderConfig* config) {
    if (!config || config->width <= 0 || config->height <= 0) {
        return nullptr;
    }

    try {
        auto* state = new SentinelNvMediaDecoder(*config);
        senseAD::adHal::VdecCfgParams params{};
        params.width = static_cast<uint32_t>(config->width);
        params.height = static_cast<uint32_t>(config->height);
        params.payload_type = selectPayloadType();

        state->drain_attempts = std::max(
            1, getenvInt("SENTINEL_NVMEDIA_DRAIN_ATTEMPTS", state->drain_attempts));
        state->use_phy_getter = getenvBool("SENTINEL_NVMEDIA_USE_PHY_GETTER", false);
        state->destroy_vdec = getenvBool("SENTINEL_NVMEDIA_DESTROY_VDEC", false);

        if (config->verbose) {
            std::cerr << "[sentinel_nvmedia_adhal] CreateVdec width="
                      << params.width << " height=" << params.height
                      << " payload=" << payloadTypeName(params.payload_type)
                      << " drain_attempts=" << state->drain_attempts
                      << " use_phy_getter=" << state->use_phy_getter
                      << " destroy_vdec=" << state->destroy_vdec << std::endl;
            std::cerr << "[sentinel_nvmedia_adhal][WARN] BGR conversion assumes tightly packed NV12 "
                      << "(Y pitch == width, UV offset == width*height). "
                      << "Use the Thor bridge or verify ADVideoDecoder output layout if images look unclear."
                      << std::endl;
        }

        const auto status = state->decoder.CreateVdec(params);
        if (status != kOk) {
            if (config->verbose) {
                std::cerr << "[sentinel_nvmedia_adhal] CreateVdec failed: "
                          << status << std::endl;
            }
            delete state;
            return nullptr;
        }

        state->initialized = true;
        return state;
    } catch (const std::exception& e) {
        if (config && config->verbose) {
            std::cerr << "[sentinel_nvmedia_adhal] create exception: "
                      << e.what() << std::endl;
        }
        return nullptr;
    }
}

extern "C" void sentinel_nvmedia_destroy(SentinelNvMediaDecoder* decoder) {
    if (!decoder) {
        return;
    }
    if (decoder->initialized && decoder->destroy_vdec) {
        decoder->decoder.DestroyVdec();
    } else if (decoder->initialized && decoder->config.verbose) {
        std::cerr << "[sentinel_nvmedia_adhal] Skip DestroyVdec"
                  << " destroy_vdec=" << decoder->destroy_vdec
                  << " saw_decoded_frame=" << decoder->saw_decoded_frame
                  << std::endl;
    }
    delete decoder;
}

extern "C" int sentinel_nvmedia_decode(SentinelNvMediaDecoder* decoder,
                                        const uint8_t* data,
                                        size_t size,
                                        SentinelNvMediaBgrFrame* output) {
    if (!decoder || !decoder->initialized || !data || size == 0 || !output) {
        return -1;
    }
    std::memset(output, 0, sizeof(*output));

    const uint64_t next_send_count = decoder->send_count + 1U;
    const bool log_send = decoder->config.verbose || shouldLogAttempt(next_send_count);
    if (log_send) {
        std::cerr << "[sentinel_nvmedia_adhal] decode enter"
                  << " send_count=" << next_send_count
                  << " size=" << size
                  << " payload=" << payloadFormatName(data, size)
                  << std::endl;
        logPayloadPrefix(data, size);
        std::cerr << "[sentinel_nvmedia_adhal] SendStream begin"
                  << " send_count=" << next_send_count
                  << " pts=" << decoder->pts
                  << " size=" << size
                  << std::endl;
    }
    const auto send_start = std::chrono::steady_clock::now();
    const auto send_status = decoder->decoder.SendStream(
        data, static_cast<uint32_t>(size), decoder->pts++, false, false);
    const auto send_elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - send_start).count();
    const uint64_t send_count = ++decoder->send_count;
    if (log_send) {
        std::cerr << "[sentinel_nvmedia_adhal] SendStream"
                  << " send_count=" << send_count
                  << " size=" << size
                  << " payload=" << payloadFormatName(data, size)
                  << " elapsed_ms=" << send_elapsed_ms
                  << " status=" << send_status << std::endl;
    }
    if (send_status != kOk) {
        ++decoder->send_fail_count;
        std::cerr << "[sentinel_nvmedia_adhal] SendStream failed"
                  << " status=" << send_status
                  << " send_fail_count=" << decoder->send_fail_count
                  << std::endl;
        return -1;
    }

    return pullDecodedFrame(decoder, output);
}

extern "C" void sentinel_nvmedia_release_frame(SentinelNvMediaDecoder* decoder,
                                               SentinelNvMediaBgrFrame* frame) {
    (void)decoder;
    if (frame) {
        std::memset(frame, 0, sizeof(*frame));
    }
}

extern "C" void sentinel_nvmedia_flush(SentinelNvMediaDecoder* decoder) {
    if (!decoder || !decoder->initialized) {
        return;
    }
    const auto send_status = decoder->decoder.SendStream(
        nullptr, 0, decoder->pts++, true, false);
    if (decoder->config.verbose) {
        std::cerr << "[sentinel_nvmedia_adhal] Flush EOS status="
                  << send_status << std::endl;
    }
    SentinelNvMediaBgrFrame ignored{};
    pullDecodedFrame(decoder, &ignored);
}
