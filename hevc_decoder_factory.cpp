#include "hevc_decoder_factory.h"

#include "nvmedia_hevc_decoder.h"
#ifdef SENTINEL_ENABLE_FFMPEG_DECODER
#include "nv_hevc_decoder.h"
#endif

#include <algorithm>
#include <cctype>

namespace {

std::string normalizeBackendName(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char ch) {
        return static_cast<char>(std::tolower(ch));
    });
    std::replace(value.begin(), value.end(), '-', '_');
    return value;
}

}  // namespace

HevcDecoderBackend parseHevcDecoderBackend(const std::string& value) {
    const std::string normalized = normalizeBackendName(value);
    if (normalized == "nvmedia" || normalized == "thor_nvmedia") {
        return HevcDecoderBackend::NvMedia;
    }
    if (normalized == "ffmpeg_software" || normalized == "software") {
        return HevcDecoderBackend::FfmpegSoftware;
    }
    if (normalized == "auto") {
        return HevcDecoderBackend::Auto;
    }
    return HevcDecoderBackend::FfmpegNvdec;
}

std::string hevcDecoderBackendName(HevcDecoderBackend backend) {
    switch (backend) {
    case HevcDecoderBackend::NvMedia:
        return "nvmedia";
    case HevcDecoderBackend::Auto:
        return "auto";
    case HevcDecoderBackend::FfmpegSoftware:
        return "ffmpeg_software";
    case HevcDecoderBackend::FfmpegNvdec:
    default:
        return "ffmpeg_nvdec";
    }
}

std::unique_ptr<IHevcDecoder> createHevcDecoder(HevcDecoderBackend backend) {
    if (backend == HevcDecoderBackend::NvMedia || backend == HevcDecoderBackend::Auto) {
        return std::make_unique<NvMediaHevcDecoder>();
    }
#ifdef SENTINEL_ENABLE_FFMPEG_DECODER
    return std::make_unique<NVHevcDecoder>(backend == HevcDecoderBackend::FfmpegSoftware);
#else
    return std::make_unique<NvMediaHevcDecoder>();
#endif
}
