#pragma once

#include "hevc_decoder.h"

#include <memory>
#include <string>

enum class HevcDecoderBackend {
    FfmpegNvdec,
    FfmpegSoftware,
    NvMedia,
    Auto,
};

HevcDecoderBackend parseHevcDecoderBackend(const std::string& value);
std::string hevcDecoderBackendName(HevcDecoderBackend backend);

std::unique_ptr<IHevcDecoder> createHevcDecoder(HevcDecoderBackend backend);
