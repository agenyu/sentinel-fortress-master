#include "../../nvmedia_decoder_bridge.h"

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <deque>
#include <iostream>
#include <list>
#include <mutex>
#include <new>
#include <string>
#include <unordered_map>
#include <vector>

#include <nvmedia_ide.h>
#include <nvmedia_2d.h>
#include <nvmedia_2d_sci.h>
#include <nvmedia_parser.h>
#include <nvscibuf.h>
#include <nvscisync.h>

constexpr uint32_t kMaxDecodeBuffers = 16;
constexpr uint32_t kInputBuffering = 1;
constexpr uint32_t kMaxReferences = 8;
constexpr uint32_t kDefaultMaxBitstreamSize = 6U * 1024U * 1024U;
constexpr int kDrainAttempts = 8;
constexpr int kSleepMs = 2;
constexpr int64_t kFenceWaitTimeoutUs = 1000000;

enum class Codec {
    H264,
    H265,
};

Codec selectCodec() {
    const char* codec = std::getenv("SENTINEL_NVMEDIA_CODEC");
    if (codec && (std::strcmp(codec, "h264") == 0 || std::strcmp(codec, "H264") == 0)) {
        return Codec::H264;
    }
    return Codec::H265;
}

NvMediaVideoCodec toNvMediaCodec(Codec codec) {
    return codec == Codec::H264 ? NVMEDIA_VIDEO_CODEC_H264 : NVMEDIA_VIDEO_CODEC_HEVC;
}

const char* codecName(Codec codec) {
    return codec == Codec::H264 ? "h264" : "h265";
}

bool shouldLogAttempt(uint64_t count) {
    return count <= 10U || count % 300U == 0U;
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
    std::cerr << "[sentinel_nvmedia_thor][payload] first_bytes=";
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

void nv12ToBgr(const uint8_t* nv12, int width, int height, int pitch,
               std::vector<uint8_t>& bgr) {
    bgr.resize(static_cast<size_t>(width) * static_cast<size_t>(height) * 3U);
    const uint8_t* y_plane = nv12;
    const uint8_t* uv_plane = nv12 + static_cast<size_t>(pitch) * height;

    for (int y = 0; y < height; ++y) {
        const int uv_row = (y / 2) * pitch;
        for (int x = 0; x < width; ++x) {
            const int y_value = static_cast<int>(y_plane[y * pitch + x]);
            const int uv_index = uv_row + (x / 2) * 2;
            const int u = static_cast<int>(uv_plane[uv_index]) - 128;
            const int v = static_cast<int>(uv_plane[uv_index + 1]) - 128;

            const int c = y_value - 16;
            const int r = (298 * c + 409 * v + 128) >> 8;
            const int g = (298 * c - 100 * u - 208 * v + 128) >> 8;
            const int b = (298 * c + 516 * u + 128) >> 8;

            const size_t out = (static_cast<size_t>(y) * width + x) * 3U;
            bgr[out + 0] = clampByte(b);
            bgr[out + 1] = clampByte(g);
            bgr[out + 2] = clampByte(r);
        }
    }
}

struct FrameBuffer {
    int ref_count = 0;
    int index = -1;
    int frame_num = 0;
    NvSciBufObj surface = nullptr;
    NvSciSyncFence eof_fence{};
};

struct SentinelNvMediaDecoder {
    explicit SentinelNvMediaDecoder(const SentinelNvMediaDecoderConfig& cfg)
        : config(cfg), codec(selectCodec()) {}

    SentinelNvMediaDecoderConfig config;
    Codec codec;

    NvMediaParserClientCb callbacks{};
    NvMediaParserParams parser_params{};
    NvMediaParser* parser = nullptr;
    NvMediaIDE* decoder = nullptr;
    NvMedia2D* blit2d = nullptr;
    NvMediaDecoderInstanceId instance_id{};
    NvSciSyncObj eof_sync = nullptr;
    NvSciSyncObj blit_eof_sync = nullptr;
    NvSciBufObj readback_surface = nullptr;
    uint64_t readback_offsets[2] = {0U, 0U};
    uint32_t readback_pitches[2] = {0U, 0U};

    NvSciBufModule buf_module = nullptr;
    NvSciSyncModule sync_module = nullptr;
    NvSciSyncCpuWaitContext cpu_wait_context = nullptr;

    uint32_t coded_width = 0;
    uint32_t coded_height = 0;
    uint32_t display_width = 0;
    uint32_t display_height = 0;
    uint32_t max_bitstream_size = kDefaultMaxBitstreamSize;
    uint32_t buffer_count = 0;
    int pic_num = 0;
    int frame_count = 0;
    uint64_t parse_count = 0;
    uint64_t parse_fail_count = 0;
    uint64_t decode_callback_count = 0;
    uint64_t decode_submit_fail_count = 0;
    uint64_t display_callback_count = 0;
    uint64_t readback_fail_count = 0;
    uint64_t readback_success_count = 0;
    uint64_t no_output_count = 0;

    FrameBuffer buffers[kMaxDecodeBuffers];
    std::mutex free_mutex;
    std::mutex valid_mutex;
    std::condition_variable valid_cv;
    std::list<NvSciBufObj> free_buffers;
    std::deque<NvSciBufObj> valid_buffers;
    std::unordered_map<NvSciBufObj, FrameBuffer*> surface_to_frame;

    std::vector<uint8_t> bgr;
    bool initialized = false;
    bool quitting = false;
};

bool statusOk(NvMediaStatus status, const char* what) {
    if (status != NVMEDIA_STATUS_OK) {
        std::cerr << "[sentinel_nvmedia_thor] " << what << " failed status="
                  << status << std::endl;
        return false;
    }
    return true;
}

bool sciOk(NvSciError status, const char* what) {
    if (status != NvSciError_Success) {
        std::cerr << "[sentinel_nvmedia_thor] " << what << " failed status="
                  << status << std::endl;
        return false;
    }
    return true;
}

bool createOutputSurface(SentinelNvMediaDecoder* state, FrameBuffer* frame) {
    NvSciBufAttrList ide_attr = nullptr;
    NvSciBufAttrList blit_attr = nullptr;
    NvSciBufAttrList reconciled = nullptr;
    NvSciBufAttrList conflict = nullptr;

    if (!sciOk(NvSciBufAttrListCreate(state->buf_module, &ide_attr),
               "NvSciBufAttrListCreate")) {
        return false;
    }
    if (!statusOk(NvMediaIDEFillNvSciBufAttrList(state->instance_id, ide_attr),
                  "NvMediaIDEFillNvSciBufAttrList")) {
        NvSciBufAttrListFree(ide_attr);
        return false;
    }
    if (!sciOk(NvSciBufAttrListCreate(state->buf_module, &blit_attr),
               "NvSciBufAttrListCreate 2D decode")) {
        NvSciBufAttrListFree(ide_attr);
        return false;
    }
    if (!statusOk(NvMedia2DFillNvSciBufAttrList(state->blit2d, blit_attr),
                  "NvMedia2DFillNvSciBufAttrList decode")) {
        NvSciBufAttrListFree(blit_attr);
        NvSciBufAttrListFree(ide_attr);
        return false;
    }

    NvSciBufType buf_type = NvSciBufType_Image;
    NvSciBufAttrValAccessPerm access = NvSciBufAccessPerm_ReadWrite;
    bool need_cpu = false;
    NvSciBufAttrValImageLayoutType layout = NvSciBufImage_BlockLinearType;
    NvSciBufAttrValImageScanType scan = NvSciBufScan_ProgressiveType;
    uint32_t plane_count = 2;
    NvSciBufAttrValColorFmt color_formats[2] = {NvSciColor_Y8, NvSciColor_V8U8};
    NvSciBufAttrValColorStd color_std[2] = {NvSciColorStd_REC601_SR, NvSciColorStd_REC601_SR};
    uint32_t width[2] = {state->coded_width, state->coded_width / 2U};
    uint32_t height[2] = {state->coded_height, state->coded_height / 2U};
    uint32_t align[2] = {256U, 256U};
    uint64_t padding[2] = {0U, 0U};
    bool vpr = false;

    NvSciBufAttrKeyValuePair attrs[] = {
        {NvSciBufGeneralAttrKey_Types, &buf_type, sizeof(buf_type)},
        {NvSciBufGeneralAttrKey_RequiredPerm, &access, sizeof(access)},
        {NvSciBufGeneralAttrKey_NeedCpuAccess, &need_cpu, sizeof(need_cpu)},
        {NvSciBufGeneralAttrKey_EnableCpuCache, &need_cpu, sizeof(need_cpu)},
        {NvSciBufImageAttrKey_Layout, &layout, sizeof(layout)},
        {NvSciBufImageAttrKey_ScanType, &scan, sizeof(scan)},
        {NvSciBufImageAttrKey_PlaneCount, &plane_count, sizeof(plane_count)},
        {NvSciBufImageAttrKey_PlaneColorFormat, color_formats, sizeof(color_formats)},
        {NvSciBufImageAttrKey_PlaneColorStd, color_std, sizeof(color_std)},
        {NvSciBufImageAttrKey_PlaneWidth, width, sizeof(width)},
        {NvSciBufImageAttrKey_PlaneHeight, height, sizeof(height)},
        {NvSciBufImageAttrKey_PlaneBaseAddrAlign, align, sizeof(align)},
        {NvSciBufImageAttrKey_TopPadding, padding, sizeof(padding)},
        {NvSciBufImageAttrKey_BottomPadding, padding, sizeof(padding)},
        {NvSciBufImageAttrKey_LeftPadding, padding, sizeof(padding)},
        {NvSciBufImageAttrKey_RightPadding, padding, sizeof(padding)},
        {NvSciBufImageAttrKey_VprFlag, &vpr, sizeof(vpr)},
    };

    bool ok = sciOk(NvSciBufAttrListSetAttrs(ide_attr, attrs,
                                             sizeof(attrs) / sizeof(attrs[0])),
                    "NvSciBufAttrListSetAttrs") &&
              sciOk(NvSciBufAttrListSetAttrs(blit_attr, attrs,
                                             sizeof(attrs) / sizeof(attrs[0])),
                    "NvSciBufAttrListSetAttrs 2D decode") &&
              [&]() {
                  NvSciBufAttrList unreconciled[] = {ide_attr, blit_attr};
                  return sciOk(NvSciBufAttrListReconcile(unreconciled, 2,
                                                         &reconciled, &conflict),
                               "NvSciBufAttrListReconcile");
              }() &&
              sciOk(NvSciBufObjAlloc(reconciled, &frame->surface),
                    "NvSciBufObjAlloc");

    if (conflict) {
        NvSciBufAttrListFree(conflict);
    }
    if (reconciled) {
        NvSciBufAttrListFree(reconciled);
    }
    if (blit_attr) {
        NvSciBufAttrListFree(blit_attr);
    }
    if (ide_attr) {
        NvSciBufAttrListFree(ide_attr);
    }
    if (!ok) {
        return false;
    }

    if (state->config.verbose) {
        std::cerr << "[sentinel_nvmedia_thor] decode surface layout=block-linear coded="
                  << state->coded_width << "x"
                  << state->coded_height << std::endl;
    }

    if (!statusOk(NvMediaIDERegisterNvSciBufObj(state->decoder, frame->surface),
                  "NvMediaIDERegisterNvSciBufObj")) {
        return false;
    }
    if (state->blit2d &&
        !statusOk(NvMedia2DRegisterNvSciBufObj(state->blit2d, frame->surface),
                  "NvMedia2DRegisterNvSciBufObj decode surface")) {
        return false;
    }
    return true;
}

bool createReadbackSurface(SentinelNvMediaDecoder* state) {
    NvSciBufAttrList attr = nullptr;
    NvSciBufAttrList reconciled = nullptr;
    NvSciBufAttrList conflict = nullptr;

    if (!sciOk(NvSciBufAttrListCreate(state->buf_module, &attr),
               "NvSciBufAttrListCreate readback")) {
        return false;
    }
    if (!statusOk(NvMedia2DFillNvSciBufAttrList(state->blit2d, attr),
                  "NvMedia2DFillNvSciBufAttrList")) {
        NvSciBufAttrListFree(attr);
        return false;
    }

    NvSciBufType buf_type = NvSciBufType_Image;
    NvSciBufAttrValAccessPerm access = NvSciBufAccessPerm_ReadWrite;
    bool need_cpu = true;
    NvSciBufAttrValImageLayoutType layout = NvSciBufImage_PitchLinearType;
    NvSciBufAttrValImageScanType scan = NvSciBufScan_ProgressiveType;
    uint32_t plane_count = 2;
    NvSciBufAttrValColorFmt color_formats[2] = {NvSciColor_Y8, NvSciColor_V8U8};
    NvSciBufAttrValColorStd color_std[2] = {NvSciColorStd_REC601_SR, NvSciColorStd_REC601_SR};
    uint32_t width[2] = {state->coded_width, state->coded_width / 2U};
    uint32_t height[2] = {state->coded_height, state->coded_height / 2U};
    uint32_t align[2] = {256U, 256U};
    uint64_t padding[2] = {0U, 0U};
    bool vpr = false;

    NvSciBufAttrKeyValuePair attrs[] = {
        {NvSciBufGeneralAttrKey_Types, &buf_type, sizeof(buf_type)},
        {NvSciBufGeneralAttrKey_RequiredPerm, &access, sizeof(access)},
        {NvSciBufGeneralAttrKey_NeedCpuAccess, &need_cpu, sizeof(need_cpu)},
        {NvSciBufGeneralAttrKey_EnableCpuCache, &need_cpu, sizeof(need_cpu)},
        {NvSciBufImageAttrKey_Layout, &layout, sizeof(layout)},
        {NvSciBufImageAttrKey_ScanType, &scan, sizeof(scan)},
        {NvSciBufImageAttrKey_PlaneCount, &plane_count, sizeof(plane_count)},
        {NvSciBufImageAttrKey_PlaneColorFormat, color_formats, sizeof(color_formats)},
        {NvSciBufImageAttrKey_PlaneColorStd, color_std, sizeof(color_std)},
        {NvSciBufImageAttrKey_PlaneWidth, width, sizeof(width)},
        {NvSciBufImageAttrKey_PlaneHeight, height, sizeof(height)},
        {NvSciBufImageAttrKey_PlaneBaseAddrAlign, align, sizeof(align)},
        {NvSciBufImageAttrKey_TopPadding, padding, sizeof(padding)},
        {NvSciBufImageAttrKey_BottomPadding, padding, sizeof(padding)},
        {NvSciBufImageAttrKey_LeftPadding, padding, sizeof(padding)},
        {NvSciBufImageAttrKey_RightPadding, padding, sizeof(padding)},
        {NvSciBufImageAttrKey_VprFlag, &vpr, sizeof(vpr)},
    };

    bool ok = sciOk(NvSciBufAttrListSetAttrs(attr, attrs,
                                             sizeof(attrs) / sizeof(attrs[0])),
                    "NvSciBufAttrListSetAttrs readback") &&
              sciOk(NvSciBufAttrListReconcile(&attr, 1, &reconciled, &conflict),
                    "NvSciBufAttrListReconcile readback");

    if (ok) {
        uint64_t plane_offsets[2] = {0U, 0U};
        uint32_t plane_pitches[2] = {0U, 0U};
        NvSciBufAttrKeyValuePair out_attrs[] = {
            {NvSciBufImageAttrKey_PlaneOffset, plane_offsets, sizeof(plane_offsets)},
            {NvSciBufImageAttrKey_PlanePitch, plane_pitches, sizeof(plane_pitches)},
        };
        ok = sciOk(NvSciBufAttrListGetAttrs(reconciled, out_attrs,
                                            sizeof(out_attrs) / sizeof(out_attrs[0])),
                   "NvSciBufAttrListGetAttrs readback");
        if (ok) {
            state->readback_offsets[0] = plane_offsets[0];
            state->readback_offsets[1] = plane_offsets[1];
            state->readback_pitches[0] = plane_pitches[0];
            state->readback_pitches[1] = plane_pitches[1];
        }
    }

    if (ok) {
        ok = sciOk(NvSciBufObjAlloc(reconciled, &state->readback_surface),
                   "NvSciBufObjAlloc readback");
    }

    if (conflict) {
        NvSciBufAttrListFree(conflict);
    }
    if (reconciled) {
        NvSciBufAttrListFree(reconciled);
    }
    if (attr) {
        NvSciBufAttrListFree(attr);
    }
    if (!ok) {
        return false;
    }

    if (state->config.verbose) {
        std::cerr << "[sentinel_nvmedia_thor] readback surface layout=pitch-linear coded="
                  << state->coded_width << "x" << state->coded_height
                  << " pitch=" << state->readback_pitches[0]
                  << "/" << state->readback_pitches[1]
                  << " offset=" << state->readback_offsets[0]
                  << "/" << state->readback_offsets[1] << std::endl;
    }

    return statusOk(NvMedia2DRegisterNvSciBufObj(state->blit2d,
                                                 state->readback_surface),
                    "NvMedia2DRegisterNvSciBufObj readback");
}

bool createSync(SentinelNvMediaDecoder* state) {
    NvSciSyncAttrList signaler = nullptr;
    NvSciSyncAttrList waiter = nullptr;
    NvSciSyncAttrList reconciled = nullptr;
    NvSciSyncAttrList conflict = nullptr;

    if (!sciOk(NvSciSyncAttrListCreate(state->sync_module, &signaler),
               "NvSciSyncAttrListCreate")) {
        return false;
    }
    if (!sciOk(NvSciSyncCpuWaitContextAlloc(state->sync_module,
                                            &state->cpu_wait_context),
               "NvSciSyncCpuWaitContextAlloc")) {
        NvSciSyncAttrListFree(signaler);
        return false;
    }
    if (!statusOk(NvMediaIDEFillNvSciSyncAttrList(state->decoder, signaler,
                                                  NVMEDIA_SIGNALER),
                  "NvMediaIDEFillNvSciSyncAttrList")) {
        NvSciSyncAttrListFree(signaler);
        return false;
    }
    if (!sciOk(NvSciSyncAttrListCreate(state->sync_module, &waiter),
               "NvSciSyncAttrListCreate")) {
        NvSciSyncAttrListFree(signaler);
        return false;
    }

    bool cpu_waiter = true;
    NvSciSyncAccessPerm wait_perm = NvSciSyncAccessPerm_WaitOnly;
    NvSciSyncAttrKeyValuePair waiter_attrs[] = {
        {NvSciSyncAttrKey_NeedCpuAccess, &cpu_waiter, sizeof(cpu_waiter)},
        {NvSciSyncAttrKey_RequiredPerm, &wait_perm, sizeof(wait_perm)},
    };
    if (!sciOk(NvSciSyncAttrListSetAttrs(waiter, waiter_attrs,
                                         sizeof(waiter_attrs) / sizeof(waiter_attrs[0])),
               "NvSciSyncAttrListSetAttrs")) {
        NvSciSyncAttrListFree(waiter);
        NvSciSyncAttrListFree(signaler);
        return false;
    }

    NvSciSyncAttrList unreconciled[] = {signaler, waiter};
    bool ok = sciOk(NvSciSyncAttrListReconcile(unreconciled, 2, &reconciled, &conflict),
                    "NvSciSyncAttrListReconcile") &&
              sciOk(NvSciSyncObjAlloc(reconciled, &state->eof_sync),
                    "NvSciSyncObjAlloc") &&
              statusOk(NvMediaIDERegisterNvSciSyncObj(state->decoder,
                                                       NVMEDIA_EOFSYNCOBJ,
                                                       state->eof_sync),
                       "NvMediaIDERegisterNvSciSyncObj") &&
              statusOk(NvMediaIDESetNvSciSyncObjforEOF(state->decoder,
                                                        state->eof_sync),
                       "NvMediaIDESetNvSciSyncObjforEOF");

    if (conflict) {
        NvSciSyncAttrListFree(conflict);
    }
    if (reconciled) {
        NvSciSyncAttrListFree(reconciled);
    }
    if (waiter) {
        NvSciSyncAttrListFree(waiter);
    }
    if (signaler) {
        NvSciSyncAttrListFree(signaler);
    }
    return ok;
}

bool createBlitSync(SentinelNvMediaDecoder* state) {
    NvSciSyncAttrList signaler = nullptr;
    NvSciSyncAttrList waiter = nullptr;
    NvSciSyncAttrList reconciled = nullptr;
    NvSciSyncAttrList conflict = nullptr;

    if (!sciOk(NvSciSyncAttrListCreate(state->sync_module, &signaler),
               "NvSciSyncAttrListCreate blit signaler")) {
        return false;
    }
    if (!statusOk(NvMedia2DFillNvSciSyncAttrList(state->blit2d, signaler,
                                                 NVMEDIA_SIGNALER),
                  "NvMedia2DFillNvSciSyncAttrList")) {
        NvSciSyncAttrListFree(signaler);
        return false;
    }
    if (!sciOk(NvSciSyncAttrListCreate(state->sync_module, &waiter),
               "NvSciSyncAttrListCreate blit waiter")) {
        NvSciSyncAttrListFree(signaler);
        return false;
    }

    bool cpu_waiter = true;
    NvSciSyncAccessPerm wait_perm = NvSciSyncAccessPerm_WaitOnly;
    NvSciSyncAttrKeyValuePair waiter_attrs[] = {
        {NvSciSyncAttrKey_NeedCpuAccess, &cpu_waiter, sizeof(cpu_waiter)},
        {NvSciSyncAttrKey_RequiredPerm, &wait_perm, sizeof(wait_perm)},
    };
    if (!sciOk(NvSciSyncAttrListSetAttrs(waiter, waiter_attrs,
                                         sizeof(waiter_attrs) / sizeof(waiter_attrs[0])),
               "NvSciSyncAttrListSetAttrs blit waiter")) {
        NvSciSyncAttrListFree(waiter);
        NvSciSyncAttrListFree(signaler);
        return false;
    }

    NvSciSyncAttrList unreconciled[] = {signaler, waiter};
    bool ok = sciOk(NvSciSyncAttrListReconcile(unreconciled, 2,
                                               &reconciled, &conflict),
                    "NvSciSyncAttrListReconcile blit") &&
              sciOk(NvSciSyncObjAlloc(reconciled, &state->blit_eof_sync),
                    "NvSciSyncObjAlloc blit") &&
              statusOk(NvMedia2DRegisterNvSciSyncObj(state->blit2d,
                                                      NVMEDIA_EOFSYNCOBJ,
                                                      state->blit_eof_sync),
                       "NvMedia2DRegisterNvSciSyncObj EOF") &&
              statusOk(NvMedia2DRegisterNvSciSyncObj(state->blit2d,
                                                      NVMEDIA_PRESYNCOBJ,
                                                      state->eof_sync),
                       "NvMedia2DRegisterNvSciSyncObj pre");

    if (conflict) {
        NvSciSyncAttrListFree(conflict);
    }
    if (reconciled) {
        NvSciSyncAttrListFree(reconciled);
    }
    if (waiter) {
        NvSciSyncAttrListFree(waiter);
    }
    if (signaler) {
        NvSciSyncAttrListFree(signaler);
    }
    return ok;
}

NvMediaStatus updateH264(NvMediaPictureInfoH264* info) {
    for (uint32_t i = 0; i < 16; ++i) {
        auto* frame = reinterpret_cast<FrameBuffer*>(info->referenceFrames[i].surface);
        info->referenceFrames[i].surface =
            frame ? reinterpret_cast<NvMediaRefSurface*>(frame->surface) : nullptr;
    }
    return NVMEDIA_STATUS_OK;
}

NvMediaStatus updateH265(NvMediaPictureInfoH265* info) {
    for (uint32_t i = 0; i < 16; ++i) {
        auto* frame = reinterpret_cast<FrameBuffer*>(info->RefPics[i]);
        info->RefPics[i] =
            frame ? reinterpret_cast<NvMediaRefSurface*>(frame->surface) : nullptr;
    }
    return NVMEDIA_STATUS_OK;
}

int32_t beginSequence(void* ptr, const NvMediaParserSeqInfo* seq) {
    auto* state = static_cast<SentinelNvMediaDecoder*>(ptr);
    if (!state || !seq) {
        return -1;
    }
    state->coded_width = (seq->uCodedWidth + 15U) & ~15U;
    state->coded_height = (seq->uCodedHeight + 15U) & ~15U;
    state->display_width = seq->uDisplayWidth ? seq->uDisplayWidth : state->config.width;
    state->display_height = seq->uDisplayHeight ? seq->uDisplayHeight : state->config.height;
    state->max_bitstream_size = seq->uMaxBitstreamSize ? seq->uMaxBitstreamSize
                                                       : kDefaultMaxBitstreamSize;
    std::cerr << "[sentinel_nvmedia_thor][begin_sequence] codec="
              << codecName(state->codec)
              << " coded=" << state->coded_width << "x" << state->coded_height
              << " display=" << state->display_width << "x" << state->display_height
              << " decode_buffers=" << seq->uDecodeBuffers
              << " max_bitstream=" << state->max_bitstream_size
              << std::endl;
    return static_cast<int32_t>(std::min<uint32_t>(seq->uDecodeBuffers,
                                                   kMaxDecodeBuffers));
}

NvMediaStatus allocPictureBuffer(void* ptr, NvMediaRefSurface** out) {
    auto* state = static_cast<SentinelNvMediaDecoder*>(ptr);
    if (!state || !out) {
        return NVMEDIA_STATUS_ERROR;
    }
    *out = nullptr;

    for (uint32_t i = 0; i < state->buffer_count; ++i) {
        FrameBuffer& frame = state->buffers[i];
        if (frame.ref_count != 0) {
            continue;
        }
        std::lock_guard<std::mutex> lock(state->free_mutex);
        auto it = std::find(state->free_buffers.begin(), state->free_buffers.end(),
                            frame.surface);
        if (it == state->free_buffers.end()) {
            continue;
        }
        frame.ref_count = 1;
        frame.index = static_cast<int>(i);
        *out = reinterpret_cast<NvMediaRefSurface*>(&frame);
        return NVMEDIA_STATUS_OK;
    }

    return NVMEDIA_STATUS_ERROR;
}

NvMediaStatus decodePicture(void* ptr, NvMediaParserPictureData* pd) {
    auto* state = static_cast<SentinelNvMediaDecoder*>(ptr);
    if (!state || !pd || !pd->pCurrPic) {
        return NVMEDIA_STATUS_ERROR;
    }
    const uint64_t callback_count = ++state->decode_callback_count;

    auto* target = reinterpret_cast<FrameBuffer*>(pd->pCurrPic);
    NvMediaStatus status = NVMEDIA_STATUS_OK;
    if (state->codec == Codec::H264) {
        status = updateH264(&pd->CodecSpecificInfo.h264);
    } else {
        status = updateH265(&pd->CodecSpecificInfo.hevc);
    }
    if (status != NVMEDIA_STATUS_OK) {
        ++state->decode_submit_fail_count;
        std::cerr << "[sentinel_nvmedia_thor][decode_picture] reference update failed"
                  << " status=" << status
                  << " callbacks=" << callback_count
                  << " submit_fail=" << state->decode_submit_fail_count
                  << std::endl;
        return status;
    }

    NvMediaBitstreamBuffer bitstream{};
    bitstream.bitstream = static_cast<uint8_t*>(pd->pBitstreamData);
    bitstream.bitstreamBytes = pd->uBitstreamDataLen;
    target->frame_num = state->pic_num++;
    if (shouldLogAttempt(callback_count)) {
        std::cerr << "[sentinel_nvmedia_thor][decode_picture] callbacks="
                  << callback_count
                  << " frame_num=" << target->frame_num
                  << " bitstream_bytes=" << bitstream.bitstreamBytes
                  << " target_index=" << target->index
                  << std::endl;
    }

    status = NvMediaIDEDecoderRender(state->decoder,
                                     target->surface,
                                     reinterpret_cast<NvMediaPictureInfo*>(
                                         &pd->CodecSpecificInfo),
                                     nullptr,
                                     1,
                                     &bitstream,
                                     nullptr,
                                     state->instance_id);
    if (status != NVMEDIA_STATUS_OK) {
        ++state->decode_submit_fail_count;
        std::cerr << "[sentinel_nvmedia_thor][decode_picture] NvMediaIDEDecoderRender failed"
                  << " status=" << status
                  << " callbacks=" << callback_count
                  << " submit_fail=" << state->decode_submit_fail_count
                  << std::endl;
        return status;
    }
    status = NvMediaIDEGetEOFNvSciSyncFence(state->decoder, state->eof_sync,
                                            &target->eof_fence);
    if (status != NVMEDIA_STATUS_OK) {
        ++state->decode_submit_fail_count;
        std::cerr << "[sentinel_nvmedia_thor][decode_picture] NvMediaIDEGetEOFNvSciSyncFence failed"
                  << " status=" << status
                  << " callbacks=" << callback_count
                  << " submit_fail=" << state->decode_submit_fail_count
                  << std::endl;
    }
    return status;
}

NvMediaStatus displayPicture(void* ptr, NvMediaRefSurface* surface, int64_t pts) {
    (void)pts;
    auto* state = static_cast<SentinelNvMediaDecoder*>(ptr);
    auto* frame = reinterpret_cast<FrameBuffer*>(surface);
    if (!state || !frame || !frame->surface) {
        return NVMEDIA_STATUS_ERROR;
    }
    {
        std::lock_guard<std::mutex> lock(state->free_mutex);
        auto it = std::find(state->free_buffers.begin(), state->free_buffers.end(),
                            frame->surface);
        if (it != state->free_buffers.end()) {
            state->free_buffers.erase(it);
        }
    }
    size_t valid_queue_size = 0;
    {
        std::lock_guard<std::mutex> lock(state->valid_mutex);
        state->valid_buffers.emplace_back(frame->surface);
        valid_queue_size = state->valid_buffers.size();
    }
    state->valid_cv.notify_one();
    ++state->frame_count;
    const uint64_t display_count = ++state->display_callback_count;
    if (shouldLogAttempt(display_count)) {
        std::cerr << "[sentinel_nvmedia_thor][display_picture] displays="
                  << display_count
                  << " frame_num=" << frame->frame_num
                  << " frame_count=" << state->frame_count
                  << " valid_queue=" << valid_queue_size
                  << std::endl;
    }
    return NVMEDIA_STATUS_OK;
}

void releasePicture(void* ptr, NvMediaRefSurface* surface) {
    (void)ptr;
    auto* frame = reinterpret_cast<FrameBuffer*>(surface);
    if (frame && frame->ref_count > 0) {
        --frame->ref_count;
    }
}

void addRefPicture(void* ptr, NvMediaRefSurface* surface) {
    (void)ptr;
    auto* frame = reinterpret_cast<FrameBuffer*>(surface);
    if (frame) {
        ++frame->ref_count;
    }
}

bool initDecoder(SentinelNvMediaDecoder* state) {
    state->instance_id =
        static_cast<NvMediaDecoderInstanceId>(std::max(0, state->config.device_id));
    state->coded_width = static_cast<uint32_t>((state->config.width + 15) & ~15);
    state->coded_height = static_cast<uint32_t>((state->config.height + 15) & ~15);
    state->display_width = static_cast<uint32_t>(state->config.width);
    state->display_height = static_cast<uint32_t>(state->config.height);

    state->callbacks = {
        &beginSequence,
        &decodePicture,
        &displayPicture,
        nullptr,
        &allocPictureBuffer,
        &releasePicture,
        &addRefPicture,
        nullptr,
        nullptr,
        nullptr,
        nullptr,
        nullptr,
        nullptr,
    };

    state->parser_params.pClient = &state->callbacks;
    state->parser_params.pClientCtx = state;
    state->parser_params.uErrorThreshold = 50;
    state->parser_params.eCodec = toNvMediaCodec(state->codec);

    state->parser = NvMediaParserCreate(&state->parser_params);
    if (!state->parser) {
        std::cerr << "[sentinel_nvmedia_thor] NvMediaParserCreate failed" << std::endl;
        return false;
    }

    float default_frame_rate = 30.0F;
    NvMediaParserSetAttribute(state->parser, NvMParseAttr_SetDefaultFramerate,
                              sizeof(default_frame_rate), &default_frame_rate);

    state->decoder = NvMediaIDECreateCtx();
    if (!state->decoder) {
        std::cerr << "[sentinel_nvmedia_thor] NvMediaIDECreateCtx failed" << std::endl;
        return false;
    }

    if (!statusOk(NvMediaIDEInit(state->decoder,
                                 toNvMediaCodec(state->codec),
                                 state->coded_width,
                                 state->coded_height,
                                 kMaxReferences,
                                 state->max_bitstream_size,
                                 kInputBuffering,
                                 0,
                                 state->instance_id),
                  "NvMediaIDEInit")) {
        return false;
    }

    if (!sciOk(NvSciBufModuleOpen(&state->buf_module), "NvSciBufModuleOpen") ||
        !sciOk(NvSciSyncModuleOpen(&state->sync_module), "NvSciSyncModuleOpen") ||
        !createSync(state)) {
        return false;
    }
    if (!statusOk(NvMedia2DCreate(&state->blit2d, nullptr),
                  "NvMedia2DCreate") ||
        !createBlitSync(state) ||
        !createReadbackSurface(state)) {
        return false;
    }

    state->buffer_count = kMaxDecodeBuffers;
    for (uint32_t i = 0; i < state->buffer_count; ++i) {
        state->buffers[i].index = static_cast<int>(i);
        if (!createOutputSurface(state, &state->buffers[i])) {
            return false;
        }
        state->surface_to_frame[state->buffers[i].surface] = &state->buffers[i];
        state->free_buffers.emplace_back(state->buffers[i].surface);
    }

    state->initialized = true;
    std::cerr << "[sentinel_nvmedia_thor][init] codec=" << codecName(state->codec)
              << " requested=" << state->config.width << "x" << state->config.height
              << " coded=" << state->coded_width << "x" << state->coded_height
              << " instance_id=" << state->instance_id
              << " buffers=" << state->buffer_count
              << " max_bitstream=" << state->max_bitstream_size
              << std::endl;
    return true;
}

bool copySurfaceToBgr(SentinelNvMediaDecoder* state, NvSciBufObj surface,
                      SentinelNvMediaBgrFrame* output) {
    auto frame_it = state->surface_to_frame.find(surface);
    if (frame_it == state->surface_to_frame.end() || !frame_it->second) {
        ++state->readback_fail_count;
        std::cerr << "[sentinel_nvmedia_thor] unknown decoded surface" << std::endl;
        return false;
    }
    const uint64_t readback_attempt =
        state->readback_success_count + state->readback_fail_count + 1U;
    if (shouldLogAttempt(readback_attempt)) {
        std::cerr << "[sentinel_nvmedia_thor][readback] attempt="
                  << readback_attempt
                  << " frame_num=" << frame_it->second->frame_num
                  << " surface=" << surface
                  << " coded=" << state->coded_width << "x" << state->coded_height
                  << " output=" << state->config.width << "x" << state->config.height
                  << std::endl;
    }

    NvMedia2DComposeParameters params{};
    NvMedia2DComposeResult compose_result{};
    NvSciSyncFence blit_fence{};

    if (!statusOk(NvMedia2DGetComposeParameters(state->blit2d, &params),
                  "NvMedia2DGetComposeParameters") ||
        !statusOk(NvMedia2DInsertPreNvSciSyncFence(state->blit2d, params,
                                                   &frame_it->second->eof_fence),
                  "NvMedia2DInsertPreNvSciSyncFence") ||
        !statusOk(NvMedia2DSetNvSciSyncObjforEOF(state->blit2d, params,
                                                 state->blit_eof_sync),
                  "NvMedia2DSetNvSciSyncObjforEOF") ||
        !statusOk(NvMedia2DSetSrcNvSciBufObj(state->blit2d, params, 0U, surface),
                  "NvMedia2DSetSrcNvSciBufObj") ||
        !statusOk(NvMedia2DSetDstNvSciBufObj(state->blit2d, params,
                                             state->readback_surface),
                  "NvMedia2DSetDstNvSciBufObj") ||
        !statusOk(NvMedia2DCompose(state->blit2d, params, &compose_result),
                  "NvMedia2DCompose") ||
        !statusOk(NvMedia2DGetEOFNvSciSyncFence(state->blit2d, &compose_result,
                                                &blit_fence),
                  "NvMedia2DGetEOFNvSciSyncFence")) {
        ++state->readback_fail_count;
        std::cerr << "[sentinel_nvmedia_thor][readback] 2D compose failed"
                  << " readback_fail=" << state->readback_fail_count
                  << std::endl;
        NvSciSyncFenceClear(&blit_fence);
        return false;
    }

    NvSciError wait_err = NvSciSyncFenceWait(&blit_fence,
                                             state->cpu_wait_context,
                                             kFenceWaitTimeoutUs);
    NvSciSyncFenceClear(&blit_fence);
    if (wait_err != NvSciError_Success) {
        ++state->readback_fail_count;
        std::cerr << "[sentinel_nvmedia_thor] NvSciSyncFenceWait blit failed status="
                  << wait_err
                  << " readback_fail=" << state->readback_fail_count
                  << std::endl;
        return false;
    }

    const int width = state->config.width;
    const int height = state->config.height;
    const int pitch = static_cast<int>(state->coded_width);
    const uint32_t y_pitch = state->readback_pitches[0] ? state->readback_pitches[0]
                                                        : state->coded_width;
    const uint32_t uv_pitch = state->readback_pitches[1] ? state->readback_pitches[1]
                                                         : state->coded_width;
    const size_t y_size = static_cast<size_t>(pitch) * state->coded_height;
    const size_t uv_size = y_size / 2U;
    std::vector<uint8_t> nv12(y_size + uv_size);

    void* cpu_ptr = nullptr;
    NvSciError err = NvSciBufObjGetCpuPtr(state->readback_surface, &cpu_ptr);
    if (err != NvSciError_Success) {
        ++state->readback_fail_count;
        std::cerr << "[sentinel_nvmedia_thor] NvSciBufObjGetCpuPtr failed status="
                  << err
                  << " readback_fail=" << state->readback_fail_count
                  << std::endl;
        return false;
    }
    if (!cpu_ptr) {
        ++state->readback_fail_count;
        std::cerr << "[sentinel_nvmedia_thor] NvSciBufObjGetCpuPtr returned null"
                  << " readback_fail=" << state->readback_fail_count
                  << std::endl;
        return false;
    }

    const size_t readback_span =
        state->readback_offsets[1] +
        static_cast<size_t>(uv_pitch) * (state->coded_height / 2U);
    NvSciBufObjFlushCpuCacheRange(state->readback_surface, 0U, readback_span);

    const auto* base = static_cast<const uint8_t*>(cpu_ptr);
    const auto* y_base = base + state->readback_offsets[0];
    const auto* uv_base = base + state->readback_offsets[1];
    uint8_t* y_dst = nv12.data();
    uint8_t* uv_dst = nv12.data() + y_size;
    for (uint32_t row = 0; row < state->coded_height; ++row) {
        std::memcpy(y_dst + static_cast<size_t>(row) * pitch,
                    y_base + static_cast<size_t>(row) * y_pitch,
                    state->coded_width);
    }
    for (uint32_t row = 0; row < state->coded_height / 2U; ++row) {
        std::memcpy(uv_dst + static_cast<size_t>(row) * pitch,
                    uv_base + static_cast<size_t>(row) * uv_pitch,
                    state->coded_width);
    }

    nv12ToBgr(nv12.data(), width, height, pitch, state->bgr);
    output->data = state->bgr.data();
    output->width = width;
    output->height = height;
    output->stride = width * 3;
    output->channels = 3;
    ++state->readback_success_count;
    if (shouldLogAttempt(state->readback_success_count)) {
        std::cerr << "[sentinel_nvmedia_thor][readback] success="
                  << state->readback_success_count
                  << " bgr=" << output->width << "x" << output->height
                  << " stride=" << output->stride
                  << std::endl;
    }
    return true;
}

void cleanup(SentinelNvMediaDecoder* state) {
    if (!state) {
        return;
    }
    state->quitting = true;
    state->valid_cv.notify_all();
    // This Thor header exposes NvMedia2D registration but not the unregister
    // helpers documented in some DRIVE OS versions. Destroying the 2D context
    // releases its registrations in this compatibility path.
    if (state->decoder) {
        for (uint32_t i = 0; i < state->buffer_count; ++i) {
            FrameBuffer& frame = state->buffers[i];
            if (frame.surface) {
                NvSciSyncFenceClear(&frame.eof_fence);
                NvMediaIDEUnregisterNvSciBufObj(state->decoder, frame.surface);
                NvSciBufObjFree(frame.surface);
                frame.surface = nullptr;
            }
        }
        if (state->eof_sync) {
            NvMediaIDEUnregisterNvSciSyncObj(state->decoder, state->eof_sync);
        }
        NvMediaIDEDestroy(state->decoder);
        state->decoder = nullptr;
    }
    if (state->parser) {
        NvMediaParserDestroy(state->parser);
        state->parser = nullptr;
    }
    if (state->readback_surface) {
        NvSciBufObjFree(state->readback_surface);
        state->readback_surface = nullptr;
    }
    if (state->blit2d) {
        NvMedia2DDestroy(state->blit2d);
        state->blit2d = nullptr;
    }
    if (state->blit_eof_sync) {
        NvSciSyncObjFree(state->blit_eof_sync);
        state->blit_eof_sync = nullptr;
    }
    if (state->eof_sync) {
        NvSciSyncObjFree(state->eof_sync);
        state->eof_sync = nullptr;
    }
    if (state->cpu_wait_context) {
        NvSciSyncCpuWaitContextFree(state->cpu_wait_context);
        state->cpu_wait_context = nullptr;
    }
    if (state->sync_module) {
        NvSciSyncModuleClose(state->sync_module);
        state->sync_module = nullptr;
    }
    if (state->buf_module) {
        NvSciBufModuleClose(state->buf_module);
        state->buf_module = nullptr;
    }
}

extern "C" SentinelNvMediaDecoder* sentinel_nvmedia_create(
    const SentinelNvMediaDecoderConfig* config) {
    if (!config || config->width <= 0 || config->height <= 0) {
        return nullptr;
    }

    auto* state = new (std::nothrow) SentinelNvMediaDecoder(*config);
    if (!state) {
        return nullptr;
    }
    if (!initDecoder(state)) {
        cleanup(state);
        delete state;
        return nullptr;
    }
    return state;
}

extern "C" void sentinel_nvmedia_destroy(SentinelNvMediaDecoder* handle) {
    cleanup(handle);
    delete handle;
}

extern "C" int sentinel_nvmedia_decode(SentinelNvMediaDecoder* handle,
                                        const uint8_t* data,
                                        size_t size,
                                        SentinelNvMediaBgrFrame* output) {
    if (!handle || !handle->initialized || !data || size == 0 || !output) {
        std::cerr << "[sentinel_nvmedia_thor][decode] invalid input"
                  << " handle=" << handle
                  << " initialized=" << (handle ? handle->initialized : false)
                  << " data=" << static_cast<const void*>(data)
                  << " size=" << size
                  << " output=" << output
                  << std::endl;
        return -1;
    }
    std::memset(output, 0, sizeof(*output));
    const uint64_t parse_count = ++handle->parse_count;
    const bool log_this = shouldLogAttempt(parse_count);
    if (log_this) {
        std::cerr << "[sentinel_nvmedia_thor][decode] parse_count=" << parse_count
                  << " bytes=" << size
                  << " payload=" << payloadFormatName(data, size)
                  << " callbacks_before=" << handle->decode_callback_count
                  << " displays_before=" << handle->display_callback_count
                  << " valid_queue_before=";
        {
            std::lock_guard<std::mutex> lock(handle->valid_mutex);
            std::cerr << handle->valid_buffers.size();
        }
        std::cerr << std::endl;
        logPayloadPrefix(data, size);
    }

    NvMediaBitStreamPkt packet{};
    packet.uDataLength = static_cast<uint32_t>(size);
    packet.pByteStream = data;
    packet.bEOS = false;
    packet.bPTSValid = 0;
    packet.llPts = 0;
    packet.bCompletePicture = 1;

    if (log_this) {
        std::cerr << "[sentinel_nvmedia_thor][decode] NvMediaParserParse begin"
                  << " parse_count=" << parse_count
                  << std::endl;
    }
    const auto parse_start = std::chrono::steady_clock::now();
    const NvMediaStatus parse_status = NvMediaParserParse(handle->parser, &packet);
    const auto parse_elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - parse_start).count();
    if (log_this) {
        std::cerr << "[sentinel_nvmedia_thor][decode] NvMediaParserParse ret="
                  << parse_status
                  << " parse_count=" << parse_count
                  << " elapsed_ms=" << parse_elapsed_ms
                  << std::endl;
    }
    if (parse_status != NVMEDIA_STATUS_OK) {
        ++handle->parse_fail_count;
        std::cerr << "[sentinel_nvmedia_thor][decode] NvMediaParserParse failed"
                  << " parse_count=" << parse_count
                  << " parse_fail=" << handle->parse_fail_count
                  << " bytes=" << size
                  << " payload=" << payloadFormatName(data, size)
                  << std::endl;
        return -1;
    }

    const uint64_t callbacks_before = handle->decode_callback_count;
    const uint64_t displays_before = handle->display_callback_count;
    for (int attempt = 0; attempt < kDrainAttempts; ++attempt) {
        NvSciBufObj surface = nullptr;
        {
            std::unique_lock<std::mutex> lock(handle->valid_mutex);
            if (handle->valid_buffers.empty()) {
                if (log_this && attempt == 0) {
                    std::cerr << "[sentinel_nvmedia_thor][decode] acquire frame begin"
                              << " parse_count=" << parse_count
                              << " wait_ms=" << kSleepMs
                              << " drain_attempts=" << kDrainAttempts
                              << std::endl;
                }
                handle->valid_cv.wait_for(lock, std::chrono::milliseconds(kSleepMs));
            }
            if (handle->valid_buffers.empty()) {
                if (log_this) {
                    std::cerr << "[sentinel_nvmedia_thor][decode] acquire frame ret=empty"
                              << " parse_count=" << parse_count
                              << " drain_attempt=" << attempt
                              << std::endl;
                }
                continue;
            }
            surface = handle->valid_buffers.front();
            handle->valid_buffers.pop_front();
        }
        if (log_this) {
            std::cerr << "[sentinel_nvmedia_thor][decode] got display surface"
                      << " parse_count=" << parse_count
                      << " drain_attempt=" << attempt
                      << " callbacks_delta="
                      << (handle->decode_callback_count - callbacks_before)
                      << " displays_delta="
                      << (handle->display_callback_count - displays_before)
                      << std::endl;
        }

        if (log_this) {
            std::cerr << "[sentinel_nvmedia_thor][decode] convert begin"
                      << " parse_count=" << parse_count
                      << " surface=" << surface
                      << std::endl;
        }
        const auto convert_start = std::chrono::steady_clock::now();
        const bool ok = copySurfaceToBgr(handle, surface, output);
        const auto convert_elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - convert_start).count();
        {
            std::lock_guard<std::mutex> lock(handle->free_mutex);
            handle->free_buffers.emplace_back(surface);
        }
        if (log_this) {
            std::cerr << "[sentinel_nvmedia_thor][decode] convert ret=" << ok
                      << " parse_count=" << parse_count
                      << " elapsed_ms=" << convert_elapsed_ms
                      << " output BGR=" << output->width << "x" << output->height
                      << " stride=" << output->stride
                      << " ptr=" << static_cast<void*>(output->data)
                      << std::endl;
        }
        if (!ok) {
            std::cerr << "[sentinel_nvmedia_thor][decode] readback returned no frame"
                      << " parse_count=" << parse_count
                      << " readback_fail=" << handle->readback_fail_count
                      << " readback_success=" << handle->readback_success_count
                      << std::endl;
            return -1;
        }
        return 1;
    }
    ++handle->no_output_count;
    if (log_this || shouldLogAttempt(handle->no_output_count)) {
        std::cerr << "[sentinel_nvmedia_thor][decode] no display surface after parse"
                  << " parse_count=" << parse_count
                  << " no_output=" << handle->no_output_count
                  << " callbacks_delta="
                  << (handle->decode_callback_count - callbacks_before)
                  << " displays_delta="
                  << (handle->display_callback_count - displays_before)
                  << " callbacks_total=" << handle->decode_callback_count
                  << " displays_total=" << handle->display_callback_count
                  << " decode_submit_fail=" << handle->decode_submit_fail_count
                  << std::endl;
    }
    return 0;
}

extern "C" void sentinel_nvmedia_release_frame(SentinelNvMediaDecoder* handle,
                                               SentinelNvMediaBgrFrame* frame) {
    (void)handle;
    if (frame) {
        std::memset(frame, 0, sizeof(*frame));
    }
}

extern "C" void sentinel_nvmedia_flush(SentinelNvMediaDecoder* handle) {
    if (!handle || !handle->parser) {
        return;
    }
    NvMediaParserFlush(handle->parser);
}
