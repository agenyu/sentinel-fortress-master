#pragma once

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct SentinelNvMediaDecoder SentinelNvMediaDecoder;

typedef struct SentinelNvMediaDecoderConfig {
    int width;
    int height;
    int device_id;
    int verbose;
} SentinelNvMediaDecoderConfig;

typedef struct SentinelNvMediaBgrFrame {
    uint8_t* data;
    int width;
    int height;
    int stride;
    int channels;
} SentinelNvMediaBgrFrame;

typedef SentinelNvMediaDecoder* (*sentinel_nvmedia_create_fn)(
    const SentinelNvMediaDecoderConfig* config);
typedef void (*sentinel_nvmedia_destroy_fn)(SentinelNvMediaDecoder* decoder);
// Decode return contract:
//   1  = one valid BGR frame is available in output
//   0  = input was accepted but no output frame is ready yet
//  <0  = decode/timeout/error condition; caller should log and may reset
typedef int (*sentinel_nvmedia_decode_fn)(
    SentinelNvMediaDecoder* decoder,
    const uint8_t* data,
    size_t size,
    SentinelNvMediaBgrFrame* output);
typedef void (*sentinel_nvmedia_release_frame_fn)(
    SentinelNvMediaDecoder* decoder,
    SentinelNvMediaBgrFrame* frame);
typedef void (*sentinel_nvmedia_flush_fn)(SentinelNvMediaDecoder* decoder);

#ifdef __cplusplus
}
#endif
