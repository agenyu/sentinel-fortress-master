#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

struct HevcDecoderConfig {
    int width = 0;
    int height = 0;
    int gpu_id = 0;
    bool verbose = false;
};

// Metadata travels through the decoder with the compressed access unit.  A
// decoder may emit an older access unit than the one supplied by the current
// decode() call because HEVC commonly buffers/reorders frames.
struct HevcPacketMetadata {
    uint64_t seq{0};
    uint64_t timestamp_ns{0};
    int frame_type{0};
};

struct HevcDecodedOutput {
    cv::Mat image;
    HevcPacketMetadata metadata;
    bool has_metadata{false};
};

class IHevcDecoder {
public:
    virtual ~IHevcDecoder() = default;

    virtual bool init(const HevcDecoderConfig& config) = 0;

    virtual bool decode(const uint8_t* data, size_t size,
                        cv::Mat& output_frame, int target_frame_index = -1) = 0;

    virtual bool decode(const std::vector<uint8_t>& data,
                        cv::Mat& output_frame, int target_frame_index = -1) {
        return decode(data.data(), data.size(), output_frame, target_frame_index);
    }

    // Backends without delayed output can use this default implementation.
    // Buffered backends override it and return the metadata belonging to the
    // frame that was actually emitted.
    virtual bool decodeWithMetadata(const uint8_t* data, size_t size,
                                    const HevcPacketMetadata& input_metadata,
                                    HevcDecodedOutput& output,
                                    int target_frame_index = -1) {
        output = {};
        if (!decode(data, size, output.image, target_frame_index)) return false;
        output.metadata = input_metadata;
        output.has_metadata = true;
        return true;
    }

    virtual bool decodeWithMetadata(const std::vector<uint8_t>& data,
                                    const HevcPacketMetadata& input_metadata,
                                    HevcDecodedOutput& output,
                                    int target_frame_index = -1) {
        return decodeWithMetadata(data.data(), data.size(), input_metadata,
                                  output, target_frame_index);
    }

    virtual bool isInitialized() const = 0;
    virtual const char* backendName() const = 0;

    virtual void flush() = 0;
    virtual void reset() = 0;
};
