#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>

namespace sentinel {

// RSCL 点云单点固定占 16 字节，但主雷达与补盲雷达的字段偏移不同。
enum class LidarPointLayout : uint8_t {
    ROBOSENSE_ALIGNED16, // x@0,y@2,z@4,pad@6,time@8,intensity@12,ring@13
    BLIND_SPOT16,        // x@0,y@2,z@4,time@6,intensity@10,ring@11,pad@12
};

struct DecodedLidarPoint16 {
    int16_t x_cm{0};
    int16_t y_cm{0};
    int16_t z_cm{0};
    int32_t time_offset_ns{0};
    uint8_t intensity{0};
    uint8_t ring{0};
};

inline LidarPointLayout lidarPointLayoutFromString(const std::string& value) {
    if (value == "robosense_aligned16") return LidarPointLayout::ROBOSENSE_ALIGNED16;
    if (value == "blind_spot16") return LidarPointLayout::BLIND_SPOT16;
    throw std::invalid_argument("Unsupported LiDAR point layout: " + value);
}

inline const char* lidarPointLayoutName(LidarPointLayout layout) {
    switch (layout) {
    case LidarPointLayout::ROBOSENSE_ALIGNED16: return "robosense_aligned16";
    case LidarPointLayout::BLIND_SPOT16: return "blind_spot16";
    }
    return "unknown";
}

inline bool decodeLidarPoint16(const uint8_t* data,
                               size_t available_bytes,
                               LidarPointLayout layout,
                               DecodedLidarPoint16& point) {
    constexpr size_t kPointStep = 16;
    if (data == nullptr || available_bytes < kPointStep) return false;

    std::memcpy(&point.x_cm, data + 0, sizeof(point.x_cm));
    std::memcpy(&point.y_cm, data + 2, sizeof(point.y_cm));
    std::memcpy(&point.z_cm, data + 4, sizeof(point.z_cm));

    size_t time_offset = 6;
    size_t intensity_offset = 10;
    size_t ring_offset = 11;
    if (layout == LidarPointLayout::ROBOSENSE_ALIGNED16) {
        time_offset = 8;
        intensity_offset = 12;
        ring_offset = 13;
    }
    std::memcpy(&point.time_offset_ns, data + time_offset,
                sizeof(point.time_offset_ns));
    point.intensity = data[intensity_offset];
    point.ring = data[ring_offset];
    return true;
}

}  // namespace sentinel
