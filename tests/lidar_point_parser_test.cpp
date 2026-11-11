#include "lidar/lidar_point_parser.h"

#include <array>
#include <cassert>
#include <cstring>
#include <iostream>

namespace {
template <typename T>
void put(std::array<uint8_t, 16>& data, size_t offset, T value) {
    std::memcpy(data.data() + offset, &value, sizeof(value));
}
}

int main() {
    using namespace sentinel;

    std::array<uint8_t, 16> robosense{};
    put<int16_t>(robosense, 0, 123);
    put<int16_t>(robosense, 2, -456);
    put<int16_t>(robosense, 4, 789);
    put<int32_t>(robosense, 8, -123456789);
    robosense[12] = 200;
    robosense[13] = 128;

    DecodedLidarPoint16 point;
    assert(decodeLidarPoint16(robosense.data(), robosense.size(),
                              LidarPointLayout::ROBOSENSE_ALIGNED16, point));
    assert(point.x_cm == 123 && point.y_cm == -456 && point.z_cm == 789);
    assert(point.time_offset_ns == -123456789);
    assert(point.intensity == 200 && point.ring == 128);

    std::array<uint8_t, 16> blind{};
    put<int16_t>(blind, 0, -321);
    put<int16_t>(blind, 2, 654);
    put<int16_t>(blind, 4, -987);
    put<int32_t>(blind, 6, 23456789);
    blind[10] = 250;
    blind[11] = 64;

    assert(decodeLidarPoint16(blind.data(), blind.size(),
                              LidarPointLayout::BLIND_SPOT16, point));
    assert(point.x_cm == -321 && point.y_cm == 654 && point.z_cm == -987);
    assert(point.time_offset_ns == 23456789);
    assert(point.intensity == 250 && point.ring == 64);
    assert(!decodeLidarPoint16(blind.data(), 15,
                               LidarPointLayout::BLIND_SPOT16, point));

    std::cout << "lidar_point_parser_test: PASS" << std::endl;
    return 0;
}
