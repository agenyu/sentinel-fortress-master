#include "orsm/single_orsm_manager.h"

#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

namespace {

using namespace sentinel;

void expectEqual(const std::string& name, int actual, int expected) {
    if (actual == expected) return;
    std::cerr << name << ": expected " << expected << ", got " << actual << std::endl;
    std::exit(1);
}

OrsmOdd readyOdd() {
    OrsmOdd odd;
    odd.power_on = true;
    odd.gear_park = true;
    odd.soc = 80.0f;
    odd.speed_kmh = 0.0f;
    odd.handbrake_on = true;
    odd.sw_enable = true;
    odd.sw_mode = OrsmSettingType::SINGLE_AUTO_SECURITY_ON;
    odd.network_ok = true;
    odd.external_locked = true;
    return odd;
}

TrackedObject lidarBackedPerson(float distance_m, float velocity_x = 0.0f) {
    TrackedObject obj;
    obj.track_id = 1;
    obj.class_id = 0;
    obj.label = "person";
    obj.confidence = 0.9f;
    obj.age = 100;
    obj.lost = 0;
    obj.has_lidar_fusion = true;
    obj.position_3d = cv::Point3f(distance_m, 0.0f, 0.0f);
    obj.velocity_3d = cv::Point3f(velocity_x, 0.0f, 0.0f);
    obj.distance_m = distance_m;
    return obj;
}

ORSMAlertOutput runConstantDistance(SingleOrsmManager& manager,
                                    const OrsmSettings& settings,
                                    float distance_m,
                                    int frames,
                                    uint64_t& timestamp_ns) {
    const ImuData imu;
    const UssData uss;
    const LidarFrame lidar;
    const auto odd = readyOdd();
    ORSMAlertOutput output;
    for (int i = 0; i < frames; ++i) {
        timestamp_ns += 100000000ULL;
        output = manager.update({lidarBackedPerson(distance_m)}, imu, uss, lidar,
                                settings, odd, timestamp_ns);
    }
    return output;
}

void expectWarning(const std::string& name,
                   const ORSMAlertOutput& output,
                   OrsmWarnLevel level) {
    expectEqual(name + " warning status",
                static_cast<int>(output.orsm_warning_st),
                static_cast<int>(OrsmWarningStatus::WARNING_ON));
    expectEqual(name + " warning level",
                static_cast<int>(output.orsm_warn_level),
                static_cast<int>(level));
    expectEqual(name + " intruder count", static_cast<int>(output.intruders_info.size()), 1);
}

}  // namespace

int main() {
    OrsmSettings settings;
    settings.detec_range_setting = DetecRangeSetting::RANGE_A;
    settings.zone_a_radius = 30.0f;
    settings.zone_b_radius = 15.0f;
    settings.zone_c_radius = 5.0f;

    {
        SingleOrsmManager manager;
        uint64_t timestamp_ns = 0;
        const auto output = runConstantDistance(manager, settings, 4.0f, 10, timestamp_ns);
        expectWarning("C-zone", output, OrsmWarnLevel::WARNLEVEL_3);
    }

    {
        SingleOrsmManager manager;
        uint64_t timestamp_ns = 0;
        const auto output = runConstantDistance(manager, settings, 10.0f, 10, timestamp_ns);
        expectWarning("B-zone", output, OrsmWarnLevel::WARNLEVEL_2);
    }

    {
        SingleOrsmManager manager;
        uint64_t timestamp_ns = 0;
        const auto output = runConstantDistance(manager, settings, 18.0f, 12, timestamp_ns);
        expectWarning("inner A-zone", output, OrsmWarnLevel::WARNLEVEL_1);
    }

    {
        SingleOrsmManager manager;
        uint64_t timestamp_ns = 0;
        const auto early = runConstantDistance(manager, settings, 25.0f, 59, timestamp_ns);
        expectEqual("outer A-zone before stable confirmation",
                    static_cast<int>(early.orsm_warning_st),
                    static_cast<int>(OrsmWarningStatus::WARNING_OFF));
        const auto confirmed = runConstantDistance(manager, settings, 25.0f, 1, timestamp_ns);
        expectWarning("outer A-zone confirmed", confirmed, OrsmWarnLevel::WARNLEVEL_1);

        const ImuData imu;
        const UssData uss;
        const LidarFrame lidar;
        timestamp_ns += 100000000ULL;
        const auto moving_away = manager.update({lidarBackedPerson(25.4f, 0.4f)},
                                                imu, uss, lidar, settings, readyOdd(),
                                                timestamp_ns);
        expectEqual("A-zone moving-away suppression",
                    static_cast<int>(moving_away.orsm_warning_st),
                    static_cast<int>(OrsmWarningStatus::WARNING_OFF));
    }

    {
        SingleOrsmManager manager;
        uint64_t timestamp_ns = 0;
        OrsmSettings b_only = settings;
        b_only.detec_range_setting = DetecRangeSetting::RANGE_B;
        const auto output = runConstantDistance(manager, b_only, 18.0f, 20, timestamp_ns);
        expectEqual("RANGE_B excludes A-zone",
                    static_cast<int>(output.orsm_warning_st),
                    static_cast<int>(OrsmWarningStatus::WARNING_OFF));
    }

    std::cout << "orsm_zone_behavior_test: PASS" << std::endl;
    return 0;
}
