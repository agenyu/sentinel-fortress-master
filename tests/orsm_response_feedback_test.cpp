#include "orsm/multi_vehicle_orsm_manager.h"
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

OrsmOdd readySingleOdd(OrsmSettingType mode) {
    OrsmOdd odd;
    odd.power_on = true;
    odd.gear_park = true;
    odd.soc = 80.0f;
    odd.speed_kmh = 0.0f;
    odd.handbrake_on = true;
    odd.sw_enable = true;
    odd.sw_mode = mode;
    odd.network_ok = true;
    odd.external_locked = true;
    return odd;
}

}  // namespace

int main() {
    const std::vector<sentinel::TrackedObject> tracked;
    const sentinel::ImuData imu;
    const sentinel::UssData uss;
    const sentinel::LidarFrame lidar;

    {
        sentinel::SingleOrsmManager manager;
        sentinel::OrsmSettings settings;
        auto odd = readySingleOdd(sentinel::OrsmSettingType::SINGLE_AUTO_SECURITY_ON);
        odd.gear_park = false;
        const auto output = manager.update(tracked, imu, uss, lidar, settings, odd, 1);
        expectEqual("single requested but blocked",
                    static_cast<int>(output.orsm_setting_st_fb), 3);
    }

    {
        sentinel::SingleOrsmManager manager;
        sentinel::OrsmSettings settings;
        auto odd = readySingleOdd(sentinel::OrsmSettingType::SINGLE_AUTO_SECURITY_ON);
        odd.sw_enable = false;
        odd.sw_mode = sentinel::OrsmSettingType::OFF;
        const auto output = manager.update(tracked, imu, uss, lidar, settings, odd, 2);
        expectEqual("single not requested",
                    static_cast<int>(output.orsm_setting_st_fb), 0);
    }

    {
        sentinel::SingleOrsmManager manager;
        sentinel::OrsmSettings settings;
        auto odd = readySingleOdd(sentinel::OrsmSettingType::SINGLE_AUTO_SECURITY_ON);
        const auto output = manager.update(tracked, imu, uss, lidar, settings, odd, 3);
        expectEqual("single active",
                    static_cast<int>(output.orsm_setting_st_fb), 1);
    }

    {
        sentinel::MultiVehicleOrsmManager manager;
        sentinel::OrsmSettings settings;
        settings.multi_vehicle_enable_network_check = true;
        auto odd = readySingleOdd(
            sentinel::OrsmSettingType::MULTI_VEHICLE_JOINT_SECURITY_ON);
        const auto output = manager.update(tracked, imu, uss, lidar, settings, odd, 4);
        expectEqual("multi requested but team not ready",
                    static_cast<int>(output.orsm_setting_st_fb), 3);
        expectEqual("multi blocked reminder",
                    static_cast<int>(output.orsm_message_rem), 0xA);
    }

    {
        sentinel::MultiVehicleOrsmManager manager;
        sentinel::OrsmSettings settings;
        settings.multi_vehicle_enable_network_check = true;
        settings.multi_vehicle_team_prepared =
            sentinel::MultiVehicleTeamPreparedEnum::PREPARED;
        settings.tbox_team_status_info = sentinel::TeamStatusEnum::TEAM_UP_COMPLETED;
        auto odd = readySingleOdd(
            sentinel::OrsmSettingType::MULTI_VEHICLE_JOINT_SECURITY_ON);
        odd.multi_vehicle_team_ready = true;
        odd.multi_vehicle_team_formed = true;
        odd.vehicle_stationary_over_60s = true;
        odd.ivi_prepared = true;
        odd.tbox_team_status = sentinel::TeamStatusEnum::TEAM_UP_COMPLETED;
        settings.multi_vehicle_team_setting =
            sentinel::MultiVehicleTeamSettingEnum::TEAM_FINISHED;
        settings.multi_vehicle_team_req = sentinel::MultiVehicleTeamReqEnum::ADD_THE_TEAM;
        auto output = manager.update(tracked, imu, uss, lidar, settings, odd, 5);
        // Managers own runtime state; SentinelNode merges settings feedback.
        // Exercise the same merge before checking the reserved team fields.
        sentinel::applyOrsmSettingsFeedback(output, settings);
        expectEqual("multi active",
                    static_cast<int>(output.orsm_setting_st_fb), 2);
        expectEqual("reserved size",
                    static_cast<int>(output.orsm_info_reserved.size()), 4);
        expectEqual("reserved team setting",
                    static_cast<int>(output.orsm_info_reserved[0]), 2);
        expectEqual("reserved team request",
                    static_cast<int>(output.orsm_info_reserved[1]), 1);
        expectEqual("reserved team prepared",
                    static_cast<int>(output.orsm_info_reserved[2]), 1);
        expectEqual("reserved tbox status",
                    static_cast<int>(output.orsm_info_reserved[3]), 2);
    }

    {
        sentinel::ORSMAlertOutput cached;
        cached.orsm_message_rem = sentinel::OrsmMessageReminder::SECURITY_MODE_ON;
        sentinel::applyOrsmEnableMessageFeedback(
            cached, true, sentinel::OrsmMessageReminder::ORSM_NONE);
        expectEqual("successful settings refresh preserves running reminder",
                    static_cast<int>(cached.orsm_message_rem), 1);
        sentinel::applyOrsmEnableMessageFeedback(
            cached, false,
            sentinel::OrsmMessageReminder::GEAR_NOT_IN_P_POSITION_FUNCTION_ENTRY_FAILED);
        expectEqual("failed settings refresh updates failure reminder",
                    static_cast<int>(cached.orsm_message_rem), 4);
    }

    std::cout << "orsm_response_feedback_test: PASS" << std::endl;
    return 0;
}
