// 多车联防越野堡垒管理器实现

#include "multi_vehicle_orsm_manager.h"

#include <algorithm>
#include <cmath>

namespace sentinel {

MultiVehicleOrsmManager::MultiVehicleOrsmManager() = default;

bool MultiVehicleOrsmManager::teamFormed(const OrsmSettings& settings, const OrsmOdd& odd) {
    // 设计文档中队长/队员请求由 IVI 发给 TBOX，HAD 以 TBOX/云端队伍状态作为最终组队依据。
    // 因此 TEAM_FINISHED / ADD_THE_TEAM 只代表用户侧请求，不单独触发多车联防激活。
    return odd.multi_vehicle_team_formed ||
           settings.tbox_team_status_info == TeamStatusEnum::TEAM_UP_COMPLETED ||
           odd.tbox_team_status == TeamStatusEnum::TEAM_UP_COMPLETED;
}

bool MultiVehicleOrsmManager::teamDisbanded(const OrsmSettings& settings, const OrsmOdd& odd) {
    if (settings.multi_vehicle_team_setting == MultiVehicleTeamSettingEnum::TEAM_DISBANDED ||
        settings.multi_vehicle_team_req == MultiVehicleTeamReqEnum::QUIT_THE_TEAM) {
        return true;
    }
    const bool has_completed_team_signal = teamFormed(settings, odd);
    const bool no_team_signal = settings.tbox_team_status_info == TeamStatusEnum::NO_TEAM_FORMED &&
                                odd.tbox_team_status == TeamStatusEnum::NO_TEAM_FORMED;
    return no_team_signal && !has_completed_team_signal;
}

bool MultiVehicleOrsmManager::iviPrepared(const OrsmSettings& settings, const OrsmOdd& odd) {
    return odd.ivi_prepared ||
           settings.multi_vehicle_team_prepared == MultiVehicleTeamPreparedEnum::PREPARED;
}

bool MultiVehicleOrsmManager::multiVehicleBaseOddReady(const OrsmSettings& settings,
                                                       const OrsmOdd& odd) {
    return odd.power_on &&
           odd.gear_park &&
           odd.soc >= settings.multi_vehicle_soc_entry_min &&
           odd.speed_kmh <= settings.multi_vehicle_speed_entry_max_kmh &&
           !odd.had_fault &&
           !odd.sensor_fault &&
           (!settings.multi_vehicle_enable_network_check || odd.network_ok);
}

bool MultiVehicleOrsmManager::multiVehicleEntryReady(const OrsmSettings& settings,
                                                     const OrsmOdd& odd) {
    return odd.sw_mode == OrsmSettingType::MULTI_VEHICLE_JOINT_SECURITY_ON &&
           odd.sw_enable &&
           multiVehicleBaseOddReady(settings, odd) &&
           teamFormed(settings, odd) &&
           iviPrepared(settings, odd) &&
           odd.multi_vehicle_team_ready &&
           odd.vehicle_stationary_over_60s &&
           odd.external_locked;
}

bool MultiVehicleOrsmManager::multiVehicleExitRequired(const OrsmSettings& settings,
                                                       const OrsmOdd& odd) {
    return !odd.sw_enable ||
           odd.sw_mode != OrsmSettingType::MULTI_VEHICLE_JOINT_SECURITY_ON ||
           odd.had_fault ||
           odd.sensor_fault ||
           !odd.gear_park ||
           odd.soc < settings.multi_vehicle_soc_exit_min ||
           odd.speed_kmh > settings.multi_vehicle_speed_exit_max_kmh ||
           (settings.multi_vehicle_enable_network_check && !odd.network_ok) ||
           teamDisbanded(settings, odd);
}

OrsmMessageReminder MultiVehicleOrsmManager::multiVehicleBlockReason(const OrsmSettings& settings,
                                                                     const OrsmOdd& odd,
                                                                     bool was_active) {
    if (!odd.power_on) {
        return OrsmMessageReminder::NOT_IN_ON_READY_STATE_FUNCTION_ENTRY_FAILED;
    }
    if (!odd.gear_park) {
        return OrsmMessageReminder::GEAR_NOT_IN_P_POSITION_FUNCTION_ENTRY_FAILED;
    }
    if (odd.had_fault) {
        return OrsmMessageReminder::INTELLIGENT_DRIVING_SYSTEM_MALFUNCTION_FUNCTION_ENTRY_FAILED;
    }
    if (odd.sensor_fault) {
        return OrsmMessageReminder::RADAR_CAMERA_MALFUNCTION_FUNCTION_ENTRY_FAILED;
    }
    if (settings.multi_vehicle_enable_network_check && !odd.network_ok) {
        return OrsmMessageReminder::NETWORK_CONNECTION_DISCONNECTED_JOINT_SECURITY_FUNCTION_EXITED;
    }
    if (odd.speed_kmh > settings.multi_vehicle_speed_exit_max_kmh) {
        return OrsmMessageReminder::VEHICLE_SPEED_TOO_HIGH_JOINT_SECURITY_FUNCTION_EXITED;
    }
    if (!was_active && odd.speed_kmh > settings.multi_vehicle_speed_entry_max_kmh) {
        return OrsmMessageReminder::VEHICLE_SPEED_TOO_HIGH_FUNCTION_CANNOT_BE_ACTIVATED;
    }
    if (odd.soc < settings.multi_vehicle_soc_exit_min) {
        return OrsmMessageReminder::LOW_BATTERY_FUNCTION_EXITED;
    }
    if (!was_active && odd.soc < settings.multi_vehicle_soc_entry_min) {
        return OrsmMessageReminder::LOW_BATTERY_FUNCTION_ENTRY_FAILED;
    }
    if (!was_active &&
        (!teamFormed(settings, odd) ||
         !iviPrepared(settings, odd) ||
         !odd.multi_vehicle_team_ready ||
         !odd.vehicle_stationary_over_60s ||
         !odd.external_locked)) {
        return OrsmMessageReminder::MULTI_VEHICLE_JOINT_SECURITY_FUNCTION_EXITED;
    }
    return was_active ? OrsmMessageReminder::MULTI_VEHICLE_JOINT_SECURITY_FUNCTION_EXITED
                      : OrsmMessageReminder::ORSM_NONE;
}

std::vector<TrackedObject> MultiVehicleOrsmManager::filterSecuritySideTargets(
    const std::vector<TrackedObject>& tracked,
    const OrsmSettings& settings,
    int image_width) {
    std::vector<TrackedObject> filtered;
    filtered.reserve(tracked.size());

    const int side = (settings.multi_vehicle_security_side > 0) ? 1
                   : (settings.multi_vehicle_security_side < 0 ? -1 : 0);
    const float no_alarm_radius = std::max(0.0f, settings.multi_vehicle_no_alarm_radius);

    for (const auto& obj : tracked) {
        float distance = obj.distance_m;
        if (distance <= 0.f && obj.bbox.height > 0) {
            distance = 6120.0f / static_cast<float>(obj.bbox.height);
        }
        if (distance > 0.f && distance <= no_alarm_radius) {
            continue;
        }

        if (side != 0) {
            float lateral = 0.f;
            bool has_lateral = false;
            if (obj.has_lidar_fusion) {
                lateral = obj.position_3d.y;
                has_lateral = true;
            } else if (image_width > 0 && obj.bbox.width > 0) {
                const float cx = obj.bbox.x + obj.bbox.width * 0.5f;
                lateral = cx - static_cast<float>(image_width) * 0.5f;
                has_lateral = true;
            }
            if (has_lateral && lateral * static_cast<float>(side) < 0.f) {
                continue;
            }
        }

        filtered.push_back(obj);
    }
    return filtered;
}

OrsmSettings MultiVehicleOrsmManager::multiVehicleRangeSettings(const OrsmSettings& settings) {
    OrsmSettings routed = settings;
    routed.zone_a_radius = settings.multi_vehicle_external_radius > 0.f
                               ? settings.multi_vehicle_external_radius
                               : settings.zone_a_radius;
    routed.zone_b_radius = settings.multi_vehicle_internal_radius > 0.f
                               ? settings.multi_vehicle_internal_radius
                               : settings.zone_b_radius;
    // 多车模式文档要求 5m 内为不报警区；过滤后不再使用 C 区三级报警。
    routed.zone_c_radius = 0.01f;
    return routed;
}

void MultiVehicleOrsmManager::normalizeExternalZoneTypes(ORSMAlertOutput& output,
                                                         const OrsmSettings& settings) {
    const float internal_radius = settings.multi_vehicle_internal_radius > 0.f
                                      ? settings.multi_vehicle_internal_radius
                                      : settings.zone_b_radius;
    for (auto& intruder : output.intruders_info) {
        if (intruder.distance > internal_radius) {
            // 外部区仅雷达可稳定检测，按设计不区分人/动物。
            intruder.intruder_type = IntruderType::INTRUDER_UNKNOWN;
        }
    }
}

ORSMAlertOutput MultiVehicleOrsmManager::update(const std::vector<TrackedObject>& tracked,
                                                const ImuData& imu,
                                                const UssData& uss,
                                                const LidarFrame& lidar,
                                                const OrsmSettings& settings,
                                                const OrsmOdd& odd,
                                                uint64_t timestamp) {
    const bool was_active = single_manager_.currentState() == OrsmState::ACTIVE ||
                            single_manager_.currentState() == OrsmState::WARNING;

    OrsmOdd multi_odd = odd;
    multi_odd.sw_mode = OrsmSettingType::MULTI_VEHICLE_JOINT_SECURITY_ON;

    if (was_active) {
        multi_odd.sw_enable = !multiVehicleExitRequired(settings, odd);
    } else {
        multi_odd.sw_enable = multiVehicleEntryReady(settings, odd);
    }

    const auto routed_settings = multiVehicleRangeSettings(settings);
    const auto security_targets = filterSecuritySideTargets(tracked, settings);
    auto output = single_manager_.update(security_targets, imu, uss, lidar,
                                         routed_settings, multi_odd, timestamp);
    normalizeExternalZoneTypes(output, settings);

    // 多车模式下，检测链路复用单车实现，但目标先裁剪为安防侧，且 5m 内不报警；状态反馈与文字提醒按多车联防 ODD 重写。
    const OrsmState state = single_manager_.currentState();
    const bool multi_requested = odd.sw_enable &&
                                 odd.sw_mode == OrsmSettingType::MULTI_VEHICLE_JOINT_SECURITY_ON;
    output.orsm_setting_st_fb = (state == OrsmState::FAILURE)
                                    ? OrsmSettingType::FUNCTION_LIMITED_ENABLE_FAILED
                                    : (state == OrsmState::ACTIVE || state == OrsmState::WARNING
                                           ? OrsmSettingType::MULTI_VEHICLE_JOINT_SECURITY_ON
                                           : (multi_requested
                                                  ? OrsmSettingType::FUNCTION_LIMITED_ENABLE_FAILED
                                                  : OrsmSettingType::OFF));

    if (state == OrsmState::ACTIVE || state == OrsmState::WARNING) {
        output.orsm_message_rem = OrsmMessageReminder::SECURITY_MODE_ON;
    } else if (multi_requested) {
        output.orsm_message_rem = multiVehicleBlockReason(settings, odd, was_active);
    }

    // OrsmStatus 目前没有独立的多车联防报警等级字段，复用 ORSMWarnLevel 承载多车联防等级。
    return output;
}

}  // namespace sentinel
