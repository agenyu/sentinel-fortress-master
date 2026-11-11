// 越野堡垒模式路由管理器实现

#include "orsm_manager.h"

#include <utility>

namespace sentinel {

OrsmManager::OrsmManager() = default;

OrsmActiveMode OrsmManager::resolveMode(const OrsmSettings& settings, const OrsmOdd& odd) {
    const OrsmSettingType mode = odd.sw_mode != OrsmSettingType::OFF
                                     ? odd.sw_mode
                                     : settings.orsm_setting_st;
    switch (mode) {
    case OrsmSettingType::SINGLE_AUTO_SECURITY_ON:
        return OrsmActiveMode::SINGLE_SECURITY;
    case OrsmSettingType::MULTI_VEHICLE_JOINT_SECURITY_ON:
        return OrsmActiveMode::MULTI_JOINT_SECURITY;
    case OrsmSettingType::OFF:
    case OrsmSettingType::FUNCTION_LIMITED_ENABLE_FAILED:
    default:
        return OrsmActiveMode::NONE;
    }
}

bool OrsmManager::multiVehicleTeamUnavailable(const OrsmSettings& settings, const OrsmOdd& odd) {
    if (settings.multi_vehicle_team_setting == MultiVehicleTeamSettingEnum::TEAM_DISBANDED ||
        settings.multi_vehicle_team_req == MultiVehicleTeamReqEnum::QUIT_THE_TEAM) {
        return true;
    }
    // 队长/队员本地请求不等于 HAD 可用队伍；按文档以 TBOX/云端队伍状态作为最终依据。
    const bool has_completed_team_signal =
        settings.tbox_team_status_info == TeamStatusEnum::TEAM_UP_COMPLETED ||
        odd.tbox_team_status == TeamStatusEnum::TEAM_UP_COMPLETED ||
        odd.multi_vehicle_team_formed;
    const bool no_team_signal = settings.tbox_team_status_info == TeamStatusEnum::NO_TEAM_FORMED &&
                                odd.tbox_team_status == TeamStatusEnum::NO_TEAM_FORMED;
    return no_team_signal && !has_completed_team_signal;
}

ORSMAlertOutput OrsmManager::update(const std::vector<TrackedObject>& tracked,
                                    const ImuData& imu,
                                    const UssData& uss,
                                    const LidarFrame& lidar,
                                    const OrsmSettings& settings,
                                    const OrsmOdd& odd,
                                    uint64_t timestamp) {
    OrsmActiveMode next_mode = resolveMode(settings, odd);
    const bool was_multi_active = active_mode_ == OrsmActiveMode::MULTI_JOINT_SECURITY &&
                                  (multi_vehicle_manager_.currentState() == OrsmState::ACTIVE ||
                                   multi_vehicle_manager_.currentState() == OrsmState::WARNING);

    // 设计文档要求：多车联防队伍解散/退出后，若本车仍满足单车安防条件，自动回落到单车安防。
    const bool team_unavailable = multiVehicleTeamUnavailable(settings, odd);
    if (next_mode != OrsmActiveMode::MULTI_JOINT_SECURITY || !team_unavailable) {
        fallback_to_single_after_multi_exit_ = false;
    } else if (was_multi_active && team_unavailable) {
        fallback_to_single_after_multi_exit_ = true;
    }

    OrsmOdd routed_odd = odd;
    if (next_mode == OrsmActiveMode::MULTI_JOINT_SECURITY &&
        fallback_to_single_after_multi_exit_) {
        next_mode = OrsmActiveMode::SINGLE_SECURITY;
        routed_odd.sw_mode = OrsmSettingType::SINGLE_AUTO_SECURITY_ON;
        routed_odd.sw_enable = true;
    }

    if (next_mode != active_mode_) {
        single_manager_.reset();
        multi_vehicle_manager_.reset();
    }
    active_mode_ = next_mode;

    switch (active_mode_) {
    case OrsmActiveMode::SINGLE_SECURITY:
        return single_manager_.update(tracked, imu, uss, lidar, settings, routed_odd, timestamp);
    case OrsmActiveMode::MULTI_JOINT_SECURITY:
        return multi_vehicle_manager_.update(tracked, imu, uss, lidar, settings, routed_odd, timestamp);
    case OrsmActiveMode::NONE:
    default: {
        single_manager_.reset();
        multi_vehicle_manager_.reset();
        ORSMAlertOutput output;
        output.timestamp_ns = timestamp;
        output.orsm_setting_st_fb = OrsmSettingType::OFF;
        return output;
    }
    }
}

OrsmState OrsmManager::currentState() const {
    switch (active_mode_) {
    case OrsmActiveMode::SINGLE_SECURITY:
        return single_manager_.currentState();
    case OrsmActiveMode::MULTI_JOINT_SECURITY:
        return multi_vehicle_manager_.currentState();
    case OrsmActiveMode::NONE:
    default:
        return OrsmState::OFF;
    }
}

void OrsmManager::setStateCallback(OrsmStateCallback cb) {
    single_manager_.setStateCallback(cb);
    multi_vehicle_manager_.setStateCallback(std::move(cb));
}

void OrsmManager::resetTracking() {
    single_manager_.reset();
    multi_vehicle_manager_.reset();
    active_mode_ = OrsmActiveMode::NONE;
    fallback_to_single_after_multi_exit_ = false;
}

}  // namespace sentinel
