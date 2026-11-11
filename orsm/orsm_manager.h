#pragma once
// 越野堡垒模式路由管理器
// 对外保持 OrsmManager API，内部按 OrsmSettingType 路由到单车/多车实现。

#include "single_orsm_manager.h"
#include "multi_vehicle_orsm_manager.h"

namespace sentinel {

enum class OrsmActiveMode : uint8_t {
    NONE = 0,
    SINGLE_SECURITY = 1,
    MULTI_JOINT_SECURITY = 2,
};

class OrsmManager {
public:
    OrsmManager();

    ORSMAlertOutput update(const std::vector<TrackedObject>& tracked,
                           const ImuData& imu,
                           const UssData& uss,
                           const LidarFrame& lidar,
                           const OrsmSettings& settings,
                           const OrsmOdd& odd,
                           uint64_t timestamp);

    OrsmState currentState() const;
    OrsmActiveMode activeMode() const { return active_mode_; }
    void setStateCallback(OrsmStateCallback cb);
    void resetTracking();

private:
    static OrsmActiveMode resolveMode(const OrsmSettings& settings, const OrsmOdd& odd);
    static bool multiVehicleTeamUnavailable(const OrsmSettings& settings, const OrsmOdd& odd);

    SingleOrsmManager single_manager_;
    MultiVehicleOrsmManager multi_vehicle_manager_;
    OrsmActiveMode active_mode_{OrsmActiveMode::NONE};
    bool fallback_to_single_after_multi_exit_{false};
};

}  // namespace sentinel
