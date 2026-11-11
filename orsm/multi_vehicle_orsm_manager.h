#pragma once
// 多车联防越野堡垒管理器
// 负责多车联防 ODD 闭环，三区入侵识别复用单车管理器。

#include "single_orsm_manager.h"

namespace sentinel {

class MultiVehicleOrsmManager {
public:
    MultiVehicleOrsmManager();

    ORSMAlertOutput update(const std::vector<TrackedObject>& tracked,
                           const ImuData& imu,
                           const UssData& uss,
                           const LidarFrame& lidar,
                           const OrsmSettings& settings,
                           const OrsmOdd& odd,
                           uint64_t timestamp);

    OrsmState currentState() const { return single_manager_.currentState(); }
    void setStateCallback(OrsmStateCallback cb) { single_manager_.setStateCallback(std::move(cb)); }
    void resetTracking() { single_manager_.resetTracking(); }
    void reset() { single_manager_.reset(); }

private:
    static bool teamFormed(const OrsmSettings& settings, const OrsmOdd& odd);
    static bool teamDisbanded(const OrsmSettings& settings, const OrsmOdd& odd);
    static bool iviPrepared(const OrsmSettings& settings, const OrsmOdd& odd);
    static bool multiVehicleBaseOddReady(const OrsmSettings& settings, const OrsmOdd& odd);
    static bool multiVehicleEntryReady(const OrsmSettings& settings, const OrsmOdd& odd);
    static bool multiVehicleExitRequired(const OrsmSettings& settings, const OrsmOdd& odd);
    static OrsmMessageReminder multiVehicleBlockReason(const OrsmSettings& settings,
                                                       const OrsmOdd& odd,
                                                       bool was_active);
    static std::vector<TrackedObject> filterSecuritySideTargets(const std::vector<TrackedObject>& tracked,
                                                                const OrsmSettings& settings,
                                                                int image_width = 1920);
    static OrsmSettings multiVehicleRangeSettings(const OrsmSettings& settings);
    static void normalizeExternalZoneTypes(ORSMAlertOutput& output, const OrsmSettings& settings);

    SingleOrsmManager single_manager_;
};

}  // namespace sentinel
