#include "alert/alert_manager.h"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

namespace {

using namespace sentinel;

void expectLevel(const std::string& name, const AlertManager& manager, ThreatLevel expected) {
    if (manager.currentLevel() == expected) return;
    std::cout << name << ": expected level=" << static_cast<int>(expected)
              << ", got=" << static_cast<int>(manager.currentLevel()) << std::endl;
    std::exit(1);
}

TrackedObject visualPerson(float calibrated_distance_m, int age = 10, int track_id = 1) {
    TrackedObject obj;
    obj.track_id = track_id;
    obj.class_id = 0;
    obj.label = "person";
    obj.bbox = cv::Rect(700, 100, 300, 1000); // bbox fallback would report a near target
    obj.confidence = 0.95f;
    obj.age = age;
    obj.lost = 0;
    obj.has_visual_ground_position = true;
    obj.visual_ground_position_3d = cv::Point3f(calibrated_distance_m, 0.f, 0.f);
    obj.visual_ground_distance_m = calibrated_distance_m;
    obj.position_3d = obj.visual_ground_position_3d;
    obj.distance_m = calibrated_distance_m;
    obj.origin = ObservationOrigin::OriginYolo;
    obj.source_camera = "front_camera_fov195";
    return obj;
}

TrackedObject visualVehicleAt(float x_m, float y_m, int age, int track_id = 2) {
    TrackedObject obj = visualPerson(std::hypot(x_m, y_m), age, track_id);
    obj.class_id = 2;
    obj.label = "vehicle";
    obj.bbox = cv::Rect(600, 500, 160, 120);
    obj.visual_ground_position_3d = cv::Point3f(x_m, y_m, 0.f);
    obj.position_3d = obj.visual_ground_position_3d;
    obj.source_camera = "left_camera_fov195";
    return obj;
}

TrackedObject visualCycleAt(float distance_m, int age, int track_id = 3) {
    TrackedObject obj = visualPerson(distance_m, age, track_id);
    obj.class_id = 1;
    obj.label = "bicycle";
    obj.bbox = cv::Rect(600, 500, 160, 220);
    obj.source_camera = "right_camera_fov195";
    return obj;
}

TrackedObject visualOther(float calibrated_distance_m, int age, int track_id = 4) {
    TrackedObject obj = visualPerson(calibrated_distance_m, age, track_id);
    obj.class_id = 99;
    obj.label = "other";
    return obj;
}

TrackedObject nearFieldContactMotion(int age = 3, int track_id = 42) {
    TrackedObject obj = visualPerson(0.8f, age, track_id);
    obj.class_id = -10;
    obj.label = "near_field_contact_motion";
    obj.bbox = cv::Rect(0, 0, 860, 1536);
    obj.confidence = 0.8f;
    obj.origin = ObservationOrigin::OriginSentryNearMotion;
    obj.source_camera = "left_camera_fov195";
    return obj;
}

TrackedObject passingMotionObject(int age = 3, int track_id = 46) {
    TrackedObject obj = visualPerson(2.5f, age, track_id);
    obj.class_id = -11;
    obj.label = "passing_motion_object";
    obj.bbox = cv::Rect(40, 720, 140, 90);
    obj.confidence = 0.6f;
    obj.origin = ObservationOrigin::OriginSentryRoadMotion;
    obj.source_camera = "rear_camera_fov195";
    obj.visual_ground_position_3d = cv::Point3f(-2.5f, 0.f, 0.f);
    obj.position_3d = obj.visual_ground_position_3d;
    return obj;
}

void evaluateRanges(AlertManager& manager,
                    const std::vector<float>& ranges,
                    uint64_t& timestamp_ns,
                    int age_start = 10) {
    const ImuData imu;
    const UssData uss;
    for (size_t index = 0; index < ranges.size(); ++index) {
        timestamp_ns += 100000000ULL;
        auto obj = visualPerson(ranges[index], age_start + static_cast<int>(index));
        manager.evaluate({obj}, imu, uss, SentryState::ACTIVE, timestamp_ns);
    }
}

} // namespace

int main() {
    {
        AlertManager manager;
        uint64_t timestamp_ns = 1000000000ULL;
        evaluateRanges(manager, {2.0f}, timestamp_ns, 3);
        expectLevel("person inside 3m triggers immediately", manager, ThreatLevel::MEDIUM);
    }

    {
        AlertManager manager;
        const ImuData imu;
        const UssData uss;
        uint64_t timestamp_ns = 1000000000ULL;
        for (int i = 0; i < 12; ++i) {
            timestamp_ns += 100000000ULL;
            manager.evaluate({visualVehicleAt(0.0f, 2.0f, 10 + i)},
                             imu, uss, SentryState::ACTIVE, timestamp_ns);
        }
        expectLevel("stationary vehicle inside 3m", manager, ThreatLevel::LOW);
    }

    {
        AlertManager manager;
        uint64_t timestamp_ns = 1000000000ULL;
        const ImuData imu;
        const UssData uss;
        for (int i = 0; i < 5; ++i) {
            timestamp_ns += 100000000ULL;
            auto obj = visualOther(2.0f + 0.1f * i, 10 + i);
            obj.has_visual_ground_position = false;
            obj.distance_m = 2.0f + 0.1f * i;
            obj.origin = ObservationOrigin::OriginYolo;
            manager.evaluate({obj}, imu, uss, SentryState::ACTIVE, timestamp_ns);
        }
        expectLevel("moving-away range fallback", manager, ThreatLevel::LOW);
    }

    {
        // A vehicle passing the left side can stay at nearly constant radial
        // distance while moving substantially in vehicle-frame X/Y.
        AlertManager manager;
        const ImuData imu;
        const UssData uss;
        uint64_t timestamp_ns = 1000000000ULL;
        for (int i = 0; i < 11; ++i) {
            timestamp_ns += 100000000ULL;
            auto obj = visualVehicleAt(0.45f - 0.045f * i, 2.40f, 10 + i);
            obj.bbox.x += 8 * i;
            manager.evaluate({obj},
                             imu, uss, SentryState::ACTIVE, timestamp_ns);
        }
        expectLevel("left-side lateral vehicle", manager, ThreatLevel::MEDIUM);
    }

    {
        AlertManager manager;
        const ImuData imu;
        const UssData uss;
        uint64_t timestamp_ns = 1000000000ULL;
        for (int i = 0; i < 4; ++i) {
            timestamp_ns += 100000000ULL;
            auto obj = visualCycleAt(2.0f, 10 + i);
            obj.bbox.x += 15 * i;
            manager.evaluate({obj}, imu, uss, SentryState::ACTIVE, timestamp_ns);
        }
        expectLevel("moving two-wheel target uses short window", manager, ThreatLevel::MEDIUM);
    }

    {
        AlertManager manager;
        const ImuData imu;
        const UssData uss;
        uint64_t timestamp_ns = 1000000000ULL;
        for (int i = 0; i < 6; ++i) {
            timestamp_ns += 100000000ULL;
            manager.evaluate({visualCycleAt(2.0f, 10 + i)},
                             imu, uss, SentryState::ACTIVE, timestamp_ns);
        }
        expectLevel("stationary two-wheel target", manager, ThreatLevel::LOW);
    }

    {
        AlertManager manager;
        const ImuData imu;
        const UssData uss;
        uint64_t timestamp_ns = 1000000000ULL;
        for (int i = 0; i < 7; ++i) {
            timestamp_ns += 100000000ULL;
            auto obj = visualVehicleAt(0.0f, 2.5f, 10 + i);
            obj.bbox.x += 8 * i;
            manager.evaluate({obj}, imu, uss, SentryState::ACTIVE, timestamp_ns);
        }
        timestamp_ns += 100000000ULL;
        auto predicted = visualVehicleAt(0.0f, 2.5f, 17);
        predicted.bbox.x += 120;
        predicted.lost = 1;
        manager.evaluate({predicted}, imu, uss, SentryState::ACTIVE, timestamp_ns);
        expectLevel("lost predicted bbox is not motion evidence", manager, ThreatLevel::LOW);
        timestamp_ns += 100000000ULL;
        auto reacquired = visualVehicleAt(0.0f, 2.5f, 18);
        reacquired.bbox.x += 56;
        manager.evaluate({reacquired}, imu, uss, SentryState::ACTIVE, timestamp_ns);
        expectLevel("reacquired target must rebuild motion history", manager, ThreatLevel::LOW);
    }

    {
        AlertManager manager;
        uint64_t timestamp_ns = 1000000000ULL;
        const ImuData imu;
        const UssData uss;
        for (int i = 0; i < 3; ++i) {
            timestamp_ns += 100000000ULL;
            manager.evaluate({visualOther(2.80f - 0.05f * i, 10 + i)},
                             imu, uss, SentryState::ACTIVE, timestamp_ns);
        }
        expectLevel("approach before minimum motion span", manager, ThreatLevel::LOW);
        timestamp_ns += 100000000ULL;
        manager.evaluate({visualOther(2.65f, 13)},
                         imu, uss, SentryState::ACTIVE, timestamp_ns);
        expectLevel("confirmed approaching target", manager, ThreatLevel::MEDIUM);
    }

    {
        // The large bbox would be estimated below 3m by the nominal-height
        // fallback.  The calibrated ground-plane distance must take precedence.
        AlertManager manager;
        uint64_t timestamp_ns = 1000000000ULL;
        evaluateRanges(manager, {4.4f, 4.3f, 4.2f, 4.1f, 4.0f}, timestamp_ns);
        expectLevel("calibrated distance overrides bbox height", manager, ThreatLevel::LOW);
    }

    {
        AlertManager manager;
        const ImuData imu;
        const UssData uss;
        auto obj = visualVehicleAt(0.0f, 2.0f, 3);
        obj.has_uss_fusion = true;
        obj.uss_distance_m = 0.8f;
        obj.origin = ObservationOrigin::OriginYolo;
        manager.evaluate({obj}, imu, uss, SentryState::ACTIVE, 1100000000ULL);
        expectLevel("USS does not bypass stationary vehicle motion check",
                    manager, ThreatLevel::LOW);
    }

    {
        AlertManager manager;
        UssData uss;
        ImuData imu;
        imu.vibration_alert = true;
        imu.vibration_accel_rms = 0.16f;
        uint64_t timestamp_ns = 1000000000ULL;

        manager.evaluate({visualPerson(1.6f, 5, 20)}, imu, uss,
                         SentryState::ACTIVE, timestamp_ns);
        expectLevel("near calibrated person with general vibration first warns",
                    manager, ThreatLevel::MEDIUM);

        timestamp_ns += 100000000ULL;
        manager.evaluate({visualPerson(1.6f, 6, 20)}, imu, uss,
                         SentryState::WARNING, timestamp_ns);
        expectLevel("near calibrated person with general vibration upgrades",
                    manager, ThreatLevel::HIGH);
    }

    {
        AlertManager manager;
        UssData uss;
        ImuData imu;
        imu.vibration_alert = true;
        imu.vibration_accel_rms = 0.16f;
        auto obj = visualPerson(1.6f, 5, 21);
        obj.has_visual_ground_position = false;
        obj.visual_ground_distance_m = 0.f;
        obj.origin = ObservationOrigin::OriginYolo;

        manager.evaluate({obj}, imu, uss, SentryState::WARNING, 1100000000ULL);
        expectLevel("bbox-only distance does not trigger severe visual impact",
                    manager, ThreatLevel::MEDIUM);
    }

    {
        AlertManager manager;
        const ImuData imu;
        const UssData uss;
        manager.evaluate({nearFieldContactMotion()}, imu, uss,
                         SentryState::ACTIVE, 1100000000ULL);
        expectLevel("near-field image motion warns without class detection",
                    manager, ThreatLevel::MEDIUM);
    }

    {
        AlertManager manager;
        UssData uss;
        ImuData imu;
        imu.vibration_alert = true;
        imu.vibration_accel_rms = 0.16f;

        manager.evaluate({nearFieldContactMotion(3, 43)}, imu, uss,
                         SentryState::ACTIVE, 1100000000ULL);
        expectLevel("near-field contact motion with vibration first warns",
                    manager, ThreatLevel::MEDIUM);

        manager.evaluate({nearFieldContactMotion(4, 43)}, imu, uss,
                         SentryState::WARNING, 1200000000ULL);
        expectLevel("near-field contact motion with vibration upgrades",
                    manager, ThreatLevel::HIGH);
    }

    {
        AlertManager manager;
        UssData uss;
        ImuData imu;
        imu.delta_accel = 0.42f;

        manager.evaluate({nearFieldContactMotion(3, 44)}, imu, uss,
                         SentryState::ACTIVE, 1100000000ULL);
        expectLevel("near-field contact motion with short imu spike first warns",
                    manager, ThreatLevel::MEDIUM);

        manager.evaluate({nearFieldContactMotion(4, 44)}, imu, uss,
                         SentryState::WARNING, 1200000000ULL);
        expectLevel("near-field contact motion with short imu spike upgrades",
                    manager, ThreatLevel::HIGH);
    }

    {
        AlertManager manager;
        const ImuData imu;
        const UssData uss;
        manager.evaluate({passingMotionObject()}, imu, uss,
                         SentryState::ACTIVE, 1100000000ULL);
        expectLevel("passing road ROI motion warns without class detection",
                    manager, ThreatLevel::MEDIUM);
    }

    {
        AlertManager manager;
        UssData uss;
        ImuData imu;
        imu.vibration_alert = true;
        imu.vibration_accel_rms = 0.16f;

        manager.evaluate({passingMotionObject(4, 47)}, imu, uss,
                         SentryState::WARNING, 1100000000ULL);
        expectLevel("passing road ROI motion does not use contact severe path",
                    manager, ThreatLevel::MEDIUM);
    }

    {
        AlertManager manager;
        const ImuData imu;
        const UssData uss;

        manager.evaluate({nearFieldContactMotion(3, 45)}, imu, uss,
                         SentryState::ACTIVE, 1100000000ULL);
        expectLevel("sustained near-field contact first warns",
                    manager, ThreatLevel::MEDIUM);

        manager.evaluate({nearFieldContactMotion(4, 45)}, imu, uss,
                         SentryState::WARNING, 1200000000ULL);
        expectLevel("sustained near-field contact without imu remains warning",
                    manager, ThreatLevel::MEDIUM);
    }

    std::cout << "sentry_alert_manager_test: PASS" << std::endl;
    return 0;
}
