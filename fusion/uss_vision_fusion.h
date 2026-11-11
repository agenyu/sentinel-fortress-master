#pragma once

#include "../detection/tracker.h"
#include "../uss/uss_subscriber.h"
#include <string>
#include <vector>

namespace sentinel {

/**
 * @brief 哨兵模式视觉-USS近距离融合。
 *
 * USS fused point cloud 只在约 1.3m 内有效，因此该模块仅用于近车身距离修正
 * 与贴近确认，不作为 3m/10m 全距离定位依据。
 */
class UssVisionFusion {
public:
    std::vector<TrackedObject> fuse(const std::vector<TrackedObject>& tracks,
                                    const UssPointCloudSnapshot& uss_cloud,
                                    const UssClusterSnapshot& uss_clusters,
                                    const std::string& camera_name,
                                    uint64_t image_timestamp_ns,
                                    int image_height,
                                    float max_uss_range_m,
                                    float candidate_vision_range_m,
                                    float max_age_ms,
                                    float confidence_boost) const;

private:
    static bool pointInCameraSector(const UssPoint& pt, const std::string& camera_name);
    static float estimateVisionDistanceM(const TrackedObject& obj, int image_height);
    static float nominalObjectHeightM(const std::string& label);
};

} // namespace sentinel
