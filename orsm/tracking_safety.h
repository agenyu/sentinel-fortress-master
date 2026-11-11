#pragma once

#include <cmath>
#include <limits>
#include <utility>
#include <vector>

#include <opencv2/core.hpp>

namespace sentinel::orsm_tracking_safety {

inline int selectLowestScore(const std::vector<std::pair<int, float>>& candidates) {
    int best_id = 0;
    float best_score = std::numeric_limits<float>::max();
    for (const auto& [id, score] : candidates) {
        if (id > 0 && std::isfinite(score) && score < best_score) {
            best_id = id;
            best_score = score;
        }
    }
    return best_id;
}

inline cv::Point3f limitTotalDisplacement(const cv::Point3f& previous,
                                          const cv::Point3f& proposed,
                                          float max_step_m) {
    const cv::Point3f delta = proposed - previous;
    const float step = std::hypot(delta.x, delta.y);
    if (!std::isfinite(step) || step <= 1e-6f || step <= max_step_m) return proposed;
    const float scale = std::max(0.f, max_step_m) / step;
    return previous + delta * scale;
}

inline bool shouldTakeDropLidarAnchor(uint64_t keep_timestamp_ns,
                                      uint64_t drop_timestamp_ns) {
    return drop_timestamp_ns > keep_timestamp_ns;
}

}  // namespace sentinel::orsm_tracking_safety
