#include "uss_vision_fusion.h"
#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>

namespace sentinel {

namespace {

struct VisionCandidate {
    size_t index{0};
    float vision_distance_m{0.f};
    int bbox_height{0};
};

} // namespace

float UssVisionFusion::nominalObjectHeightM(const std::string& label) {
    if (label == "person" || label == "human" || label == "pedestrian") return 1.70f;
    if (label == "bicycle" || label == "motorcycle") return 1.40f;
    if (label == "vehicle" || label == "car" || label == "sedan") return 1.50f;
    if (label == "truck" || label == "bus" || label == "suv") return 2.20f;
    if (label == "animal" || label == "dog" || label == "cat") return 0.60f;
    if (label == "horse" || label == "cow" || label == "sheep") return 1.40f;
    if (label == "ball" || label == "sports ball") return 0.25f;
    return 1.0f;
}

float UssVisionFusion::estimateVisionDistanceM(const TrackedObject& obj, int image_height) {
    if (obj.bbox.height <= 0 || image_height <= 0) return 0.f;
    const float effective_focal_px = static_cast<float>(image_height) * 0.45f;
    const float distance = nominalObjectHeightM(obj.label) * effective_focal_px /
                           static_cast<float>(obj.bbox.height);
    return std::isfinite(distance) && distance > 0.f ? distance : 0.f;
}

bool UssVisionFusion::pointInCameraSector(const UssPoint& pt, const std::string& camera_name) {
    // 车体坐标约定：x 前向为正，y 左向为正。保留 0.2m 余量以兼容角点/安装误差。
    constexpr float margin_m = 0.2f;
    if (camera_name.find("front") != std::string::npos) return pt.x_m >= -margin_m;
    if (camera_name.find("rear")  != std::string::npos) return pt.x_m <=  margin_m;
    if (camera_name.find("left")  != std::string::npos) return pt.y_m >= -margin_m;
    if (camera_name.find("right") != std::string::npos) return pt.y_m <=  margin_m;
    return true;
}

std::vector<TrackedObject> UssVisionFusion::fuse(const std::vector<TrackedObject>& tracks,
                                                 const UssPointCloudSnapshot& uss_cloud,
                                                 const UssClusterSnapshot& uss_clusters,
                                                 const std::string& camera_name,
                                                 uint64_t image_timestamp_ns,
                                                 int image_height,
                                                 float max_uss_range_m,
                                                 float candidate_vision_range_m,
                                                 float max_age_ms,
                                                 float confidence_boost) const {
    std::vector<TrackedObject> fused = tracks;
    if (!uss_cloud.valid() || !uss_clusters.valid() || fused.empty() || image_timestamp_ns == 0) return fused;
    static std::atomic<uint64_t> diagnostic_count{0};
    if (uss_cloud.timestamp_ns > image_timestamp_ns) {
        const float future_ms = static_cast<float>(uss_cloud.timestamp_ns - image_timestamp_ns) / 1.0e6f;
        if (future_ms > max_age_ms) {
            const uint64_t idx = ++diagnostic_count;
            if (idx <= 10 || idx % 60 == 0) {
                std::cout << "[UssVisionFusion][Skip] reason=timestamp_future camera=" << camera_name
                          << " tracks=" << fused.size()
                          << " uss_points=" << uss_cloud.points.size()
                          << " future_ms=" << future_ms
                          << " max_age_ms=" << max_age_ms << std::endl;
            }
            return fused;
        }
    } else {
        const float age_ms = static_cast<float>(image_timestamp_ns - uss_cloud.timestamp_ns) / 1.0e6f;
        if (age_ms > max_age_ms) {
            const uint64_t idx = ++diagnostic_count;
            if (idx <= 10 || idx % 60 == 0) {
                std::cout << "[UssVisionFusion][Skip] reason=timestamp_stale camera=" << camera_name
                          << " tracks=" << fused.size()
                          << " uss_points=" << uss_cloud.points.size()
                          << " age_ms=" << age_ms
                          << " max_age_ms=" << max_age_ms << std::endl;
            }
            return fused;
        }
    }

    int near_uss_count = 0;
    std::vector<UssCluster> clusters;
    clusters.reserve(uss_clusters.clusters.size());
    for (const auto& cluster : uss_clusters.clusters) {
        if (cluster.min_distance_m <= 0.f || cluster.min_distance_m > max_uss_range_m) continue;
        UssPoint sector_probe;
        sector_probe.x_m = cluster.center_x_m;
        sector_probe.y_m = cluster.center_y_m;
        sector_probe.distance_m = cluster.min_distance_m;
        if (!pointInCameraSector(sector_probe, camera_name)) continue;
        near_uss_count += std::max(0, cluster.point_count);
        clusters.push_back(cluster);
    }

    const float min_uss_distance = clusters.empty() ? std::numeric_limits<float>::max() : clusters.front().min_distance_m;

    std::vector<VisionCandidate> candidates;
    for (size_t i = 0; i < fused.size(); ++i) {
        auto& obj = fused[i];
        const float vision_distance_m = estimateVisionDistanceM(obj, image_height);
        const bool near_by_distance = vision_distance_m > 0.f &&
            vision_distance_m <= candidate_vision_range_m;
        const bool near_by_bbox = image_height > 0 && obj.bbox.height > image_height * 0.35f;
        if (!near_by_distance && !near_by_bbox) continue;
        candidates.push_back(VisionCandidate{i, vision_distance_m, obj.bbox.height});
    }

    std::sort(candidates.begin(), candidates.end(), [](const VisionCandidate& a, const VisionCandidate& b) {
        const bool a_has_dist = a.vision_distance_m > 0.f;
        const bool b_has_dist = b.vision_distance_m > 0.f;
        if (a_has_dist && b_has_dist && a.vision_distance_m != b.vision_distance_m) {
            return a.vision_distance_m < b.vision_distance_m;
        }
        if (a_has_dist != b_has_dist) return a_has_dist;
        return a.bbox_height > b.bbox_height;
    });

    std::vector<bool> cluster_used(clusters.size(), false);
    int fused_count = 0;
    for (const auto& candidate : candidates) {
        auto& obj = fused[candidate.index];
        int best_cluster_idx = -1;
        float best_cost = std::numeric_limits<float>::max();
        for (size_t ci = 0; ci < clusters.size(); ++ci) {
            if (cluster_used[ci]) continue;
            const auto& cluster = clusters[ci];
            // USS only confirms near-body distance. Vision distance is coarse, so keep cost dominated by
            // the closest unassigned cluster, with a small penalty for large vision/USS disagreement.
            float cost = cluster.min_distance_m;
            if (candidate.vision_distance_m > 0.f) {
                cost += 0.1f * std::fabs(candidate.vision_distance_m - cluster.min_distance_m);
            }
            cost -= std::min(0.05f, 0.005f * static_cast<float>(cluster.point_count));
            if (cost < best_cost) {
                best_cost = cost;
                best_cluster_idx = static_cast<int>(ci);
            }
        }
        if (best_cluster_idx < 0) continue;

        auto& cluster = clusters[best_cluster_idx];
        cluster_used[best_cluster_idx] = true;
        obj.has_uss_fusion = true;
        obj.near_body_confirmed = true;
        obj.uss_distance_m = cluster.min_distance_m;
        obj.uss_x_m = cluster.nearest_x_m;
        obj.uss_y_m = cluster.nearest_y_m;
        obj.uss_point_count = cluster.point_count;
        obj.uss_cluster_id = cluster.id;
        obj.uss_cluster_radius_m = cluster.radius_m;
        obj.uss_association_cost = best_cost;
        obj.uss_fusion_quality = std::min(1.0f, std::max(0.1f, confidence_boost + obj.confidence));
        obj.uss_timestamp_ns = uss_cloud.timestamp_ns;
        fused_count++;

        std::cout << "[UssVisionFusion] camera=" << camera_name
                  << " track=" << obj.track_id
                  << " vision_dist=" << candidate.vision_distance_m
                  << " uss_dist=" << obj.uss_distance_m
                  << " uss_xy=(" << obj.uss_x_m << "," << obj.uss_y_m << ")"
                  << " cluster=" << obj.uss_cluster_id
                  << " cluster_radius=" << obj.uss_cluster_radius_m
                  << " points=" << obj.uss_point_count
                  << " assoc_cost=" << obj.uss_association_cost
                  << " quality=" << obj.uss_fusion_quality << std::endl;
    }

    if (near_uss_count > 0 && fused_count == 0) {
        const uint64_t idx = ++diagnostic_count;
        if (idx <= 10 || idx % 60 == 0) {
            std::cout << "[UssVisionFusion][NoMatch] camera=" << camera_name
                      << " tracks=" << fused.size()
                      << " candidates=" << candidates.size()
                      << " near_uss_points=" << near_uss_count
                      << " clusters=" << clusters.size()
                      << " min_uss_dist=" << min_uss_distance
                      << " candidate_vision_range=" << candidate_vision_range_m
                      << " max_uss_range=" << max_uss_range_m << std::endl;
        }
    }

    return fused;
}

} // namespace sentinel
