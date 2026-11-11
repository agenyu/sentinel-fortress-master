#include "lidar_camera_fusion.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <numeric>
#include <queue>
#include <string>
#include <unordered_map>
#include <unordered_set>

namespace sentinel {

namespace {

struct ProjectedLidarPoint {
    const LidarPoint* point{nullptr};
    cv::Point2f uv{0.f, 0.f};
    float distance_m{0.f};
    int cluster_id{-1};
};

struct LidarCluster {
    int id{-1};
    std::vector<int> indices;
    cv::Rect image_box;
    cv::Point2f image_center{0.f, 0.f};
    cv::Point3f center{0.f, 0.f, 0.f};
    cv::Point3f nearest{0.f, 0.f, 0.f};
    float distance_m{0.f};
    float radius_m{0.f};
};

struct ProjectionContext {
    cv::Matx44d T_car_to_cam;
    double fx{0.0};
    double fy{0.0};
    double cx{0.0};
    double cy{0.0};
    int img_w{0};
    int img_h{0};
    float max_distance_m{0.f};
};

long long gridKey(int ix, int iy, int iz) {
    constexpr long long kOffset = 1LL << 20;
    constexpr long long kMask = (1LL << 21) - 1LL;
    return (((static_cast<long long>(ix) + kOffset) & kMask) << 42) |
           (((static_cast<long long>(iy) + kOffset) & kMask) << 21) |
           (((static_cast<long long>(iz) + kOffset) & kMask));
}

float rectIoU(const cv::Rect& a, const cv::Rect& b) {
    const cv::Rect inter = a & b;
    const float inter_area = static_cast<float>(inter.area());
    if (inter_area <= 0.f) return 0.f;
    const float union_area = static_cast<float>(a.area() + b.area()) - inter_area;
    return union_area > 0.f ? inter_area / union_area : 0.f;
}

float normalizedCenterDistance(const cv::Rect& box, const cv::Point2f& p) {
    const cv::Point2f c(box.x + box.width * 0.5f, box.y + box.height * 0.5f);
    const float dx = (p.x - c.x) / std::max(1.f, static_cast<float>(box.width));
    const float dy = (p.y - c.y) / std::max(1.f, static_cast<float>(box.height));
    return std::sqrt(dx * dx + dy * dy);
}

bool pointInAnyGate(const cv::Point2f& uv, const std::vector<cv::Rect>& gates) {
    const cv::Point pt(static_cast<int>(std::round(uv.x)), static_cast<int>(std::round(uv.y)));
    for (const auto& gate : gates) {
        if (gate.contains(pt)) return true;
    }
    return false;
}

bool projectCarPointToImageFast(const LidarPoint& p,
                                const ProjectionContext& ctx,
                                cv::Point2f& uv,
                                float& distance_m) {
    distance_m = std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
    if (!std::isfinite(distance_m) || distance_m <= 0.05f || distance_m > ctx.max_distance_m) {
        return false;
    }

    const cv::Vec4d P_car(static_cast<double>(p.x), static_cast<double>(p.y), static_cast<double>(p.z), 1.0);
    const cv::Vec4d P_cam = ctx.T_car_to_cam * P_car;
    const double x = P_cam[0];
    const double y = P_cam[1];
    const double z = P_cam[2];
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) || z <= 0.05) return false;

    const double u = ctx.fx * x / z + ctx.cx;
    const double v = ctx.fy * y / z + ctx.cy;
    if (!std::isfinite(u) || !std::isfinite(v)) return false;
    if (u < 0 || v < 0 || u >= ctx.img_w || v >= ctx.img_h) return false;

    uv = cv::Point2f(static_cast<float>(u), static_cast<float>(v));
    return true;
}

}  // namespace

LidarCameraFusion::LidarCameraFusion(const LidarCameraFusionConfig& cfg)
    : cfg_(cfg)
{}

void LidarCameraFusion::setVisualGroundConsistencyGate(bool enabled,
                                                       float min_gate_m,
                                                       float range_ratio) {
    std::lock_guard<std::mutex> lk(mutex_);
    cfg_.visual_ground_gate_enabled = enabled;
    cfg_.visual_ground_gate_min_m = std::max(0.f, min_gate_m);
    cfg_.visual_ground_gate_range_ratio = std::max(0.f, range_ratio);
}

cv::Rect LidarCameraFusion::expandAndClip(const cv::Rect& box, float ratio, const cv::Size& size) {
    const int dx = static_cast<int>(std::round(box.width * ratio));
    const int dy = static_cast<int>(std::round(box.height * ratio));
    cv::Rect expanded(box.x - dx, box.y - dy, box.width + 2 * dx, box.height + 2 * dy);
    return expanded & cv::Rect(0, 0, size.width, size.height);
}

float LidarCameraFusion::median(std::vector<float>& values) {
    if (values.empty()) return 0.f;
    const size_t mid = values.size() / 2;
    std::nth_element(values.begin(), values.begin() + mid, values.end());
    float m = values[mid];
    if (values.size() % 2 == 0) {
        auto max_lower = std::max_element(values.begin(), values.begin() + mid);
        if (max_lower != values.begin() + mid) m = 0.5f * (m + *max_lower);
    }
    return m;
}

bool LidarCameraFusion::projectCarPointToImage(const LidarPoint& p,
                                                const FisheyeCalib& camera_calib,
                                                cv::Point2f& uv,
                                                float& distance_m) const {
    if (camera_calib.extrinsic.empty() || camera_calib.K_new.empty()) return false;
    if (camera_calib.extrinsic.rows != 4 || camera_calib.extrinsic.cols != 4) return false;

    distance_m = std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
    if (!std::isfinite(distance_m) || distance_m <= 0.05f || distance_m > cfg_.max_point_distance_m) {
        return false;
    }

    cv::Mat T_cam_to_car;
    camera_calib.extrinsic.convertTo(T_cam_to_car, CV_64F);
    cv::Mat T_car_to_cam = T_cam_to_car.inv();

    const cv::Mat P_car = (cv::Mat_<double>(4, 1) << p.x, p.y, p.z, 1.0);
    cv::Mat P_cam = T_car_to_cam * P_car;
    const double x = P_cam.at<double>(0, 0);
    const double y = P_cam.at<double>(1, 0);
    const double z = P_cam.at<double>(2, 0);
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) || z <= 0.05) return false;

    const double fx = camera_calib.K_new.at<double>(0, 0);
    const double fy = camera_calib.K_new.at<double>(1, 1);
    const double cx = camera_calib.K_new.at<double>(0, 2);
    const double cy = camera_calib.K_new.at<double>(1, 2);
    const double u = fx * x / z + cx;
    const double v = fy * y / z + cy;
    if (!std::isfinite(u) || !std::isfinite(v)) return false;
    if (u < 0 || v < 0 || u >= camera_calib.img_w || v >= camera_calib.img_h) return false;

    uv = cv::Point2f(static_cast<float>(u), static_cast<float>(v));
    return true;
}

std::vector<LidarPoint> LidarCameraFusion::cachedDownsampledRoiPoints(const LidarFrame& lidar) {
    const float leaf = std::max(0.02f, cfg_.voxel_leaf_m);

    {
        std::lock_guard<std::mutex> lk(mutex_);
        if (downsample_cache_.timestamp_ns == lidar.timestamp_ns &&
            downsample_cache_.raw_point_count == lidar.points.size() &&
            std::abs(downsample_cache_.voxel_leaf_m - leaf) < 1e-6f) {
            return downsample_cache_.points;
        }
    }

    std::vector<LidarPoint> points;
    points.reserve(std::min<size_t>(lidar.points.size(), 120000));
    std::unordered_set<long long> occupied_voxels;
    occupied_voxels.reserve(lidar.points.size() / 8 + 1);

    for (const auto& point : lidar.points) {
        if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) continue;
        if (point.x < -cfg_.roi_rear_m || point.x > cfg_.roi_front_m) continue;
        if (std::abs(point.y) > cfg_.roi_side_m) continue;
        if (point.z < cfg_.roi_min_z_m || point.z > cfg_.roi_max_z_m) continue;
        const float dist2 = point.x * point.x + point.y * point.y + point.z * point.z;
        if (dist2 <= 0.05f * 0.05f || dist2 > cfg_.max_point_distance_m * cfg_.max_point_distance_m) continue;

        const int vx = static_cast<int>(std::floor(point.x / leaf));
        const int vy = static_cast<int>(std::floor(point.y / leaf));
        const int vz = static_cast<int>(std::floor(point.z / leaf));
        const long long voxel_key = gridKey(vx, vy, vz);
        if (!occupied_voxels.insert(voxel_key).second) continue;
        points.push_back(point);
    }

    {
        std::lock_guard<std::mutex> lk(mutex_);
        downsample_cache_.timestamp_ns = lidar.timestamp_ns;
        downsample_cache_.raw_point_count = lidar.points.size();
        downsample_cache_.voxel_leaf_m = leaf;
        downsample_cache_.points = points;
    }
    return points;
}

std::vector<TrackedObject> LidarCameraFusion::fuse(const std::vector<TrackedObject>& tracked,
                                                    const LidarFrame& lidar,
                                                    const FisheyeCalib& camera_calib,
                                                    const std::string& camera_name,
                                                    uint64_t timestamp_ns) {
    std::vector<TrackedObject> fused = tracked;
    if (fused.empty() || !lidar.valid() || camera_calib.K_new.empty() || camera_calib.extrinsic.empty()) {
        return fused;
    }

    const cv::Size image_size(camera_calib.img_w, camera_calib.img_h);
    std::vector<cv::Rect> gates;
    gates.reserve(fused.size());
    for (const auto& obj : fused) {
        gates.push_back(expandAndClip(obj.bbox, cfg_.bbox_expand_ratio, image_size));
    }

    cv::Mat T_cam_to_car;
    camera_calib.extrinsic.convertTo(T_cam_to_car, CV_64F);
    cv::Mat T_car_to_cam_mat = T_cam_to_car.inv();
    cv::Matx44d T_car_to_cam;
    for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 4; ++c) {
            T_car_to_cam(r, c) = T_car_to_cam_mat.at<double>(r, c);
        }
    }
    ProjectionContext proj_ctx;
    proj_ctx.T_car_to_cam = T_car_to_cam;
    proj_ctx.fx = camera_calib.K_new.at<double>(0, 0);
    proj_ctx.fy = camera_calib.K_new.at<double>(1, 1);
    proj_ctx.cx = camera_calib.K_new.at<double>(0, 2);
    proj_ctx.cy = camera_calib.K_new.at<double>(1, 2);
    proj_ctx.img_w = camera_calib.img_w;
    proj_ctx.img_h = camera_calib.img_h;
    proj_ctx.max_distance_m = cfg_.max_point_distance_m;

    const std::vector<LidarPoint> roi_points = cachedDownsampledRoiPoints(lidar);

    const char* lcf_debug_env = std::getenv("ORSM_LCF_DEBUG");
    const bool lcf_debug = lcf_debug_env && *lcf_debug_env &&
                           camera_name.find(lcf_debug_env) != std::string::npos;
    size_t diag_roi_x_neg = 0;
    size_t diag_behind_camera = 0;
    size_t diag_outside_image = 0;
    size_t diag_projected_image = 0;
    size_t diag_gate_reject = 0;
    size_t diag_projected_gate = 0;

    if (lcf_debug) {
        for (const auto& point : roi_points) {
            if (point.x < 0.f) ++diag_roi_x_neg;
            const cv::Vec4d p_car(point.x, point.y, point.z, 1.0);
            const cv::Vec4d p_cam = proj_ctx.T_car_to_cam * p_car;
            if (!std::isfinite(p_cam[2]) || p_cam[2] <= 0.05) {
                ++diag_behind_camera;
                continue;
            }
            const double u = proj_ctx.fx * p_cam[0] / p_cam[2] + proj_ctx.cx;
            const double v = proj_ctx.fy * p_cam[1] / p_cam[2] + proj_ctx.cy;
            if (!std::isfinite(u) || !std::isfinite(v) ||
                u < 0.0 || v < 0.0 || u >= proj_ctx.img_w || v >= proj_ctx.img_h) {
                ++diag_outside_image;
                continue;
            }
            ++diag_projected_image;
            const cv::Point2f uv(static_cast<float>(u), static_cast<float>(v));
            if (pointInAnyGate(uv, gates)) ++diag_projected_gate;
            else ++diag_gate_reject;
        }
    }

    // 先做车体 ROI + voxel 缓存，再只保留落入视觉目标 gate 的点，避免对整幅可见点云聚类。
    std::vector<ProjectedLidarPoint> projected;
    projected.reserve(std::min<size_t>(roi_points.size(), 20000));
    std::unordered_map<long long, std::vector<int>> grid;
    grid.reserve(roi_points.size() / 4 + 1);
    const float tol = std::max(0.05f, cfg_.cluster_tolerance_m);

    for (const auto& point : roi_points) {
        cv::Point2f uv;
        float distance_m = 0.f;
        if (!projectCarPointToImageFast(point, proj_ctx, uv, distance_m)) continue;
        if (!pointInAnyGate(uv, gates)) continue;

        const int idx = static_cast<int>(projected.size());
        projected.push_back(ProjectedLidarPoint{&point, uv, distance_m, -1});
        const int ix = static_cast<int>(std::floor(point.x / tol));
        const int iy = static_cast<int>(std::floor(point.y / tol));
        const int iz = static_cast<int>(std::floor(point.z / tol));
        grid[gridKey(ix, iy, iz)].push_back(idx);
    }

    std::vector<LidarCluster> clusters;
    std::vector<int> queue;
    queue.reserve(1024);
    const float tol2 = tol * tol;

    for (int seed = 0; seed < static_cast<int>(projected.size()); ++seed) {
        if (projected[seed].cluster_id >= 0) continue;

        const int cluster_id = static_cast<int>(clusters.size());
        LidarCluster cluster;
        cluster.id = cluster_id;
        queue.clear();
        queue.push_back(seed);
        projected[seed].cluster_id = cluster_id;

        for (size_t q = 0; q < queue.size(); ++q) {
            const int cur = queue[q];
            cluster.indices.push_back(cur);
            const auto& cp = *projected[cur].point;
            const int ix = static_cast<int>(std::floor(cp.x / tol));
            const int iy = static_cast<int>(std::floor(cp.y / tol));
            const int iz = static_cast<int>(std::floor(cp.z / tol));
            for (int dx = -1; dx <= 1; ++dx) {
                for (int dy = -1; dy <= 1; ++dy) {
                    for (int dz = -1; dz <= 1; ++dz) {
                        auto it = grid.find(gridKey(ix + dx, iy + dy, iz + dz));
                        if (it == grid.end()) continue;
                        for (int nb : it->second) {
                            if (projected[nb].cluster_id >= 0) continue;
                            const auto& np = *projected[nb].point;
                            const float ddx = cp.x - np.x;
                            const float ddy = cp.y - np.y;
                            const float ddz = cp.z - np.z;
                            if (ddx * ddx + ddy * ddy + ddz * ddz > tol2) continue;
                            projected[nb].cluster_id = cluster_id;
                            queue.push_back(nb);
                        }
                    }
                }
            }
        }

        if (static_cast<int>(cluster.indices.size()) < cfg_.min_cluster_points) {
            for (int idx : cluster.indices) projected[idx].cluster_id = -2;
            continue;
        }

        std::vector<float> xs, ys, zs, ds;
        xs.reserve(cluster.indices.size());
        ys.reserve(cluster.indices.size());
        zs.reserve(cluster.indices.size());
        ds.reserve(cluster.indices.size());
        float min_u = std::numeric_limits<float>::max();
        float min_v = std::numeric_limits<float>::max();
        float max_u = std::numeric_limits<float>::lowest();
        float max_v = std::numeric_limits<float>::lowest();
        float nearest_d = std::numeric_limits<float>::max();
        for (int idx : cluster.indices) {
            const auto& pp = projected[idx];
            const auto& p = *pp.point;
            xs.push_back(p.x); ys.push_back(p.y); zs.push_back(p.z); ds.push_back(pp.distance_m);
            min_u = std::min(min_u, pp.uv.x); min_v = std::min(min_v, pp.uv.y);
            max_u = std::max(max_u, pp.uv.x); max_v = std::max(max_v, pp.uv.y);
            if (pp.distance_m < nearest_d) {
                nearest_d = pp.distance_m;
                cluster.nearest = cv::Point3f(p.x, p.y, p.z);
            }
        }
        cluster.center = cv::Point3f(median(xs), median(ys), median(zs));
        cluster.distance_m = median(ds);
        cluster.image_box = cv::Rect(cv::Point(static_cast<int>(std::floor(min_u)), static_cast<int>(std::floor(min_v))),
                                     cv::Point(static_cast<int>(std::ceil(max_u)) + 1, static_cast<int>(std::ceil(max_v)) + 1)) &
                            cv::Rect(0, 0, image_size.width, image_size.height);
        cluster.image_center = cv::Point2f(cluster.image_box.x + cluster.image_box.width * 0.5f,
                                           cluster.image_box.y + cluster.image_box.height * 0.5f);
        float radius = 0.f;
        for (int idx : cluster.indices) {
            const auto& p = *projected[idx].point;
            const float dx = p.x - cluster.center.x;
            const float dy = p.y - cluster.center.y;
            const float dz = p.z - cluster.center.z;
            radius = std::max(radius, std::sqrt(dx * dx + dy * dy + dz * dz));
        }
        cluster.radius_m = radius;
        clusters.push_back(std::move(cluster));
    }

    struct Candidate {
        int target_idx{-1};
        int cluster_idx{-1};
        float cost{0.f};
        float iou{0.f};
    };
    // Protect both history-assisted candidate selection and the history update below.  The old
    // implementation used history only after association to calculate velocity, so a completely
    // different 3D cluster could win on 2D IoU alone and generate a large position jump.
    std::lock_guard<std::mutex> lk(mutex_);
    std::vector<Candidate> candidates;
    int visual_ground_rejected = 0;
    for (size_t ti = 0; ti < fused.size(); ++ti) {
        const std::string history_key = camera_name + ":" + std::to_string(fused[ti].track_id);
        const auto hist = history_.find(history_key);
        for (size_t ci = 0; ci < clusters.size(); ++ci) {
            const auto& c = clusters[ci];
            if (c.image_box.empty()) continue;
            const float iou = rectIoU(gates[ti], c.image_box);
            const bool center_inside = gates[ti].contains(c.image_center);
            if (iou <= 0.f && !center_inside) continue;
            const float center_cost = normalizedCenterDistance(gates[ti], c.image_center);
            float visual_ground_cost = 0.f;
            if (cfg_.visual_ground_gate_enabled &&
                fused[ti].distance_source == "vision_ground_plane" &&
                fused[ti].distance_m > 0.3f) {
                const float dx = c.center.x - fused[ti].position_3d.x;
                const float dy = c.center.y - fused[ti].position_3d.y;
                const float ground_error_m = std::hypot(dx, dy);
                const float ground_gate_m = std::max(
                    cfg_.visual_ground_gate_min_m,
                    cfg_.visual_ground_gate_range_ratio * fused[ti].distance_m);
                if (ground_error_m > ground_gate_m) {
                    ++visual_ground_rejected;
                    continue;
                }
                visual_ground_cost = 0.35f * ground_error_m /
                                     std::max(0.1f, ground_gate_m);
            }
            float temporal_cost = 0.f;
            if (hist != history_.end() && hist->second.timestamp_ns > 0 &&
                timestamp_ns > hist->second.timestamp_ns) {
                const double dt = static_cast<double>(timestamp_ns - hist->second.timestamp_ns) * 1e-9;
                if (dt > 1e-3 && dt <= cfg_.max_track_dt_sec) {
                    const float dx = c.center.x - hist->second.position.x;
                    const float dy = c.center.y - hist->second.position.y;
                    const float dz = c.center.z - hist->second.position.z;
                    const float displacement_m = std::sqrt(dx * dx + dy * dy + dz * dz);
                    const float temporal_gate_m = static_cast<float>(0.6 + 6.0 * dt);
                    if (displacement_m > temporal_gate_m) continue;
                    temporal_cost = 0.75f * displacement_m /
                                    std::max(0.1f, temporal_gate_m);
                }
            }
            const float cost = center_cost + (1.0f - std::min(1.0f, iou)) +
                               temporal_cost + visual_ground_cost;
            if (cost > cfg_.max_association_cost && !center_inside) continue;
            candidates.push_back(Candidate{static_cast<int>(ti), static_cast<int>(ci), cost, iou});
        }
    }
    std::sort(candidates.begin(), candidates.end(), [](const Candidate& a, const Candidate& b) {
        return a.cost < b.cost;
    });

    std::vector<bool> used_target(fused.size(), false), used_cluster(clusters.size(), false);
    int matched = 0;
    for (const auto& cand : candidates) {
        if (used_target[cand.target_idx] || used_cluster[cand.cluster_idx]) continue;
        const auto& c = clusters[cand.cluster_idx];
        if (static_cast<int>(c.indices.size()) < cfg_.min_points_per_target) continue;

        auto& obj = fused[cand.target_idx];
        obj.has_lidar_fusion = true;
        obj.position_3d = c.center;
        obj.distance_m = c.distance_m;
        obj.fusion_timestamp_ns = timestamp_ns;
        obj.lidar_point_count = static_cast<int>(c.indices.size());
        obj.lidar_cluster_id = c.id;
        obj.lidar_cluster_radius_m = c.radius_m;
        obj.lidar_association_cost = cand.cost;

        const std::string history_key = camera_name + ":" + std::to_string(obj.track_id);
        auto hist = history_.find(history_key);
        if (hist != history_.end() && hist->second.timestamp_ns > 0 && timestamp_ns > hist->second.timestamp_ns) {
            const double dt = static_cast<double>(timestamp_ns - hist->second.timestamp_ns) * 1e-9;
            if (dt > 1e-3 && dt <= cfg_.max_track_dt_sec) {
                obj.velocity_3d = cv::Point3f(
                    static_cast<float>((c.center.x - hist->second.position.x) / dt),
                    static_cast<float>((c.center.y - hist->second.position.y) / dt),
                    static_cast<float>((c.center.z - hist->second.position.z) / dt));
                obj.speed_mps = std::sqrt(obj.velocity_3d.x * obj.velocity_3d.x +
                                          obj.velocity_3d.y * obj.velocity_3d.y +
                                          obj.velocity_3d.z * obj.velocity_3d.z);
            }
        }
        history_[history_key] = Track3DState{c.center, timestamp_ns};
        used_target[cand.target_idx] = true;
        used_cluster[cand.cluster_idx] = true;
        matched++;
    }

    const int velocity_ready = static_cast<int>(std::count_if(
        fused.begin(), fused.end(), [](const TrackedObject& obj) { return obj.speed_mps > 1e-2f; }));

    static uint64_t log_count = 0;
    if (matched > 0 || ++log_count <= 20 || log_count % 120 == 0) {
        std::cout << "[LidarCameraFusion] camera=" << camera_name
                  << " tracks=" << fused.size()
                  << " projected=" << projected.size()
                  << " roi_points=" << roi_points.size()
                  << " raw_points=" << lidar.points.size()
                  << " clusters=" << clusters.size()
                  << " candidates=" << candidates.size()
                  << " matched=" << matched
                  << " visual_ground_rejected=" << visual_ground_rejected
                  << " velocity_ready=" << velocity_ready
                  << std::endl;
    }
    if (lcf_debug) {
        std::cout << "[LidarCameraFusionDiag] camera=" << camera_name
                  << " roi_x_neg=" << diag_roi_x_neg
                  << " behind_camera=" << diag_behind_camera
                  << " outside_image=" << diag_outside_image
                  << " projected_image=" << diag_projected_image
                  << " gate_reject=" << diag_gate_reject
                  << " projected_gate=" << diag_projected_gate
                  << " matched=" << matched
                  << std::endl;
    }
    return fused;
}

void LidarCameraFusion::reset() {
    std::lock_guard<std::mutex> lk(mutex_);
    history_.clear();
    downsample_cache_ = DownsampleCache{};
}

}  // namespace sentinel
