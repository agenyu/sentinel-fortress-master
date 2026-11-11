#pragma once
// 视觉-LiDAR 融合：把车体坐标系点云投影到去畸变相机图像，
// 为视频跟踪目标补充真实 3D 位置、距离和速度。

#include "camera/fisheye_undistort.h"
#include "detection/tracker.h"
#include "lidar/lidar_subscriber.h"

#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

namespace sentinel {

struct LidarCameraFusionConfig {
    float bbox_expand_ratio{0.10f};       // bbox 外扩，容忍标定/时间误差
    int   min_points_per_target{3};       // 每个目标至少关联点数
    float max_point_distance_m{80.0f};    // ORSM 关注范围内的最大投影距离
    float max_track_dt_sec{1.0f};         // 速度差分最大时间间隔
    float cluster_tolerance_m{0.60f};     // LiDAR 欧式聚类邻域半径
    float voxel_leaf_m{0.20f};            // 聚类前体素下采样，避免高线束点云过密导致耗时过高
    int   min_cluster_points{5};          // 有效 cluster 最少点数
    float max_association_cost{1.25f};    // cluster 与视觉 bbox 贪心匹配最大代价
    bool  visual_ground_gate_enabled{false}; // 用视觉地面交点约束点云簇，防止近人框吸附远处背景
    float visual_ground_gate_min_m{2.5f};    // 视觉位置与点云簇中心允许的最小平面误差
    float visual_ground_gate_range_ratio{0.6f}; // 随视觉距离放宽的比例

    // 性能保护：先在车体坐标系做 ROI + voxel，下采样结果按 LiDAR timestamp 缓存，供多路相机复用。
    float roi_front_m{40.0f};             // 车前 ROI，上限覆盖 ORSM A 区并留余量
    float roi_rear_m{10.0f};              // 车后 ROI
    float roi_side_m{20.0f};              // 左右 ROI
    float roi_min_z_m{0.15f};             // 车辆坐标地面上方过滤，切断人体与地面背景连通
    float roi_max_z_m{3.0f};              // 车高以上噪声过滤
};

class LidarCameraFusion {
public:
    explicit LidarCameraFusion(const LidarCameraFusionConfig& cfg = {});

    void setVisualGroundConsistencyGate(bool enabled,
                                        float min_gate_m,
                                        float range_ratio);

    std::vector<TrackedObject> fuse(const std::vector<TrackedObject>& tracked,
                                    const LidarFrame& lidar,
                                    const FisheyeCalib& camera_calib,
                                    const std::string& camera_name,
                                    uint64_t timestamp_ns);

    void reset();

private:
    struct Track3DState {
        cv::Point3f position{0.f, 0.f, 0.f};
        uint64_t timestamp_ns{0};
    };

    static cv::Rect expandAndClip(const cv::Rect& box, float ratio, const cv::Size& size);
    static float median(std::vector<float>& values);
    bool projectCarPointToImage(const LidarPoint& p,
                                const FisheyeCalib& camera_calib,
                                cv::Point2f& uv,
                                float& distance_m) const;
    std::vector<LidarPoint> cachedDownsampledRoiPoints(const LidarFrame& lidar);

    struct DownsampleCache {
        uint64_t timestamp_ns{0};
        size_t raw_point_count{0};
        float voxel_leaf_m{0.f};
        std::vector<LidarPoint> points;
    };

    LidarCameraFusionConfig cfg_;
    std::mutex mutex_;
    std::unordered_map<std::string, Track3DState> history_;
    DownsampleCache downsample_cache_;
};

}  // namespace sentinel
