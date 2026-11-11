#pragma once

#include "detector.h"
#include <opencv2/opencv.hpp>
#include <vector>
#include <deque>
#include <unordered_map>
#include <cstdint>
#include <string>

namespace sentinel {

// 观测来源，用于替代旧的 observation_source/distance_source 字符串字段。
enum class ObservationOrigin : uint8_t {
    OriginNone = 0,
    OriginYolo,              // YOLO 检测框（含标定/框高估距的视觉测距）
    OriginDiffbox,           // 图像运动差值框（远距运动提案）
    OriginLidarCluster,      // LiDAR 聚类簇
    OriginSentryNearMotion,  // 哨兵近场运动候选
    OriginSentryRoadMotion,  // 哨兵过路运动候选
    OriginTrackerPrediction, // ByteTrack 预测帧（非真实检测）
};

// 当前帧的雷达簇关联结果，仅对 OriginLidarCluster 有效。
enum class LidarClaim : uint8_t {
    LidarClaimNone = 0,      // 未被其他通道关联，类别由 L# 历史状态补充
    LidarClaimYolo,          // 本帧被 YOLO 框认领（中心距离门内）
    LidarClaimDiffbox,       // 本帧被差值框 InBox 匹配
};

// 单帧观测结构，不持有跨帧状态。
// 相机观测仅作为 ORSM 语义锚点；LiDAR 聚类观测关联到 L# 后进入候选队列。
// 轨迹、类别和发布租约由 L# 管理，origin/lidar_claim 仅描述当前帧。
struct TrackedObject {
    int      track_id{-1};
    int      class_id{-1};
    std::string label;
    cv::Rect bbox;
    float    confidence{0.f};

    // 轨迹历史（最近 N 帧中心点）
    std::deque<cv::Point2f> trajectory;

    // 运动向量（当前帧相对上一帧的位移，像素/帧）
    cv::Point2f velocity{0.f, 0.f};

    // 连续出现帧数（用于过滤误检）
    int age{0};

    // 连续丢失帧数
    int lost{0};

    // 视觉-LiDAR 融合结果（车体坐标系，单位：m / m/s）
    bool has_lidar_fusion{false};
    cv::Point3f position_3d{0.f, 0.f, 0.f};
    cv::Point3f velocity_3d{0.f, 0.f, 0.f};
    float distance_m{0.f};
    float speed_mps{0.f};
    // 保留标定后的框底接地点；融合覆盖 position_3d 后仍用于近场视觉轨迹。
    bool has_visual_ground_position{false};
    cv::Point3f visual_ground_position_3d{0.f, 0.f, 0.f};
    float visual_ground_distance_m{0.f};
    int lidar_point_count{0};
    int lidar_cluster_id{-1};
    float lidar_cluster_radius_m{0.f};
    float lidar_association_cost{0.f};
    uint64_t fusion_timestamp_ns{0};
    std::string source_camera;
    ObservationOrigin origin{ObservationOrigin::OriginNone}; // 观测出身（哪条通道产出本对象）
    LidarClaim lidar_claim{LidarClaim::LidarClaimNone}; // 雷达簇的当帧关联结果（仅 origin==OriginLidarCluster 有效）
    int orsm_semantic_anchor_id{-1};     // ORSM LiDAR-primary: one video/latched semantic anchor owns one candidate
    int orsm_visual_track_id{-1};        // ORSM LiDAR-primary: stable camera-scoped video track that confirmed this candidate
    // Persistent identity assigned by ORSM's LiDAR-cluster temporal associator.
    // Unlike lidar_cluster_id / track_id this survives a new clustering pass.
    int orsm_lidar_temporal_track_id{-1};
    bool orsm_far_image_motion_lidar_confirmed{false}; // far unknown track has image-motion evidence and LiDAR support

    // 视觉-USS融合结果：哨兵模式用于近车身确认，ORSM用于C区关联目标的测距。
    bool has_uss_fusion{false};
    bool near_body_confirmed{false};
    float uss_distance_m{0.f};
    float uss_x_m{0.f};
    float uss_y_m{0.f};
    int uss_point_count{0};
    int uss_cluster_id{-1};
    float uss_cluster_radius_m{0.f};
    float uss_association_cost{0.f};
    float uss_fusion_quality{0.f};
    uint64_t uss_timestamp_ns{0};
};

/**
 * @brief ByteTrack 多目标跟踪器
 *
 * 实现 ByteTrack 核心流程：Kalman 预测、按高/低置信度检测框二阶段关联、
 * 匈牙利匹配、轨迹激活/丢失/移除状态管理，并保持对外 TrackedObject 接口稳定。
 *
 * 说明：ByteTrack 本身不是神经网络模型，不需要 .pt/.onnx/.engine 文件；它消费
 * Detector 输出的 Detection 列表，生成稳定 track_id、轨迹历史和像素速度。
 */
class Tracker {
public:
    explicit Tracker(int max_lost_frames = 30,
                     int min_hits        = 3,
                     float iou_threshold = 0.3f,
                     int output_lost_frames = 2,
                     float high_conf_threshold = 0.5f,
                     float low_conf_threshold = 0.1f,
                     float new_track_threshold = 0.5f,
                     float low_match_iou_threshold = 0.2f);

    /**
     * @brief 更新跟踪状态
     * @param detections 当前帧检测结果
     * @return 当前帧所有活跃跟踪目标
     */
    std::vector<TrackedObject> update(const std::vector<Detection>& detections);

    void reset();

private:
    enum class TrackLifecycle {
        Tentative,
        Tracked,
        Lost,
        Removed,
    };

    struct TrackState {
        TrackedObject object;
        cv::KalmanFilter kf;
        TrackLifecycle state{TrackLifecycle::Tentative};
        int hits{0};
        int frame_id{0};
        int start_frame{0};
    };

    struct MatchResult {
        std::vector<std::pair<int, int>> matches;       // track index, detection index
        std::vector<int> unmatched_tracks;
        std::vector<int> unmatched_detections;
    };

    static float iou(const cv::Rect& a, const cv::Rect& b);
    static cv::Point2f centerOf(const cv::Rect& r);
    static void appendTrajectory(TrackedObject& obj, const cv::Point2f& center);

    static cv::Vec4f bboxToMeasurement(const cv::Rect& bbox);
    static cv::Rect measurementToBbox(const cv::Vec4f& m);
    static cv::KalmanFilter createKalman(const cv::Rect& bbox);
    static cv::Rect predictBbox(cv::KalmanFilter& kf);
    static void correctKalman(cv::KalmanFilter& kf, const cv::Rect& bbox);

    static MatchResult associate(const std::vector<int>& track_ids,
                                 const std::vector<int>& detection_indices,
                                 const std::unordered_map<int, TrackState>& tracks,
                                 const std::vector<Detection>& detections,
                                 float min_iou);

    static std::vector<std::pair<int, int>> hungarianMinCost(const std::vector<std::vector<float>>& cost);

    void activateTrack(const Detection& det);
    void updateTrack(TrackState& track, const Detection& det);
    void markLostOrRemoved(TrackState& track);
    void pruneRemoved();

    int   max_lost_frames_;
    int   min_hits_;
    int   output_lost_frames_;
    float iou_threshold_;

    // ByteTrack 阈值：默认保持原行为，可由上层配置覆盖。
    float high_conf_threshold_{0.5f};
    float low_conf_threshold_{0.1f};
    float new_track_threshold_{0.5f};
    float low_match_iou_threshold_{0.2f};

    int   next_id_{0};
    int   frame_id_{0};

    std::unordered_map<int, TrackState> tracks_;
};

}  // namespace sentinel
