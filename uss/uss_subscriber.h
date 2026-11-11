#pragma once

/**
 * USS 超声波信号接入
 *
 * 接口来源：东风哨兵和越野堡垒项目-软件接口设计说明书
 *
 * 订阅两个 topic：
 *   /parking/pdc/pdc_info  → PdcOutput        扇区距离报警（avp_pdc_meb 节点）
 *   /parking/meb/meb_info  → MebCollisionMsg  防碰撞距离信息（avp_pdc_meb 节点）
 *
 * PdcOutput 消息结构：
 *   timestampNs  : UInt64   时间戳
 *   seq          : UInt16   序号
 *   sectorDis    : List(UInt16)  各扇区距离（单位：mm）
 *   sourceFlag   : List(UInt8)   各扇区数据来源标志
 *
 * MebCollisionMsg 消息结构：
 *   header       : StdHeader.Header
 *   collision    : Collision
 *     distance       : Float32   预期碰撞点距离（m）
 *     obstacleType   : UInt8     障碍物类型
 *     car            : Point     车辆碰撞点坐标
 *     obj            : Point     障碍物坐标
 *   direction    : Direction  轨迹方向（forward/backward/...）
 *   pathInfo     : PathInfo   瞬时轨迹边界信息
 */

#include "rscl_access/rscl_access.h"
#include <ad_msg_idl/avp_pdc_meb/pdc_distance.capnp.h>
#include <ad_msg_idl/avp_pdc_meb/meb_collision.capnp.h>

#include <mutex>
#include <atomic>
#include <functional>
#include <vector>
#include <deque>
#include <algorithm>
#include <cstdint>
#include <string>
#include <limits>

namespace sentinel {

// ---- PDC 扇区距离快照（对应 PdcOutput） ----
struct UssData {
    uint64_t              timestamp_ns{0};
    uint16_t              seq{0};
    std::vector<uint16_t> sector_dis;    // 各扇区距离（mm）
    std::vector<uint8_t>  source_flag;   // 各扇区数据来源

    // 所有扇区中最近的非零距离（mm），无有效数据时返回 UINT16_MAX。
    // PDC sectorDis 中 0 表示无效/未使用扇区，不能参与最近距离判断。
    uint16_t minDistance() const {
        if (sector_dis.empty()) return std::numeric_limits<uint16_t>::max();
        uint16_t min_dist = std::numeric_limits<uint16_t>::max();
        for (const uint16_t dist : sector_dis) {
            if (dist == 0) continue;
            min_dist = std::min(min_dist, dist);
        }
        return min_dist;
    }

    bool valid() const { return timestamp_ns > 0; }
};

// ---- MEB 防碰撞快照（对应 MebCollisionMsg.collision） ----
struct UssCollision {
    uint64_t timestamp_ns{0};
    float    distance{999.f};     // 预期碰撞点距离（m）
    uint8_t  obstacle_type{0};    // 障碍物类型
    bool     valid{false};
};

// ---- USS fused point cloud 快照（/sensor/uss/fused_points） ----
struct UssPoint {
    float x_m{0.f};
    float y_m{0.f};
    float distance_m{0.f};
    uint64_t timestamp_ns{0};
};

struct UssPointCloudSnapshot {
    uint64_t timestamp_ns{0};
    std::vector<UssPoint> points;
    bool valid() const { return timestamp_ns > 0 && !points.empty(); }
};

// USS 回调侧预聚类结果。USS 不做固定间隔丢弃；融合侧按视频帧时间取最近 cluster。
struct UssCluster {
    int id{-1};
    float center_x_m{0.f};
    float center_y_m{0.f};
    float min_distance_m{0.f};
    float nearest_x_m{0.f};
    float nearest_y_m{0.f};
    float radius_m{0.f};
    int point_count{0};
};

struct UssClusterSnapshot {
    uint64_t timestamp_ns{0};
    std::vector<UssCluster> clusters;
    bool valid() const { return timestamp_ns > 0 && !clusters.empty(); }
};

// 近距离报警回调
using UssAlertCallback = std::function<void(const UssData&, uint16_t min_dist_mm)>;

/**
 * @brief 超声波 USS 订阅器
 *
 * 哨兵模式用途：
 *   - 补充视觉盲区（近距离 <1m 接触检测）
 *   - 与 IMU 振动联合触发 HIGH 级别报警
 *   - sectorDis 可映射到车辆四周方位，实现方向感知
 */
class UssSubscriber {
public:
    // alert_threshold_mm: 触发报警的最近距离阈值（mm），默认 500mm
    explicit UssSubscriber(uint16_t alert_threshold_mm = 500);

    // 注册订阅，必须在 RsclAccess::init() 之后调用
    void subscribe(const std::string& pdc_topic = "/parking/pdc/pdc_info",
                   const std::string& meb_topic = "/parking/meb/meb_info");

    // 注册近距离报警回调（可选）
    void setAlertCallback(UssAlertCallback cb) { alert_cb_ = std::move(cb); }

    // 从 /sensor/uss/fused_points 更新 USS 点云快照，并兼容合成 PDC 单扇区最近距离。
    void updateFromFusedPoints(uint64_t timestamp_ns, const std::vector<UssPoint>& points);

    // 线程安全读取最新快照
    UssData      latestPdc()       const;
    UssCollision latestCollision() const;
    UssPointCloudSnapshot latestPointCloud() const;
    UssPointCloudSnapshot closestPointCloud(uint64_t target_timestamp_ns, float max_age_ms) const;
    UssClusterSnapshot closestClusters(uint64_t target_timestamp_ns, float max_age_ms) const;

    uint64_t alertCount() const { return alert_count_.load(); }

private:
    void onPdc(const std::shared_ptr<ReceivedMsg<senseAD::msg::avp_pdc::PdcOutput>>& msg);
    void onMeb(const std::shared_ptr<ReceivedMsg<senseAD::msg::avp_meb::MebCollisionMsg>>& msg);

    uint16_t              alert_threshold_mm_;
    std::atomic<uint64_t> alert_count_{0};
    UssAlertCallback      alert_cb_;

    mutable std::mutex pdc_mutex_;
    UssData            latest_pdc_;

    mutable std::mutex meb_mutex_;
    UssCollision       latest_collision_;

    mutable std::mutex point_cloud_mutex_;
    UssPointCloudSnapshot latest_point_cloud_;
    std::deque<UssPointCloudSnapshot> point_cloud_history_;
    UssClusterSnapshot latest_clusters_;
    std::deque<UssClusterSnapshot> cluster_history_;
};

}  // namespace sentinel
