#include "uss_subscriber.h"
#include <ad_log/ad_log.hpp>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>

namespace sentinel {

namespace {

std::vector<UssCluster> buildUssClusters(const std::vector<UssPoint>& points) {
    constexpr float kClusterRadiusM = 0.25f;
    std::vector<UssCluster> clusters;
    std::vector<bool> visited(points.size(), false);
    for (size_t seed = 0; seed < points.size(); ++seed) {
        if (visited[seed]) continue;
        if (points[seed].distance_m <= 0.f) continue;
        std::vector<size_t> queue{seed};
        std::vector<size_t> members;
        visited[seed] = true;
        for (size_t q = 0; q < queue.size(); ++q) {
            const size_t cur = queue[q];
            members.push_back(cur);
            for (size_t i = 0; i < points.size(); ++i) {
                if (visited[i] || points[i].distance_m <= 0.f) continue;
                const float dx = points[cur].x_m - points[i].x_m;
                const float dy = points[cur].y_m - points[i].y_m;
                if (std::sqrt(dx * dx + dy * dy) <= kClusterRadiusM) {
                    visited[i] = true;
                    queue.push_back(i);
                }
            }
        }

        UssCluster cluster;
        cluster.id = static_cast<int>(clusters.size());
        cluster.point_count = static_cast<int>(members.size());
        cluster.min_distance_m = std::numeric_limits<float>::max();
        float sum_x = 0.f, sum_y = 0.f;
        for (const size_t idx : members) {
            const auto& pt = points[idx];
            sum_x += pt.x_m;
            sum_y += pt.y_m;
            if (pt.distance_m < cluster.min_distance_m) {
                cluster.min_distance_m = pt.distance_m;
                cluster.nearest_x_m = pt.x_m;
                cluster.nearest_y_m = pt.y_m;
            }
        }
        cluster.center_x_m = sum_x / std::max(1, cluster.point_count);
        cluster.center_y_m = sum_y / std::max(1, cluster.point_count);
        for (const size_t idx : members) {
            const auto& pt = points[idx];
            const float dx = pt.x_m - cluster.center_x_m;
            const float dy = pt.y_m - cluster.center_y_m;
            cluster.radius_m = std::max(cluster.radius_m, std::sqrt(dx * dx + dy * dy));
        }
        clusters.push_back(cluster);
    }
    std::sort(clusters.begin(), clusters.end(), [](const UssCluster& a, const UssCluster& b) {
        if (a.min_distance_m != b.min_distance_m) return a.min_distance_m < b.min_distance_m;
        return a.point_count > b.point_count;
    });
    for (size_t i = 0; i < clusters.size(); ++i) clusters[i].id = static_cast<int>(i);
    return clusters;
}

template <typename SnapshotT>
SnapshotT closestSnapshot(const std::deque<SnapshotT>& history, uint64_t target_timestamp_ns, float max_age_ms) {
    if (target_timestamp_ns == 0 || history.empty()) return SnapshotT{};
    const uint64_t max_delta_ns = static_cast<uint64_t>(std::max(0.0f, max_age_ms) * 1.0e6f);
    const SnapshotT* best = nullptr;
    uint64_t best_delta = std::numeric_limits<uint64_t>::max();
    for (const auto& snapshot : history) {
        const uint64_t delta = snapshot.timestamp_ns > target_timestamp_ns
            ? snapshot.timestamp_ns - target_timestamp_ns
            : target_timestamp_ns - snapshot.timestamp_ns;
        if (delta < best_delta) {
            best_delta = delta;
            best = &snapshot;
        }
    }
    if (!best || best_delta > max_delta_ns) return SnapshotT{};
    return *best;
}

} // namespace

UssSubscriber::UssSubscriber(uint16_t alert_threshold_mm)
    : alert_threshold_mm_(alert_threshold_mm)
{}

void UssSubscriber::subscribe(const std::string& pdc_topic,
                               const std::string& meb_topic)
{
    RsclAccess::Instance()->subscribe(pdc_topic, &UssSubscriber::onPdc, this);
    RsclAccess::Instance()->subscribe(meb_topic, &UssSubscriber::onMeb, this);
    std::cout << "[UssSubscriber] Subscribed:"
              << "\n  PDC: " << pdc_topic
              << "\n  MEB: " << meb_topic << std::endl;
}

UssData UssSubscriber::latestPdc() const {
    std::lock_guard<std::mutex> lk(pdc_mutex_);
    return latest_pdc_;
}

UssCollision UssSubscriber::latestCollision() const {
    std::lock_guard<std::mutex> lk(meb_mutex_);
    return latest_collision_;
}

UssPointCloudSnapshot UssSubscriber::latestPointCloud() const {
    std::lock_guard<std::mutex> lk(point_cloud_mutex_);
    return latest_point_cloud_;
}

UssPointCloudSnapshot UssSubscriber::closestPointCloud(uint64_t target_timestamp_ns, float max_age_ms) const {
    std::lock_guard<std::mutex> lk(point_cloud_mutex_);
    return closestSnapshot(point_cloud_history_, target_timestamp_ns, max_age_ms);
}

UssClusterSnapshot UssSubscriber::closestClusters(uint64_t target_timestamp_ns, float max_age_ms) const {
    std::lock_guard<std::mutex> lk(point_cloud_mutex_);
    return closestSnapshot(cluster_history_, target_timestamp_ns, max_age_ms);
}

void UssSubscriber::updateFromFusedPoints(uint64_t timestamp_ns, const std::vector<UssPoint>& points) {
    uint16_t min_distance_mm = std::numeric_limits<uint16_t>::max();
    for (const auto& pt : points) {
        if (pt.distance_m <= 0.f) continue;
        const uint32_t dist_mm = static_cast<uint32_t>(pt.distance_m * 1000.0f);
        if (dist_mm < min_distance_mm) {
            min_distance_mm = dist_mm > 65534u ? 65535u : static_cast<uint16_t>(dist_mm);
        }
    }

    {
        std::lock_guard<std::mutex> lk(point_cloud_mutex_);
        latest_point_cloud_.timestamp_ns = timestamp_ns;
        latest_point_cloud_.points = points;
        latest_clusters_.timestamp_ns = timestamp_ns;
        latest_clusters_.clusters = buildUssClusters(points);
        if (!points.empty()) {
            point_cloud_history_.push_back(latest_point_cloud_);
            cluster_history_.push_back(latest_clusters_);
            constexpr size_t kMaxPointCloudHistory = 200;
            while (point_cloud_history_.size() > kMaxPointCloudHistory) {
                point_cloud_history_.pop_front();
            }
            while (cluster_history_.size() > kMaxPointCloudHistory) {
                cluster_history_.pop_front();
            }
        }
    }

    UssData data;
    data.timestamp_ns = timestamp_ns;
    data.sector_dis.push_back(min_distance_mm);
    data.source_flag.push_back(1);  // synthetic fused USS source

    {
        std::lock_guard<std::mutex> lk(pdc_mutex_);
        data.seq = static_cast<uint16_t>(latest_pdc_.seq + 1);
        latest_pdc_ = data;
    }

    static std::atomic<uint64_t> fused_log_count{0};
    const uint64_t log_idx = ++fused_log_count;
    if (!points.empty() && (log_idx % 30 == 1 || min_distance_mm < alert_threshold_mm_)) {
        std::cout << "[UssSubscriber] fused_points points=" << points.size()
                  << " min_dist=" << min_distance_mm << "mm" << std::endl;
    }

    if (min_distance_mm > 0 && min_distance_mm < alert_threshold_mm_) {
        alert_count_++;
        AD_LINFO(Sentinel) << "[Sentinel][USS] FUSED PROXIMITY ALERT"
                   << " min_dist=" << min_distance_mm << "mm"
                   << " points=" << points.size();
        if (alert_cb_) alert_cb_(data, min_distance_mm);
    }
}

// ---- /parking/pdc/pdc_info 回调 ----
// 消息结构：PdcOutput
//   timestampNs : UInt64
//   seq         : UInt16
//   sectorDis   : List(UInt16)  各扇区距离（mm）
//   sourceFlag  : List(UInt8)   各扇区数据来源标志

void UssSubscriber::onPdc(
    const std::shared_ptr<ReceivedMsg<senseAD::msg::avp_pdc::PdcOutput>>& msg)
{
    auto m = msg->Msg();

    UssData data;
    data.timestamp_ns = m.getTimestampNs();
    data.seq          = m.getSeq();

    for (auto d : m.getSectorDis())  data.sector_dis.push_back(d);
    for (auto f : m.getSourceFlag()) data.source_flag.push_back(f);

    const uint16_t min_dist = data.minDistance();

    if (min_dist > 0 && min_dist < alert_threshold_mm_) {
        alert_count_++;
        AD_LINFO(Sentinel) << "[Sentinel][USS] PROXIMITY ALERT"
                   << " min_dist=" << min_dist << "mm"
                   << " sectors=" << data.sector_dis.size();
        if (alert_cb_) alert_cb_(data, min_dist);
    }

    {
        std::lock_guard<std::mutex> lk(pdc_mutex_);
        latest_pdc_ = std::move(data);
    }
}

// ---- /parking/meb/meb_info 回调 ----
// 消息结构：MebCollisionMsg
//   header    : StdHeader.Header
//   collision : Collision
//     distance      : Float32  预期碰撞点距离（m）
//     obstacleType  : UInt8    障碍物类型
//     car           : Point    车辆碰撞点
//     obj           : Point    障碍物坐标
//   direction : Direction  轨迹方向
//   pathInfo  : PathInfo   瞬时轨迹边界

void UssSubscriber::onMeb(
    const std::shared_ptr<ReceivedMsg<senseAD::msg::avp_meb::MebCollisionMsg>>& msg)
{
    auto m = msg->Msg();

    UssCollision col;
    col.timestamp_ns  = m.getHeader().getTime().getNanoSec();
    col.distance      = m.getCollision().getDistance();
    col.obstacle_type = m.getCollision().getObstacleType();
    col.valid         = true;

    AD_LINFO(Sentinel) << "[Sentinel][USS] MEB collision"
               << " distance=" << col.distance << "m"
               << " obstacle_type=" << static_cast<int>(col.obstacle_type);

    {
        std::lock_guard<std::mutex> lk(meb_mutex_);
        latest_collision_ = col;
    }
}

}  // namespace sentinel
