#include "lidar_subscriber.h"
#include <ad_log/ad_log.hpp>
#include <iostream>
#include <cstdlib>
#include <limits>

namespace sentinel {

namespace {
constexpr size_t kMaxLidarHistoryFrames = 40;
constexpr uint64_t kMaxLidarHistorySpanNs = 5ULL * 1000000000ULL;
constexpr uint32_t kRsclPointStep = 16;

uint64_t absDiffNs(uint64_t a, uint64_t b) {
    return a > b ? a - b : b - a;
}
}  // namespace

LidarSubscriber::LidarSubscriber(float near_alert_distance_m,
                                 int   near_alert_min_points,
                                 LidarPointLayout point_layout,
                                 float yaw_degrees,
                                 uint8_t source_id)
    : near_alert_distance_m_(near_alert_distance_m)
    , near_alert_min_points_(near_alert_min_points)
    , point_layout_(point_layout)
    , yaw_degrees_(yaw_degrees)
    , source_id_(source_id)
{
    const float radians = yaw_degrees * static_cast<float>(M_PI) / 180.0f;
    const float c = std::cos(radians);
    const float s = std::sin(radians);
    xy_a_ = c;
    xy_b_ = -s;
    xy_c_ = s;
    xy_d_ = c;
}

void LidarSubscriber::setXYTransform(float a, float b, float c, float d) {
    xy_a_ = a;
    xy_b_ = b;
    xy_c_ = c;
    xy_d_ = d;
    has_explicit_xy_transform_ = true;
}

void LidarSubscriber::subscribe(const std::string& topic) {
    topic_ = topic;
    RsclAccess::Instance()->subscribe(topic, &LidarSubscriber::onLidar, this);
    std::cout << "[LidarSubscriber] Subscribed to " << topic << std::endl;
}

void LidarSubscriber::setRangeLimits(float min_range_m, float max_range_m) {
    calib_min_range_m_ = min_range_m;
    calib_max_range_m_ = max_range_m;
    has_range_limits_ = (max_range_m > min_range_m);
    std::cout << "[LidarSubscriber] Range limits: [" << min_range_m << ", " << max_range_m << "]m" << std::endl;
}

LidarFrame LidarSubscriber::latestFrame() const {
    std::lock_guard<std::mutex> lk(mutex_);
    return latest_frame_;
}

LidarFrame LidarSubscriber::closestFrame(uint64_t target_timestamp_ns, float max_age_ms) const {
    std::lock_guard<std::mutex> lk(mutex_);
    if (target_timestamp_ns == 0 || frame_history_.empty()) {
        return latest_frame_;
    }

    const uint64_t max_age_ns = max_age_ms > 0.0f
        ? static_cast<uint64_t>(max_age_ms * 1.0e6f)
        : 0;
    uint64_t best_delta = std::numeric_limits<uint64_t>::max();
    const LidarFrame* best = nullptr;
    for (const auto& frame : frame_history_) {
        if (!frame.valid()) continue;
        const uint64_t delta = absDiffNs(frame.timestamp_ns, target_timestamp_ns);
        if (max_age_ns > 0 && delta > max_age_ns) continue;
        if (delta < best_delta) {
            best_delta = delta;
            best = &frame;
        }
    }
    return best ? *best : LidarFrame{};
}

// ---- /sensor/lidar/front_lidar 回调 ----
// 消息结构：AdsfiLidarPointCloud
//   header      : StdHeader.Header
//   isBigEndian : UInt32
//   height      : UInt32
//   width       : UInt32   无序点云时 = 点数
//   pointStep   : UInt32   单点字节数（RSCL文档规定为16）
//   rowStep     : UInt32
//   isDense     : UInt32
//   data        : Data     原始字节流；按point_layout_选择速腾/补盲字段偏移
//   parityFlag  : Bool     false=奇帧, true=偶帧

void LidarSubscriber::onLidar(
    const std::shared_ptr<ReceivedMsg<senseAD::msg::sensor::AdsfiLidarPointCloud>>& msg)
{
    auto m = msg->Msg();

    LidarFrame frame;
    frame.timestamp_ns = m.getHeader().getTime().getNanoSec();
    frame.width        = m.getWidth();
    frame.height       = m.getHeight();
    // 当前 rscl 容器内的 AdsfiLidarPointCloud 未提供 parityFlag 字段，保留默认 false。
    frame.parity_flag  = false;

    const uint32_t point_step = m.getPointStep();
    const auto     raw_data   = m.getData();
    const size_t   data_size  = raw_data.size();

    static std::atomic<uint64_t> callback_count{0};
    const uint64_t cb_idx = ++callback_count;
    if (cb_idx <= 12 || cb_idx % 200 == 0) {
        std::cout << "[LidarSubscriber][Raw] topic=" << topic_
                  << " cb=" << cb_idx
                  << " ts=" << frame.timestamp_ns
                  << " width=" << frame.width
                  << " height=" << frame.height
                  << " point_step=" << point_step
                  << " point_layout=" << lidarPointLayoutName(point_layout_)
                  << " data_size=" << data_size
                  << std::endl;
    }

    if (point_step < kRsclPointStep) {
        std::cout << "[LidarSubscriber][Skip] topic=" << topic_
                  << " point_step=" << point_step
                  << " < required=" << kRsclPointStep
                  << std::endl;
        AD_LINFO(Sentinel) << "[LidarSubscriber] WARNING: pointStep=" << point_step
                   << " < required=" << kRsclPointStep
                   << ", skipping frame";
        return;
    }
    if (data_size % point_step != 0) {
        std::cout << "[LidarSubscriber][Warn] topic=" << topic_
                  << " data_size=" << data_size
                  << " is not divisible by point_step=" << point_step
                  << std::endl;
    }

    const size_t buffer_points = point_step > 0 ? data_size / point_step : 0;
    const uint64_t declared_points_u64 =
        static_cast<uint64_t>(frame.width) * static_cast<uint64_t>(frame.height);
    const size_t declared_points = declared_points_u64 > std::numeric_limits<size_t>::max()
        ? std::numeric_limits<size_t>::max()
        : static_cast<size_t>(declared_points_u64);
    const size_t num_points = declared_points > 0
        ? std::min(declared_points, buffer_points)
        : buffer_points;
    if (declared_points > buffer_points) {
        std::cout << "[LidarSubscriber][Warn] topic=" << topic_
                  << " declared_points=" << declared_points
                  << " exceeds buffer_points=" << buffer_points
                  << ", truncating" << std::endl;
    } else if (declared_points > 0 && declared_points < buffer_points &&
               (cb_idx <= 12 || cb_idx % 200 == 0)) {
        std::cout << "[LidarSubscriber][Padding] topic=" << topic_
                  << " valid_points=" << declared_points
                  << " buffer_capacity_points=" << buffer_points
                  << std::endl;
    }
    frame.points.reserve(num_points);

    const char* lidar_debug_env = std::getenv("ORSM_LIDAR_DEBUG");
    const bool lidar_debug = lidar_debug_env && std::string(lidar_debug_env) == "1" &&
                             frame_count_.load() < 3;
    float raw_min_x = std::numeric_limits<float>::max();
    float raw_max_x = std::numeric_limits<float>::lowest();
    float raw_min_y = std::numeric_limits<float>::max();
    float raw_max_y = std::numeric_limits<float>::lowest();
    float car_min_x = std::numeric_limits<float>::max();
    float car_max_x = std::numeric_limits<float>::lowest();
    float car_min_y = std::numeric_limits<float>::max();
    float car_max_y = std::numeric_limits<float>::lowest();
    int32_t min_time_offset_ns = std::numeric_limits<int32_t>::max();
    int32_t max_time_offset_ns = std::numeric_limits<int32_t>::lowest();
    uint8_t min_intensity = std::numeric_limits<uint8_t>::max();
    uint8_t max_intensity = std::numeric_limits<uint8_t>::lowest();
    uint8_t min_ring = std::numeric_limits<uint8_t>::max();
    uint8_t max_ring = std::numeric_limits<uint8_t>::lowest();

    const uint8_t* ptr = reinterpret_cast<const uint8_t*>(raw_data.begin());

    for (size_t i = 0; i < num_points; ++i) {
        const uint8_t* point_data = ptr + i * point_step;
        DecodedLidarPoint16 raw_point;
        if (!decodeLidarPoint16(point_data, point_step, point_layout_, raw_point)) continue;

        const float sensor_x = raw_point.x_cm * 0.01f;
        const float sensor_y = raw_point.y_cm * 0.01f;

        LidarPoint p;
        // 部分RSCL包按每个雷达自身坐标系保存点云（局部x朝雷达正前方）。
        // yaw=0保持输入不变；非零时绕车辆z轴旋转到x前/y左的统一车体方向。
        p.x              = xy_a_ * sensor_x + xy_b_ * sensor_y;
        p.y              = xy_c_ * sensor_x + xy_d_ * sensor_y;
        p.z              = raw_point.z_cm * 0.01f;
        p.time_offset_ns = raw_point.time_offset_ns;
        p.intensity      = raw_point.intensity;
        p.ring           = raw_point.ring;
        p.source_id      = source_id_;

        if (lidar_debug) {
            raw_min_x = std::min(raw_min_x, sensor_x);
            raw_max_x = std::max(raw_max_x, sensor_x);
            raw_min_y = std::min(raw_min_y, sensor_y);
            raw_max_y = std::max(raw_max_y, sensor_y);
            min_time_offset_ns = std::min(min_time_offset_ns, p.time_offset_ns);
            max_time_offset_ns = std::max(max_time_offset_ns, p.time_offset_ns);
            min_intensity = std::min(min_intensity, p.intensity);
            max_intensity = std::max(max_intensity, p.intensity);
            min_ring = std::min(min_ring, p.ring);
            max_ring = std::max(max_ring, p.ring);
        }

        // 量程过滤（使用标定参数中的 min/max_range_m）
        if (has_range_limits_) {
            float dist = std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
            if (dist < calib_min_range_m_ || dist > calib_max_range_m_) continue;
        }

        if (lidar_debug) {
            car_min_x = std::min(car_min_x, p.x);
            car_max_x = std::max(car_max_x, p.x);
            car_min_y = std::min(car_min_y, p.y);
            car_max_y = std::max(car_max_y, p.y);
        }

        frame.points.push_back(p);
    }

    if (lidar_debug) {
        std::cout << "[LiDARDiag] topic=" << topic_
                  << " raw_x_range=[" << raw_min_x << "," << raw_max_x << "]"
                  << " raw_y_range=[" << raw_min_y << "," << raw_max_y << "]"
                  << " car_x_range=[" << car_min_x << "," << car_max_x << "]"
                  << " car_y_range=[" << car_min_y << "," << car_max_y << "]"
                  << " time_offset_ns_range=[" << min_time_offset_ns << "," << max_time_offset_ns << "]"
                  << " intensity_range=[" << static_cast<unsigned>(min_intensity)
                  << "," << static_cast<unsigned>(max_intensity) << "]"
                  << " ring_range=[" << static_cast<unsigned>(min_ring)
                  << "," << static_cast<unsigned>(max_ring) << "]"
                  << " point_layout=" << lidarPointLayoutName(point_layout_)
                  << " yaw_degrees=" << yaw_degrees_
                  << " xy_transform=[" << xy_a_ << "," << xy_b_
                  << ";" << xy_c_ << "," << xy_d_ << "]"
                  << " coordinate_frame=vehicle("
                  << (has_explicit_xy_transform_ ? "explicit_xy_transform" : "yaw_rotated")
                  << ")"
                  << std::endl;
    }

    frame_count_++;

    // 近距离入侵检测
    const int near_count = frame.countNearPoints(near_alert_distance_m_);
    if (near_count >= near_alert_min_points_) {
        alert_count_++;
        AD_LINFO(Sentinel) << "[Sentinel][LiDAR] PROXIMITY ALERT"
                   << " near_points=" << near_count
                   << " within " << near_alert_distance_m_ << "m"
                   << " total_points=" << num_points;
    }

    std::cout << "[LiDAR] ts=" << (frame.timestamp_ns / 1000000) << "ms"
              << " points=" << num_points
              << " parity=" << frame.parity_flag
              << " near(" << near_alert_distance_m_ << "m)=" << near_count
              << std::endl;

    if (frame_cb_) frame_cb_(frame);

    {
        std::lock_guard<std::mutex> lk(mutex_);
        latest_frame_ = frame;
        frame_history_.push_back(std::move(frame));
        while (frame_history_.size() > kMaxLidarHistoryFrames) {
            frame_history_.pop_front();
        }
        while (frame_history_.size() > 1 &&
               frame_history_.back().timestamp_ns > frame_history_.front().timestamp_ns &&
               frame_history_.back().timestamp_ns - frame_history_.front().timestamp_ns > kMaxLidarHistorySpanNs) {
            frame_history_.pop_front();
        }
    }
}

}  // namespace sentinel
