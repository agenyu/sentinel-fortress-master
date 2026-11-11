#pragma once

/**
 * 激光雷达信号接入
 *
 * 接口来源：东风哨兵和越野堡垒项目-软件接口设计说明书
 *
 * Topic: /sensor/lidar/*_lidar → AdsfiLidarPointCloud
 *
 * 当前车端上报点云已是车辆坐标系；默认不需要 LiDAR 外参变换。
 *
 * 消息结构：
 *   header      : StdHeader.Header
 *   isBigEndian : UInt32   是否大端
 *   height      : UInt32   点云二维结构高度
 *   width       : UInt32   点云宽度（无序点云时 = 点数）
 *   pointStep   : UInt32   单点数据字节数
 *   rowStep     : UInt32   单行数据字节数
 *   isDense     : UInt32   是否无无效点
 *   data        : Data     点云原始字节流，pointStep=16；速腾与补盲雷达padding位置不同。
 *   parityFlag  : Bool     奇偶帧标志（false=奇帧, true=偶帧）
 */

#include "rscl_access/rscl_access.h"
#include <ad_msg_idl/ad_sensor/sensor.capnp.h>
#include "lidar_point_parser.h"

#include <mutex>
#include <atomic>
#include <functional>
#include <vector>
#include <deque>
#include <cstdint>
#include <string>
#include <cmath>

namespace sentinel {

// 单个激光点（解析后，单位转换为 m）
struct LidarPoint {
    float   x{0.f};          // m（原始 int16 * 0.01）
    float   y{0.f};          // m
    float   z{0.f};          // m
    int32_t time_offset_ns{0}; // 相对 header.stamp 的时间偏移（ns）
    uint8_t intensity{0};    // 点云强度
    uint8_t ring{0};         // 激光线束编号（从1开始）
    uint8_t source_id{0};    // 来源：0=前，1=左，2=后，3=右
};

// 点云帧快照
struct LidarFrame {
    uint64_t              timestamp_ns{0};
    uint32_t              width{0};
    uint32_t              height{0};
    bool                  parity_flag{false};  // false=奇帧, true=偶帧
    std::vector<LidarPoint> points;            // 解析后的点云

    bool valid() const { return timestamp_ns > 0 && !points.empty(); }

    // 统计近距离点数（distance_m 以内）
    int countNearPoints(float distance_m) const {
        int cnt = 0;
        for (const auto& p : points) {
            if (std::sqrt(p.x*p.x + p.y*p.y) < distance_m) cnt++;
        }
        return cnt;
    }
};

// 点云帧就绪回调
using LidarFrameCallback = std::function<void(const LidarFrame&)>;

/**
 * @brief 激光雷达订阅器
 *
 * 订阅 /sensor/lidar/front_lidar，解析 AdsfiLidarPointCloud 消息，
 * 输出解析后的 LidarFrame（含 LidarPoint 数组）。
 *
 * ORSM 用途：
 *   - 与视觉目标检测融合，提升 3D 定位/测速精度
 *   - 近距离点云统计
 *
 * 哨兵模式禁止使用 LiDAR。
 */
class LidarSubscriber {
public:
    // near_alert_distance_m: 近距离报警阈值（m），默认 3m
    // near_alert_min_points: 触发报警的最少点数，默认 50 个点
    explicit LidarSubscriber(float near_alert_distance_m = 3.0f,
                             int   near_alert_min_points = 50,
                             LidarPointLayout point_layout = LidarPointLayout::BLIND_SPOT16,
                             float yaw_degrees = 0.0f,
                             uint8_t source_id = 0);

    // 注册订阅，必须在 RsclAccess::init() 之后调用
    void subscribe(const std::string& topic = "/sensor/lidar/front_lidar");

    // 设置有效量程范围（从标定参数加载）
    void setRangeLimits(float min_range_m, float max_range_m);

    // Override the yaw-derived XY mapping with an explicit 2x2 matrix:
    // [car_x, car_y]^T = [[a,b],[c,d]] * [sensor_x,sensor_y]^T.
    // This is primarily used to compare coordinate-contract hypotheses in bag replay.
    void setXYTransform(float a, float b, float c, float d);

    // 注册帧就绪回调（在回调线程调用，勿做耗时操作）
    void setFrameCallback(LidarFrameCallback cb) { frame_cb_ = std::move(cb); }

    // 线程安全读取最新帧
    LidarFrame latestFrame() const;

    // 线程安全读取最接近目标时间戳的历史帧；max_age_ms<=0 表示只选最近、不做窗口过滤。
    LidarFrame closestFrame(uint64_t target_timestamp_ns, float max_age_ms = 0.0f) const;

    uint64_t frameCount() const { return frame_count_.load(); }
    uint64_t alertCount() const { return alert_count_.load(); }

private:
    void onLidar(const std::shared_ptr<ReceivedMsg<senseAD::msg::sensor::AdsfiLidarPointCloud>>& msg);

    float near_alert_distance_m_;
    int   near_alert_min_points_;
    LidarPointLayout point_layout_;
    float yaw_degrees_{0.0f};
    uint8_t source_id_{0};
    float xy_a_{1.0f};
    float xy_b_{0.0f};
    float xy_c_{0.0f};
    float xy_d_{1.0f};
    bool  has_explicit_xy_transform_{false};

    std::string           topic_;
    LidarFrameCallback    frame_cb_;
    std::atomic<uint64_t> frame_count_{0};
    std::atomic<uint64_t> alert_count_{0};

    mutable std::mutex mutex_;
    LidarFrame         latest_frame_;
    std::deque<LidarFrame> frame_history_;

    float   calib_min_range_m_{0.f};
    float   calib_max_range_m_{0.f};
    bool    has_range_limits_{false};
};

}  // namespace sentinel
