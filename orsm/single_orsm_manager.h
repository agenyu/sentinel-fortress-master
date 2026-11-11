#pragma once
// 单车越野堡垒管理器
// 整合状态机 + 三区报警逻辑 + 人/动物分类
// 对齐 OrsmInfo.proto / senseauto-msgs: ad_msg_idl/l3_msgs/orsm_info.capnp.h

#include "orsm_state_machine.h"
#include "orsm_output.h"
#include "orsm_settings.h"
#include "detection/tracker.h"
#include "imu/imu_subscriber.h"
#include "uss/uss_subscriber.h"
#include "lidar/lidar_subscriber.h"

#include <vector>
#include <cmath>
#include <deque>
#include <unordered_map>
#include <utility>
#include <cstdint>

namespace sentinel {

/**
 * @brief 单车越野堡垒管理器
 *
 * 每帧调用 update()，输入跟踪目标、传感器数据、用户设置和ODD条件，
 * 输出完整的 ORSMAlertOutput（SOME/IP 信号）。
 *
 * 逻辑流程：
 *   1. 驱动 OrsmStateMachine
 *   2. 遍历 tracked 目标，优先使用视频-LiDAR融合位置/速度，判断所在区域（A/B/C）
 *   3. 根据 humans_recog_setting/animals_recog_setting 过滤目标类型
 *   4. 生成 IntruderInfo 列表
 *   5. 确定最高报警等级（最近目标所在区域）
 *   6. 填充 ORSMAlertOutput
 */
class SingleOrsmManager {
public:
    SingleOrsmManager();

    /**
     * @brief 每帧更新
     * @param tracked   跟踪目标列表（来自 Tracker）
     * @param imu       最新 IMU 数据（堡垒模式辅助事件/状态输入，不用于入侵目标定位）
     * @param uss       最新超声波数据
     * @param lidar     最新激光雷达帧
     * @param settings  用户设置（来自 IVI/TBOX SOME/IP 输入）
     * @param odd       ODD 条件
     * @param timestamp 当前时间戳（ns）
     * @return 完整报警输出
     */
    ORSMAlertOutput update(const std::vector<TrackedObject>& tracked,
                           const ImuData& imu,
                           const UssData& uss,
                           const LidarFrame& lidar,
                           const OrsmSettings& settings,
                           const OrsmOdd& odd,
                           uint64_t timestamp);

    OrsmState currentState() const { return sm_.currentState(); }
    void setStateCallback(OrsmStateCallback cb) { sm_.setStateCallback(std::move(cb)); }
    void resetTracking();
    void reset();

private:
    struct TrackHistory {
        float last_distance{0.f};
        float last_pose_x{0.f};
        float last_pose_y{0.f};
        uint64_t last_timestamp_ns{0};
        int stable_observations{0};
    };

    struct WarningSnapshot {
        std::vector<IntruderInfo> intruders;
        OrsmWarnLevel level{OrsmWarnLevel::LEVEL_NONE};
        uint64_t timestamp_ns{0};
    };

    // 将检测标签映射为入侵物类型
    static IntruderType classifyIntruder(const std::string& label);

    // 根据距离判断所在区域
    static OrsmWarnLevel distanceToZone(float distance,
                                        float zone_a, float zone_b, float zone_c);

    OrsmStateMachine sm_;
    std::unordered_map<int, TrackHistory> track_history_;

    // ORSM威胁输入稳定化：不改变 T3/T4 语义，只对 threat_detected 输入做确认/保持。
    // 使用滑动窗口吸收多相机/多传感器帧间抖动，避免 T,-,T,-,T 这类真实威胁被空帧打断。
    std::deque<bool> threat_confirm_window_;
    int threat_confirm_hits_{0};
    int clear_confirm_count_{0};
    uint64_t warning_hold_until_ns_{0};
    WarningSnapshot pending_warning_snapshot_;
    WarningSnapshot last_warning_snapshot_;
};

}  // namespace sentinel
