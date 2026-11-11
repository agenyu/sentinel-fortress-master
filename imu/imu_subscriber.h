#pragma once

#include <ad_msg_idl/ad_sensor/sensor.capnp.h>
#include "rscl_access/rscl_access.h"

#include <mutex>
#include <atomic>
#include <functional>
#include <cmath>
#include <deque>
#include <optional>

namespace sentinel {

// IMU 标定补偿参数（从 imu-intrinsic.json 加载）
struct ImuBiasCompensation {
    float accel_bias_x{0.f}, accel_bias_y{0.f}, accel_bias_z{0.f};
    float gyro_bias_x{0.f},  gyro_bias_y{0.f},  gyro_bias_z{0.f};
    float accel_scale_x{0.f}, accel_scale_y{0.f}, accel_scale_z{0.f};
    float gyro_scale_x{0.f},  gyro_scale_y{0.f},  gyro_scale_z{0.f};
};

// IMU 数据快照（POD，可安全拷贝）
struct ImuData {
    float    accel_x{0.f}, accel_y{0.f}, accel_z{0.f};  // m/s²，车体坐标系 X前/Y左/Z上
    float    gyro_x{0.f},  gyro_y{0.f},  gyro_z{0.f};   // rad/s
    uint64_t timestamp_ns{0};
    uint64_t raw_timestamp_ns{0};       // IMU actualTimeNs（可能回退）
    bool     timestamp_fallback{false}; // 本帧是否使用了单调时间回退
    bool     vibration_alert{false};
    bool     severe_vibration_alert{false};
    bool     impact_alert{false};       // 瞬时冲击/拉门类事件（delta/jerk）
    float    delta_accel{0.f};          // 与上一帧加速度向量差值模长
    float    delta_gyro{0.f};           // 与上一帧角速度向量差值模长
    float    vibration_accel_rms{0.f};  // 高通后三轴加速度滑窗 RMS
    float    vibration_gyro_rms{0.f};   // 高通后三轴角速度滑窗 RMS

    float accelMagnitudeXY() const {
        return std::sqrt(accel_x * accel_x + accel_y * accel_y);
    }
    float gyroMagnitude() const {
        return std::sqrt(gyro_x * gyro_x + gyro_y * gyro_y + gyro_z * gyro_z);
    }
};

// 基于 5 个实测包的在线振动检测参数。RMS 阈值与原始瞬时幅值阈值分开，
// 避免把重力/姿态分量当成振动，也避免稳定的机械背景振动误报。
struct ImuDetectionConfig {
    float highpass_cutoff_hz{2.0f};
    int   rms_window_ms{500};
    int   min_duration_ms{300};
    int   latch_ms{3000};
    float accel_rms_threshold{0.14f};
    float gyro_rms_threshold{0.0015f};
    float severe_accel_rms_threshold{0.20f};
    float severe_gyro_rms_threshold{0.0040f};
    float accel_guard_rms{0.04f};
    float impact_delta_accel_threshold{1.0f};
    float impact_delta_gyro_threshold{0.05f};
};

// 振动事件回调类型
using VibrationCallback = std::function<void(const ImuData&)>;

/**
 * @brief IMU 订阅器
 *
 * 订阅 /sensor/imu，维护最新 IMU 快照，并在振动超阈值时触发回调。
 * 线程安全：latest() 可在任意线程调用。
 */
class ImuSubscriber {
public:
    explicit ImuSubscriber(const ImuDetectionConfig& config = ImuDetectionConfig{});

    // 设置标定补偿参数（可选，在 subscribe 前调用）
    void setCalibration(const ImuBiasCompensation& calib) { calib_ = calib; }

    // 注册到 RSCL，必须在 RsclAccess::init() 之后调用
    void subscribe(const std::string& topic = "/sensor/imu");

    // 注册振动事件回调（可选）
    void setVibrationCallback(VibrationCallback cb) { vibration_cb_ = std::move(cb); }

    // 获取最新 IMU 快照（线程安全）
    ImuData latest() const;
    ImuData latestData() const;

    // 累计振动事件次数
    uint64_t vibrationCount() const { return vibration_count_.load(); }

private:
    struct EnergySample {
        uint64_t timestamp_ns{0};
        float accel_sq{0.f};
        float gyro_sq{0.f};
    };

    void onImu(const std::shared_ptr<ReceivedMsg<senseAD::msg::sensor::Imu>>& msg);

    ImuDetectionConfig config_;

    mutable std::mutex mutex_;
    ImuData            latest_;
    bool               has_prev_{false};
    ImuData            prev_;
    bool               filter_initialized_{false};
    float              lowpass_accel_[3]{0.f, 0.f, 0.f};
    float              lowpass_gyro_[3]{0.f, 0.f, 0.f};
    std::deque<EnergySample> energy_window_;
    float              accel_energy_sum_{0.f};
    float              gyro_energy_sum_{0.f};
    uint64_t           last_raw_timestamp_ns_{0};
    uint64_t           effective_timestamp_ns_{0};
    uint64_t           general_candidate_since_ns_{0};
    uint64_t           severe_candidate_since_ns_{0};
    uint64_t           vibration_latch_until_ns_{0};
    uint64_t           severe_latch_until_ns_{0};
    uint64_t           impact_latch_until_ns_{0};
    int                confirmed_class_{0}; // 0=none, 1=general, 2=severe
    bool               impact_active_{false};
    uint64_t           timestamp_fallback_count_{0};

    std::atomic<uint64_t> vibration_count_{0};
    VibrationCallback     vibration_cb_;
    std::optional<ImuBiasCompensation> calib_;
};

}  // namespace sentinel
