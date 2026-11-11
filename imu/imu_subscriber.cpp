#include "imu_subscriber.h"
#include <ad_log/ad_log.hpp>
#include <algorithm>
#include <array>
#include <iostream>

namespace sentinel {

ImuSubscriber::ImuSubscriber(const ImuDetectionConfig& config)
    : config_(config)
{
    config_.highpass_cutoff_hz = std::max(0.1f, config_.highpass_cutoff_hz);
    config_.rms_window_ms = std::max(100, config_.rms_window_ms);
    config_.min_duration_ms = std::max(0, config_.min_duration_ms);
    config_.latch_ms = std::max(0, config_.latch_ms);
}

void ImuSubscriber::subscribe(const std::string& topic) {
    RsclAccess::Instance()->subscribe(topic, &ImuSubscriber::onImu, this);
    std::cout << "[ImuSubscriber] Subscribed to " << topic
              << " detector=highpass_rms"
              << " cutoff_hz=" << config_.highpass_cutoff_hz
              << " window_ms=" << config_.rms_window_ms
              << " min_duration_ms=" << config_.min_duration_ms
              << " accel_rms=" << config_.accel_rms_threshold
              << " gyro_rms=" << config_.gyro_rms_threshold
              << " severe_accel_rms=" << config_.severe_accel_rms_threshold
              << " severe_gyro_rms=" << config_.severe_gyro_rms_threshold
              << std::endl;
}

ImuData ImuSubscriber::latest() const {
    std::lock_guard<std::mutex> lk(mutex_);
    return latest_;
}

ImuData ImuSubscriber::latestData() const {
    return latest();
}

void ImuSubscriber::onImu(
    const std::shared_ptr<ReceivedMsg<senseAD::msg::sensor::Imu>>& msg)
{
    auto m = msg->Msg();

    ImuData data;
    data.accel_x      = m.getAccelX();
    data.accel_y      = m.getAccelY();
    data.accel_z      = m.getAccelZ();
    data.gyro_x       = m.getGyroX();
    data.gyro_y       = m.getGyroY();
    data.gyro_z       = m.getGyroZ();
    data.raw_timestamp_ns = m.getActualTimeNs();

    if (!std::isfinite(data.accel_x) || !std::isfinite(data.accel_y) ||
        !std::isfinite(data.accel_z) || !std::isfinite(data.gyro_x) ||
        !std::isfinite(data.gyro_y) || !std::isfinite(data.gyro_z)) {
        AD_LERROR(Sentinel) << "[Sentinel][IMU] drop non-finite sample";
        return;
    }

    // 应用标定补偿：去除零偏和标度因子误差
    // corrected = (raw - bias) / (1 + scale)
    if (calib_.has_value()) {
        const auto& c = calib_.value();
        data.accel_x = (data.accel_x - c.accel_bias_x) / (1.0f + c.accel_scale_x);
        data.accel_y = (data.accel_y - c.accel_bias_y) / (1.0f + c.accel_scale_y);
        data.accel_z = (data.accel_z - c.accel_bias_z) / (1.0f + c.accel_scale_z);
        data.gyro_x  = (data.gyro_x  - c.gyro_bias_x)  / (1.0f + c.gyro_scale_x);
        data.gyro_y  = (data.gyro_y  - c.gyro_bias_y)  / (1.0f + c.gyro_scale_y);
        data.gyro_z  = (data.gyro_z  - c.gyro_bias_z)  / (1.0f + c.gyro_scale_z);
    }

    bool new_vibration_event = false;
    bool new_severe_event = false;
    bool raw_impact = false;
    bool new_impact_event = false;
    uint64_t timestamp_fallback_count_snapshot = 0;

    {
        std::lock_guard<std::mutex> lk(mutex_);

        // 实测 rsclbag 的 actualTimeNs 周期性回退 60~110 ms。滤波、持续时间和锁存
        // 不能直接用它计时：对不在 2~30 ms 的相邻间隔按 100 Hz 回退，
        // 但保留 raw_timestamp_ns 便于排查源时钟。
        constexpr uint64_t kFallbackPeriodNs = 10000000ULL;
        uint64_t step_ns = kFallbackPeriodNs;
        bool valid_raw_delta = false;
        if (last_raw_timestamp_ns_ > 0 && data.raw_timestamp_ns > last_raw_timestamp_ns_) {
            const uint64_t raw_delta = data.raw_timestamp_ns - last_raw_timestamp_ns_;
            valid_raw_delta = raw_delta >= 2000000ULL && raw_delta <= 30000000ULL;
            if (valid_raw_delta) step_ns = raw_delta;
        }
        if (effective_timestamp_ns_ == 0) {
            effective_timestamp_ns_ = data.raw_timestamp_ns > 0
                ? data.raw_timestamp_ns : kFallbackPeriodNs;
            data.timestamp_fallback = data.raw_timestamp_ns == 0;
        } else {
            effective_timestamp_ns_ += step_ns;
            data.timestamp_fallback = !valid_raw_delta;
        }
        last_raw_timestamp_ns_ = data.raw_timestamp_ns;
        data.timestamp_ns = effective_timestamp_ns_;
        if (data.timestamp_fallback) {
            ++timestamp_fallback_count_;
            if (timestamp_fallback_count_ <= 3 || timestamp_fallback_count_ % 500 == 0) {
                std::cout << "[Sentinel][IMU][TimestampFallback] count="
                          << timestamp_fallback_count_
                          << " raw_ns=" << data.raw_timestamp_ns
                          << " effective_ns=" << data.timestamp_ns << std::endl;
            }
        }

        if (has_prev_) {
            const float dax = data.accel_x - prev_.accel_x;
            const float day = data.accel_y - prev_.accel_y;
            const float daz = data.accel_z - prev_.accel_z;
            const float dgx = data.gyro_x - prev_.gyro_x;
            const float dgy = data.gyro_y - prev_.gyro_y;
            const float dgz = data.gyro_z - prev_.gyro_z;
            data.delta_accel = std::sqrt(dax * dax + day * day + daz * daz);
            data.delta_gyro = std::sqrt(dgx * dgx + dgy * dgy + dgz * dgz);
        }

        const float dt_sec = static_cast<float>(step_ns) * 1.0e-9f;
        constexpr float kPi = 3.14159265358979323846f;
        const float alpha = std::exp(-2.0f * kPi *
                                     config_.highpass_cutoff_hz * dt_sec);
        const std::array<float, 3> accel{data.accel_x, data.accel_y, data.accel_z};
        const std::array<float, 3> gyro{data.gyro_x, data.gyro_y, data.gyro_z};
        float accel_sq = 0.f;
        float gyro_sq = 0.f;
        if (!filter_initialized_) {
            for (size_t i = 0; i < 3; ++i) {
                lowpass_accel_[i] = accel[i];
                lowpass_gyro_[i] = gyro[i];
            }
            filter_initialized_ = true;
        } else {
            for (size_t i = 0; i < 3; ++i) {
                lowpass_accel_[i] = alpha * lowpass_accel_[i] + (1.f - alpha) * accel[i];
                lowpass_gyro_[i] = alpha * lowpass_gyro_[i] + (1.f - alpha) * gyro[i];
                const float highpass_accel = accel[i] - lowpass_accel_[i];
                const float highpass_gyro = gyro[i] - lowpass_gyro_[i];
                accel_sq += highpass_accel * highpass_accel;
                gyro_sq += highpass_gyro * highpass_gyro;
            }
        }

        energy_window_.push_back({data.timestamp_ns, accel_sq, gyro_sq});
        accel_energy_sum_ += accel_sq;
        gyro_energy_sum_ += gyro_sq;
        const uint64_t window_ns = static_cast<uint64_t>(config_.rms_window_ms) * 1000000ULL;
        while (!energy_window_.empty() &&
               data.timestamp_ns - energy_window_.front().timestamp_ns > window_ns) {
            accel_energy_sum_ -= energy_window_.front().accel_sq;
            gyro_energy_sum_ -= energy_window_.front().gyro_sq;
            energy_window_.pop_front();
        }
        if (!energy_window_.empty()) {
            const float count = static_cast<float>(energy_window_.size());
            data.vibration_accel_rms = std::sqrt(std::max(0.f, accel_energy_sum_) / count);
            data.vibration_gyro_rms = std::sqrt(std::max(0.f, gyro_energy_sum_) / count);
        }

        const bool window_ready = energy_window_.size() >= 30 &&
            data.timestamp_ns - energy_window_.front().timestamp_ns >=
                std::min<uint64_t>(window_ns, 250000000ULL);
        const bool gyro_general = data.vibration_gyro_rms >= config_.gyro_rms_threshold &&
                                  data.vibration_accel_rms >= config_.accel_guard_rms;
        const bool gyro_severe = data.vibration_gyro_rms >= config_.severe_gyro_rms_threshold &&
                                 data.vibration_accel_rms >= config_.accel_guard_rms;
        const bool general_above = window_ready &&
            (data.vibration_accel_rms >= config_.accel_rms_threshold || gyro_general);
        const bool severe_above = window_ready &&
            (data.vibration_accel_rms >= config_.severe_accel_rms_threshold || gyro_severe);

        auto confirmedAfter = [this, &data](bool above, uint64_t& since_ns) {
            if (!above) {
                since_ns = 0;
                return false;
            }
            if (since_ns == 0) since_ns = data.timestamp_ns;
            const uint64_t duration_ns = static_cast<uint64_t>(config_.min_duration_ms) * 1000000ULL;
            return data.timestamp_ns - since_ns >= duration_ns;
        };
        const bool general_confirmed = confirmedAfter(general_above, general_candidate_since_ns_);
        const bool severe_confirmed = confirmedAfter(severe_above, severe_candidate_since_ns_);
        const int current_class = severe_confirmed ? 2 : (general_confirmed ? 1 : 0);
        new_severe_event = current_class == 2 && confirmed_class_ < 2;
        new_vibration_event = current_class > confirmed_class_;
        if (current_class > confirmed_class_) {
            confirmed_class_ = current_class;
        } else if (current_class == 0 &&
                   (vibration_latch_until_ns_ == 0 ||
                    data.timestamp_ns > vibration_latch_until_ns_)) {
            // 低于阈值且锁存也结束后才重新布防。避免同一段连续摇晃
            // 因 RMS 在阈值上下波动而被重复计数。
            confirmed_class_ = 0;
        }

        // 瞬时冲击用于识别拉门/踢碰/砸窗等高频、横向/车身平面扰动。
        // 斜坡/姿态变化可能产生较大的 3D delta_accel，但通常主要体现在 Z 轴
        // 重力分量变化，XY 平面扰动很小；若仅看 3D delta 会误报严重威胁。
        const float delta_accel_xy = has_prev_ ? std::sqrt(
            (data.accel_x - prev_.accel_x) * (data.accel_x - prev_.accel_x) +
            (data.accel_y - prev_.accel_y) * (data.accel_y - prev_.accel_y)) : 0.f;
        const bool accel_impact = has_prev_ &&
            delta_accel_xy > config_.impact_delta_accel_threshold &&
            data.accelMagnitudeXY() > 0.45f;
        const bool gyro_impact = has_prev_ &&
            data.delta_gyro > config_.impact_delta_gyro_threshold;
        raw_impact = accel_impact || gyro_impact;
        new_impact_event = raw_impact && !impact_active_;
        impact_active_ = raw_impact;

        const uint64_t latch_ns = static_cast<uint64_t>(config_.latch_ms) * 1000000ULL;
        if (current_class > 0 || raw_impact) {
            // 锁存使相机同步组不会错过 100 Hz IMU 事件。
            vibration_latch_until_ns_ = data.timestamp_ns + latch_ns;
        }
        if (current_class == 2 || raw_impact) {
            severe_latch_until_ns_ = data.timestamp_ns + latch_ns;
        }
        if (raw_impact) {
            impact_latch_until_ns_ = data.timestamp_ns + latch_ns;
        }
        const bool vibration_latched = vibration_latch_until_ns_ > 0 &&
                                       data.timestamp_ns <= vibration_latch_until_ns_;
        const bool impact_latched = impact_latch_until_ns_ > 0 &&
                                    data.timestamp_ns <= impact_latch_until_ns_;
        const bool severe_latched = severe_latch_until_ns_ > 0 &&
                                    data.timestamp_ns <= severe_latch_until_ns_;
        data.impact_alert = raw_impact || impact_latched;
        data.severe_vibration_alert = current_class == 2 || raw_impact || severe_latched;
        data.vibration_alert = current_class > 0 || raw_impact || vibration_latched;

        prev_ = data;
        has_prev_ = true;
        latest_ = data;
        timestamp_fallback_count_snapshot = timestamp_fallback_count_;
    }

    if (new_vibration_event || new_impact_event) {
        vibration_count_++;
        AD_LINFO(Sentinel) << "[Sentinel][IMU] VIBRATION ALERT"
                   << " accel_rms=" << data.vibration_accel_rms
                   << " gyro_rms=" << data.vibration_gyro_rms
                   << " delta_accel=" << data.delta_accel
                   << " delta_gyro=" << data.delta_gyro
                   << " severe=" << data.severe_vibration_alert
                   << " impact=" << raw_impact
                   << " timestamp_fallback_count=" << timestamp_fallback_count_snapshot;
        std::cout << "[Sentinel][IMU] VIBRATION ALERT"
                  << " accel_rms=" << data.vibration_accel_rms
                  << " gyro_rms=" << data.vibration_gyro_rms
                  << " delta_accel=" << data.delta_accel
                  << " delta_gyro=" << data.delta_gyro
                  << " severe=" << data.severe_vibration_alert
                  << " impact=" << raw_impact << std::endl;
        if (vibration_cb_) vibration_cb_(data);
    }
}

}  // namespace sentinel
