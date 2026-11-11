#include "rscl_publisher.h"

#include <algorithm>
#include <iostream>

// senseauto-msgs V2.6.0 已提供正式 l3_msgs 消息定义。
#include <ad_msg_idl/l3_msgs/alarm_ctrl.capnp.h>
#include <ad_msg_idl/l3_msgs/orsm_info.capnp.h>
#include <ad_msg_idl/l3_msgs/sentry_mode.capnp.h>
#include <state_broadcast.capnp.h>

namespace sentinel {

namespace l3 = senseAD::msg::l3;
namespace sb = senseAD::msg::sentinel;

RsclPublisher::RsclPublisher(const PublishConfig& cfg)
    : cfg_(cfg)
{}

bool RsclPublisher::init() {
    if (!cfg_.enable_rscl_publish) {
        std::cout << "[RsclPublisher] RSCL publish DISABLED (enable_rscl_publish=false)" << std::endl;
        return true;
    }

    bool ok = true;
    ok &= RsclAccess::Instance()->advertise<l3::SentinelResStatus>(cfg_.sentry_event_topic);
    ok &= RsclAccess::Instance()->advertise<sb::StateBroadcast>(cfg_.sentry_state_topic);
    ok &= RsclAccess::Instance()->advertise<l3::AlarmCtrl>(cfg_.sentry_action_topic);
    ok &= RsclAccess::Instance()->advertise<l3::OrsmStatus>(cfg_.orsm_state_topic);

    if (ok) {
        initialized_ = true;
        std::cout << "[RsclPublisher] RSCL status publish ENABLED, topics advertised:" << std::endl;
        std::cout << "[RsclPublisher]   " << cfg_.sentry_event_topic << std::endl;
        std::cout << "[RsclPublisher]   " << cfg_.sentry_state_topic
                  << " (StateBroadcast Cap'n Proto ~10Hz)" << std::endl;
        std::cout << "[RsclPublisher]   " << cfg_.sentry_action_topic << std::endl;
        std::cout << "[RsclPublisher]   " << cfg_.orsm_state_topic << std::endl;
        std::cout << "[RsclPublisher] ORSM action topic remains disabled: "
                  << cfg_.orsm_action_topic << std::endl;
    } else {
        std::cerr << "[RsclPublisher] WARNING: some topics failed to advertise" << std::endl;
    }
    return ok;
}

void RsclPublisher::publishSentryEvent(const AlertEvent& evt,
                                       const SentinelResStatusView& res,
                                       uint64_t event_id) {
    if (!enabled()) return;

    (void)evt;
    (void)event_id;

    SendMsg<l3::SentinelResStatus> send_msg;
    auto builder = send_msg.MsgBuilder();
    auto header = builder.initHeader();
    header.setFrameId("sentinel_perception");
    if (evt.timestamp_ns > 0) {
        header.initTime().setNanoSec(evt.timestamp_ns);
    }
    builder.setSentinelResMsgType(
        static_cast<l3::SentinelResMsgType>(static_cast<uint16_t>(res.sentinel_res_msg_type)));
    builder.setSentinelOddStatus(
        static_cast<l3::SentinelODDStatus>(static_cast<uint16_t>(res.sentinel_odd_status)));
    builder.setSentinelEnableFailed(
        static_cast<l3::SentinelEnableFailed>(static_cast<uint16_t>(res.sentinel_enable_failed)));
    builder.setSentinelGeneralThreat(
        static_cast<l3::SentinelGeneralThreat>(static_cast<uint16_t>(res.sentinel_general_threat)));
    builder.setSentinelSevereThreat(
        static_cast<l3::SentinelGeneralThreat>(static_cast<uint16_t>(res.sentinel_severe_threat)));
    builder.setSentinelAudioVisualAlarm(
        static_cast<l3::SentinelGeneralThreat>(static_cast<uint16_t>(res.sentinel_audio_visual_alarm)));
    builder.setSentinelObstacleType(
        static_cast<l3::SentinelObstacleType>(static_cast<uint16_t>(res.sentinel_obstacle_type)));

    RsclAccess::Instance()->publish(cfg_.sentry_event_topic, send_msg);
}

void RsclPublisher::publishSentryStateBroadcast(const StateBroadcastView& msg) {
    if (!enabled()) return;

    SendMsg<sb::StateBroadcast> send_msg;
    auto builder = send_msg.MsgBuilder();
    builder.setTimestampNs(msg.timestamp_ns);
    builder.setSentryState(msg.sentry_state);
    builder.setSentryStateName(msg.sentry_state_name);
    builder.setThreatLevel(msg.threat_level);
    builder.setThreatLevelName(msg.threat_level_name);
    builder.setRawThreatId(msg.raw_threat_id);
    builder.setSmThreatId(msg.sm_threat_id);
    builder.setLocalTrackId(msg.local_track_id);
    builder.setThreatCamera(msg.threat_camera);
    builder.setTargetCount(msg.target_count);
    builder.setObstacleType(msg.obstacle_type);
    builder.setObstacleTypeName(msg.obstacle_type_name);
    builder.setReasonCode(msg.reason_code);
    builder.setReason(msg.reason);
    builder.setSummary(msg.summary);

    RsclAccess::Instance()->publish(cfg_.sentry_state_topic, send_msg);
}

void RsclPublisher::publishSentryAction(const AlertEvent& evt,
                                        const SentinelResStatusView& res) {
    if (!enabled()) return;
    if (evt.level < ThreatLevel::HIGH) return;

    (void)res;

    SendMsg<l3::AlarmCtrl> send_msg;
    auto builder = send_msg.MsgBuilder();
    auto header = builder.initHeader();
    header.setFrameId("sentinel_perception");
    builder.setReqModule("sentinel_perception");
    // 复用 L3 AlarmCtrl 中已有声光控制字段承载哨兵/OIB 调测请求。
    // 设计文档：一般威胁只唤醒中控/录像，不触发音响、喇叭、大灯；严重威胁触发声光报警。
    constexpr uint16_t kSentrySevereHazardTicks = 50;  // 5s, AlarmCtrl uses 100ms resolution.
    constexpr uint8_t kSentrySevereHornTicks = 50;
    builder.setRdaHzrdLampTime(evt.hazard_lamp_req ? kSentrySevereHazardTicks : 0);
    builder.setRdaHzrdLamponDurn(evt.hazard_lamp_req ? 1 : 0);
    builder.setRdaHzrdLampoffDurn(evt.hazard_lamp_req ? 1 : 0);
    builder.setPedWarnToneCtrl(evt.horn_req ? kSentrySevereHornTicks : 0);
    builder.setAebAlertReq(evt.light_req || evt.horn_req || evt.hazard_lamp_req ? 1 : 0);

    RsclAccess::Instance()->publish(cfg_.sentry_action_topic, send_msg);
}

namespace {

uint16_t orsmDurationTicks(AlarmSoundDuration duration) {
    switch (duration) {
    case AlarmSoundDuration::REMINDER_3S:  return 30;   // AlarmCtrl uses 100ms resolution.
    case AlarmSoundDuration::REMINDER_5S:  return 50;
    case AlarmSoundDuration::REMINDER_10S: return 100;
    case AlarmSoundDuration::NO_REMINDER:
    default:                               return 0;
    }
}

uint16_t defaultOrsmWarningDurationTicks(OrsmWarnLevel level) {
    // 设计文档：I/II/III 级报警的声音提醒时长由用户设置的 3s/5s/10s 决定；
    // 当设置为 NO_REMINDER 但仍需要灯光/双闪动作时，给 OIB 一个保守的默认动作时长。
    switch (level) {
    case OrsmWarnLevel::WARNLEVEL_1: return 30;
    case OrsmWarnLevel::WARNLEVEL_2: return 50;
    case OrsmWarnLevel::WARNLEVEL_3: return 100;
    case OrsmWarnLevel::LEVEL_NONE:
    default:                         return 0;
    }
}

bool orsmIsMultiVehicle(const ORSMAlertOutput& output) {
    return output.orsm_setting_st_fb == OrsmSettingType::MULTI_VEHICLE_JOINT_SECURITY_ON;
}

bool orsmNeedsHazardLamp(const ORSMAlertOutput& output) {
    if (output.alarm_light_setting_fb != BasicSwitchResp::ON) return false;
    if (output.orsm_warning_st != OrsmWarningStatus::WARNING_ON) return false;
    if (output.orsm_warn_level == OrsmWarnLevel::LEVEL_NONE) return false;

    // 设计摘录：
    // - 单车：多个入侵物 I/II/III 级均发送 OIB 双闪灯控制请求；单个 B/C 区更多是位置灯/近光灯。
    // - 多车：I/II 级报警均发送 OIB 双闪灯控制请求。
    // 当前 AlarmCtrl 没有独立 HAD_LightReq(位置灯/近光灯) 字段，所以双闪只在文档明确要求
    // 的“多车联防”或“多个入侵物”场景输出；单个入侵物的灯光等级通过 AebAlertReq 透传。
    return orsmIsMultiVehicle(output) ||
           output.intruders_num == IntrudersNumType::MULTIPLE_INTRUDERS;
}

bool orsmIsSafeCrossingAlert(const ORSMAlertOutput& output) {
    // 单车安防：激活瞬间已有的安全生物越过 A 区边界时，OrsmStatus 只置 WARNING_ON，
    // 不给普通入侵物等级/列表。该形态用于触发位置灯/近光灯 5s + 声音提醒的降级动作。
    return output.orsm_setting_st_fb == OrsmSettingType::SINGLE_AUTO_SECURITY_ON &&
           output.orsm_warning_st == OrsmWarningStatus::WARNING_ON &&
           output.orsm_warn_level == OrsmWarnLevel::LEVEL_NONE &&
           output.intruders_num == IntrudersNumType::NO_INTRUDER &&
           output.intruders_info.empty();
}

bool orsmNeedsSound(const ORSMAlertOutput& output) {
    if (output.alarm_horn_setting_fb != BasicSwitchResp::ON) return false;
    if (output.orsm_warning_st != OrsmWarningStatus::WARNING_ON) return false;
    if (orsmIsSafeCrossingAlert(output)) return true;
    if (output.orsm_warn_level == OrsmWarnLevel::LEVEL_NONE) return false;

    // 单车单个入侵物 A 区：设计仅唤醒 APP/HMI、推送提醒，不强制声光；
    // 单车单个入侵物 B/C 区、多入侵物、以及多车联防 I/II 级均按用户设置声音提醒。
    if (!orsmIsMultiVehicle(output) &&
        output.intruders_num == IntrudersNumType::SINGLE_INTRUDER &&
        output.orsm_warn_level == OrsmWarnLevel::WARNLEVEL_1) {
        return false;
    }
    return true;
}

}  // namespace

void RsclPublisher::publishOrsmState(const ORSMAlertOutput& output, uint64_t timestamp_ns) {
    if (!enabled()) return;

    SendMsg<l3::OrsmStatus> send_msg;
    auto builder = send_msg.MsgBuilder();
    auto header = builder.initHeader();
    header.setFrameId("sentinel_perception");
    if (timestamp_ns > 0) {
        header.initTime().setNanoSec(timestamp_ns);
    }
    builder.setOrsmSettingStFb(
        static_cast<l3::OrsmSettingType>(static_cast<uint16_t>(output.orsm_setting_st_fb)));
    builder.setHumansRecogSettingFb(
        static_cast<l3::BasicSwitchResp>(static_cast<uint16_t>(output.humans_recog_setting_fb)));
    builder.setAnimalsRecogSettingFb(
        static_cast<l3::BasicSwitchResp>(static_cast<uint16_t>(output.animals_recog_setting_fb)));
    builder.setDetecRangeSettingFb(
        static_cast<l3::DetecRangeSetting>(static_cast<uint16_t>(output.detec_range_setting_fb)));
    builder.setAlarmLightSettingFb(
        static_cast<l3::BasicSwitchResp>(static_cast<uint16_t>(output.alarm_light_setting_fb)));
    builder.setAlarmHornSettingFb(
        static_cast<l3::BasicSwitchResp>(static_cast<uint16_t>(output.alarm_horn_setting_fb)));
    builder.setAlarmSoundSettingFb(
        static_cast<l3::AlarmSoundType>(static_cast<uint16_t>(output.alarm_sound_setting_fb)));
    builder.setAlarmSndDuraSettingFb(
        static_cast<l3::AlarmSoundDuration>(static_cast<uint16_t>(output.alarm_snd_dura_setting_fb)));
    builder.setOrsmMessageRem(
        static_cast<l3::OrsmMessageReminder>(static_cast<uint16_t>(output.orsm_message_rem)));
    builder.setOrsmWarningSt(
        static_cast<l3::OrsmWarningStatus>(static_cast<uint16_t>(output.orsm_warning_st)));
    builder.setIntrudersNum(
        static_cast<l3::IntrudersNumType>(static_cast<uint16_t>(output.intruders_num)));
    builder.setOrsmWarnLevel(
        static_cast<l3::OrsmWarnLevel>(static_cast<uint16_t>(output.orsm_warn_level)));

    // Repeated fields are initialized even when empty, so downstream adapters/JSON bridges
    // consistently observe [] rather than an absent/uninitialized list.
    auto intruders_list = builder.initIntrudersInfo(output.intruders_info.size());
    for (size_t i = 0; i < output.intruders_info.size(); ++i) {
        const auto& src = output.intruders_info[i];
        auto entry = intruders_list[i];
        entry.setIntruderId(src.intruder_id);
        entry.setIntruderType(
            static_cast<l3::IntruderType>(static_cast<uint16_t>(src.intruder_type)));
        entry.setIntruderPoseX(src.intruder_pose_x);
        entry.setIntruderPoseY(src.intruder_pose_y);
        entry.setIntruderVelocityX(src.intruder_velocity_x);
        entry.setIntruderVelocityY(src.intruder_velocity_y);
        entry.setTimestampNsec(src.timestamp_nsec);
    }

    auto reserved_list = builder.initOrsmInfoReserved(output.orsm_info_reserved.size());
    for (size_t i = 0; i < output.orsm_info_reserved.size(); ++i) {
        reserved_list.set(i, output.orsm_info_reserved[i]);
    }

    RsclAccess::Instance()->publish(cfg_.orsm_state_topic, send_msg);
}

void RsclPublisher::publishOrsmAction(const ORSMAlertOutput& output, uint64_t /*timestamp_ns*/) {
    if (!enabled()) return;

    const bool warning = output.orsm_warning_st == OrsmWarningStatus::WARNING_ON;
    const bool has_level = output.orsm_warn_level != OrsmWarnLevel::LEVEL_NONE;
    const bool low_battery_exit =
        output.orsm_message_rem == OrsmMessageReminder::LOW_BATTERY_FUNCTION_EXITED;
    const bool safe_crossing_alert = orsmIsSafeCrossingAlert(output);

    if (!warning && !low_battery_exit) {
        return;
    }

    const uint16_t user_duration = orsmDurationTicks(output.alarm_snd_dura_setting_fb);
    const uint16_t warning_duration = safe_crossing_alert
                                          ? 50
                                          : (user_duration > 0
                                                 ? user_duration
                                                 : defaultOrsmWarningDurationTicks(output.orsm_warn_level));

    SendMsg<l3::AlarmCtrl> send_msg;
    auto builder = send_msg.MsgBuilder();
    auto header = builder.initHeader();
    header.setFrameId("sentinel_perception_orsm");
    builder.setReqModule("sentinel_perception_orsm");

    if (low_battery_exit) {
        // 设计要求：SOC < 20% 退出时双闪 5s + 喇叭 1s，提醒用户功能退出。
        builder.setRdaHzrdLampTime(50);
        builder.setRdaHzrdLamponDurn(1);
        builder.setRdaHzrdLampoffDurn(1);
        builder.setPedWarnToneCtrl(10);
        builder.setAebAlertReq(1);
        RsclAccess::Instance()->publish(cfg_.orsm_action_topic, send_msg);
        return;
    }

    if (orsmNeedsHazardLamp(output)) {
        builder.setRdaHzrdLampTime(warning_duration > 0 ? warning_duration : 50);
        builder.setRdaHzrdLamponDurn(1);
        builder.setRdaHzrdLampoffDurn(1);
    }

    if (orsmNeedsSound(output)) {
        builder.setPedWarnToneCtrl(warning_duration > 0 ? static_cast<uint8_t>(std::min<uint16_t>(warning_duration, 255)) : 50);
    }

    if (safe_crossing_alert) {
        // 设计要求安全生物越线点亮位置灯/近光灯 5s。当前 AlarmCtrl 没有独立 HAD_LightReq，
        // 暂用 AebAlertReq=1 作为下游适配层的“单车安全生物越线/灯光提醒”触发信号。
        builder.setAebAlertReq(1);
    } else if (has_level) {
        // AlarmCtrl 当前没有独立 HAD_LightReq/APP/HMI 字段，先把 ORSM 报警等级透传到
        // AebAlertReq，供下游调测适配层区分 I/II/III 级并映射位置灯/近光灯、APP/HMI 提示。
        builder.setAebAlertReq(static_cast<uint8_t>(output.orsm_warn_level));
    }

    RsclAccess::Instance()->publish(cfg_.orsm_action_topic, send_msg);
}

}  // namespace sentinel
