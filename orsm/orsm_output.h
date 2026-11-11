#pragma once
// 越野堡垒 ORSM 输出/状态结构
// 对齐 OrsmInfo.proto / senseauto-msgs: ad_msg_idl/l3_msgs/orsm_info.capnp.h

#include <cstdint>
#include <vector>

namespace sentinel {

// BasicSwitchResp 开关响应枚举（响应方向）
enum class BasicSwitchResp : uint8_t {
    NONE     = 0x0,
    ON       = 0x1,
    OFF      = 0x2,
    REVERSED = 0x3,
};

// OrsmSettingType 越野堡垒模式开关类型
enum class OrsmSettingType : uint8_t {
    OFF                              = 0x0,
    SINGLE_AUTO_SECURITY_ON          = 0x1,
    MULTI_VEHICLE_JOINT_SECURITY_ON  = 0x2,
    FUNCTION_LIMITED_ENABLE_FAILED   = 0x3,
};

// DetecRangeSetting 监测范围设置类型
enum class DetecRangeSetting : uint8_t {
    NONE_RANGE = 0x0,
    RANGE_A    = 0x1,
    RANGE_B    = 0x2,
    RANGE_C    = 0x3,
};

// AlarmSoundType 危险报警音类型
enum class AlarmSoundType : uint8_t {
    URGENT = 0x0,
    GENTLE = 0x1,
};

// AlarmSoundDuration 危险报警音时长提醒类型
enum class AlarmSoundDuration : uint8_t {
    NO_REMINDER = 0x0,
    REMINDER_3S = 0x1,
    REMINDER_5S = 0x2,
    REMINDER_10S = 0x3,
};

// OrsmMessageReminder 越野堡垒模式文言提醒/状态
enum class OrsmMessageReminder : uint8_t {
    ORSM_NONE = 0x0,
    SECURITY_MODE_ON = 0x1,
    LOW_BATTERY = 0x2,
    NOT_IN_ON_READY_STATE_FUNCTION_ENTRY_FAILED = 0x3,
    GEAR_NOT_IN_P_POSITION_FUNCTION_ENTRY_FAILED = 0x4,
    LOW_BATTERY_FUNCTION_ENTRY_FAILED = 0x5,
    INTELLIGENT_DRIVING_SYSTEM_MALFUNCTION_FUNCTION_ENTRY_FAILED = 0x6,
    RADAR_CAMERA_MALFUNCTION_FUNCTION_ENTRY_FAILED = 0x7,
    SECURITY_FUNCTION_EXITED = 0x8,
    LOW_BATTERY_FUNCTION_EXITED = 0x9,
    MULTI_VEHICLE_JOINT_SECURITY_FUNCTION_EXITED = 0xA,
    VEHICLE_SPEED_TOO_HIGH_FUNCTION_CANNOT_BE_ACTIVATED = 0xB,
    VEHICLE_SPEED_TOO_HIGH_JOINT_SECURITY_FUNCTION_EXITED = 0xC,
    NETWORK_CONNECTION_DISCONNECTED_JOINT_SECURITY_FUNCTION_EXITED = 0xD,
};

// OrsmWarningStatus 安全生物越线报警状态
enum class OrsmWarningStatus : uint8_t {
    WARNING_OFF = 0x0,
    WARNING_ON  = 0x1,
};

// IntrudersNumType 入侵物数量类型
enum class IntrudersNumType : uint8_t {
    NO_INTRUDER        = 0x0,
    SINGLE_INTRUDER    = 0x1,
    MULTIPLE_INTRUDERS = 0x2,
};

// OrsmWarnLevel 安防报警等级
enum class OrsmWarnLevel : uint8_t {
    LEVEL_NONE = 0x0,
    WARNLEVEL_1 = 0x1,
    WARNLEVEL_2 = 0x2,
    WARNLEVEL_3 = 0x3,
};

// IntruderType 入侵物类型枚举
enum class IntruderType : uint8_t {
    INTRUDER_UNKNOWN  = 0x0,
    INTRUDER_ANIMAL   = 0x1,
    INTRUDER_HUMANKIND = 0x2,
};

// 单个入侵物信息，对齐 IntrudersInfo；distance/zone 仅作内部调试字段，不进正式消息。
struct IntruderInfo {
    uint32_t     intruder_id{0};
    IntruderType intruder_type{IntruderType::INTRUDER_UNKNOWN};
    float        intruder_pose_x{0.f};       // 入侵物位置X坐标
    float        intruder_pose_y{0.f};       // 入侵物位置Y坐标
    float        intruder_velocity_x{0.f};   // 入侵物X向速度
    float        intruder_velocity_y{0.f};   // 入侵物Y向速度
    uint64_t     timestamp_nsec{0};          // 时间戳_纳秒

    // Internal-only diagnostics.
    float        distance{0.f};
    OrsmWarnLevel zone{OrsmWarnLevel::LEVEL_NONE};
};

// OrsmStatus 越野堡垒模式状态消息的内部等价结构。
struct ORSMAlertOutput {
    OrsmSettingType      orsm_setting_st_fb{OrsmSettingType::OFF};
    BasicSwitchResp      humans_recog_setting_fb{BasicSwitchResp::NONE};
    BasicSwitchResp      animals_recog_setting_fb{BasicSwitchResp::NONE};
    DetecRangeSetting    detec_range_setting_fb{DetecRangeSetting::NONE_RANGE};
    BasicSwitchResp      alarm_light_setting_fb{BasicSwitchResp::NONE};
    BasicSwitchResp      alarm_horn_setting_fb{BasicSwitchResp::NONE};
    AlarmSoundType       alarm_sound_setting_fb{AlarmSoundType::URGENT};
    AlarmSoundDuration   alarm_snd_dura_setting_fb{AlarmSoundDuration::NO_REMINDER};
    OrsmMessageReminder  orsm_message_rem{OrsmMessageReminder::ORSM_NONE};
    OrsmWarningStatus    orsm_warning_st{OrsmWarningStatus::WARNING_OFF};
    IntrudersNumType     intruders_num{IntrudersNumType::NO_INTRUDER};
    OrsmWarnLevel        orsm_warn_level{OrsmWarnLevel::LEVEL_NONE};
    std::vector<IntruderInfo> intruders_info;
    std::vector<uint32_t> orsm_info_reserved;
    uint64_t             timestamp_ns{0};
};

// 设置刷新成功时保留运行态提醒；启用失败时更新失败原因。
inline void applyOrsmEnableMessageFeedback(ORSMAlertOutput& cached,
                                           bool enable_ready,
                                           OrsmMessageReminder enable_failure) {
    if (!enable_ready) {
        cached.orsm_message_rem = enable_failure;
    }
}

// Merge only runtime/state-machine fields. Settings feedback is owned by the
// IVI/TBOX callbacks and must survive a late perception result produced from
// an older OrsmSettings snapshot.
inline void applyOrsmRuntimeOutput(ORSMAlertOutput& cached,
                                   const ORSMAlertOutput& runtime) {
    cached.orsm_setting_st_fb = runtime.orsm_setting_st_fb;
    cached.orsm_message_rem = runtime.orsm_message_rem;
    cached.orsm_warning_st = runtime.orsm_warning_st;
    cached.intruders_num = runtime.intruders_num;
    cached.orsm_warn_level = runtime.orsm_warn_level;
    cached.intruders_info = runtime.intruders_info;
    cached.timestamp_ns = runtime.timestamp_ns;
}

}  // namespace sentinel
