#pragma once
// 越野堡垒 ORSM 输入/设置结构
// 对齐 OrsmInfo.proto: SetOffRoadMdInfoReq / OffRoadSetInfoStruct

#include <cstdint>
#include "orsm_output.h"

namespace sentinel {

// BasicSwitchReq 开关请求枚举（请求方向）
enum class BasicSwitchReq : uint8_t {
    NONE     = 0x0,
    ON       = 0x1,
    OFF      = 0x2,
    REVERSED = 0x3,
};

// MultiVehicleTeamSettingEnum 多车联防组队设置枚举
enum class MultiVehicleTeamSettingEnum : uint8_t {
    TEAM_NO_REQUEST = 0x0,
    TEAM_FORMING    = 0x1,
    TEAM_FINISHED   = 0x2,
    TEAM_DISBANDED  = 0x3,
};

// MultiVehicleTeamReqEnum 多车联防组队请求枚举
enum class MultiVehicleTeamReqEnum : uint8_t {
    TEAM_REQ_NO_REQUEST = 0x0,
    ADD_THE_TEAM        = 0x1,
    QUIT_THE_TEAM       = 0x2,
};

// MultiVehicleTeamPreparedEnum 多车联防功能准备完成枚举
enum class MultiVehicleTeamPreparedEnum : uint8_t {
    PREPARED_NO_REQUEST = 0x0,
    PREPARED            = 0x1,
    NOT_PREPARED        = 0x2,
    PREPARED_REVERSED   = 0x3,
};

// TeamStatusEnum 多车联防组队状态枚举
enum class TeamStatusEnum : uint8_t {
    NO_TEAM_FORMED    = 0x0,
    TEAM_UP           = 0x1,
    TEAM_UP_COMPLETED = 0x2,
};

// 本车在多车联防中的角色。该字段不是 OrsmInfo.proto 正式信号，
// 由 SetOffRoadMdInfoReq 中“队长车辆使用 MultiVehicleTeamSetting / 成员车辆使用 MultiVehicleTeamReq”的文档约定推断。
enum class MultiVehicleRole : uint8_t {
    UNKNOWN = 0x0,
    LEADER  = 0x1,
    MEMBER  = 0x2,
};

// SetOffRoadMdInfoReq 越野堡垒模式开关设置请求的内部等价结构。
struct OrsmSettings {
    OrsmSettingType              orsm_setting_st{OrsmSettingType::OFF};
    BasicSwitchReq               humans_recog_setting{BasicSwitchReq::ON};
    BasicSwitchReq               animals_recog_setting{BasicSwitchReq::ON};
    DetecRangeSetting            detec_range_setting{DetecRangeSetting::RANGE_B};
    BasicSwitchReq               alarm_light_setting{BasicSwitchReq::ON};
    BasicSwitchReq               alarm_horn_setting{BasicSwitchReq::ON};
    AlarmSoundType               alarm_sound_setting{AlarmSoundType::URGENT};
    AlarmSoundDuration           alarm_snd_dura_setting{AlarmSoundDuration::REMINDER_5S};
    MultiVehicleTeamSettingEnum  multi_vehicle_team_setting{MultiVehicleTeamSettingEnum::TEAM_NO_REQUEST};
    MultiVehicleTeamReqEnum      multi_vehicle_team_req{MultiVehicleTeamReqEnum::TEAM_REQ_NO_REQUEST};
    MultiVehicleTeamPreparedEnum multi_vehicle_team_prepared{MultiVehicleTeamPreparedEnum::PREPARED_NO_REQUEST};
    TeamStatusEnum               tbox_team_status_info{TeamStatusEnum::NO_TEAM_FORMED};
    MultiVehicleRole             multi_vehicle_role{MultiVehicleRole::UNKNOWN};

    // 三区半径（m），可通过 config.yaml 配置；不是 OrsmInfo.proto 字段。
    float zone_a_radius{30.0f};   // A区（最外围）
    float zone_b_radius{15.0f};   // B区（中间）
    float zone_c_radius{5.0f};    // C区（核心）
    float intruder_dedup_distance_m{1.0f}; // 入侵物空间去重距离阈值；<=0 表示关闭去重。
    bool  lidar_only_far_output_enabled{true}; // 远距离LiDAR-only稳定轨迹可作为unknown入侵物输出
    float lidar_only_far_output_min_distance_m{10.0f};
    float far_dynamic_min_distance_m{10.0f};
    bool  far_image_motion_required{true};

    // 多车联防 ODD 参数（来自越野堡垒功能详细说明书，默认值按 TBD 当前实现）。
    bool  multi_vehicle_enable_network_check{true};
    float multi_vehicle_speed_entry_max_kmh{10.0f};
    float multi_vehicle_speed_exit_max_kmh{20.0f};
    float multi_vehicle_soc_entry_min{30.0f};
    float multi_vehicle_soc_exit_min{20.0f};
    int   multi_vehicle_stationary_required_sec{60};

    // 多车联防监控区域：安防侧外部区/内部区/不报警区。
    // security_side: 1=左侧(Y+), -1=右侧(Y-), 0=不限制侧向（台架/无标定回退）。
    int   multi_vehicle_security_side{1};
    float multi_vehicle_external_radius{30.0f};
    float multi_vehicle_internal_radius{15.0f};
    float multi_vehicle_no_alarm_radius{5.0f};
};

inline bool switchOn(BasicSwitchReq value) {
    return value == BasicSwitchReq::ON;
}

inline BasicSwitchResp toResp(BasicSwitchReq value) {
    switch (value) {
    case BasicSwitchReq::ON:       return BasicSwitchResp::ON;
    case BasicSwitchReq::OFF:      return BasicSwitchResp::OFF;
    case BasicSwitchReq::REVERSED: return BasicSwitchResp::REVERSED;
    case BasicSwitchReq::NONE:
    default:                       return BasicSwitchResp::NONE;
    }
}

inline void applyOrsmSettingsFeedback(ORSMAlertOutput& output,
                                      const OrsmSettings& settings) {
    output.humans_recog_setting_fb = toResp(settings.humans_recog_setting);
    output.animals_recog_setting_fb = toResp(settings.animals_recog_setting);
    output.detec_range_setting_fb = settings.detec_range_setting;
    output.alarm_light_setting_fb = toResp(settings.alarm_light_setting);
    output.alarm_horn_setting_fb = toResp(settings.alarm_horn_setting);
    output.alarm_sound_setting_fb = settings.alarm_sound_setting;
    output.alarm_snd_dura_setting_fb = settings.alarm_snd_dura_setting;

    // OrsmStatus has no formal fields for these multi-vehicle coordination signals.
    // Publish them in fixed order through orsm_info_reserved:
    // [0]=multi_vehicle_team_setting, [1]=multi_vehicle_team_req,
    // [2]=multi_vehicle_team_prepared, [3]=tbox_team_status_info.
    output.orsm_info_reserved = {
        static_cast<uint32_t>(settings.multi_vehicle_team_setting),
        static_cast<uint32_t>(settings.multi_vehicle_team_req),
        static_cast<uint32_t>(settings.multi_vehicle_team_prepared),
        static_cast<uint32_t>(settings.tbox_team_status_info),
    };
}

inline bool orsmEnabled(OrsmSettingType value) {
    return value == OrsmSettingType::SINGLE_AUTO_SECURITY_ON ||
           value == OrsmSettingType::MULTI_VEHICLE_JOINT_SECURITY_ON;
}

}  // namespace sentinel
