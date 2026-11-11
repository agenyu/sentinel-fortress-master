#pragma once
// 哨兵模式正式接口内部映射
// 对齐 SentryMode.proto / senseauto-msgs: ad_msg_idl/l3_msgs/sentry_mode.capnp.h

#include <cstdint>

namespace sentinel {

// SentinelReqMsgType 哨兵请求消息类型枚举
// 当前单节点主要使用 user_enabled/user_shutdown_requested 调试配置承载用户意愿；
// 后续接 IVI/TBOX 时应直接映射 SentinelRequest。
enum class SentinelReqMsgType : uint8_t {
    UNKNOWN_SENTINEL_REQ = 0,
    ENABLE_SENTINEL_IMMEDIATELY = 1,
    DISABLE_SENTINEL_IMMEDIATELY = 2,
    ENABLE_SENTINEL_AUTOMATICALLY = 3,
    DISABLE_SENTINEL_AUTOMATICALLY = 4,
    ENABLE_SENTINEL_SCHEDULED_START = 5,
    DISABLE_SENTINEL_SCHEDULED_START = 6,
    ENTER_SENTINEL_MODE_FEATURE = 7,
    EXIT_SENTINEL_MODE_FEATURE = 8,
    ENTER_VIDEO_VIEW = 9,
    EXIT_VIDEO_VIEW = 10,
    REQ_RESERVED = 11,
};

// SentinelResMsgType 哨兵响应消息类型枚举
enum class SentinelResMsgType : uint8_t {
    UNKNOWN_SENTINEL_RES = 0,
    APPROVE_SENTINEL_IMMEDIATELY = 1,
    DECLINE_SENTINEL_IMMEDIATELY = 2,
    APPROVE_SENTINEL_AUTOMATICALLY = 3,
    DECLINE_SENTINEL_AUTOMATICALLY = 4,
    APPROVE_SENTINEL_SCHEDULED_START = 5,
    DECLINE_SENTINEL_SCHEDULED_START = 6,
    APPROVE_SENTINEL_MODE_FEATURE = 7,
    DECLINE_SENTINEL_MODE_FEATURE = 8,
    APPROVE_VIDEO_VIEW = 9,
    DECLINE_VIDEO_VIEW = 10,
    RES_RESERVED = 11,
};

// SentinelODDStatus 哨兵 ODD 状态枚举
enum class SentinelODDStatus : uint8_t {
    SATISFY_ALL_ODD = 0,
    INSUFFICIENT_BATTERY_POWER = 1,
    INSUFFICIENT_HYBRID_POWER = 2,
    SPEED_REQUIREMENT_NOT_MET = 3,
    GEAR_POSITION_NOT_MET = 4,
    HANDBRAKE_ENGAGED = 5,
    KEY_OUT_OF_RANGE = 6,
    EV_START_CONDITIONS_NOT_MET = 7,
    HYBRID_START_CONDITIONS_NOT_MET = 8,
    SENSOR_MALFUNCTION = 9,
    TEMPORARY_STORAGE_NOT_AVAILABLE = 10,
    PERMANENT_STORAGE_NOT_AVAILABLE = 11,
    NO_PERSON_IN_VEHICLE = 12,
    NO_DRIVER_DETECTED = 13,
    ODD_RESERVED = 14,
};

// SentinelEnableFailed 哨兵开启失败原因枚举
enum class SentinelEnableFailed : uint8_t {
    SENTINEL_ENABLE_FAILED_UNKNOWN = 0,
    VEHICLE_NOT_IN_PARK_GEAR = 1,
    LOW_BATTERY_LEVEL = 2,
    SYSTEM_UPGRADE_OR_SENSOR_CALIBRATION_IN_PROGRESS = 3,
    SYSTEM_FAULT = 4,
    HAD_NOT_WORKING = 5,
    PET_MODE_ACTIVE = 6,
    MAINTENANCE_MODE_ACTIVE = 7,
    ULTRA_POWER_SAVING_MODE_ACTIVE = 8,
    VEHICLE_NOT_LOCKED = 9,
    GEAR_SHIFTING_IN_PROGRESS = 10,
    INSUFFICIENT_VEHICLE_ENERGY = 11,
    RPA_FUNCTION_ACTIVE_RPA = 12,
    AVP_FUNCTION_ACTIVE_AVP = 13,
    ABNORMAL_VEHICLE_STATUS = 14,
    SYSTEM_MALFUNCTION = 15,
    ENA_RESERVED = 16,
};

// SentinelGeneralThreat 哨兵威胁/声光报警状态枚举
enum class SentinelGeneralThreat : uint8_t {
    OFF = 0,
    ON = 1,
    ACTIVE = 2,
    FAILURE = 3,
};

// SentinelObstacleType 哨兵模式触发障碍物类型枚举
enum class SentinelObstacleType : uint8_t {
    UNKNOWN_OBSTACLE = 0,
    SEDAN = 1,
    SUV = 2,
    ADULT = 3,
    CHILD = 4,
    BICYCLE = 5,
    MOTORCYCLE = 6,
    E_BIKE = 7,
    ANIMAL = 8,
    UNIDENTIFIED_COLLISION = 9,
    UNIDENTIFIED_SCRAPING = 10,
};

struct SentinelResStatusView {
    SentinelResMsgType sentinel_res_msg_type{SentinelResMsgType::UNKNOWN_SENTINEL_RES};
    SentinelODDStatus sentinel_odd_status{SentinelODDStatus::SATISFY_ALL_ODD};
    SentinelEnableFailed sentinel_enable_failed{SentinelEnableFailed::SENTINEL_ENABLE_FAILED_UNKNOWN};
    SentinelGeneralThreat sentinel_general_threat{SentinelGeneralThreat::OFF};
    SentinelGeneralThreat sentinel_severe_threat{SentinelGeneralThreat::OFF};
    SentinelGeneralThreat sentinel_audio_visual_alarm{SentinelGeneralThreat::OFF};
    SentinelObstacleType sentinel_obstacle_type{SentinelObstacleType::UNKNOWN_OBSTACLE};
};

inline bool sentinelRequestKnown(uint16_t req_value) {
    switch (req_value) {
        case static_cast<uint16_t>(SentinelReqMsgType::ENABLE_SENTINEL_IMMEDIATELY):
        case static_cast<uint16_t>(SentinelReqMsgType::DISABLE_SENTINEL_IMMEDIATELY):
        case static_cast<uint16_t>(SentinelReqMsgType::ENABLE_SENTINEL_AUTOMATICALLY):
        case static_cast<uint16_t>(SentinelReqMsgType::DISABLE_SENTINEL_AUTOMATICALLY):
        case static_cast<uint16_t>(SentinelReqMsgType::ENABLE_SENTINEL_SCHEDULED_START):
        case static_cast<uint16_t>(SentinelReqMsgType::DISABLE_SENTINEL_SCHEDULED_START):
        case static_cast<uint16_t>(SentinelReqMsgType::ENTER_SENTINEL_MODE_FEATURE):
        case static_cast<uint16_t>(SentinelReqMsgType::EXIT_SENTINEL_MODE_FEATURE):
        case static_cast<uint16_t>(SentinelReqMsgType::ENTER_VIDEO_VIEW):
        case static_cast<uint16_t>(SentinelReqMsgType::EXIT_VIDEO_VIEW):
            return true;
        default:
            return false;
    }
}

inline SentinelResMsgType sentinelResponseTypeForRequest(uint16_t req_value, bool approve) {
    switch (req_value) {
        case static_cast<uint16_t>(SentinelReqMsgType::ENABLE_SENTINEL_IMMEDIATELY):
            return approve ? SentinelResMsgType::APPROVE_SENTINEL_IMMEDIATELY
                           : SentinelResMsgType::DECLINE_SENTINEL_IMMEDIATELY;
        case static_cast<uint16_t>(SentinelReqMsgType::DISABLE_SENTINEL_IMMEDIATELY):
            // 关闭哨兵模式需反馈关闭态 2，而不是沿用开启确认 1。
            return SentinelResMsgType::DECLINE_SENTINEL_IMMEDIATELY;
        case static_cast<uint16_t>(SentinelReqMsgType::ENABLE_SENTINEL_AUTOMATICALLY):
            return approve ? SentinelResMsgType::APPROVE_SENTINEL_AUTOMATICALLY
                           : SentinelResMsgType::DECLINE_SENTINEL_AUTOMATICALLY;
        case static_cast<uint16_t>(SentinelReqMsgType::DISABLE_SENTINEL_AUTOMATICALLY):
            // 关闭自动开启需反馈关闭态 4，而不是沿用自动开启确认 3。
            return SentinelResMsgType::DECLINE_SENTINEL_AUTOMATICALLY;
        case static_cast<uint16_t>(SentinelReqMsgType::ENABLE_SENTINEL_SCHEDULED_START):
            return approve ? SentinelResMsgType::APPROVE_SENTINEL_SCHEDULED_START
                           : SentinelResMsgType::DECLINE_SENTINEL_SCHEDULED_START;
        case static_cast<uint16_t>(SentinelReqMsgType::DISABLE_SENTINEL_SCHEDULED_START):
            // 关闭预约开启需反馈关闭态 6，而不是沿用预约开启确认 5。
            return SentinelResMsgType::DECLINE_SENTINEL_SCHEDULED_START;
        case static_cast<uint16_t>(SentinelReqMsgType::ENTER_SENTINEL_MODE_FEATURE):
        case static_cast<uint16_t>(SentinelReqMsgType::EXIT_SENTINEL_MODE_FEATURE):
            return approve ? SentinelResMsgType::APPROVE_SENTINEL_MODE_FEATURE
                           : SentinelResMsgType::DECLINE_SENTINEL_MODE_FEATURE;
        case static_cast<uint16_t>(SentinelReqMsgType::ENTER_VIDEO_VIEW):
        case static_cast<uint16_t>(SentinelReqMsgType::EXIT_VIDEO_VIEW):
            return approve ? SentinelResMsgType::APPROVE_VIDEO_VIEW
                           : SentinelResMsgType::DECLINE_VIDEO_VIEW;
        default:
            return SentinelResMsgType::UNKNOWN_SENTINEL_RES;
    }
}

inline bool sentinelRequestNeedsOddApproval(uint16_t req_value) {
    switch (req_value) {
        case static_cast<uint16_t>(SentinelReqMsgType::ENABLE_SENTINEL_IMMEDIATELY):
        case static_cast<uint16_t>(SentinelReqMsgType::ENABLE_SENTINEL_AUTOMATICALLY):
        case static_cast<uint16_t>(SentinelReqMsgType::ENABLE_SENTINEL_SCHEDULED_START):
        case static_cast<uint16_t>(SentinelReqMsgType::ENTER_SENTINEL_MODE_FEATURE):
        case static_cast<uint16_t>(SentinelReqMsgType::ENTER_VIDEO_VIEW):
            return true;
        default:
            return false;
    }
}

}  // namespace sentinel
