#include "sentinel_node.h"
#include <yaml-cpp/yaml.h>
#include <iostream>
#include <csignal>
#include <atomic>
#include <filesystem>
#include <unistd.h>

static std::atomic<bool> g_running{true};

void signalHandler(int) {
    std::cout << "\n[main] Shutting down..." << std::endl;
    g_running = false;
}

void applyOrsmSafetyDefaults(sentinel::SentinelConfig& cfg) {
    if (!cfg.orsm_safety_defaults_enabled) {
        std::cerr << "[main][ORSM][WARN] orsm_safety_defaults_enabled=false; "
                  << "critical fortress perception settings will not be enforced." << std::endl;
        return;
    }

}

sentinel::SentinelConfig loadConfig(const std::string& path) {
    sentinel::SentinelConfig cfg;
    try {
        YAML::Node node = YAML::LoadFile(path);
        if (node["camera_topics"]) {
            cfg.camera_topics.clear();
            for (const auto& topic : node["camera_topics"]) {
                cfg.camera_topics.push_back(topic.as<std::string>());
            }
        } else if (node["camera_topic"]) {
            // 兼容旧版单路配置。
            cfg.camera_topics = {node["camera_topic"].as<std::string>()};
        }
        if (node["camera_names"]) {
            cfg.camera_names.clear();
            for (const auto& name : node["camera_names"]) {
                cfg.camera_names.push_back(name.as<std::string>());
            }
        }
        if (node["calib_base_dir"])         cfg.calib_base_dir        = node["calib_base_dir"].as<std::string>();
        if (node["inbox_calib_dir"])        cfg.inbox_calib_dir       = node["inbox_calib_dir"].as<std::string>();
        if (node["imu_topic"])             cfg.imu_topic             = node["imu_topic"].as<std::string>();
        if (node["uss_pdc_topic"])         cfg.uss_pdc_topic         = node["uss_pdc_topic"].as<std::string>();
        if (node["uss_meb_topic"])         cfg.uss_meb_topic         = node["uss_meb_topic"].as<std::string>();
        if (node["uss_fused_topic"])       cfg.uss_fused_topic       = node["uss_fused_topic"].as<std::string>();
        if (node["uss_fused_legacy_topic"]) cfg.uss_fused_legacy_topic = node["uss_fused_legacy_topic"].as<std::string>();
        if (node["uss_alert_threshold_mm"]) cfg.uss_alert_threshold_mm = node["uss_alert_threshold_mm"].as<uint16_t>();
        if (node["uss_fused_coordinate_scale_m"]) cfg.uss_fused_coordinate_scale_m = node["uss_fused_coordinate_scale_m"].as<float>();
        if (node["uss_fused_min_exist_prob"]) cfg.uss_fused_min_exist_prob = node["uss_fused_min_exist_prob"].as<int>();
        if (node["enable_uss_vision_fusion"]) cfg.enable_uss_vision_fusion = node["enable_uss_vision_fusion"].as<bool>();
        if (node["uss_fusion_max_range_m"]) cfg.uss_fusion_max_range_m = node["uss_fusion_max_range_m"].as<float>();
        if (node["uss_fusion_candidate_vision_range_m"]) cfg.uss_fusion_candidate_vision_range_m = node["uss_fusion_candidate_vision_range_m"].as<float>();
        if (node["uss_fusion_max_age_ms"]) cfg.uss_fusion_max_age_ms = node["uss_fusion_max_age_ms"].as<float>();
        if (node["uss_fusion_confidence_boost"]) cfg.uss_fusion_confidence_boost = node["uss_fusion_confidence_boost"].as<float>();
        if (node["lidar_topics"]) {
            cfg.lidar_topics.clear();
            for (const auto& topic : node["lidar_topics"]) {
                cfg.lidar_topics.push_back(topic.as<std::string>());
            }
        } else if (node["lidar_topic"]) {
            // 兼容旧版单路配置。
            cfg.lidar_topics = {node["lidar_topic"].as<std::string>()};
        }
        if (node["lidar_names"]) {
            cfg.lidar_names.clear();
            for (const auto& name : node["lidar_names"]) {
                cfg.lidar_names.push_back(name.as<std::string>());
            }
        }
        if (node["lidar_point_layouts"]) {
            cfg.lidar_point_layouts.clear();
            for (const auto& layout : node["lidar_point_layouts"]) {
                cfg.lidar_point_layouts.push_back(layout.as<std::string>());
            }
        }
        if (node["lidar_yaw_degrees"]) {
            cfg.lidar_yaw_degrees.clear();
            for (const auto& yaw : node["lidar_yaw_degrees"]) {
                cfg.lidar_yaw_degrees.push_back(yaw.as<float>());
            }
        }
        if (node["lidar_xy_transforms"]) {
            cfg.lidar_xy_transforms.clear();
            for (const auto& transform : node["lidar_xy_transforms"]) {
                if (!transform.IsSequence() || transform.size() != 4) {
                    throw std::runtime_error(
                        "Each lidar_xy_transforms entry must be [a,b,c,d]");
                }
                cfg.lidar_xy_transforms.push_back({
                    transform[0].as<float>(), transform[1].as<float>(),
                    transform[2].as<float>(), transform[3].as<float>()});
            }
        }
        if (node["lidar_near_distance_m"]) cfg.lidar_near_distance_m = node["lidar_near_distance_m"].as<float>();
        if (node["lidar_near_min_points"]) cfg.lidar_near_min_points = node["lidar_near_min_points"].as<int>();
        if (node["lidar_time_sync_mode"]) cfg.lidar_time_sync_mode = node["lidar_time_sync_mode"].as<std::string>();
        if (node["lidar_max_age_ms"]) cfg.lidar_max_age_ms = node["lidar_max_age_ms"].as<float>();
        if (node["lidar_bag_replay_max_age_ms"]) cfg.lidar_bag_replay_max_age_ms = node["lidar_bag_replay_max_age_ms"].as<float>();
        if (node["lidar_merge_voxel_leaf_m"]) cfg.lidar_merge_voxel_leaf_m = node["lidar_merge_voxel_leaf_m"].as<float>();
        if (node["vehicle_report_topic"])  cfg.vehicle_report_topic  = node["vehicle_report_topic"].as<std::string>();
        if (node["chassis_info_topic"])    cfg.chassis_info_topic    = node["chassis_info_topic"].as<std::string>();
        if (node["sentry_request_topic"])  cfg.sentry_request_topic  = node["sentry_request_topic"].as<std::string>();
        if (node["sentry_state_topic"])    cfg.sentry_state_topic    = node["sentry_state_topic"].as<std::string>();
        if (node["sentry_event_topic"])    cfg.sentry_event_topic    = node["sentry_event_topic"].as<std::string>();
        if (node["sentry_action_topic"])   cfg.sentry_action_topic   = node["sentry_action_topic"].as<std::string>();
        if (node["sentry_media_cmd_topic"]) cfg.sentry_media_cmd_topic = node["sentry_media_cmd_topic"].as<std::string>();
        if (node["sentry_media_result_topic"]) cfg.sentry_media_result_topic = node["sentry_media_result_topic"].as<std::string>();
        if (node["orsm_setting_topic"])    cfg.orsm_setting_topic    = node["orsm_setting_topic"].as<std::string>();
        if (node["orsm_set_info_topic"])   cfg.orsm_set_info_topic   = node["orsm_set_info_topic"].as<std::string>();
        if (node["orsm_state_topic"])      cfg.orsm_state_topic      = node["orsm_state_topic"].as<std::string>();
        if (node["orsm_action_topic"])     cfg.orsm_action_topic     = node["orsm_action_topic"].as<std::string>();
        if (node["camera_input_mode"])     cfg.camera_input_mode     = node["camera_input_mode"].as<std::string>();
        if (node["camera_decoder_backend"]) cfg.camera_decoder_backend = node["camera_decoder_backend"].as<std::string>();
        if (node["camera_width"])          cfg.camera_width          = node["camera_width"].as<int>();
        if (node["camera_height"])         cfg.camera_height         = node["camera_height"].as<int>();
        if (node["gpu_id"])                cfg.gpu_id                = node["gpu_id"].as<int>();
        if (node["camera_decode_debug_enabled"]) cfg.camera_decode_debug_enabled = node["camera_decode_debug_enabled"].as<bool>();
        if (node["camera_decode_debug_dir"]) cfg.camera_decode_debug_dir = node["camera_decode_debug_dir"].as<std::string>();
        if (node["camera_decode_debug_max_frames_per_camera"]) cfg.camera_decode_debug_max_frames_per_camera = node["camera_decode_debug_max_frames_per_camera"].as<int>();
        if (node["camera_decode_debug_save_hevc"]) cfg.camera_decode_debug_save_hevc = node["camera_decode_debug_save_hevc"].as<bool>();
        if (node["camera_decode_debug_save_decoded_bgr"]) cfg.camera_decode_debug_save_decoded_bgr = node["camera_decode_debug_save_decoded_bgr"].as<bool>();
        if (node["camera_decode_debug_save_node_images"]) cfg.camera_decode_debug_save_node_images = node["camera_decode_debug_save_node_images"].as<bool>();
        if (node["enable_camera_sync"])    cfg.enable_camera_sync    = node["enable_camera_sync"].as<bool>();
        if (node["camera_sync_tolerance_ms"]) cfg.camera_sync_tolerance_ms = node["camera_sync_tolerance_ms"].as<int>();
        if (node["camera_sync_timeout_ms"]) cfg.camera_sync_timeout_ms = node["camera_sync_timeout_ms"].as<int>();
        if (node["camera_sync_max_buffered_frames"]) cfg.camera_sync_max_buffered_frames = node["camera_sync_max_buffered_frames"].as<int>();
        if (node["camera_sync_enable_hold"]) cfg.camera_sync_enable_hold = node["camera_sync_enable_hold"].as<bool>();
        if (node["camera_sync_hold_ms"]) cfg.camera_sync_hold_ms = node["camera_sync_hold_ms"].as<int>();
        if (node["camera_sync_enable_degrade"]) cfg.camera_sync_enable_degrade = node["camera_sync_enable_degrade"].as<bool>();
        if (node["camera_sync_degrade_ms"]) cfg.camera_sync_degrade_ms = node["camera_sync_degrade_ms"].as<int>();
        if (node["camera_sync_offsets_ms"]) {
            cfg.camera_sync_offsets_ms.clear();
            for (const auto& offset : node["camera_sync_offsets_ms"]) {
                cfg.camera_sync_offsets_ms.push_back(offset.as<int>());
            }
        }
        if (node["camera_sync_half_rate_cameras"]) {
            cfg.camera_sync_half_rate_cameras.clear();
            for (const auto& cam : node["camera_sync_half_rate_cameras"]) {
                cfg.camera_sync_half_rate_cameras.push_back(cam.as<std::string>());
            }
        }
        if (node["sentry_process_interval_ms"]) cfg.sentry_process_interval_ms = node["sentry_process_interval_ms"].as<int>();
        if (node["orsm_process_interval_ms"]) cfg.orsm_process_interval_ms = node["orsm_process_interval_ms"].as<int>();
        if (node["visual_track_confirm_min_age"]) cfg.visual_track_confirm_min_age = node["visual_track_confirm_min_age"].as<int>();
        if (node["visual_track_max_lost_frames"]) cfg.visual_track_max_lost_frames = node["visual_track_max_lost_frames"].as<int>();
        if (node["visual_track_min_confidence"]) cfg.visual_track_min_confidence = node["visual_track_min_confidence"].as<float>();
        if (node["visual_tracker_iou_threshold"]) cfg.visual_tracker_iou_threshold = node["visual_tracker_iou_threshold"].as<float>();
        if (node["visual_tracker_high_conf_threshold"]) cfg.visual_tracker_high_conf_threshold = node["visual_tracker_high_conf_threshold"].as<float>();
        if (node["visual_tracker_low_conf_threshold"]) cfg.visual_tracker_low_conf_threshold = node["visual_tracker_low_conf_threshold"].as<float>();
        if (node["visual_tracker_new_track_threshold"]) cfg.visual_tracker_new_track_threshold = node["visual_tracker_new_track_threshold"].as<float>();
        if (node["visual_tracker_low_match_iou_threshold"]) cfg.visual_tracker_low_match_iou_threshold = node["visual_tracker_low_match_iou_threshold"].as<float>();
        if (node["visual_fallback_min_streak"]) cfg.visual_fallback_min_streak = node["visual_fallback_min_streak"].as<int>();
        if (node["sentry_near_field_motion_enabled"]) cfg.sentry_near_field_motion_enabled = node["sentry_near_field_motion_enabled"].as<bool>();
        if (node["sentry_near_field_motion_min_changed_ratio"]) cfg.sentry_near_field_motion_min_changed_ratio = node["sentry_near_field_motion_min_changed_ratio"].as<float>();
        if (node["sentry_near_field_motion_min_mean_delta"]) cfg.sentry_near_field_motion_min_mean_delta = node["sentry_near_field_motion_min_mean_delta"].as<float>();
        if (node["sentry_near_field_motion_min_streak"]) cfg.sentry_near_field_motion_min_streak = node["sentry_near_field_motion_min_streak"].as<int>();
        if (node["sentry_passing_motion_enabled"]) cfg.sentry_passing_motion_enabled = node["sentry_passing_motion_enabled"].as<bool>();
        if (node["sentry_passing_motion_min_changed_ratio"]) cfg.sentry_passing_motion_min_changed_ratio = node["sentry_passing_motion_min_changed_ratio"].as<float>();
        if (node["sentry_passing_motion_min_mean_delta"]) cfg.sentry_passing_motion_min_mean_delta = node["sentry_passing_motion_min_mean_delta"].as<float>();
        if (node["sentry_passing_motion_min_component_ratio"]) cfg.sentry_passing_motion_min_component_ratio = node["sentry_passing_motion_min_component_ratio"].as<float>();
        if (node["sentry_passing_motion_min_streak"]) cfg.sentry_passing_motion_min_streak = node["sentry_passing_motion_min_streak"].as<int>();
        if (node["orsm_visual_confirm_min_age"]) cfg.orsm_visual_confirm_min_age = node["orsm_visual_confirm_min_age"].as<int>();
        if (node["orsm_lidar_primary_fallback_semantic_gate_m"]) cfg.orsm_lidar_primary_fallback_semantic_gate_m = node["orsm_lidar_primary_fallback_semantic_gate_m"].as<float>();
        if (node["orsm_lidar_primary_fallback_min_confidence"]) cfg.orsm_lidar_primary_fallback_min_confidence = node["orsm_lidar_primary_fallback_min_confidence"].as<float>();
        if (node["orsm_debug_track"]) cfg.orsm_debug_track = node["orsm_debug_track"].as<bool>();
        if (node["imu_accel_threshold"])   cfg.imu_accel_threshold   = node["imu_accel_threshold"].as<float>();
        if (node["imu_accel_severe_threshold"]) cfg.imu_accel_severe_threshold = node["imu_accel_severe_threshold"].as<float>();
        if (node["imu_gyro_threshold"])    cfg.imu_gyro_threshold    = node["imu_gyro_threshold"].as<float>();
        if (node["imu_gyro_severe_threshold"]) cfg.imu_gyro_severe_threshold = node["imu_gyro_severe_threshold"].as<float>();
        if (node["imu_handle_freq_min"])   cfg.imu_handle_freq_min   = node["imu_handle_freq_min"].as<float>();
        if (node["imu_handle_freq_max"])   cfg.imu_handle_freq_max   = node["imu_handle_freq_max"].as<float>();
        if (node["imu_highpass_cutoff_hz"]) cfg.imu_highpass_cutoff_hz = node["imu_highpass_cutoff_hz"].as<float>();
        if (node["imu_rms_window_ms"]) cfg.imu_rms_window_ms = node["imu_rms_window_ms"].as<int>();
        if (node["imu_vibration_min_duration_ms"]) cfg.imu_vibration_min_duration_ms = node["imu_vibration_min_duration_ms"].as<int>();
        if (node["imu_vibration_latch_ms"]) cfg.imu_vibration_latch_ms = node["imu_vibration_latch_ms"].as<int>();
        if (node["imu_accel_rms_threshold"]) cfg.imu_accel_rms_threshold = node["imu_accel_rms_threshold"].as<float>();
        if (node["imu_gyro_rms_threshold"]) cfg.imu_gyro_rms_threshold = node["imu_gyro_rms_threshold"].as<float>();
        if (node["imu_accel_severe_rms_threshold"]) cfg.imu_accel_severe_rms_threshold = node["imu_accel_severe_rms_threshold"].as<float>();
        if (node["imu_gyro_severe_rms_threshold"]) cfg.imu_gyro_severe_rms_threshold = node["imu_gyro_severe_rms_threshold"].as<float>();
        if (node["imu_accel_guard_rms"]) cfg.imu_accel_guard_rms = node["imu_accel_guard_rms"].as<float>();
        if (node["engine_path"]) {
            std::filesystem::path engine_path(node["engine_path"].as<std::string>());
            if (engine_path.is_relative()) {
                const auto config_dir =
                    std::filesystem::absolute(std::filesystem::path(path)).parent_path();
                engine_path = config_dir / engine_path;
            }
            cfg.engine_path = engine_path.lexically_normal().string();
        }
        if (node["detector_model_type"]) cfg.detector_model_type = node["detector_model_type"].as<std::string>();
        if (node["conf_threshold"])        cfg.conf_threshold        = node["conf_threshold"].as<float>();
        if (node["nms_threshold"])         cfg.nms_threshold         = node["nms_threshold"].as<float>();
        if (node["enable_display"])        cfg.enable_display        = node["enable_display"].as<bool>();
        if (node["enable_save"])           cfg.enable_save           = node["enable_save"].as<bool>();
        if (node["save_interval"])         cfg.save_interval         = node["save_interval"].as<int>();
        if (node["save_dir"])              cfg.save_dir              = node["save_dir"].as<std::string>();
        if (node["vehicle_report_timeout_sec"]) cfg.vehicle_report_timeout_sec = node["vehicle_report_timeout_sec"].as<float>();
        if (node["soc_min_sentry"])        cfg.soc_min_sentry        = node["soc_min_sentry"].as<float>();
        if (node["soc_min_orsm"])          cfg.soc_min_orsm          = node["soc_min_orsm"].as<float>();
        if (node["soc_exit_orsm"])         cfg.soc_exit_orsm         = node["soc_exit_orsm"].as<float>();
        if (node["orsm_speed_exit_threshold_kmh"]) cfg.orsm_speed_exit_threshold_kmh = node["orsm_speed_exit_threshold_kmh"].as<float>();
        if (node["speed_zero_threshold_kmh"]) cfg.speed_zero_threshold_kmh = node["speed_zero_threshold_kmh"].as<float>();
        if (node["override_orsm_power_on"]) cfg.override_orsm_power_on = node["override_orsm_power_on"].as<bool>();
        if (node["default_orsm_power_on"]) cfg.default_orsm_power_on = node["default_orsm_power_on"].as<bool>();
        if (node["override_power_ok"])     cfg.override_power_ok     = node["override_power_ok"].as<bool>();
        if (node["default_power_ok"])      cfg.default_power_ok      = node["default_power_ok"].as<bool>();
        if (node["override_speed_zero"])   cfg.override_speed_zero   = node["override_speed_zero"].as<bool>();
        if (node["default_speed_zero"])    cfg.default_speed_zero    = node["default_speed_zero"].as<bool>();
        if (node["override_gear_park"])    cfg.override_gear_park    = node["override_gear_park"].as<bool>();
        if (node["default_gear_park"])     cfg.default_gear_park     = node["default_gear_park"].as<bool>();
        if (node["override_handbrake_on"]) cfg.override_handbrake_on = node["override_handbrake_on"].as<bool>();
        if (node["default_handbrake_on"])  cfg.default_handbrake_on  = node["default_handbrake_on"].as<bool>();
        if (node["override_power_off"])    cfg.override_power_off    = node["override_power_off"].as<bool>();
        if (node["default_power_off"])     cfg.default_power_off     = node["default_power_off"].as<bool>();
        if (node["override_no_sensor_fault"]) cfg.override_no_sensor_fault = node["override_no_sensor_fault"].as<bool>();
        if (node["default_no_sensor_fault"]) cfg.default_no_sensor_fault = node["default_no_sensor_fault"].as<bool>();
        if (node["override_no_occupant"])  cfg.override_no_occupant  = node["override_no_occupant"].as<bool>();
        if (node["default_no_occupant"])   cfg.default_no_occupant   = node["default_no_occupant"].as<bool>();
        if (node["override_vehicle_locked"]) cfg.override_vehicle_locked = node["override_vehicle_locked"].as<bool>();
        if (node["default_vehicle_locked"]) cfg.default_vehicle_locked = node["default_vehicle_locked"].as<bool>();
        if (node["override_temp_ok"])      cfg.override_temp_ok      = node["override_temp_ok"].as<bool>();
        if (node["default_temp_ok"])       cfg.default_temp_ok       = node["default_temp_ok"].as<bool>();
        if (node["override_key_far"])      cfg.override_key_far      = node["override_key_far"].as<bool>();
        if (node["default_key_far"])       cfg.default_key_far       = node["default_key_far"].as<bool>();
        if (node["override_storage_ok"])   cfg.override_storage_ok   = node["override_storage_ok"].as<bool>();
        if (node["default_storage_ok"])    cfg.default_storage_ok    = node["default_storage_ok"].as<bool>();
        if (node["user_enabled"])          cfg.user_enabled          = node["user_enabled"].as<bool>();
        if (node["user_shutdown_requested"]) cfg.user_shutdown_requested = node["user_shutdown_requested"].as<bool>();
        if (node["odd_temp_min_c"])        cfg.odd_temp_min_c        = node["odd_temp_min_c"].as<float>();
        if (node["odd_temp_max_c"])        cfg.odd_temp_max_c        = node["odd_temp_max_c"].as<float>();
        if (node["odd_light_min_lux"])     cfg.odd_light_min_lux     = node["odd_light_min_lux"].as<float>();
        if (node["min_temp_storage_bytes"]) cfg.min_temp_storage_bytes = node["min_temp_storage_bytes"].as<uint64_t>();
        if (node["min_persistent_storage_bytes"]) cfg.min_persistent_storage_bytes = node["min_persistent_storage_bytes"].as<uint64_t>();
        if (node["persistent_storage_dir"]) cfg.persistent_storage_dir = node["persistent_storage_dir"].as<std::string>();
        if (node["enable_event_recording"]) cfg.enable_event_recording = node["enable_event_recording"].as<bool>();
        if (node["event_pre_seconds"])     cfg.event_pre_seconds     = node["event_pre_seconds"].as<int>();
        if (node["event_post_seconds"])    cfg.event_post_seconds    = node["event_post_seconds"].as<int>();
        if (node["event_record_fps"])      cfg.event_record_fps      = node["event_record_fps"].as<int>();
        if (node["event_record_dir"])      cfg.event_record_dir      = node["event_record_dir"].as<std::string>();
        if (node["event_log_path"])        cfg.event_log_path        = node["event_log_path"].as<std::string>();
        if (node["event_record_max_bytes"]) cfg.event_record_max_bytes = node["event_record_max_bytes"].as<uint64_t>();
        if (node["event_log_max_bytes"])    cfg.event_log_max_bytes    = node["event_log_max_bytes"].as<uint64_t>();
        if (node["sentry_state_tick_ms"])  cfg.sentry_state_tick_ms  = node["sentry_state_tick_ms"].as<int>();
        if (node["orsm_replay_force_entry_odd"]) cfg.orsm_replay_force_entry_odd = node["orsm_replay_force_entry_odd"].as<bool>();
        if (node["orsm_safety_defaults_enabled"]) cfg.orsm_safety_defaults_enabled = node["orsm_safety_defaults_enabled"].as<bool>();
        if (node["orsm"]) {
            auto orsm = node["orsm"];
            if (orsm["zone_a_radius"]) cfg.orsm_zone_a_radius = orsm["zone_a_radius"].as<float>();
            if (orsm["zone_b_radius"]) cfg.orsm_zone_b_radius = orsm["zone_b_radius"].as<float>();
            if (orsm["zone_c_radius"]) cfg.orsm_zone_c_radius = orsm["zone_c_radius"].as<float>();
            if (orsm["intruder_dedup_distance_m"]) cfg.orsm_intruder_dedup_distance_m = orsm["intruder_dedup_distance_m"].as<float>();
            if (orsm["default_orsm_setting_st"]) cfg.orsm_default_setting_st = orsm["default_orsm_setting_st"].as<int>();
            if (orsm["default_humans_recog_setting"]) cfg.orsm_default_humans_recog_setting = orsm["default_humans_recog_setting"].as<int>();
            if (orsm["default_animals_recog_setting"]) cfg.orsm_default_animals_recog_setting = orsm["default_animals_recog_setting"].as<int>();
            if (orsm["default_detec_range_setting"]) cfg.orsm_default_detec_range_setting = orsm["default_detec_range_setting"].as<int>();
            if (orsm["default_alarm_light_setting"]) cfg.orsm_default_alarm_light_setting = orsm["default_alarm_light_setting"].as<int>();
            if (orsm["default_alarm_horn_setting"]) cfg.orsm_default_alarm_horn_setting = orsm["default_alarm_horn_setting"].as<int>();
            if (orsm["default_alarm_sound_setting"]) cfg.orsm_default_alarm_sound_setting = orsm["default_alarm_sound_setting"].as<int>();
            if (orsm["default_alarm_snd_dura_setting"]) cfg.orsm_default_alarm_snd_dura_setting = orsm["default_alarm_snd_dura_setting"].as<int>();
            if (orsm["default_multi_vehicle_team_setting"]) cfg.orsm_default_multi_vehicle_team_setting = orsm["default_multi_vehicle_team_setting"].as<int>();
            if (orsm["default_multi_vehicle_team_req"]) cfg.orsm_default_multi_vehicle_team_req = orsm["default_multi_vehicle_team_req"].as<int>();
            if (orsm["default_multi_vehicle_team_prepared"]) cfg.orsm_default_multi_vehicle_team_prepared = orsm["default_multi_vehicle_team_prepared"].as<int>();
            if (orsm["default_tbox_team_status_info"]) cfg.orsm_default_tbox_team_status_info = orsm["default_tbox_team_status_info"].as<int>();
            if (orsm["default_multi_vehicle_role"]) cfg.orsm_default_multi_vehicle_role = orsm["default_multi_vehicle_role"].as<int>();
            if (orsm["default_multi_vehicle_team_ready"]) cfg.orsm_default_multi_vehicle_team_ready = orsm["default_multi_vehicle_team_ready"].as<bool>();
            if (orsm["default_vehicle_stationary_over_60s"]) cfg.orsm_default_vehicle_stationary_over_60s = orsm["default_vehicle_stationary_over_60s"].as<bool>();
            if (orsm["default_external_locked"]) cfg.orsm_default_external_locked = orsm["default_external_locked"].as<bool>();
            if (orsm["replay_force_entry_odd"]) cfg.orsm_replay_force_entry_odd = orsm["replay_force_entry_odd"].as<bool>();
            if (orsm["sensor_data_timeout_ms"]) cfg.orsm_sensor_data_timeout_ms = orsm["sensor_data_timeout_ms"].as<int>();
            if (orsm["safety_defaults_enabled"]) cfg.orsm_safety_defaults_enabled = orsm["safety_defaults_enabled"].as<bool>();
            if (orsm["lidar_fusion_period_ms"]) cfg.orsm_lidar_fusion_period_ms = orsm["lidar_fusion_period_ms"].as<int>();
            if (orsm["lidar_continuation_max_age_ms"]) cfg.orsm_lidar_continuation_max_age_ms = orsm["lidar_continuation_max_age_ms"].as<int>();
            if (orsm["far_image_motion_tracking_enabled"]) cfg.orsm_far_image_motion_tracking_enabled = orsm["far_image_motion_tracking_enabled"].as<bool>();
            if (orsm["far_image_motion_allow_lidar_backed_new_targets"]) cfg.orsm_far_image_motion_allow_lidar_backed_new_targets = orsm["far_image_motion_allow_lidar_backed_new_targets"].as<bool>();
            if (orsm["far_image_motion_min_streak"]) cfg.orsm_far_image_motion_min_streak = orsm["far_image_motion_min_streak"].as<int>();
            if (orsm["far_image_motion_debug_every_frame"]) cfg.orsm_far_image_motion_debug_every_frame = orsm["far_image_motion_debug_every_frame"].as<bool>();
            if (orsm["far_image_motion_lidar_assoc_min_distance_m"]) cfg.orsm_far_image_motion_lidar_assoc_min_distance_m = orsm["far_image_motion_lidar_assoc_min_distance_m"].as<float>();
            if (orsm["far_image_motion_unclassed_min_distance_m"]) cfg.orsm_far_image_motion_unclassed_min_distance_m = orsm["far_image_motion_unclassed_min_distance_m"].as<float>();
            if (orsm["far_image_motion_lidar_assoc_min_streak"]) cfg.orsm_far_image_motion_lidar_assoc_min_streak = orsm["far_image_motion_lidar_assoc_min_streak"].as<int>();
            if (orsm["far_image_motion_near_filter_ratio"]) cfg.orsm_far_image_motion_near_filter_ratio = orsm["far_image_motion_near_filter_ratio"].as<float>();
            if (orsm["far_image_motion_far_filter_ratio"]) cfg.orsm_far_image_motion_far_filter_ratio = orsm["far_image_motion_far_filter_ratio"].as<float>();
            if (orsm["far_image_motion_lidar_assoc_max_range_gap_m"]) cfg.orsm_far_image_motion_lidar_assoc_max_range_gap_m = orsm["far_image_motion_lidar_assoc_max_range_gap_m"].as<float>();
            if (orsm["far_image_motion_inbox_lease_priority_enabled"]) cfg.orsm_far_image_motion_inbox_lease_priority_enabled = orsm["far_image_motion_inbox_lease_priority_enabled"].as<bool>();
            if (orsm["far_image_motion_classed_box_split_ownership_enabled"]) cfg.orsm_far_image_motion_classed_box_split_ownership_enabled = orsm["far_image_motion_classed_box_split_ownership_enabled"].as<bool>();
            if (orsm["far_image_motion_max_box_area_ratio"]) cfg.orsm_far_image_motion_max_box_area_ratio = orsm["far_image_motion_max_box_area_ratio"].as<float>();
            if (orsm["far_image_motion_max_visual_range_front_m"]) cfg.orsm_far_image_motion_max_visual_range_front_m = orsm["far_image_motion_max_visual_range_front_m"].as<float>();
            if (orsm["far_image_motion_max_visual_range_other_m"]) cfg.orsm_far_image_motion_max_visual_range_other_m = orsm["far_image_motion_max_visual_range_other_m"].as<float>();
            if (orsm["rear_motion_range_override_enabled"]) cfg.orsm_rear_motion_range_override_enabled = orsm["rear_motion_range_override_enabled"].as<bool>();
            if (orsm["rear_motion_range_override_min_center_u_px"]) cfg.orsm_rear_motion_range_override_min_center_u_px = orsm["rear_motion_range_override_min_center_u_px"].as<float>();
            if (orsm["rear_motion_range_override_max_center_u_px"]) cfg.orsm_rear_motion_range_override_max_center_u_px = orsm["rear_motion_range_override_max_center_u_px"].as<float>();
            if (orsm["rear_motion_range_override_min_bottom_v_px"]) cfg.orsm_rear_motion_range_override_min_bottom_v_px = orsm["rear_motion_range_override_min_bottom_v_px"].as<float>();
            if (orsm["rear_motion_range_override_max_bottom_v_px"]) cfg.orsm_rear_motion_range_override_max_bottom_v_px = orsm["rear_motion_range_override_max_bottom_v_px"].as<float>();
            if (orsm["rear_motion_range_override_distance_m"]) cfg.orsm_rear_motion_range_override_distance_m = orsm["rear_motion_range_override_distance_m"].as<float>();
            if (orsm["far_image_motion_min_changed_ratio"]) cfg.orsm_far_image_motion_min_changed_ratio = orsm["far_image_motion_min_changed_ratio"].as<float>();
            if (orsm["far_image_motion_min_mean_delta"]) cfg.orsm_far_image_motion_min_mean_delta = orsm["far_image_motion_min_mean_delta"].as<float>();
            if (orsm["far_image_motion_min_component_ratio"]) cfg.orsm_far_image_motion_min_component_ratio = orsm["far_image_motion_min_component_ratio"].as<float>();
            if (orsm["far_image_motion_confidence"]) cfg.orsm_far_image_motion_confidence = orsm["far_image_motion_confidence"].as<float>();
            if (orsm["far_image_motion_class_transfer_min_overlap"]) cfg.orsm_far_image_motion_class_transfer_min_overlap = orsm["far_image_motion_class_transfer_min_overlap"].as<float>();
            if (orsm["far_image_motion_classed_single_match_enabled"]) cfg.orsm_far_image_motion_classed_single_match_enabled = orsm["far_image_motion_classed_single_match_enabled"].as<bool>();
            if (orsm["class_label_lock_enabled"]) cfg.orsm_class_label_lock_enabled = orsm["class_label_lock_enabled"].as<bool>();
            if (orsm["alarm_first_publish_class_required"]) cfg.orsm_alarm_first_publish_class_required = orsm["alarm_first_publish_class_required"].as<bool>();
            if (orsm["alarm_first_publish_min_distance_m"]) cfg.orsm_alarm_first_publish_min_distance_m = orsm["alarm_first_publish_min_distance_m"].as<float>();
            if (orsm["alarm_first_publish_max_distance_m"]) cfg.orsm_alarm_first_publish_max_distance_m = orsm["alarm_first_publish_max_distance_m"].as<float>();
            if (orsm["lidar_semantic_latch_output_enabled"]) cfg.orsm_lidar_semantic_latch_output_enabled = orsm["lidar_semantic_latch_output_enabled"].as<bool>();
            if (orsm["lidar_semantic_latch_refresh_ms"]) cfg.orsm_lidar_semantic_latch_refresh_ms = orsm["lidar_semantic_latch_refresh_ms"].as<int>();
            if (orsm["far_image_motion_lease_ms"]) cfg.orsm_far_image_motion_lease_ms = orsm["far_image_motion_lease_ms"].as<int>();
            if (orsm["front_lidar_motion_lease_enabled"]) cfg.orsm_front_lidar_motion_lease_enabled = orsm["front_lidar_motion_lease_enabled"].as<bool>();
            if (orsm["front_lidar_motion_min_x_m"]) cfg.orsm_front_lidar_motion_min_x_m = orsm["front_lidar_motion_min_x_m"].as<float>();
            if (orsm["front_lidar_motion_max_abs_y_m"]) cfg.orsm_front_lidar_motion_max_abs_y_m = orsm["front_lidar_motion_max_abs_y_m"].as<float>();
            if (orsm["front_lidar_motion_max_y_span_m"]) cfg.orsm_front_lidar_motion_max_y_span_m = orsm["front_lidar_motion_max_y_span_m"].as<float>();
            if (orsm["front_lidar_motion_min_x_progress_m"]) cfg.orsm_front_lidar_motion_min_x_progress_m = orsm["front_lidar_motion_min_x_progress_m"].as<float>();
            if (orsm["front_lidar_motion_min_history_points"]) cfg.orsm_front_lidar_motion_min_history_points = orsm["front_lidar_motion_min_history_points"].as<int>();
            if (orsm["front_lidar_motion_min_toward_steps"]) cfg.orsm_front_lidar_motion_min_toward_steps = orsm["front_lidar_motion_min_toward_steps"].as<int>();
            if (orsm["front_lidar_motion_history_max_age_ms"]) cfg.orsm_front_lidar_motion_history_max_age_ms = orsm["front_lidar_motion_history_max_age_ms"].as<int>();
            if (orsm["front_lidar_motion_lease_ms"]) cfg.orsm_front_lidar_motion_lease_ms = orsm["front_lidar_motion_lease_ms"].as<int>();
            if (orsm["lidar_semantic_latch_max_active_targets"]) cfg.orsm_lidar_semantic_latch_max_active_targets = orsm["lidar_semantic_latch_max_active_targets"].as<int>();
            if (orsm["lidar_only_far_output_enabled"]) cfg.orsm_lidar_only_far_output_enabled = orsm["lidar_only_far_output_enabled"].as<bool>();
            if (orsm["lidar_only_far_output_min_distance_m"]) cfg.orsm_lidar_only_far_output_min_distance_m = orsm["lidar_only_far_output_min_distance_m"].as<float>();
            if (orsm["lidar_only_far_output_max_distance_m"]) cfg.orsm_lidar_only_far_output_max_distance_m = orsm["lidar_only_far_output_max_distance_m"].as<float>();
            if (orsm["lidar_only_near_output_enabled"]) cfg.orsm_lidar_only_near_output_enabled = orsm["lidar_only_near_output_enabled"].as<bool>();
            if (orsm["lidar_only_near_output_max_distance_m"]) cfg.orsm_lidar_only_near_output_max_distance_m = orsm["lidar_only_near_output_max_distance_m"].as<float>();
            if (orsm["lidar_only_output_min_observations"]) cfg.orsm_lidar_only_output_min_observations = orsm["lidar_only_output_min_observations"].as<int>();
            if (orsm["lidar_only_output_min_motion_m"]) cfg.orsm_lidar_only_output_min_motion_m = orsm["lidar_only_output_min_motion_m"].as<float>();
            if (orsm["lidar_only_output_motion_window_ms"]) cfg.orsm_lidar_only_output_motion_window_ms = orsm["lidar_only_output_motion_window_ms"].as<int>();
            if (orsm["lidar_only_output_max_speed_mps"]) cfg.orsm_lidar_only_output_max_speed_mps = orsm["lidar_only_output_max_speed_mps"].as<float>();
            if (orsm["crop_yolo_enabled"]) cfg.orsm_crop_yolo_enabled = orsm["crop_yolo_enabled"].as<bool>();
            if (orsm["crop_yolo_min_streak"]) cfg.orsm_crop_yolo_min_streak = orsm["crop_yolo_min_streak"].as<int>();
            if (orsm["crop_yolo_pad_ratio"]) cfg.orsm_crop_yolo_pad_ratio = orsm["crop_yolo_pad_ratio"].as<float>();
            if (orsm["crop_yolo_min_confidence"]) cfg.orsm_crop_yolo_min_confidence = orsm["crop_yolo_min_confidence"].as<float>();
            if (orsm["crop_yolo_every_n_frames"]) cfg.orsm_crop_yolo_every_n_frames = orsm["crop_yolo_every_n_frames"].as<int>();
            if (orsm["crop_yolo_max_boxes"]) cfg.orsm_crop_yolo_max_boxes = orsm["crop_yolo_max_boxes"].as<int>();
            if (orsm["crop_yolo_max_distance_m"]) cfg.orsm_crop_yolo_max_distance_m = orsm["crop_yolo_max_distance_m"].as<float>();
            if (orsm["near_static_visual_refresh_ms"]) cfg.orsm_near_static_visual_refresh_ms = orsm["near_static_visual_refresh_ms"].as<int>();
            if (orsm["far_dynamic_min_distance_m"]) cfg.orsm_far_dynamic_min_distance_m = orsm["far_dynamic_min_distance_m"].as<float>();
            if (orsm["far_image_motion_required"]) cfg.orsm_far_image_motion_required = orsm["far_image_motion_required"].as<bool>();
            if (orsm["track_stable_min_age_ms"]) cfg.orsm_track_stable_min_age_ms = orsm["track_stable_min_age_ms"].as<int>();
            if (orsm["track_stable_min_observations"]) cfg.orsm_track_stable_min_observations = orsm["track_stable_min_observations"].as<int>();
            if (orsm["track_stable_lidar_min_observations"]) cfg.orsm_track_stable_lidar_min_observations = orsm["track_stable_lidar_min_observations"].as<int>();
            if (orsm["track_stable_max_uncertainty_m"]) cfg.orsm_track_stable_max_uncertainty_m = orsm["track_stable_max_uncertainty_m"].as<float>();
            if (orsm["track_lost_timeout_ms"]) cfg.orsm_track_lost_timeout_ms = orsm["track_lost_timeout_ms"].as<int>();
            if (orsm["track_prune_timeout_ms"]) cfg.orsm_track_prune_timeout_ms = orsm["track_prune_timeout_ms"].as<int>();
            if (orsm["visual_lidar_ground_gate_enabled"]) cfg.orsm_visual_lidar_ground_gate_enabled = orsm["visual_lidar_ground_gate_enabled"].as<bool>();
            if (orsm["visual_lidar_ground_gate_min_m"]) cfg.orsm_visual_lidar_ground_gate_min_m = orsm["visual_lidar_ground_gate_min_m"].as<float>();
            if (orsm["visual_lidar_ground_gate_range_ratio"]) cfg.orsm_visual_lidar_ground_gate_range_ratio = orsm["visual_lidar_ground_gate_range_ratio"].as<float>();
            if (orsm["lidar_visual_roi_enabled"]) cfg.orsm_lidar_visual_roi_enabled = orsm["lidar_visual_roi_enabled"].as<bool>();
            if (orsm["lidar_history_anchor_enabled"]) cfg.orsm_lidar_history_anchor_enabled = orsm["lidar_history_anchor_enabled"].as<bool>();
            if (orsm["lidar_history_anchor_max_age_ms"]) cfg.orsm_lidar_history_anchor_max_age_ms = orsm["lidar_history_anchor_max_age_ms"].as<int>();
            if (orsm["lidar_history_anchor_gate_m"]) cfg.orsm_lidar_history_anchor_gate_m = orsm["lidar_history_anchor_gate_m"].as<float>();
            if (orsm["lidar_primary_static_semantic_release_ms"]) cfg.orsm_lidar_primary_static_semantic_release_ms = orsm["lidar_primary_static_semantic_release_ms"].as<int>();
            if (orsm["lidar_primary_static_semantic_max_displacement_m"]) cfg.orsm_lidar_primary_static_semantic_max_displacement_m = orsm["lidar_primary_static_semantic_max_displacement_m"].as<float>();
            if (orsm["lidar_primary_voxel_leaf_m"]) cfg.orsm_lidar_primary_voxel_leaf_m = orsm["lidar_primary_voxel_leaf_m"].as<float>();
            if (orsm["lidar_primary_cluster_tolerance_m"]) cfg.orsm_lidar_primary_cluster_tolerance_m = orsm["lidar_primary_cluster_tolerance_m"].as<float>();
            if (orsm["lidar_primary_min_cluster_points"]) cfg.orsm_lidar_primary_min_cluster_points = orsm["lidar_primary_min_cluster_points"].as<int>();
            if (orsm["lidar_primary_mid_min_distance_m"]) cfg.orsm_lidar_primary_mid_min_distance_m = orsm["lidar_primary_mid_min_distance_m"].as<float>();
            if (orsm["lidar_primary_mid_min_cluster_points"]) cfg.orsm_lidar_primary_mid_min_cluster_points = orsm["lidar_primary_mid_min_cluster_points"].as<int>();
            if (orsm["lidar_primary_far_min_distance_m"]) cfg.orsm_lidar_primary_far_min_distance_m = orsm["lidar_primary_far_min_distance_m"].as<float>();
            if (orsm["lidar_primary_far_min_cluster_points"]) cfg.orsm_lidar_primary_far_min_cluster_points = orsm["lidar_primary_far_min_cluster_points"].as<int>();
            if (orsm["lidar_primary_far_voxel_leaf_m"]) cfg.orsm_lidar_primary_far_voxel_leaf_m = orsm["lidar_primary_far_voxel_leaf_m"].as<float>();
            if (orsm["lidar_primary_far_cluster_tolerance_m"]) cfg.orsm_lidar_primary_far_cluster_tolerance_m = orsm["lidar_primary_far_cluster_tolerance_m"].as<float>();
            if (orsm["lidar_primary_anchor_min_cluster_points"]) cfg.orsm_lidar_primary_anchor_min_cluster_points = orsm["lidar_primary_anchor_min_cluster_points"].as<int>();
            if (orsm["lidar_primary_anchor_max_radius_m"]) cfg.orsm_lidar_primary_anchor_max_radius_m = orsm["lidar_primary_anchor_max_radius_m"].as<float>();
            if (orsm["lidar_primary_anchor_min_height_m"]) cfg.orsm_lidar_primary_anchor_min_height_m = orsm["lidar_primary_anchor_min_height_m"].as<float>();
            // 【侧向稀疏聚类放宽】见 sentinel_node.h 注释；默认开启，可通过配置关闭
            if (orsm["lidar_primary_blindside_sparse_enabled"]) cfg.orsm_lidar_primary_blindside_sparse_enabled = orsm["lidar_primary_blindside_sparse_enabled"].as<bool>();
            if (orsm["lidar_primary_blindside_min_cluster_points"]) cfg.orsm_lidar_primary_blindside_min_cluster_points = orsm["lidar_primary_blindside_min_cluster_points"].as<int>();
            if (orsm["lidar_primary_blindside_max_radius_m"]) cfg.orsm_lidar_primary_blindside_max_radius_m = orsm["lidar_primary_blindside_max_radius_m"].as<float>();
            if (orsm["lidar_primary_blindside_min_lateral_ratio"]) cfg.orsm_lidar_primary_blindside_min_lateral_ratio = orsm["lidar_primary_blindside_min_lateral_ratio"].as<float>();
            if (orsm["lidar_primary_static_background_filter_enabled"]) cfg.orsm_lidar_primary_static_background_filter_enabled = orsm["lidar_primary_static_background_filter_enabled"].as<bool>();
            if (orsm["lidar_primary_static_background_min_hits"]) cfg.orsm_lidar_primary_static_background_min_hits = orsm["lidar_primary_static_background_min_hits"].as<int>();
            if (orsm["lidar_primary_static_background_max_displacement_m"]) cfg.orsm_lidar_primary_static_background_max_displacement_m = orsm["lidar_primary_static_background_max_displacement_m"].as<float>();
            if (orsm["lidar_primary_static_background_max_speed_mps"]) cfg.orsm_lidar_primary_static_background_max_speed_mps = orsm["lidar_primary_static_background_max_speed_mps"].as<float>();
            if (orsm["static_cluster_semantic_lockout_enabled"]) cfg.orsm_static_cluster_semantic_lockout_enabled = orsm["static_cluster_semantic_lockout_enabled"].as<bool>();
            if (orsm["static_cluster_semantic_lockout_min_hits"]) cfg.orsm_static_cluster_semantic_lockout_min_hits = orsm["static_cluster_semantic_lockout_min_hits"].as<int>();
            if (orsm["static_cluster_semantic_lockout_max_displacement_m"]) cfg.orsm_static_cluster_semantic_lockout_max_displacement_m = orsm["static_cluster_semantic_lockout_max_displacement_m"].as<float>();
            if (orsm["lidar_primary_min_z_m"]) cfg.orsm_lidar_primary_min_z_m = orsm["lidar_primary_min_z_m"].as<float>();
            if (orsm["lidar_primary_max_z_m"]) cfg.orsm_lidar_primary_max_z_m = orsm["lidar_primary_max_z_m"].as<float>();
            if (orsm["lidar_primary_front_fixed_ground_plane_enabled"]) cfg.orsm_lidar_primary_front_fixed_ground_plane_enabled = orsm["lidar_primary_front_fixed_ground_plane_enabled"].as<bool>();
            if (orsm["lidar_primary_front_fixed_ground_plane_a"]) cfg.orsm_lidar_primary_front_fixed_ground_plane_a = orsm["lidar_primary_front_fixed_ground_plane_a"].as<float>();
            if (orsm["lidar_primary_front_fixed_ground_plane_b"]) cfg.orsm_lidar_primary_front_fixed_ground_plane_b = orsm["lidar_primary_front_fixed_ground_plane_b"].as<float>();
            if (orsm["lidar_primary_front_fixed_ground_plane_c"]) cfg.orsm_lidar_primary_front_fixed_ground_plane_c = orsm["lidar_primary_front_fixed_ground_plane_c"].as<float>();
            if (orsm["lidar_primary_front_fixed_ground_plane_min_height_m"]) cfg.orsm_lidar_primary_front_fixed_ground_plane_min_height_m = orsm["lidar_primary_front_fixed_ground_plane_min_height_m"].as<float>();
            if (orsm["lidar_primary_source_z_split_distance_m"]) cfg.orsm_lidar_primary_source_z_split_distance_m = orsm["lidar_primary_source_z_split_distance_m"].as<float>();
            if (orsm["lidar_primary_side_rear_min_z_m"]) cfg.orsm_lidar_primary_side_rear_min_z_m = orsm["lidar_primary_side_rear_min_z_m"].as<float>();
            if (orsm["lidar_primary_left_min_z_m"]) cfg.orsm_lidar_primary_left_min_z_m = orsm["lidar_primary_left_min_z_m"].as<float>();
            if (orsm["lidar_primary_semantic_gate_m"]) cfg.orsm_lidar_primary_semantic_gate_m = orsm["lidar_primary_semantic_gate_m"].as<float>();
            if (orsm["multi_vehicle_enable_network_check"]) cfg.orsm_multi_vehicle_enable_network_check = orsm["multi_vehicle_enable_network_check"].as<bool>();
            if (orsm["multi_vehicle_speed_entry_max_kmh"]) cfg.orsm_multi_vehicle_speed_entry_max_kmh = orsm["multi_vehicle_speed_entry_max_kmh"].as<float>();
            if (orsm["multi_vehicle_speed_exit_max_kmh"]) cfg.orsm_multi_vehicle_speed_exit_max_kmh = orsm["multi_vehicle_speed_exit_max_kmh"].as<float>();
            if (orsm["multi_vehicle_soc_entry_min"]) cfg.orsm_multi_vehicle_soc_entry_min = orsm["multi_vehicle_soc_entry_min"].as<float>();
            if (orsm["multi_vehicle_soc_exit_min"]) cfg.orsm_multi_vehicle_soc_exit_min = orsm["multi_vehicle_soc_exit_min"].as<float>();
            if (orsm["multi_vehicle_stationary_required_sec"]) cfg.orsm_multi_vehicle_stationary_required_sec = orsm["multi_vehicle_stationary_required_sec"].as<int>();
            if (orsm["multi_vehicle_security_side"]) cfg.orsm_multi_vehicle_security_side = orsm["multi_vehicle_security_side"].as<int>();
            if (orsm["multi_vehicle_external_radius"]) cfg.orsm_multi_vehicle_external_radius = orsm["multi_vehicle_external_radius"].as<float>();
            if (orsm["multi_vehicle_internal_radius"]) cfg.orsm_multi_vehicle_internal_radius = orsm["multi_vehicle_internal_radius"].as<float>();
            if (orsm["multi_vehicle_no_alarm_radius"]) cfg.orsm_multi_vehicle_no_alarm_radius = orsm["multi_vehicle_no_alarm_radius"].as<float>();
        }
        if (node["enable_rscl_publish"]) cfg.enable_rscl_publish = node["enable_rscl_publish"].as<bool>();
        if (node["status_publish_mode"]) cfg.status_publish_mode = node["status_publish_mode"].as<std::string>();
        if (node["enable_algorithm_time_log"]) cfg.enable_algorithm_time_log = node["enable_algorithm_time_log"].as<bool>();
        if (node["algorithm_time_log_interval_frames"]) cfg.algorithm_time_log_interval_frames = node["algorithm_time_log_interval_frames"].as<int>();
        std::cout << "[main] Config loaded from " << path << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "[main] Config load failed: " << e.what()
                  << ", using defaults." << std::endl;
    }
    applyOrsmSafetyDefaults(cfg);
    std::cout << "[main][ORSM] effective safety defaults: enabled="
              << (cfg.orsm_safety_defaults_enabled ? "true" : "false")
              << " conf_threshold=" << cfg.conf_threshold
              << std::endl;
    return cfg;
}

int main(int argc, char* argv[]) {
    std::signal(SIGINT,  signalHandler);
    std::signal(SIGTERM, signalHandler);

    std::string config_path = "./config.yaml";
    if (argc > 1) config_path = argv[1];

    auto cfg = loadConfig(config_path);
    sentinel::SentinelNode node(cfg);

    if (!node.start(argc, argv)) {
        std::cerr << "[main] Failed to start SentinelNode." << std::endl;
        return 1;
    }

    std::cout << "[main] Sentinel running. Press Ctrl+C to stop." << std::endl;
    while (g_running) {
        sleep(1);
    }

    // stop() 会释放 IMU 订阅器；先保存本次运行统计。
    const auto final_frames = node.frameCount();
    const auto final_alerts = node.alertCount();
    const auto final_vibrations = node.vibrationCount();
    node.stop();
    std::cout << "[main] frames=" << final_frames
              << " alerts=" << final_alerts
              << " vibrations=" << final_vibrations
              << std::endl;
    return 0;
}
