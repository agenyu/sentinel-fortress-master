#pragma once

#include "imu/imu_subscriber.h"
#include "uss/uss_subscriber.h"
#include "lidar/lidar_subscriber.h"
#include "camera/camera_subscriber.h"
#include "camera/fisheye_undistort.h"
#include "calib/sensor_calib_manager.h"
#include "fusion/uss_vision_fusion.h"
#include "recording/video_recorder.h"
#include "detection/detector.h"
#include "detection/tracker.h"
#include "alert/alert_manager.h"
#include "state/sentry_state_machine.h"
#include "orsm/orsm_manager.h"
#include "publish/rscl_publisher.h"
#include "rscl_access/rscl_access.h"

#include <opencv2/opencv.hpp>
#include <memory>
#include <string>
#include <atomic>
#include <mutex>
#include <optional>
#include <array>
#include <deque>
#include <map>
#include <vector>
#include <thread>
#include <condition_variable>
#include <unordered_map>
#include <functional>
#include <limits>

#include <ad_msg_idl/ad_vehicle/vehicle.capnp.h>
#include <ad_msg_idl/avp_uss/uss_perception.capnp.h>
#ifndef SENTINEL_DIAGNOSTIC_NO_L3
#include <ad_msg_idl/l3_msgs/orsm_info.capnp.h>
#include <ad_msg_idl/l3_msgs/sentry_mode.capnp.h>
#include <ad_msg_idl/l3_msgs/chassis_info_m18.capnp.h>
#endif

namespace sentinel {

struct SentinelConfig {
    // 摄像头（哨兵模式使用 4 路鱼眼环视 FOV195）
    std::vector<std::string> camera_topics{
        "/sensor/camera/front_camera_fov195/encode",
        "/sensor/camera/rear_camera_fov195/encode",
        "/sensor/camera/left_camera_fov195/encode",
        "/sensor/camera/right_camera_fov195/encode",
    };
    // topic → 相机名称映射（与标定参数目录名一致）
    std::vector<std::string> camera_names{
        "front_camera_fov195",
        "rear_camera_fov195",
        "left_camera_fov195",
        "right_camera_fov195",
    };
    std::string calib_base_dir{"param/calib"};  // 标定参数根目录
    // Inbox 差值框专用相机标定目录；普通视觉/LiDAR关联仍使用 calib_base_dir。
    std::string inbox_calib_dir{"param/calib_inbox"};
    std::string imu_topic{"/sensor/imu"};

    // USS topics(来自接口设计说明书 + V2.3.0 bag fused static map)
    std::string uss_pdc_topic{"/parking/pdc/pdc_info"};
    std::string uss_meb_topic{"/parking/meb/meb_info"};
    std::string uss_fused_topic{"/parking/uss/fused_points"};
    std::string uss_fused_legacy_topic{"/sensor/uss/fused_points"}; // 兼容既有 bag/旧接口
    uint16_t    uss_alert_threshold_mm{500};
    float       uss_fused_coordinate_scale_m{0.001f}; // UssFusedPoint x/y 原始单位：mm
    int         uss_fused_min_exist_prob{50};
    bool        enable_uss_vision_fusion{false};
    float       uss_fusion_max_range_m{1.3f};
    float       uss_fusion_candidate_vision_range_m{3.0f};
    float       uss_fusion_max_age_ms{200.0f};
    float       uss_fusion_confidence_boost{0.25f};

    // LiDAR topics(仅越野堡垒 ORSM 使用；哨兵模式禁止使用 LiDAR)
    // 车端当前上报点云已是车辆坐标系，订阅侧不再依赖 LiDAR 外参标定。
    std::vector<std::string> lidar_topics{
        "/sensor/lidar/front_lidar",
        "/sensor/lidar/left_lidar",
        "/sensor/lidar/rear_lidar",
        "/sensor/lidar/right_lidar",
    };
    std::vector<std::string> lidar_names{
        "front_lidar",
        "left_lidar",
        "rear_lidar",
        "right_lidar",
    };
    // RSCL单点16字节布局，与lidar_topics/lidar_names一一对应。
    // 0721实测bag的四路点云均使用4字节对齐布局；若后续车端切换补盲紧凑布局，
    // 可按topic将对应项显式改为 blind_spot16。
    std::vector<std::string> lidar_point_layouts{
        "robosense_aligned16",
        "robosense_aligned16",
        "robosense_aligned16",
        "robosense_aligned16",
    };
    // 每路点云若仍在雷达自身坐标系，可按topic配置绕车辆z轴的安装方向角。
    // 输入已是统一车辆坐标系时全部保持0；正角为逆时针，车辆坐标约定x前/y左。
    std::vector<float> lidar_yaw_degrees{0.0f, 0.0f, 0.0f, 0.0f};
    // 可选的每路2x2 XY显式映射，优先于lidar_yaw_degrees；用于验证bag坐标契约。
    std::vector<std::array<float, 4>> lidar_xy_transforms;
    float       lidar_near_distance_m{3.0f};
    int         lidar_near_min_points{50};
    // LiDAR/Camera 时间同步策略：
    // - real_vehicle：实车默认，使用较小窗口，避免过旧点云参与 ORSM 判断。
    // - bag_replay：仅用于离线 bag 回放验证，允许更大窗口容忍录制/回放时间戳偏差。
    std::string lidar_time_sync_mode{"real_vehicle"};
    float       lidar_max_age_ms{200.0f};             // 实车与视频帧融合时允许的 LiDAR 最大时间差
    float       lidar_bag_replay_max_age_ms{3000.0f}; // bag 回放验证窗口，禁止作为实车默认兜底
    float       lidar_merge_voxel_leaf_m{0.10f}; // 多路 LiDAR 合并去重体素，0 表示不去重

    float effectiveLidarMaxAgeMs() const {
        return lidar_time_sync_mode == "bag_replay" ? lidar_bag_replay_max_age_ms : lidar_max_age_ms;
    }

    // 对外 RSCL topic(借鉴 development 分支命名,但由单节点内部直接发布/订阅)
    std::string vehicle_report_topic{"/canbus/vehicle_report"};
    std::string chassis_info_topic{"/driver/chassis_info_m18v2"};
    std::string sentry_request_topic{"/network/sentry_req"};
    std::string sentry_state_topic{"/sentry/state_broadcast"};
    std::string sentry_event_topic{"/network/sentry_resp"};
    std::string sentry_action_topic{"/sentry/action_request"};
    std::string sentry_media_cmd_topic{"/sentry/internal/media_cmd"};
    std::string sentry_media_result_topic{"/sentry/internal/media_result"};
    std::string orsm_setting_topic{"/network/fortress_req_ivi"};
    std::string orsm_set_info_topic{"/network/fortress_req_tbox"};
    std::string orsm_state_topic{"/network/fortress_resp"};
    std::string orsm_action_topic{"/orsm/action_request"};

    // ORSM 配置（从 config.yaml orsm: 节加载）
    float orsm_zone_a_radius{30.0f};
    float orsm_zone_b_radius{15.0f};
    float orsm_zone_c_radius{5.0f};
    float orsm_intruder_dedup_distance_m{1.0f}; // 入侵物空间去重距离阈值；<=0 表示关闭去重
    int   orsm_default_setting_st{0};  // OrsmSettingType: 0=OFF,1=单车,2=多车
    int   orsm_default_humans_recog_setting{1};
    int   orsm_default_animals_recog_setting{1};
    int   orsm_default_detec_range_setting{2};
    int   orsm_default_alarm_light_setting{1};
    int   orsm_default_alarm_horn_setting{1};
    int   orsm_default_alarm_sound_setting{0};
    int   orsm_default_alarm_snd_dura_setting{2};
    int   orsm_default_multi_vehicle_team_setting{0};
    int   orsm_default_multi_vehicle_team_req{0};
    int   orsm_default_multi_vehicle_team_prepared{0};
    int   orsm_default_tbox_team_status_info{0};
    int   orsm_default_multi_vehicle_role{0}; // 0=unknown,1=leader,2=member
    bool  orsm_default_multi_vehicle_team_ready{false};
    bool  orsm_default_vehicle_stationary_over_60s{true};
    bool  orsm_default_external_locked{true};
    bool  orsm_replay_force_entry_odd{false}; // 仅离线回放验证使用：强制满足 ORSM 进入 ODD
    bool  orsm_safety_defaults_enabled{true}; // 堡垒生产安全默认：防止部署配置漂移关闭关键感知链路
    // 开启 ORSM 时：相机/雷达墙钟新鲜度超时则置 sensor_fault，反馈 0x7。
    // 与 CAN override_no_sensor_fault 独立；<=0 关闭该本地检查（台架无传感器时可关）。
    int   orsm_sensor_data_timeout_ms{2000};
    int   orsm_lidar_fusion_period_ms{100}; // ORSM LiDAR 融合限频，默认 10Hz，避免逐 camera frame 全量融合
    int   orsm_lidar_continuation_max_age_ms{15000}; // 静态释放判定：语义确认后允许保持“最近动态”记忆的时长
    bool  orsm_far_image_motion_tracking_enabled{true}; // 远距小目标检测短时失效时，用图像运动延续已有稳定轨迹
    int   orsm_far_image_motion_max_age_ms{1500}; // 远距图像运动状态机：前后帧间隔超过该时长则重置连续计数
    bool  orsm_far_image_motion_allow_lidar_backed_new_targets{true}; // 远距图像运动+LiDAR稳定轨迹允许新建unknown候选
    int   orsm_far_image_motion_min_streak{3};
    // 逐帧打印差值框 bbox 高度、底边像素位置和单目地面估距；默认关闭，避免增加日志 I/O。
    bool  orsm_far_image_motion_debug_every_frame{false};
    // YOLO/差值框重合时的近距/远距归属分界：<= 该距离由 YOLO 负责，>
    // 该距离由差值框负责。无 YOLO 重合的差值框使用下方独立的 15m 下限。
    float orsm_far_image_motion_lidar_assoc_min_distance_m{15.0f};
    float orsm_far_image_motion_unclassed_min_distance_m{15.0f};
    // 近/远分区边界(米)。现在只用于"YOLO/差值框候选关联到已有 global 轨迹"时的
    // 同区判断：跨分区的候选不会被优先匹配到发布中的轨迹。<=0 表示关闭该分区。
    // 注意：它不再限制 YOLO 检测框给雷达簇贴类别的距离范围。
    int   orsm_far_image_motion_lidar_assoc_min_streak{1}; // 单次Inbox匹配即可授予租约
    // 同一个框可能同时框住近处干扰和远处的目标（同一条射线）。丢弃明显比框估距
    // 更近或更远的簇，只保留接近框估距的那些。far ratio <= 0 表示关闭远端过滤。
    float orsm_far_image_motion_near_filter_ratio{0.80f};
    float orsm_far_image_motion_far_filter_ratio{2.0f};
    // 关联残差上限(m)：差值框自身的单目地面估距允许偏短几米（框底边落在脚面
    // 以上、腰高植被遮挡等），但同一射线上远出该残差的簇已属于另一个物体，
    // 不能再被这个框认领。与 near/far ratio 是"且"关系，<= 0 表示关闭。
    float orsm_far_image_motion_lidar_assoc_max_range_gap_m{5.0f};
    // 单目估距抖动可能使静态背景簇获得更小残差。InBox 选簇时优先匹配
    // 活跃租约或已绑定全局轨迹的 L#，未命中再按残差排序。
    bool  orsm_far_image_motion_inbox_lease_priority_enabled{true};
    // 重叠的 YOLO 框和差值框只保留一个关联入口，避免语义状态冲突。
    // 根据差值框底边估距和近远分界选择入口；带类别差值框不受后续分界门限制。
    bool  orsm_far_image_motion_classed_box_split_ownership_enabled{true};
    // 差值框面积超过整幅图像该比例时视为明显近场大框，不参与远距 LiDAR 关联。
    // <= 0 表示关闭。
    float orsm_far_image_motion_max_box_area_ratio{0.05f};
    // 差值框估距上限。单目地面射线本身封顶约 30m，而 bbox 高度兜底估算在目标
    // 离开视野后可能编出 70~138m 的假框，必须拒绝，避免用不可信的框去建/挡轨迹。
    float orsm_far_image_motion_max_visual_range_front_m{55.0f};
    float orsm_far_image_motion_max_visual_range_other_m{35.0f};
    // 后相机右侧特定图像区域的框高兜底估距偏近。命中该区域时将差值框估距
    // 修正为固定距离；使用框中心横坐标和框底纵坐标，避免依赖框宽高抖动。
    bool  orsm_rear_motion_range_override_enabled{false};
    float orsm_rear_motion_range_override_min_center_u_px{1550.0f};
    float orsm_rear_motion_range_override_max_center_u_px{1710.0f};
    float orsm_rear_motion_range_override_min_bottom_v_px{512.0f};
    float orsm_rear_motion_range_override_max_bottom_v_px{540.0f};
    float orsm_rear_motion_range_override_distance_m{30.0f};
    float orsm_far_image_motion_min_changed_ratio{0.0015f};
    float orsm_far_image_motion_min_mean_delta{0.6f};
    float orsm_far_image_motion_min_component_ratio{0.00025f};
    float orsm_far_image_motion_confidence{0.30f};
    // 同相机 YOLO 与差值框的重合阈值。当前重合后始终保留 YOLO、丢弃差值框。
    // 使用“交集 / 较小框面积”，因为两个框形状差异较大，普通 IoU 会偏低。
    float orsm_far_image_motion_class_transfer_min_overlap{0.50f};
    // 已继承真实检测类别的差值框只需一次 InBox 匹配即可获得租约；无类别的
    // 差值框仍按 far_image_motion_lidar_assoc_min_streak 要求连续匹配。
    bool  orsm_far_image_motion_classed_single_match_enabled{true};
    // 类别按置信度单调提升：unknown 可被真实类别升级；已有真实类别只能被"严格
    // 更高置信度"的真实类别替换；unknown 永远不能覆盖或清除已有类别。
    bool  orsm_class_label_lock_enabled{true};
    // 当前策略不要求首次发布必须带类别；差值框可先以 unknown 进入发布流程。
    // 下面的距离字段仅在重新开启类别门槛时生效。
    bool  orsm_alarm_first_publish_class_required{false};
    float orsm_alarm_first_publish_min_distance_m{0.0f};
    float orsm_alarm_first_publish_max_distance_m{0.0f}; // <=0 表示不设上限
    float orsm_global_max_position_step_m{0.35f};
    // Offline policy: the first true visual semantic association immediately
    // qualifies the same temporal LiDAR trajectory for output.  Every later
    // true association renews this lease; no semantic refresh within the
    // window stops external publication.
    bool  orsm_lidar_semantic_latch_output_enabled{false};
    int   orsm_lidar_semantic_latch_refresh_ms{15000};
    // 差值框(绿圈)刷新统一发布租约时长；YOLO语义刷新使用上面的20s租约时长，
    // 两者写入同一个租约并取更晚的过期时间。
    int   orsm_far_image_motion_lease_ms{15000};
    // 前向 LiDAR 垂直接近兜底：仅在车头外侧 X>20m 的走廊内，
    // 用同一 L# 的连续轨迹确认 X 向车辆靠近、Y 基本不变后发放短租约。
    bool  orsm_front_lidar_motion_lease_enabled{true};
    float orsm_front_lidar_motion_min_x_m{20.0f};
    float orsm_front_lidar_motion_max_abs_y_m{10.0f};
    float orsm_front_lidar_motion_max_y_span_m{1.0f};
    float orsm_front_lidar_motion_min_x_progress_m{1.0f};
    int   orsm_front_lidar_motion_min_history_points{5};
    int   orsm_front_lidar_motion_min_toward_steps{3};
    int   orsm_front_lidar_motion_history_max_age_ms{3000};
    int   orsm_front_lidar_motion_lease_ms{3000};
    // 0 leaves multi-intruder operation unrestricted.  A positive value is
    // an offline validation guard limiting simultaneously leased L# sources.
    int   orsm_lidar_semantic_latch_max_active_targets{0};
    bool  orsm_lidar_only_far_output_enabled{true}; // 10m以上稳定LiDAR-only轨迹可无语义输出
    float orsm_lidar_only_far_output_min_distance_m{10.0f}; // 动态目标需累计N次视觉语义关联后才允许视觉丢失后继续输出
    float orsm_lidar_only_far_output_max_distance_m{0.0f}; // lidar-only 远场发布距离上限；0=不设限。bag-1008 实测 >20m 停车簇中心随机游走（多雷达观测偏差+稀疏聚类）可伪装成慢速运动，20m 内外噪声/信号可分性差异大，超过后交由差值框/语义租约路径
    // 近场 lidar-only 发布：bag-1008 实测 YOLO 全程零检出(conf 0.10)、无类别差值框
    // 关联下限 15m，导致 10m 内无语义稳定行人轨迹零发布。此开关让近场 LiDAR 稳定
    // 且有真实运动证据的轨迹绕过语义租约发布；不采用“调低 10m 远场门槛”方案以控
    // 制近车误报。
    bool  orsm_lidar_only_near_output_enabled{false}; // 近场(≤max_distance)无语义稳定LiDAR轨迹带运动证据直接发布
    float orsm_lidar_only_near_output_max_distance_m{10.0f}; // 近场发布距离上限，与 lidar_only_far_output_min_distance_m 构成连续覆盖带
    int   orsm_lidar_only_output_min_observations{8}; // lidar-only 发布最少累计观测(约1s@10Hz)，抑制单帧噪声
    float orsm_lidar_only_output_min_motion_m{0.6f};  // 运动窗口内相对最新观测的净位移下限，排除静止背景/停放车辆
    int   orsm_lidar_only_output_motion_window_ms{4000}; // 净位移统计窗口
    float orsm_lidar_only_output_max_speed_mps{3.0f}; // 平滑速度上限，排除机动车流与点云跳变
    int   orsm_near_static_visual_refresh_ms{3000}; // 近车静态目标需在该窗口内有视觉语义复核
    float orsm_far_dynamic_min_distance_m{10.0f}; // 远距动态/图像运动目标输出距离阈值
    bool  orsm_far_image_motion_required{true}; // 远距unknown目标必须有图像运动证据才输出
    int   orsm_track_stable_min_age_ms{500};
    int   orsm_track_stable_min_observations{4};
    int   orsm_track_stable_lidar_min_observations{3};
    float orsm_track_stable_max_uncertainty_m{1.20f};
    int   orsm_track_lost_timeout_ms{2200};
    int   orsm_track_prune_timeout_ms{3200};
    bool  orsm_visual_lidar_ground_gate_enabled{true}; // 视觉地面位置约束点云关联，阻止人体框吸附远处背景簇
    float orsm_visual_lidar_ground_gate_min_m{1.5f};
    float orsm_visual_lidar_ground_gate_range_ratio{0.35f};
    // 视觉 ROI 只作为真实 LiDAR 簇的关联门，不会从 ROI 内的点单独生成目标。
    bool  orsm_lidar_visual_roi_enabled{true};
    // 历史锚点只用于接纳/延续当前帧已经聚出的真实稀疏簇；不贴类别、不授予租约、
    // 不创建没有 LiDAR 测量的目标。0 表示关闭历史锚点。
    bool  orsm_lidar_history_anchor_enabled{true};
    int   orsm_lidar_history_anchor_max_age_ms{3000};
    float orsm_lidar_history_anchor_gate_m{1.8f};
    int   orsm_lidar_primary_static_semantic_release_ms{2000};
    float orsm_lidar_primary_static_semantic_max_displacement_m{0.20f};
    float orsm_lidar_primary_voxel_leaf_m{0.30f};
    float orsm_lidar_primary_cluster_tolerance_m{0.65f};
    int   orsm_lidar_primary_min_cluster_points{12};
    float orsm_lidar_primary_mid_min_distance_m{15.0f};
    int   orsm_lidar_primary_mid_min_cluster_points{7};
    float orsm_lidar_primary_far_min_distance_m{18.5f};
    int   orsm_lidar_primary_far_min_cluster_points{3};
    float orsm_lidar_primary_far_voxel_leaf_m{0.30f};
    float orsm_lidar_primary_far_cluster_tolerance_m{0.65f};
    int   orsm_lidar_primary_anchor_min_cluster_points{3}; // 补盲雷达可信生物轨迹预测邻域稀疏簇下限
    float orsm_lidar_primary_anchor_max_radius_m{1.5f}; // 稀疏簇最大三维半径，避免吸收墙面/地面
    float orsm_lidar_primary_anchor_min_height_m{0.15f}; // 稀疏簇最小垂直跨度
    // 【侧向稀疏聚类放宽】行人横穿车头走出前向主雷达 FOV 后，落入补盲雷达覆盖的
    // 侧向远距区域，每帧可能只剩 1~2 个回波点，被 anchor_min_points=3 硬门直接
    // 拒绝导致 L# 断档（前左30m bag +31.4~+39.9s 实测 8.3s 零聚类）。开启后，
    // 侧向(|y|≥blindside_min_lateral_ratio×距离)且远于 far_min_distance_m 的簇，
    // 点数达到 blindside_min_cluster_points 即可通过两道点数门；仍受紧凑半径与
    // 高度约束，且发布层 11 道门控不变。默认开启，可通过配置关闭。
    bool  orsm_lidar_primary_blindside_sparse_enabled{true};
    int   orsm_lidar_primary_blindside_min_cluster_points{2};  // 侧向稀疏簇点数下限
    float orsm_lidar_primary_blindside_max_radius_m{1.0f};     // 侧向稀疏簇最大三维半径
    float orsm_lidar_primary_blindside_min_lateral_ratio{0.8f};// 判定侧向区域的 |y|/距离 比例
    bool  orsm_lidar_primary_static_background_filter_enabled{true};
    int   orsm_lidar_primary_static_background_min_hits{5};
    float orsm_lidar_primary_static_background_max_displacement_m{0.50f};
    float orsm_lidar_primary_static_background_max_speed_mps{0.35f};
    // 差值框局部放大二次检测（crop-YOLO）：全图 YOLO 对 10~20m 侧向小目标
    // 检不出（bag-1008 实测全程零检出）时，把稳定差值框周边区域裁剪放大后
    // 再送一次检测器，命中生物类则作为 YOLO 语义观测注入 ORSM 管线，走既有
    // 20s 语义租约通道。仅堡垒 ORSM 管线使用，不影响哨兵模式。
    bool  orsm_crop_yolo_enabled{false};          // 默认关闭；开启后差值框获得语义支持
    int   orsm_crop_yolo_min_streak{2};           // 差值框 streak 达到该值才做二次检测，抑制噪声框
    float orsm_crop_yolo_pad_ratio{0.60f};        // 裁剪区域四周各外扩差值框宽高的该比例
    float orsm_crop_yolo_min_confidence{0.30f};   // 二次检测置信度下限；放大图纹理更清晰，用高于全图的门槛抑制误检
    int   orsm_crop_yolo_every_n_frames{2};       // 每相机每 N 帧最多做一次二次检测，限制推理开销
    int   orsm_crop_yolo_max_boxes{1};            // 单帧最多处理的差值框数量
    // crop 语义距离上限：差值框估距超过该值时不做二次检测（0=不设限）。
    // 远场静止物体（停车簇/植被晃动）的裁剪放大误检为 person/animal 会经语义
    // 租约发布假目标（v4 实测 21.4m person / 23.9m animal，speed≤0.19）。
    float orsm_crop_yolo_max_distance_m{0.f};
    // 差值框关联前的静止簇语义锁定：仅依据附近 L# 的命中次数、历史位移
    // 以及其与当前帧簇中心的距离，不使用速度/语义/时效附加条件。
    bool  orsm_static_cluster_semantic_lockout_enabled{true};
    int   orsm_static_cluster_semantic_lockout_min_hits{5};
    float orsm_static_cluster_semantic_lockout_max_displacement_m{0.50f};
    float orsm_lidar_primary_min_z_m{0.15f};
    float orsm_lidar_primary_max_z_m{2.5f};
    // 临时前雷达固定地面补偿：ground_z = a*x + b*y + c。
    // 仅改变聚类前的高度过滤，不修改原始点坐标。
    bool  orsm_lidar_primary_front_fixed_ground_plane_enabled{false};
    float orsm_lidar_primary_front_fixed_ground_plane_a{0.00529f};
    float orsm_lidar_primary_front_fixed_ground_plane_b{0.01740f};
    float orsm_lidar_primary_front_fixed_ground_plane_c{0.00473f};
    float orsm_lidar_primary_front_fixed_ground_plane_min_height_m{0.30f};
    float orsm_lidar_primary_source_z_split_distance_m{25.0f};
    float orsm_lidar_primary_side_rear_min_z_m{-0.15f};
    float orsm_lidar_primary_left_min_z_m{-0.20f};
    float orsm_lidar_primary_semantic_gate_m{1.5f};
    bool  orsm_multi_vehicle_enable_network_check{true};
    float orsm_multi_vehicle_speed_entry_max_kmh{10.0f};
    float orsm_multi_vehicle_speed_exit_max_kmh{20.0f};
    float orsm_multi_vehicle_soc_entry_min{30.0f};
    float orsm_multi_vehicle_soc_exit_min{20.0f};
    int   orsm_multi_vehicle_stationary_required_sec{60};
    int   orsm_multi_vehicle_security_side{1};
    float orsm_multi_vehicle_external_radius{30.0f};
    float orsm_multi_vehicle_internal_radius{15.0f};
    float orsm_multi_vehicle_no_alarm_radius{5.0f};

    // 全局模式互斥：单车/多车堡垒模式优先于哨兵模式。
    // 当前设计范围仅包含哨兵、单车堡垒、多车堡垒三种模式。

    // RSCL publish 开关（上车调测时设为 true）
    bool enable_rscl_publish{false};
    // 状态输出模式：event_driven=仅状态变化/请求响应/告警变化发布；
    // fixed_10hz=独立线程固定10Hz发布；per_frame=每处理一帧图像发布一次，时间戳使用图像帧时间戳。
    std::string status_publish_mode{"event_driven"};
    bool enable_algorithm_time_log{true};
    int algorithm_time_log_interval_frames{50};

    // 摄像头 / 解码（鱼眼 FOV195 原始分辨率 1920x1536）
    // encoded_hevc: CameraEncodedStruct 编码流 + NVDEC；cuda_frame: 车端 Memblock raw BGR/NV12。
    std::string camera_input_mode{"encoded_hevc"};
    std::string camera_decoder_backend{"ffmpeg_nvdec"};
    int camera_width{1920};
    int camera_height{1536};
    int gpu_id{0};
    bool camera_decode_debug_enabled{false};
    std::string camera_decode_debug_dir{"./nvmedia_decode_debug"};
    int camera_decode_debug_max_frames_per_camera{5};
    bool camera_decode_debug_save_hevc{true};
    bool camera_decode_debug_save_decoded_bgr{true};
    bool camera_decode_debug_save_node_images{true};

    // 多路相机时间同步：4 路相机必须按时间戳同步成组处理；同步不齐时丢弃该组并记录缺失路，不能单路降级作为正常感知结果。
    bool enable_camera_sync{true};
    int  camera_sync_tolerance_ms{80};   // offset 校正后同一组多路帧允许的最大时间差
    int  camera_sync_timeout_ms{1000};    // front 基准帧等待其它相机的最大时间
    int  camera_sync_max_buffered_frames{30};
    std::vector<int> camera_sync_offsets_ms{0, 0, 0, 0}; // 顺序对应 camera_names；校正时间戳=原始时间戳+offset
    bool camera_sync_enable_hold{true};    // 短时缺帧时可使用上一帧补齐同步组，仅用于维持目标
    int  camera_sync_hold_ms{300};
    bool camera_sync_enable_degrade{true}; // 长时缺一路时允许降级为剩余相机组
    int  camera_sync_degrade_ms{2500};
    // 右后原始约20Hz；入感知同步队列前以前视相位对齐到约10Hz。
    // 告警录像在VideoRecorder内按event_record_fps独立抽帧，不受此路径影响。
    std::vector<std::string> camera_sync_half_rate_cameras{
        "rear_camera_fov195",
        "right_camera_fov195",
    };

    // 在线感知按固定频率消费最新同步组；多余同步组直接丢弃，避免旧帧排队造成高延迟。
    // 0 表示不节流。哨兵默认 10Hz，ORSM 默认 5Hz。
    int sentry_process_interval_ms{100};
    int orsm_process_interval_ms{50};

    // 多帧视觉目标稳定策略：减少单帧误检，同时用短时轨迹保留降低漏检。
    int   visual_track_confirm_min_age{3};      // 正常 track 至少连续确认 N 帧后参与报警判断
    int   visual_track_max_lost_frames{2};      // 已确认 track 短时漏检时最多保留 N 帧预测结果
    float visual_track_min_confidence{0.10f};   // 低置信稳定目标过滤阈值
    float visual_tracker_iou_threshold{0.3f};
    float visual_tracker_high_conf_threshold{0.5f};
    float visual_tracker_low_conf_threshold{0.1f};
    float visual_tracker_new_track_threshold{0.5f};
    float visual_tracker_low_match_iou_threshold{0.2f};
    int   visual_fallback_min_streak{3};        // 负 track detection fallback 连续 N 帧后才参与哨兵判断
    bool  sentry_near_field_motion_enabled{true}; // 检测框缺失时，用车身近场 ROI 图像变化作为接触型证据
    float sentry_near_field_motion_min_changed_ratio{0.05f};
    float sentry_near_field_motion_min_mean_delta{7.0f};
    int   sentry_near_field_motion_min_streak{2};
    bool  sentry_passing_motion_enabled{true}; // 检测框缺失时，用道路 ROI 连续运动作为路过目标证据
    float sentry_passing_motion_min_changed_ratio{0.003f};
    float sentry_passing_motion_min_mean_delta{0.8f};
    float sentry_passing_motion_min_component_ratio{0.0004f};
    int   sentry_passing_motion_min_streak{2};
    int   orsm_visual_confirm_min_age{3};
    float orsm_lidar_primary_fallback_semantic_gate_m{4.0f};
    float orsm_lidar_primary_fallback_min_confidence{0.6f};
    bool  orsm_debug_track{false};

    // IMU 阈值（对齐设计文档表14/16，区分一般/严重威胁）
    float imu_accel_threshold{2.0f};         // 一般威胁加速度阈值
    float imu_accel_severe_threshold{5.0f};  // 严重威胁加速度阈值
    float imu_gyro_threshold{0.5f};          // 一般威胁角速度阈值
    float imu_gyro_severe_threshold{2.0f};   // 严重威胁角速度阈值
    float imu_handle_freq_min{10.0f};        // 门把手震动最低频率
    float imu_handle_freq_max{50.0f};        // 门把手震动最高频率
    float imu_highpass_cutoff_hz{2.0f};      // 去除重力/姿态变化的高通截止频率
    int   imu_rms_window_ms{500};            // 振动 RMS 滑窗
    int   imu_vibration_min_duration_ms{300};
    int   imu_vibration_latch_ms{3000};
    float imu_accel_rms_threshold{0.14f};
    float imu_gyro_rms_threshold{0.0015f};
    float imu_accel_severe_rms_threshold{0.20f};
    float imu_gyro_severe_rms_threshold{0.0040f};
    float imu_accel_guard_rms{0.04f};

    // 检测器
    std::string engine_path{"yolov8n.engine"};
    std::string detector_model_type{"rfdetr"}; // rfdetr / yolo
    float       conf_threshold{0.5f};
    float       nms_threshold{0.45f};

    // 可视化 / 保存
    bool        enable_display{true};
    bool        enable_save{true};
    int         save_interval{10};
    std::string save_dir{"./output"};

    // 车辆状态 ODD 默认值:当 /canbus/vehicle_report 字段暂未解析或消息超时时使用。
    float vehicle_report_timeout_sec{2.0f};
    float soc_min_sentry{15.0f};
    float soc_min_orsm{30.0f};
    float soc_exit_orsm{20.0f};
    float orsm_speed_exit_threshold_kmh{20.0f};
    float speed_zero_threshold_kmh{0.1f};
    // ORSM 专用电源覆盖；与 Sentinel 的 power_off/override_power_off 独立。
    bool override_orsm_power_on{false};
    bool default_orsm_power_on{true};
    bool override_power_ok{false};
    bool default_power_ok{true};
    bool override_speed_zero{false};
    bool default_speed_zero{true};
    bool override_gear_park{false};
    bool default_gear_park{true};
    bool override_handbrake_on{false};
    bool default_handbrake_on{true};
    bool override_power_off{false};
    bool default_power_off{true};
    bool override_no_sensor_fault{false};
    bool default_no_sensor_fault{true};
    bool override_no_occupant{false};
    bool default_no_occupant{true};
    bool override_vehicle_locked{false};
    bool default_vehicle_locked{true};
    bool override_temp_ok{false};
    bool default_temp_ok{true};
    bool override_key_far{false};
    bool default_key_far{false};
    bool override_storage_ok{false};
    bool default_storage_ok{true};
    bool user_enabled{false};
    bool user_shutdown_requested{false};
    float odd_temp_min_c{-40.0f};
    float odd_temp_max_c{85.0f};
    float odd_light_min_lux{70.0f};
    uint64_t min_temp_storage_bytes{5ULL * 1024 * 1024 * 1024};
    uint64_t min_persistent_storage_bytes{20ULL * 1024 * 1024 * 1024};
    std::string persistent_storage_dir{"/mnt/sentry"};

    // 告警录像:按相机缓存原始鱼眼帧，触发后输出每路 pre/post 窗口 MP4。
    bool enable_event_recording{true};
    int  event_pre_seconds{30};
    int  event_post_seconds{30};
    int  event_record_fps{10};
    std::string event_record_dir{"./events"};
    std::string event_log_path{"./events/sentry_events.jsonl"};
    uint64_t event_record_max_bytes{20ULL * 1024 * 1024 * 1024}; // 0 表示关闭录像目录自动清理
    uint64_t event_log_max_bytes{50ULL * 1024 * 1024};           // 0 表示关闭日志轮转
    int sentry_state_tick_ms{100}; // 独立状态机 tick，避免只由摄像头帧驱动
};

/**
 * @brief 哨兵/越野堡垒感知主节点
 *
 * 数据流:
 *   哨兵模式：Camera/Video + IMU + USS → AlertManager（禁止使用 LiDAR）
 *   堡垒模式：Camera/Video + LiDAR → ORSM 目标位置/速度融合；IMU 仅作辅助事件/状态输入
 *                                                        │
 *                                                   SentryStateMachine
 *                                                        │
 *                                              AlertEvent → IVI/TBOX (SOME/IP)
 */
class SentinelNode {
public:
    explicit SentinelNode(const SentinelConfig& cfg = {});

    bool start(int argc, char* argv[]);
    void stop();
    ~SentinelNode() { stop(); }

    uint64_t frameCount()     const { return frame_count_.load(); }
    uint64_t alertCount()     const { return alert_manager_ ? alert_manager_->alertCount() : 0; }
    uint64_t vibrationCount() const { return imu_ ? imu_->vibrationCount() : 0; }
    SentryState currentState() const { return state_machine_ ? state_machine_->currentState()
                                                              : SentryState::UNAVAILABLE; }

private:
    struct VehicleOddSnapshot {
        std::optional<float> speed_kmh;
        std::optional<float> soc_percent;
        std::optional<bool> gear_park;
        std::optional<bool> handbrake_on;
        std::optional<bool> power_off;
        std::optional<bool> no_sensor_fault;
        std::optional<bool> no_occupant;
        std::optional<bool> key_far;
        std::optional<bool> vehicle_locked;
        std::optional<bool> non_ota_mode;
        std::optional<bool> user_enabled;
        std::optional<bool> user_shutdown_requested;
        std::optional<uint8_t> scheduled_start_day;
        std::optional<uint32_t> scheduled_start_hour;
        std::optional<uint32_t> scheduled_start_minute;
        std::optional<long long> scheduled_request_time_s;
        bool scheduled_start_pending{false};
        uint64_t soc_timestamp_ns{0};
        uint64_t timestamp_ns{0};
        uint64_t chassis_timestamp_ns{0};
        uint64_t sequence{0};
    };

    struct SentryRequestControl {
        bool immediate_enable_pending_once{false};
        bool automatic_enable_enabled{false};
        bool automatic_enable_pending_once{false};
        bool scheduled_enable_pending{false};
        std::optional<uint8_t> scheduled_start_day;
        std::optional<uint32_t> scheduled_start_hour;
        std::optional<uint32_t> scheduled_start_minute;
        std::optional<long long> scheduled_request_time_s;
        bool sentry_enabled_by_request{false};
        bool sentry_enabled_by_automatic_request{false};
        bool sentry_enabled_by_scheduled_request{false};
        bool last_power_off_locked{false};
    };

    void onFrame(const DecodedFrame& frame);
    void enqueueFrameForProcessing(const DecodedFrame& frame);
    void enqueueFrameGroupForProcessing(const std::vector<DecodedFrame>& group);
    bool shouldProcessFrameGroup(const std::vector<DecodedFrame>& group);
    uint64_t frameGroupSyncTimestampNs(const std::vector<DecodedFrame>& group) const;
    void frameProcessingLoop();
    void processFrame(const DecodedFrame& frame);
    std::optional<TrackedObject> updateNearFieldMotionCandidate(const DecodedFrame& frame,
                                                                const cv::Mat& image);
    std::optional<TrackedObject> updatePassingMotionCandidate(const DecodedFrame& frame,
                                                              const cv::Mat& image);
    std::optional<TrackedObject> updateOrsmFarImageMotionCandidate(const DecodedFrame& frame,
                                                                   const cv::Mat& image);
    // 差值框局部放大二次检测（crop-YOLO）：把稳定差值框周边区域裁剪后送检测器，
    // 命中生物类则生成 OriginYolo 语义观测加入 ORSM 管线，走既有 20s 语义租约通道。
    void runOrsmCropYoloSemantic(const DecodedFrame& frame,
                                 const cv::Mat& rectified,
                                 const TrackedObject& diff_box,
                                 const FisheyeCalib* calib,
                                 const std::vector<TrackedObject>& yolo_entries,
                                 std::vector<TrackedObject>& out);
    void saveCameraDecodeDebugImage(const char* stage, const DecodedFrame& frame, const cv::Mat& image);
    std::vector<DecodedFrame> pushFrameAndCollectSynchronized(const DecodedFrame& frame);
    std::string cameraSyncKey(const DecodedFrame& frame) const;
    int64_t cameraSyncOffsetNs(const std::string& camera_key) const;
    bool isCameraSyncHalfRate(const std::string& camera_key) const;
    void onAlert(const AlertEvent& evt);
    int stableThreatIdForStateMachine(ThreatLevel level, int raw_threat_id, uint64_t timestamp_ns);
    void publishSentryOutputs(uint64_t event_id,
                              const AlertEvent& evt,
                              const std::vector<std::string>& video_paths,
                              bool publish_action);
    void publishSentryRequestResponse(uint16_t req_value, uint64_t timestamp_ns,
                                      bool approve, const OddConditions& odd,
                                      std::optional<SentinelEnableFailed> forced_failed = std::nullopt);
    void publishSentryStatusTick(uint64_t output_timestamp_ns = 0);  // 状态发布；fixed_10hz 模式下由独立线程调用
    void publishSentryStateBroadcastTick(uint64_t output_timestamp_ns = 0);  // /sentry/state_broadcast ~10Hz
    void publishOrsmStatusTick(uint64_t output_timestamp_ns = 0);    // ORSM 状态发布；独立于哨兵 status_publish_mode，固定10Hz发送最近状态
    void publishFrameStatusOutputs(uint64_t frame_timestamp_ns);     // 兼容入口：仅发送哨兵状态，ORSM 独立固定10Hz发送
    void updateCachedSentryStatus();
    void logSentryModeLifecycle(const SentryStateEvent& evt) const;
    void logOrsmModeLifecycle(const OrsmStateEvent& evt) const;
    void logOrsmOddIfNeeded(const OrsmOdd& odd, OrsmState state, uint64_t timestamp_ns) const;
    void forceOrsmOffFeedback(uint64_t timestamp_ns, const char* reason);
    void refreshCachedOrsmSettingsFeedback(uint64_t timestamp_ns = 0);
    // 本地相机/雷达收数新鲜度；缺失时 OR 进 odd.sensor_fault（复用 0x7 文言）。
    bool localSensorDataMissing() const;
    void applyLocalSensorDataFault(OrsmOdd& odd) const;
    void statusPublishLoop();
    void orsmStatusPublishLoop();
    void stateBroadcastLoop();
    void pushFrameForEventRecording(const DecodedFrame& frame);
    std::string sentryRecordingReason(const AlertEvent& evt) const;
    int sentryManifestThreatLevel(ThreatLevel level) const;
    bool statusPublishFixed10Hz() const;
    bool statusPublishPerFrame() const;
    void handleRecordedVideoDone(uint64_t event_id, const std::vector<std::string>& paths);
    void stateTickLoop();
    std::vector<std::string> recordEventClip(const AlertEvent& evt);
    void rememberFrameForRecording(const DecodedFrame& frame, const cv::Mat& vis);
    bool checkStorageOk() const;
    void cleanupEventRecordingStorage(const std::vector<std::string>& protected_paths = {}) const;
    void rotateEventLogIfNeeded() const;
    void onVehicleReport(const std::shared_ptr<ReceivedMsg<senseAD::msg::vehicle::VehicleReport>>& msg);
#ifndef SENTINEL_DIAGNOSTIC_NO_L3
    void onChassisInfo(const std::shared_ptr<ReceivedMsg<senseAD::msg::l3::ChassisInfoM18V2>>& msg);
    void onSentryRequest(const std::shared_ptr<ReceivedMsg<senseAD::msg::l3::SentinelRequest>>& msg);
    void onOrsmSettingRequest(const std::shared_ptr<ReceivedMsg<senseAD::msg::l3::SetOffRoadMdInfoReq>>& msg);
    void onOrsmSetInfo(const std::shared_ptr<ReceivedMsg<senseAD::msg::l3::OffRoadSetInfoStruct>>& msg);
#endif
    void onUssFusedStaticMap(const std::shared_ptr<ReceivedMsg<senseAD::msg::avp_perception::UssFusedStaticMap>>& msg);

    bool fortressModeRequested() const;
    bool fortressModeActiveForProcessing() const;
    bool sentrySuppressedByHigherPriorityMode() const;
    bool shouldRunHeavyPerception(const OddConditions& odd) const;
    std::vector<TrackedObject> updateOrsmGlobalTracks(const std::vector<TrackedObject>& camera_tracks,
                                                       const LidarFrame& lidar,
                                                       const std::string& camera_name,
                                                       uint64_t timestamp_ns);
    std::vector<TrackedObject> buildLidarPrimaryOrsmCandidates(const LidarFrame& lidar,
                                                               const std::vector<TrackedObject>& visual_tracks,
                                                               uint64_t timestamp_ns) const;
    void associateOrsmLidarTemporalTracks(std::vector<TrackedObject>& candidates,
                                          uint64_t timestamp_ns);
    std::vector<TrackedObject> dedupeOrsmGroupCandidates(const std::vector<TrackedObject>& candidates,
                                                          const char* log_tag = "ORSMGroupDedupe") const;
    std::vector<TrackedObject> dedupeSentryGroupCandidates(const std::vector<TrackedObject>& candidates) const;
    void resetOrsmGlobalTracks();
    void clearOrsmStaticFootprints(const char* reason);

    OddConditions buildOdd() const;
    void evaluateSentryRequestTriggers(uint64_t timestamp_ns = 0);
    bool vehiclePowerOffLocked(const VehicleOddSnapshot& vehicle) const;
    static OddConditions oddIgnoringDeferredEntryConditions(OddConditions odd);
    std::optional<std::time_t> scheduledStartEpoch(uint8_t day, uint32_t hour, uint32_t minute,
                                                   long long request_time_s) const;
    bool scheduledStartIsFuture(uint8_t day, uint32_t hour, uint32_t minute,
                                long long request_time_s) const;
    bool scheduledStartDue(const VehicleOddSnapshot& vehicle) const;
    LidarFrame mergedLidarFrame(uint64_t target_timestamp_ns) const;
    void recordPerfMetric(const std::string& name, double value_ms) const;
    void logPerfSummaryIfNeeded(uint64_t processed_frames);

    void visualize(cv::Mat& image,
                   const DecodedFrame& frame,
                   const std::vector<TrackedObject>& tracked,
                   ThreatLevel level,
                   SentryState state);

    SentinelConfig cfg_;

    std::shared_ptr<ImuSubscriber>      imu_;
    std::shared_ptr<UssSubscriber>      uss_;
    std::shared_ptr<LidarSubscriber>    lidar_;   // 主 LiDAR（第一路，兼容旧代码）
    std::vector<std::shared_ptr<LidarSubscriber>> lidars_;  // 全部 LiDAR
    std::shared_ptr<CameraSubscriber>   camera_;  // 主相机（第一路）
    std::vector<std::shared_ptr<CameraSubscriber>> cameras_;  // 全部 4 路
    std::shared_ptr<FisheyeUndistortManager> fisheye_mgr_;
    // 仅用于 far image-motion Inbox 与 LiDAR 点投影的独立标定。
    std::shared_ptr<FisheyeUndistortManager> inbox_fisheye_mgr_;
    std::shared_ptr<SensorCalibManager>       sensor_calib_;
    UssVisionFusion                           uss_vision_fusion_;
    std::shared_ptr<VideoRecorder>        video_recorder_;
    std::shared_ptr<Detector>           detector_;
    // crop-YOLO 每相机节流计数（orsm_crop_yolo_every_n_frames）
    std::unordered_map<std::string, uint64_t> orsm_crop_yolo_frame_counter_;
    std::unordered_map<std::string, std::shared_ptr<Tracker>> trackers_by_camera_;
    std::unordered_map<std::string, int> detection_fallback_streak_;
    std::shared_ptr<AlertManager>       alert_manager_;
    std::shared_ptr<SentryStateMachine> state_machine_;
    std::shared_ptr<OrsmManager>        orsm_manager_;
    OrsmSettings                        orsm_settings_;
    mutable std::mutex                  orsm_settings_mutex_;
    uint64_t                            orsm_settings_generation_{0};
    mutable std::mutex                  latest_orsm_output_mutex_;
    ORSMAlertOutput                     latest_orsm_output_;
    uint64_t                            latest_orsm_output_ts_ns_{0};
    mutable std::mutex                  latest_sentry_status_mutex_;
    AlertEvent                          latest_sentry_status_event_;
    SentinelResStatusView               latest_sentry_status_res_;
    std::shared_ptr<RsclPublisher>      rscl_publisher_;

    mutable std::mutex processing_mutex_;
    std::mutex frame_processing_mutex_;
    std::condition_variable frame_processing_cv_;
    std::deque<DecodedFrame> frame_processing_queue_;
    std::thread frame_processing_thread_;
    std::atomic<bool> frame_processing_running_{false};
    static constexpr size_t kMaxFrameProcessingQueueSize = 32;

    struct PerfMetricStats {
        uint64_t count{0};
        uint64_t over_threshold_count{0};
        double sum_ms{0.0};
        double min_ms{std::numeric_limits<double>::max()};
        double max_ms{0.0};
        std::vector<double> samples_ms;
    };

    mutable std::mutex perf_mutex_;
    mutable std::unordered_map<std::string, PerfMetricStats> perf_metrics_;
    std::unordered_map<std::string, uint64_t> camera_last_timestamp_ns_;
    std::unordered_map<std::string, uint64_t> camera_sync_timeout_missing_counts_;
    std::unordered_map<std::string, uint64_t> frame_processing_drop_counts_;
    std::atomic<uint64_t> camera_sync_timeout_count_{0};
    std::atomic<uint64_t> camera_sync_group_ok_count_{0};
    std::atomic<uint64_t> camera_sync_buffer_overflow_count_{0};
    std::atomic<uint64_t> camera_sync_unmatchable_master_drop_count_{0};
    std::atomic<uint64_t> camera_sync_unmatchable_left_drop_count_{0};
    std::atomic<uint64_t> camera_sync_half_rate_drop_count_{0};
    std::atomic<uint64_t> frame_processing_overflow_count_{0};
    std::atomic<uint64_t> frame_interval_drop_count_{0};
    std::atomic<uint64_t> frame_interval_drop_group_count_{0};
    std::atomic<uint64_t> orsm_latest_group_replace_count_{0};
    std::atomic<uint64_t> orsm_latest_group_drop_frame_count_{0};
    std::atomic<uint64_t> sentry_latest_group_replace_count_{0};
    std::atomic<uint64_t> sentry_latest_group_drop_frame_count_{0};
    std::atomic<uint64_t> current_processing_group_id_{0};
    mutable std::atomic<uint64_t> lidar_frame_candidate_count_{0};
    mutable std::atomic<uint64_t> lidar_stale_skip_count_{0};
    mutable std::atomic<uint64_t> lidar_merge_output_count_{0};
    uint64_t last_perf_summary_frame_{0};
    uint64_t last_lidar_fusion_steady_ns_{0};
    std::unordered_map<std::string, uint64_t> camera_decode_debug_node_save_counts_;
    std::atomic<bool> detector_ready_{false};

    struct NearFieldMotionState {
        cv::Mat previous_gray;
        cv::Point2f previous_motion_center{0.f, 0.f};
        int streak{0};
        uint64_t last_timestamp_ns{0};
    };
    std::mutex near_field_motion_mutex_;
    std::unordered_map<std::string, NearFieldMotionState> near_field_motion_states_;

    struct PassingMotionState {
        cv::Mat previous_gray;
        cv::Point2f previous_motion_center{0.f, 0.f};
        int streak{0};
        uint64_t last_timestamp_ns{0};
    };
    std::mutex passing_motion_mutex_;
    std::unordered_map<std::string, PassingMotionState> passing_motion_states_;

    std::mutex orsm_far_image_motion_mutex_;
    std::unordered_map<std::string, PassingMotionState> orsm_far_image_motion_states_;

    struct PendingOrsmGroup {
        uint64_t group_id{0};
        uint64_t timestamp_ns{0};
        size_t expected_size{1};
        size_t processed_count{0};
        std::vector<TrackedObject> candidates;
        LidarFrame lidar;
    };
    std::unordered_map<uint64_t, PendingOrsmGroup> pending_orsm_groups_;

    struct PendingSentryGroup {
        uint64_t group_id{0};
        uint64_t timestamp_ns{0};
        size_t expected_size{1};
        size_t processed_count{0};
        std::vector<TrackedObject> internal_tracks;
        ImuData imu;
        UssData uss;
    };
    std::unordered_map<uint64_t, PendingSentryGroup> pending_sentry_groups_;

    enum class OrsmTrackLifecycle {
        Candidate,
        Stable,
        Lost,
    };

    struct OrsmGlobalTrack {
        int stable_id{0};
        OrsmTrackLifecycle lifecycle{OrsmTrackLifecycle::Candidate};
        std::string label;
        int class_id{-1};
        // Confidence at which the current class was established.  A different
        // real class may replace it only with a strictly higher confidence;
        // an unknown observation may never replace it at all.
        float class_confidence{0.f};
        cv::Rect bbox;
        float confidence{0.f};
        cv::Point3f position{0.f, 0.f, 0.f};
        cv::Point3f velocity{0.f, 0.f, 0.f};
        float position_uncertainty_m{0.60f};
        float distance_m{0.f};
        float speed_mps{0.f};
        bool has_lidar_recent{false};
        // 类别权威在 L#：全局轨迹的 label/class_id 由 L# 投影或出生观测确立，
        // 不再单独维护 semantic_confirmed/语义时间戳/视觉命中数那本账。
        // Back-pointer to the bound OrsmLidarTemporalTrack (L#).  The L# track
        // is the single evidence holder and motion authority; the global track
        // only projects it for publication.
        int lidar_temporal_id{-1};
        // 唯一的发布租约：本帧 YOLO 认领与差值框证据都延长此期限，取晚者胜。
        uint64_t lidar_publication_lease_until_ts_ns{0};
        // 10~15m 带内首次发布必须是"带类别的差值框"(label 非 unknown)。一旦
        // 该条件满足过，后续保活只依赖租约，不再重复要求差值框证据。
        bool class_established_publication{false};
        uint64_t last_lidar_ts_ns{0};
        int last_lidar_point_count{0};
        float last_lidar_cluster_radius_m{0.f};
        uint64_t first_seen_ts_ns{0};
        uint64_t last_seen_ts_ns{0};
        uint64_t stable_since_ts_ns{0};
        uint64_t last_camera_seen_ts_ns{0};
        int hits{0};
        int lidar_hits{0};
        int last_source_track_id{-1};
        std::string last_camera;
        std::deque<std::pair<uint64_t, cv::Point3f>> motion_history;
        uint64_t last_dynamic_motion_confirmed_ts_ns{0};
        bool has_lidar_tracking_anchor{false};
        cv::Point3f lidar_tracking_anchor{0.f, 0.f, 0.f};
        cv::Point3f lidar_stationary_anchor{0.f, 0.f, 0.f};
        uint64_t lidar_stationary_since_ts_ns{0};
    };
    std::unordered_map<int, OrsmGlobalTrack> orsm_global_tracks_;
    int next_orsm_global_track_id_{1};

    // The primary LiDAR clusterer is deliberately stateless per scan.  Keep a
    // separate, conservative temporal identity here so a real cluster which
    // receives video semantics does not lose that evidence merely because its
    // local cluster index changed on the next scan.
    struct OrsmLidarTemporalTrack {
        int stable_id{0};
        cv::Point3f position{0.f, 0.f, 0.f};
        cv::Point3f initial_position{0.f, 0.f, 0.f};
        cv::Point3f velocity{0.f, 0.f, 0.f};
        float max_displacement_m{0.f};
        // 仅用于前向垂直接近判定；这是 L# 自己的历史，不借用全局 ID 历史。
        std::deque<std::pair<uint64_t, cv::Point3f>> front_motion_history;
        // lidar-only 发布判定的运动窗口历史（每次真实 LiDAR 观测追加一条）。
        std::deque<std::pair<uint64_t, cv::Point3f>> output_motion_history;
        uint64_t first_seen_ts_ns{0};
        uint64_t last_seen_ts_ns{0};
        uint64_t last_semantic_ts_ns{0};
        uint64_t last_far_image_motion_ts_ns{0};
        // 唯一的发布租约：本帧 YOLO 认领与差值框证据都延长此期限，取晚者胜。
        uint64_t lidar_publication_lease_until_ts_ns{0};
        int hits{0};
        int semantic_hits{0};
        int far_image_motion_streak{0};
        int bound_global_track_id{0};
        int class_id{-1};
        std::string label{"unknown"};
        float confidence{0.f};
        // Confidence at which this temporal identity's class was established.
        // Same monotonic rule as OrsmGlobalTrack::class_confidence.
        float class_confidence{0.f};
    };
    std::unordered_map<int, OrsmLidarTemporalTrack> orsm_lidar_temporal_tracks_;
    int next_orsm_lidar_temporal_track_id_{1};

    // In-memory occupancy left behind by confirmed static unknown LiDAR tracks.
    // Unlike L# temporal identities, these footprints survive ordinary temporal
    // pruning and are cleared only when fortress mode exits/resets.
    struct OrsmStaticFootprint {
        int source_lidar_temporal_id{0};
        cv::Point2f center{0.f, 0.f};
        float radius_m{0.f};
        uint64_t confirmed_ts_ns{0};
        uint64_t last_seen_ts_ns{0};
    };
    mutable std::mutex orsm_static_footprints_mutex_;
    std::vector<OrsmStaticFootprint> orsm_static_footprints_;

    // Events are appended by the fusion worker and written verbatim into the
    // opt-in offline trace by the status publisher.  This makes ownership and
    // termination decisions auditable without changing the RSCL payload.
    mutable std::mutex orsm_binding_trace_mutex_;
    std::vector<std::string> pending_orsm_binding_trace_events_;

    mutable std::mutex frame_process_interval_mutex_;
    uint64_t last_processed_group_sync_ts_ns_{0};

    struct CameraSyncFrame {
        DecodedFrame frame;
        uint64_t arrival_steady_ns{0};
        int64_t sync_timestamp_ns{0};  // 原始 timestamp + 每路 camera offset，用于 4 路同步匹配
    };

    mutable std::mutex camera_sync_mutex_;
    std::map<std::string, std::deque<CameraSyncFrame>> camera_sync_buffers_;
    std::unordered_map<std::string, CameraSyncFrame> camera_sync_last_frames_;
    struct HalfRateKeep {
        uint64_t slot{0};
        uint64_t frame_ts{0};
        uint64_t front_anchor_ts{0};
        uint64_t frame_seq{0};
    };
    std::unordered_map<std::string, HalfRateKeep> camera_sync_half_rate_kept_;
    uint64_t camera_sync_latest_ts_ns_{0};

    mutable std::mutex vehicle_mutex_;
    VehicleOddSnapshot latest_vehicle_;
    OddConditions latest_odd_;
    mutable std::mutex sentry_request_mutex_;
    SentryRequestControl sentry_request_control_;

    struct FrameRecord {
        uint64_t seq{0};
        uint64_t timestamp_ns{0};
        std::string path;
    };
    mutable std::mutex record_mutex_;
    std::deque<FrameRecord> recent_frames_;
    uint64_t next_event_id_{1};

    // 设计文档中的“同一威胁”是一个持续窗口，而不是视觉 tracker 的瞬时 track_id。
    // 这里维护稳定 ThreatSession，用于吸收 track_id 抖动，并让外部告警/录像按
    // ACTIVE→WARNING、WARNING→ALARM 等状态转换发布，而不是每次 track_id 切换都发布。
    struct ThreatSession {
        bool active{false};
        int session_id{0};
        int last_raw_threat_id{0};
        ThreatLevel max_level{ThreatLevel::NONE};
        SentryState first_state{SentryState::ACTIVE};
        uint64_t first_seen_ns{0};
        uint64_t last_seen_ns{0};
        bool warning_event_published{false};
        bool alarm_event_published{false};
        bool video_triggered{false};
    };

    mutable std::mutex threat_session_mutex_;
    ThreatSession active_threat_session_;
    int next_threat_session_id_{1};
    uint64_t last_video_record_trigger_ns_{0};
    uint64_t next_recording_group_id_{1};

    mutable std::mutex pending_event_mutex_;
    std::unordered_map<uint64_t, AlertEvent> pending_record_events_;
    std::unordered_map<uint64_t, uint64_t> pending_record_trigger_wall_ns_;

    std::thread state_tick_thread_;
    std::atomic<bool> state_tick_running_{false};
    std::thread status_publish_thread_;
    std::atomic<bool> status_publish_running_{false};
    std::thread orsm_status_publish_thread_;
    std::atomic<bool> orsm_status_publish_running_{false};
    std::thread state_broadcast_thread_;
    std::atomic<bool> state_broadcast_running_{false};

    std::atomic<uint64_t> frame_count_{0};
    std::atomic<uint64_t> last_sensor_time_ns_{0};
    std::atomic<uint64_t> camera_last_receive_steady_ns_{0};
    std::atomic<uint64_t> lidar_last_receive_steady_ns_{0};
    std::atomic<bool>     started_{false};
};

}  // namespace sentinel
