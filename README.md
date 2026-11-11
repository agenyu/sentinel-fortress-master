# sentinel-fortress

> 哨兵模式 + 越野堡垒模式 — 车载多传感器融合安防感知框架

基于 SenseAuto RSCL 消息总线，融合摄像头、激光雷达、超声波、IMU 多路传感器数据，实现车辆停驻后的智能安防感知。

---

## 功能概述

### 哨兵模式（Sentry Mode）
来源：M18-3先导车哨兵模式功能详细说明书V1.3

驾驶员离车落锁后持续检测车辆周围情况，若检测到可疑事件，根据威胁严重程度进行警示、录制视频。

- 5状态机：不可用 → 待机 → 激活 → 警戒 → 报警
- 一般威胁（物体速度≥0.1m/s且距离<3m）→ 中控屏警戒提示
- 目标距离/速度：按目标类别名义高度 + 鱼眼等效焦距估距，并基于 track_id 时间序列估算 m/s 速度，替代原 bbox 比例/像素速度阈值
- 4 路环视相机同步：以 `front_camera_fov195` 为基准，支持 per-camera offset 校正、短时帧保持和超时降级；是否允许降级由配置控制
- 严重威胁（车辆震动/碰撞）→ 双闪灯 + 喇叭 + 中控屏报警
- 威胁解除（速度≤0.1m/s或距离>10m，持续>10s）→ 回到激活
- 同一威胁超时（>3min）→ 按 track_id/传感器事件ID 独立计时，停止对应持续报警

### 越野堡垒模式（Off-Road Security Mode / ORSM）
来源：M18-3先导车越野堡垒功能详细说明书V1.2

户外无人区停车休息时，监控车辆周边区域，检测异常入侵物分级报警。支持单车安防和多车联防。

- 4状态机：关闭 → 激活 → 报警 → 故障
- 单车堡垒：A区(30m)/B区(15m)/C区(5m)，对应I/II/III级报警
- 多车堡垒：按组队状态、5G网络、IVI prepared、外部锁车、静止60s、SOC/车速等 ODD 进入；监控安防侧外部区/内部区，5m 内不报警
- 入侵物分类：人/动物/未知，可通过用户设置过滤；多车外部区按设计方向不强行区分人/动物
- 报警方式可配置：灯光/鸣笛/声音类型/持续时长

### 三模式互斥关系

当前设计范围仅包含三种模式：**哨兵模式、单车堡垒模式、多车堡垒模式**。

- 单车/多车堡垒模式优先于哨兵模式；任一堡垒模式被请求时，哨兵模式不再激活。
- 单车堡垒与多车堡垒由 `OrsmManager` 根据 `orsmSettingSt`、TBOX/云端组队状态和多车 ODD 路由。
- 多车联防激活后若队伍解散/不可用，会在单车 ODD 满足时回落到单车堡垒。

---

## 系统架构

> 当前 master 分支定位为 **单节点/单进程交付版本**：只启动 `sentinel_perception` 一个可执行程序。
> `development` 分支中的多节点 RSCL 工程可作为 topic、Cap'n Proto 消息和验证流程参考，但不直接改变 master 的单节点部署形态。

```
IMU  /sensor/imu ─────────────────────────────────────────────┐
USS  /parking/pdc/pdc_info ───────────────────────────────────┤
USS  /parking/meb/meb_info ───────────────────────────────────┤
USS  /parking/uss/fused_points（兼容 /sensor/uss/fused_points） ───────────────┤
LiDAR /sensor/lidar/{front,left,rear,right}_lidar ──────────────┤
                                                               ▼
Camera /sensor/camera/... → encoded_hevc(NVDEC) 或 cuda_frame(Memblock raw) → DecodedFrame → Detector → Tracker
                                                               │
                                                    ┌──────────┴──────────┐
                                                    ▼                     ▼
                                             AlertManager          OrsmManager
                                             (哨兵模式)           (越野堡垒)
                                                    │                     │
                                          SentryStateMachine    OrsmStateMachine
                                                    │                     │
                                      事件/动作 JSONL + RSCL publish
                                      (enable_rscl_publish 控制)  ORSM 状态/动作 RSCL publish
```

---

## 目录结构

```
sentinel_perception/
├── main.cpp                      # 入口
├── sentinel_node.h/cpp           # 主节点，组装所有子模块
├── CMakeLists.txt
├── build/
│   └── config.yaml               # 运行时配置
├── publish/                      # RSCL publish 封装（Sentry/ORSM 官方 l3_msgs）
├── docs/
│   └── development-single-node-port.md # development → master 单节点借鉴说明
├── scripts/
│   ├── run_single_node.sh        # 单节点启动脚本
│   └── stop_single_node.sh       # 单节点停止脚本
│
├── imu/
│   └── imu_subscriber.h/cpp      # IMU 振动检测
│
├── uss/
│   └── uss_subscriber.h/cpp      # 超声波 USS（PDC + MEB）
│
├── lidar/
│   └── lidar_subscriber.h/cpp    # 激光雷达点云
│
├── camera/
│   ├── camera_subscriber.h/cpp   # 多路摄像头输入：HEVC+NVDEC 或车端 CUDA Memblock raw BGR/NV12（每路独立线程）
│   └── fisheye_undistort.h/cpp   # 鱼眼去畸变（cv::fisheye remap，预计算查找表）
│
├── calib/
│   └── sensor_calib_manager.h/cpp  # 传感器标定参数统一加载器（LiDAR/IMU/车辆）
│
├── detection/
│   ├── detector.h/cpp            # RF-DETR Base TensorRT（输出 Detection 接口保持稳定）
│   └── tracker.h/cpp             # ByteTrack 多目标跟踪（Kalman + 二阶段匹配）
│
├── alert/
│   └── alert_manager.h/cpp       # 哨兵模式：多传感器融合威胁分级
│
├── state/
│   ├── sentry_state_machine.h/cpp # 哨兵模式：5状态机 + M1-M8切换
│   └── sentry_mode_contract.h    # 对齐 SentryMode.proto 的内部枚举/响应映射
│
├── orsm/
│   ├── orsm_state_machine.h/cpp       # 越野堡垒：4状态机 + T1-T6切换
│   ├── orsm_manager.h/cpp             # 越野堡垒：单车/多车模式路由 facade
│   ├── single_orsm_manager.h/cpp      # 单车堡垒：A/B/C 三区报警 + 入侵物分类
│   ├── multi_vehicle_orsm_manager.h/cpp # 多车堡垒：组队ODD + 安防侧外/内部区 + 5m不报警区
│   ├── orsm_output.h                  # 对齐 OrsmInfo.proto 的 OrsmStatus 输出结构
│   └── orsm_settings.h                # 对齐 OrsmInfo.proto 的 SetOffRoadMdInfoReq 输入结构
│
├── param/
│   └── calib/                    # 相机标定参数（内参 + 外参）
│       ├── front_camera_fov195/  # 前鱼眼
│       ├── rear_camera_fov195/   # 后鱼眼
│       ├── left_camera_fov195/   # 左鱼眼
│       ├── right_camera_fov195/  # 右鱼眼
│       ├── front_lidar/          # 前向激光雷达 (ATX)
│       ├── left_lidar/           # 左侧激光雷达
│       ├── rear_lidar/           # 后向激光雷达
│       ├── right_lidar/          # 右侧激光雷达
│       ├── imu/                  # IMU 标定（bias/scale）+ 里程计 + 双天线
│       └── vehicle/              # 车辆参数（尺寸/轴距）
│
├── proto/
│   ├── OrsmInfo.proto            # ORSM 正式接口参考
│   └── SentryMode.proto          # 哨兵模式正式接口参考
│
├── nv_hevc_decoder.h/cpp         # NVIDIA NVDEC 硬件解码器
└── rscl_access/
    └── rscl_access.h/cpp         # RSCL 订阅/发布访问层
```

---

## 哨兵模式状态机

来源：功能详细说明书V1.3 表格7/8

```
                    M1+M2                M3
  UNAVAILABLE ──────────→ STANDBY ──────────→ ACTIVE
       ↑                    ↑                  │  ↑
       │                    │            M5    │  │  M6(>10s)
       │                    │           ┌──────↓  │──────┐
       │                    │           │ WARNING  │      │
       │                    │           │    │     │      │
       │                    │           │ M7 ↓     │      │
       │                    │           │  ALARM ──┘      │
       │                    │           │    │  M8(>3min)  │
       │                    │           └────┴────────────┘
       └────────────────────┴──── M4（用户关闭/ODD失效）
```

| 状态 | 说明 | 传感器 | 中控屏 | 声光 |
|------|------|--------|--------|------|
| UNAVAILABLE | 自检/ODD不通过 | — | 提示不可用 | — |
| STANDBY | 等待钥匙>10m | — | 提示准备激活 | — |
| ACTIVE | 正常监测 | 全部开启 | 无提示 | — |
| WARNING | 一般威胁 | 全部开启 | 点亮，显示"警戒状态" | — |
| ALARM | 严重威胁 | 全部开启 | 闪烁，显示"报警状态" | 双闪+喇叭 |

### 哨兵模式威胁分级

| 等级 | 触发条件 | CAN输出信号 |
|------|----------|-------------|
| HIGH（严重） | 整车/门把手震动、碰撞等明确侵犯本车事件 | HAD_HornReq=1, HAD_HzrdLampReq=2 |
| MEDIUM（一般） | 移动目标接近车辆约3m内（按类别名义高度+等效焦距估距，按 track_id 时间序列估速） | 中控屏警戒 + HAD_LightReq |
| LOW | 监测范围内出现目标，持续观察 | — |

### 哨兵模式 ODD 条件

| 条件 | 要求 | 配置项 / 回退项 |
|------|------|----------------|
| 能源 | SOC ≥ 配置阈值 | `soc_min_sentry`，默认 15% |
| 车速 | 近似 0 km/h | `speed_zero_threshold_kmh`，默认 0.1 km/h；缺信号回退 `default_speed_zero` |
| 档位 | P档 | 缺信号回退 `default_gear_park` |
| 手刹 | 拉起 | 缺信号回退 `default_handbrake_on` |
| 钥匙 | 离车 > 10m | 缺信号回退 `default_key_far` |
| 启动 | 整车下电 | 缺信号回退 `default_power_off` |
| 传感器 | 无故障 | 缺信号回退 `default_no_sensor_fault` |
| 存储 | ≥ 临时缓存阈值 + 永久/U盘阈值 | `min_temp_storage_bytes` / `min_persistent_storage_bytes` / `persistent_storage_dir`，也受 `default_storage_ok` 影响 |
| 车内 | 无人（DMS + 座椅） | 缺信号回退 `default_no_occupant`；立即开启请求预检查阶段暂不作为拒绝条件，待下电锁车触发后再核查 |
| 整车锁 | 落锁 | 当前有真实车身信号时使用真实值；无信号时回退通过 |
| 温度 | 配置范围内 | `odd_temp_min_c` / `odd_temp_max_c` |

ODD 判断优先使用 `/canbus/vehicle_report` 与 `/driver/chassis_info_m18v2` 的实时信号；字段缺失或车辆状态超时时，按 `build/config.yaml` 中的 `default_*` 与阈值配置回退，便于台架、bag 回放和联调。`user_enabled` 默认为 `false`，避免无请求时默认进入哨兵；哨兵实际开启意图由立即/自动/预约请求状态驱动。

### 哨兵正式接口（对齐 SentryMode.proto / senseauto-msgs）

当前代码中的 `state/sentry_mode_contract.h`、`OddConditions::oddStatus()`、`AlertEvent` 与 `SentinelNode::publishSentryOutputs()` 已按 `ad_msg_idl/l3_msgs/sentry_mode.capnp.h` 的字段语义对齐；`SentinelNode` 已订阅 `/network/sentry_req`，可由 `SentinelRequest` 更新哨兵开启/关闭意图。

> 当前发布状态：已接入真实 RSCL publish，并通过 `config.yaml` 的 `enable_rscl_publish` 控制。默认关闭时仍保留原有 JSONL、日志和控制台输出；开启后发布 `/network/sentry_resp`（`l3_msgs/SentinelResStatus`）与 `/sentry/action_request`（`l3_msgs/AlarmCtrl`，承载声光/双闪调测请求）。`/network/fortress_resp`（`l3_msgs/OrsmStatus`）与哨兵状态发布互相独立，由 ORSM 固定 10Hz 线程发送最近状态。`/sentry/internal/media_cmd` 当前仍为内部 JSONL/日志骨架，未发布正式 RSCL topic。`status_publish_mode` 仅控制哨兵状态，默认 `event_driven`，仍保留 `fixed_10hz` 与 `per_frame` 兼容模式。

**请求输入：`SentinelRequest`（IVI/TBOX → HAD）**

| 字段 | 值定义 |
|------|--------|
| sentinelReqMsgType | 立即开启/关闭、自动开启/关闭、预约开启/关闭、进入/退出 APP、进入/退出视频查看 |
| sentinelScheduledStartTime | 预约日期、小时、分钟 |

请求处理语义：

- 立即开启：收到 `ENABLE_SENTINEL_IMMEDIATELY` 后先核查除“下电、锁车、无人”外的 ODD；通过则返回 `APPROVE_SENTINEL_IMMEDIATELY` 并保存一次性 pending。后续检测到“下电+锁车”后再核查完整 ODD（包含无人），满足则开启，不满足则返回 `DECLINE_SENTINEL_IMMEDIATELY` 与失败原因；无论成功/失败都会清掉一次性 pending。
- 立即关闭：仅清除“立即开启”一次性 pending，不影响自动开启和预约开启状态。
- 自动开启：收到 `ENABLE_SENTINEL_AUTOMATICALLY` 立即返回 `APPROVE_SENTINEL_AUTOMATICALLY` 并保存自动开启状态；若当前已经处于“下电+锁车+钥匙远离”则立即核查完整 ODD，满足则开启；若仅下电、锁车或钥匙距离未满足，则不返回 `DECLINE_SENTINEL_AUTOMATICALLY`，继续保留 pending 等待触发；其它 ODD 不满足时返回 `DECLINE_SENTINEL_AUTOMATICALLY` 与失败原因。后续每次检测到“下电+锁车”上升沿或自动开启 pending 下“下电+锁车+钥匙远离”满足时也会按同样逻辑触发。自动开启状态持续保留，直到收到关闭自动开启请求。
- 预约开启：收到 `ENABLE_SENTINEL_SCHEDULED_START` 立即返回 `APPROVE_SENTINEL_SCHEDULED_START` 并保存预约时间；到点后核查完整 ODD，满足则开启，不满足则返回 `DECLINE_SENTINEL_SCHEDULED_START` 与失败原因；触发处理后清除预约 pending。
- 关闭自动开启/关闭预约开启：分别只清除对应状态，不影响其它请求状态。

**响应输出：`SentinelResStatus`（HAD → IVI/TBOX）**

| 字段 | 当前映射 |
|------|----------|
| sentinelResMsgType | 状态信息发布时固定 UNKNOWN_SENTINEL_RES(0)；仅哨兵模式请求响应帧按实际响应赋值 |
| sentinelOddStatus | 由 `OddConditions::oddStatus()` 生成，覆盖电量/车速/档位/手刹/钥匙/电源/传感器/存储/车内人员 |
| sentinelEnableFailed | 由 `OddConditions::enableFailedReason()` 映射开启失败原因 |
| sentinelGeneralThreat | 一般威胁：OFF/ON/ACTIVE |
| sentinelSevereThreat | 严重威胁：震动/碰撞时 ACTIVE |
| sentinelAudioVisualAlarm | 声光报警状态：警戒 ON，报警 ACTIVE |
| sentinelObstacleType | 由目标标签/IMU 触发映射成人、车辆、两轮车、动物、未识别碰撞等 |

---

## 越野堡垒模式状态机

来源：功能详细说明书V1.2 表格6

```
         T1                T3
  OFF ──────→ ACTIVE ──────────→ WARNING
   ↑           │  ↑               │
   │     T2    │  │  T4           │ T2
   │←──────────┘  └───────────────┘
   │           │                  │
   │     T6    ↓            T6   │
   │         FAILURE ←───────────┘
   │           │
   │     T5    │
   │←──────────┘
```

| 切换 | 条件 |
|------|------|
| T1 Off→Active | 电源ON + P档 + SOC≥`soc_min_orsm` + 手刹 + 无故障 + 软开关=1 |
| T2 →Off | 软开关=0 / 故障 / 非P档 / SOC<`soc_exit_orsm` / 车速>`orsm_speed_exit_threshold_kmh` |
| T3 Active→Warning | 检测到入侵物 |
| T4 Warning→Active | 入侵物离开 |
| T5 Failure→Active | 故障解除 |
| T6 →Failure | 存在故障 |

ORSM 单车堡垒 ODD 阈值已配置化，默认保持设计值：

```yaml
soc_min_orsm: 30.0                 # T1 激活 SOC 阈值
soc_exit_orsm: 20.0                # T2 退出 SOC 阈值
orsm_speed_exit_threshold_kmh: 20.0 # T2 退出车速阈值
```

这些配置由 `SentinelNode` 写入 `OrsmOdd`，`OrsmStateMachine` 直接使用配置后的阈值；状态提醒文案中的低电量/车速过高判断也同步使用同一组阈值，避免状态机和提示逻辑不一致。

离线回放若需要验证 WARNING/THREAT 链路，可在 `orsm:` 节临时启用：

```yaml
orsm:
  replay_force_entry_odd: true
```

该开关会强制满足 ORSM 进入 ODD，仅用于 bag 回放验证；实车配置必须保持 `false`。

### 三级检测区域

```
        ┌─────────────────────────────┐
        │         A区 (30m)           │  I级报警
        │    ┌───────────────────┐    │
        │    │     B区 (15m)     │    │  II级报警
        │    │   ┌───────────┐   │    │
        │    │   │  C区 (5m) │   │    │  III级报警
        │    │   │   [车辆]   │   │    │
        │    │   └───────────┘   │    │
        │    └───────────────────┘    │
        └─────────────────────────────┘
```

### 当前代码实现逻辑

当前堡垒模式在 `SentinelNode` 单进程内闭环实现：`SentinelNode` 负责订阅外部请求、汇总车辆 ODD 和多传感器目标，`OrsmManager` 负责单车/多车模式路由，`SingleOrsmManager` 与 `MultiVehicleOrsmManager` 生成对齐 `OrsmStatus` 的 `ORSMAlertOutput`。

1. 配置与默认设置初始化

- `build/config.yaml` 中的 `orsm:` 配置会在启动时写入 `OrsmSettings`，包含 A/B/C 区半径、入侵物去重距离、人员/动物识别开关、检测范围、声光/鸣笛设置、多车组队默认状态和多车 ODD 阈值。
- 当前默认 topic 为：IVI 请求 `/network/fortress_req_ivi`，TBOX/云端组队状态 `/network/fortress_req_tbox`，状态反馈 `/network/fortress_resp`，动作请求 `/orsm/action_request`。
- `SentinelNode` 启动后会独立创建 ORSM 状态发布线程，按固定 10Hz 发送最近一次 `ORSMAlertOutput`；`status_publish_mode` 只影响哨兵状态发布，不影响堡垒状态发布节奏。
- 视觉检测短时丢失时，动态目标需先累计 `dynamic_visual_association_min_hits` 次视觉语义关联，之后只要 LiDAR 仍持续支撑且轨迹运动状态成立，即可通过 `orsm_dynamic_visual_lidar_continued` 继续作为预警目标输出；近车静态目标必须在 `near_static_visual_refresh_ms` 窗口内重新被视觉语义关联，否则释放为 `unknown` 并停止作为近车预警输出。
- 远距小目标可通过 `far_image_motion_tracking_enabled` 和 `far_image_motion_min_distance_m` 开启图像运动补轨，默认从 10m 起生效。开启 `far_image_motion_allow_lidar_backed_new_targets` 后，远距图像运动可生成 `unknown` 候选，但必须与 LiDAR-primary cluster 关联后才进入全局轨迹；`far_image_motion_required` 默认要求远距 `unknown` 预警同时具备图像运动证据。
- `orsm.replay_force_entry_odd` 只用于离线 bag 验证：它会强制满足进入 ODD 并在未显式请求时默认进入单车安防，实车配置必须保持关闭。

2. 请求闭环与 ODD 汇总

- `/network/fortress_req_ivi` 订阅 `SetOffRoadMdInfoReq`，更新堡垒开关、单车/多车模式、人员/动物识别、检测范围、灯光/鸣笛/提示音设置，以及队长 `multiVehicleTeamSetting`、队员 `multiVehicleTeamReq`、`multiVehicleTeamPrepared`。
- `/network/fortress_req_tbox` 订阅 `OffRoadSetInfoStruct`，同步 TBOX/云端确认后的 `tboxTeamStatusInfo`。代码把该字段作为“车队是否真正组队完成”的最终依据，而不是只看 IVI 侧请求。
- ORSM 每个相机同步组完成后，从车辆状态快照构造 `OrsmOdd`：电源、P 档、SOC、车速、手刹、传感器故障、外部锁车状态、多车 prepared、TBOX 组队状态等都会被写入状态机输入。
- 堡垒模式优先级高于哨兵模式。只要单车或多车堡垒被请求，哨兵感知和状态机会被抑制；堡垒内部再由 `OrsmManager` 判断具体走单车还是多车。

3. 单车/多车路由

- `OrsmManager::resolveMode()` 优先使用当前 ODD 中的 `sw_mode`，否则回退到 `OrsmSettings::orsm_setting_st`。模式为 `SINGLE_AUTO_SECURITY_ON` 时进入单车安防，模式为 `MULTI_VEHICLE_JOINT_SECURITY_ON` 时进入多车联防，其余返回关闭输出。
- 单车和多车共用同一套 `OrsmStateMachine` 状态语义：`OFF`、`ACTIVE`、`WARNING`、`FAILURE`。模式切换时会重置对应 manager，避免旧目标、旧窗口和旧告警保持到新模式。
- 多车联防激活后，如果 TBOX/云端显示车队解散、队员退出或车队不可用，`OrsmManager` 会尝试把 `sw_mode` 路由为单车安防；只要单车 ODD 仍满足，就自动回落到单车堡垒。

4. 目标输入与 ORSM 全局轨迹

- ORSM 消费四路相机同步组的检测/跟踪结果，并按同步组聚合目标。哨兵模式只使用视频/IMU/USS，LiDAR 融合结果仅供 ORSM 目标定位、测距和测速使用。
- 默认启用 LiDAR-primary ORSM 跟踪：先从 LiDAR 点云生成空间候选，再用当前相机语义确认人员/动物；也支持视频语义 + LiDAR 投影融合、近距离视觉地面点优先、C 区 USS 距离修正和远距离 LiDAR-only unknown 输出。
- `updateOrsmGlobalTracks()` 在车体坐标系内维护稳定 `intruder_id`，通过空间门限、视觉 track、LiDAR anchor 和运动预测做跨相机关联；目标需要达到最小年龄、观测次数、LiDAR 命中次数和不确定度门限后才允许对外输出。
- 为减少跳点和闪烁，代码会限制位置更新速度/单步位移，对短时丢帧目标做 hold，对重复目标做合并，并在有新鲜视觉语义时抑制过旧的 LiDAR 语义锚点。

5. 单车安防告警生成

- `SingleOrsmManager` 先按用户设置过滤人/动物类别；普通 unknown 不直接告警，只有满足配置的远距离 LiDAR-only 稳定目标才会作为 unknown 入侵物输出。
- 按距离映射 A/B/C 区：C 区为 III 级，B 区为 II 级，A 区为 I 级；`detecRangeSetting` 会限制最大有效检测范围，例如只看 B+C 或只看 C。
- A 区目标需要更严格稳定性：正在远离本车的 A 区目标不触发报警，外 A 区还要求更多稳定观测、更低速度和更小位置跳变。
- 告警输入不是单帧直接触发：ACTIVE 态下使用 5 帧滑窗、至少 3 次命中确认威胁；WARNING 态下需要连续 5 帧无威胁才清除，并保持至少 3 秒告警窗口。
- `intrudersInfo` 会做物理去重和稳定排序；WARNING 状态下最多保留短时 LiDAR 支撑的旧输出 1.5 秒，避免多目标数组在相机切换或 LiDAR 短暂失配时抖动。

6. 多车联防告警生成

- 多车进入条件比单车更严格：ON/Ready、P 档、SOC 达到多车进入阈值、车速不超过进入阈值、无智驾/传感器故障、网络检查通过、TBOX/云端组队完成、IVI prepared、车辆静止超过 60 秒、外部锁车完成。
- 多车退出条件包括：用户关闭、模式不再是多车、故障、非 P 档、SOC 低于退出阈值、车速超过退出阈值、网络断开、车队解散或队员退出。
- 多车仅监测本车安防侧：`multi_vehicle_security_side=1` 表示 Y+ 侧/左侧，`-1` 表示 Y- 侧/右侧，`0` 表示不做侧向过滤。`multi_vehicle_no_alarm_radius` 默认 5m，范围内不报警。
- 多车会把外部区半径映射为 I 级、内部区半径映射为 II 级，并把 C 区近场报警关闭；外部区目标会被规范为 unknown，内部区保留语义类型。
- 当前实现只输出本车雷达范围和本车入侵物信息，不做车间入侵物共享、编队几何融合或跨车报警同步。

7. 状态、动作与录像

- `OrsmStateMachine` 负责 T1-T6：满足 ODD 从 `OFF` 进 `ACTIVE`，检测到确认威胁进入 `WARNING`，威胁清除回 `ACTIVE`，故障进入 `FAILURE`，故障解除且 ODD 满足后恢复。
- `ORSMAlertOutput` 会回填设置反馈、状态提醒、告警开关、入侵物数量、报警等级、`intrudersInfo` 和 `orsmInfoReserved`；`RsclPublisher::publishOrsmState()` 将其转换成 `l3_msgs/OrsmStatus` 并发布到 `/network/fortress_resp`。
- `RsclPublisher::publishOrsmAction()` 保留 WARNING 或低电量退出时的 `l3_msgs/AlarmCtrl` 映射，但当前未 advertise `/orsm/action_request`，主流程也未调用该函数。现阶段稳定输出为 `/network/fortress_resp` 的 `OrsmStatus`。
- 堡垒模式不触发录像；事件录像仅服务哨兵模式预警和报警。

### ORSM 正式接口（对齐 OrsmInfo.proto / senseauto-msgs）

当前代码中的 `orsm/orsm_output.h` 与 `orsm/orsm_settings.h` 已按 `ad_msg_idl/l3_msgs/orsm_info.capnp.h` 的字段语义对齐。

> 当前发布状态：已接入真实 RSCL publish，并通过 `config.yaml` 的 `enable_rscl_publish` 控制。默认关闭时仍保留原有周期性/报警控制台日志；开启后固定约 10Hz 发布 `/network/fortress_resp`（`l3_msgs/OrsmStatus`）。`/orsm/action_request` 的 `AlarmCtrl` 映射代码当前保留为预留骨架，尚未作为真实对外 topic 启用。

**状态输出：`OrsmStatus`（HAD → IVI/TBOX）**

| 字段 | 值定义 |
|------|--------|
| orsmSettingStFb | 0=OFF, 1=单车安防ON, 2=多车联防ON, 3=功能受限 |
| humansRecogSettingFb / animalsRecogSettingFb | BasicSwitchResp: 0=NONE,1=ON,2=OFF,3=Reversed |
| detecRangeSettingFb | 0=无设置, 1=A区, 2=B区, 3=C区 |
| alarmLightSettingFb / alarmHornSettingFb | BasicSwitchResp |
| alarmSoundSettingFb | 0=急促, 1=温和 |
| alarmSndDuraSettingFb | 0=无, 1=3s, 2=5s, 3=10s |
| orsmMessageRem | 越野堡垒文言提醒/状态 |
| orsmWarningSt | 0=Warning_OFF, 1=Warning_ON |
| intrudersNum | 0=无, 1=单个, 2=多个 |
| orsmWarnLevel | 0=None, 1=I级(A区), 2=II级(B区), 3=III级(C区) |
| intrudersInfo | intruderId/type/poseX/poseY/velocityX/velocityY/timestampNsec |
| orsmInfoReserved | 预留数组；当前固定编码：`[0]=multiVehicleTeamSetting`、`[1]=multiVehicleTeamReq`、`[2]=multiVehicleTeamPrepared`、`[3]=tboxTeamStatusInfo` |

说明：`SetOffRoadMdInfoReq` / `OffRoadSetInfoStruct` 中没有正式 `OrsmStatus` 对应字段的组队相关信号，会保存到节点内部设置，并通过 `orsmInfoReserved` 固定下标随 `/network/fortress_resp` 发出。无论堡垒模式是否开启，节点都会处理 `/network/fortress_req_ivi` 与 `/network/fortress_req_tbox`，刷新已保存设置；堡垒关闭时状态仍为 `OFF`、报警/入侵物字段清空，但设置反馈字段和 `orsmInfoReserved` 继续按保存值发布。

**设置请求：`SetOffRoadMdInfoReq`（IVI/TBOX → HAD）**

| 字段 | 值定义 |
|------|--------|
| orsmSettingSt | 0=关闭, 1=单车安防, 2=多车联防, 3=功能受限 |
| humansRecogSetting / animalsRecogSetting | BasicSwitchReq: 0=NONE,1=ON,2=OFF,3=Reversed |
| detecRangeSetting | 0=无设置, 1=A区, 2=B区, 3=C区 |
| alarmLightSetting / alarmHornSetting | BasicSwitchReq |
| alarmSoundSetting | 0=急促, 1=温和 |
| alarmSndDuraSetting | 0=无, 1=3s, 2=5s, 3=10s |
| multiVehicleTeamSetting | 0=无请求, 1=组队中, 2=组队完成, 3=解散 |
| multiVehicleTeamReq | 0=无请求, 1=加入队伍, 2=退出队伍 |
| multiVehicleTeamPrepared | 0=无请求, 1=已准备, 2=未准备, 3=预留 |

**当前已订阅输入 topic**

| Topic | 消息 | 用途 |
|-------|------|------|
| `/network/fortress_req_ivi` | `SetOffRoadMdInfoReq` | IVI/TBOX 下发 ORSM 开关、单车/多车模式、人员/动物识别、检测范围、声光/鸣笛设置、多车组队请求与 prepared 状态；其中队长使用 `multiVehicleTeamSetting`，队员使用 `multiVehicleTeamReq` |
| `/network/fortress_req_tbox` | `OffRoadSetInfoStruct` | 回填/同步 TBOX/云端队伍状态 `tboxTeamStatusInfo`，这是 HAD 判断组队中/组队完成/未组队的最终依据 |

**多车堡垒当前实现**

- 队长/队员职责对齐文档：队长车辆通过 `MultiVehicleTeamSetting` 发起“组队中/组队完成/解散”请求；队员车辆通过 `MultiVehicleTeamReq` 发起“加入/退出队伍”请求。代码据此推断本车角色，仅作为日志/意图记录。
- 组队最终依据：HAD 不把队长 `TEAM_FINISHED` 或队员 `ADD_THE_TEAM` 直接当作有效车队；必须收到 TBOX/云端 `TboxTeamStatusInfo=2 组队完成` 后才允许进入多车联防。
- 停车准备依据：队长确认队伍创建完成后，IVI 负责显示推荐停放位置和雷达图；HAD 不判断实际停放位置，只接收 `MultiVehicleTeamPrepared=1` 作为准备完成条件。
- 进入门槛：ON/Ready、P档、SOC≥30%、车速≤10km/h、无智驾/传感器故障、网络检查通过、TBOX/云端组队完成、IVI prepared、车辆静止超过60s、外部锁车完成。
- 退出门槛：用户关闭、队伍解散/不可用、队员退出队伍、网络断开、车速>20km/h、SOC<20%、智驾/传感器故障等。
- 安防区域：按 `multi_vehicle_security_side` 过滤安防侧，外部区/内部区分别由 `multi_vehicle_external_radius`、`multi_vehicle_internal_radius` 配置；`multi_vehicle_no_alarm_radius` 默认 5m，范围内不报警。
- 监测与报警：按文档要求，各车只监测并显示本车雷达范围与入侵物信息；代码不做车间入侵物/报警信息共享。
- 回落策略：多车联防激活后若队伍解散/不可用，且单车堡垒 ODD 满足，会自动尝试回落到单车堡垒。

---

## 传感器接口

### IMU — `/sensor/imu`

| 字段 | 类型 | 说明 |
|------|------|------|
| accelX/Y/Z | float32 | 加速度（m/s²） |
| gyroX/Y/Z | float32 | 角速度（rad/s） |

### 超声波 USS — `/parking/pdc/pdc_info`

| 字段 | 类型 | 说明 |
|------|------|------|
| sectorDis | List(UInt16) | 各扇区距离（mm） |
| sourceFlag | List(UInt8) | 各扇区数据来源 |

### 超声波 USS — `/parking/meb/meb_info`

| 字段 | 类型 | 说明 |
|------|------|------|
| collision.distance | Float32 | 预期碰撞点距离（m） |
| collision.obstacleType | UInt8 | 障碍物类型 |


### 超声波 USS 点云 — `/parking/uss/fused_points`（兼容 `/sensor/uss/fused_points`）

接口来源为新增超声波信号表中的 `UssFusedStaticMap`。当前代码同时订阅新接口 `/parking/uss/fused_points` 与旧 bag 常见的 `/sensor/uss/fused_points`。0806 实测 bag 中所有帧的 `fusedPointsSize == 0`、实际 list 长度固定为 150，因此兼容模式会扫描完整 list，但只接收达到 `uss_fused_min_exist_prob` 的点，以排除全零槽和 `(-2000,-2000)` 填充值。该 bag 的有效 `x/y` 原始单位按 mm 解析，并通过 `uss_fused_coordinate_scale_m` 转为 m。

哨兵模式 USS-视觉融合仅用于 1.3m 内近车身距离修正/贴近确认：

- `UssSubscriber` 保存 USS 点云时间历史，并按相机帧时间选择最近快照，避免异步回放时直接取 latest 导致 USS 时间戳超前/滞后；
- `UssVisionFusion` 先按 0.25m 邻域对同相机方位、≤1.3m 的 USS 点云做轻量聚类，再将近距离视觉候选与 USS cluster 做贪心唯一分配，给视觉轨迹标记 `has_uss_fusion / near_body_confirmed / distance_source=uss_pointcloud_cluster`；
- USS 点云可提升贴车、拉门、碰撞/近距离经过等场景的距离稳定性，但不作为 3m/10m 全距离定位精度依据。

ORSM 可通过 `enable_orsm_uss_distance` 单独启用 C 区 USS 测距：只给已由视频确认、且在 `orsm_zone_c_radius` 内关联到新鲜 USS 点的目标替换径向距离；目标方位仍取视觉/LiDAR，避免在 USS 坐标轴安装关系尚未标定时直接使用原始 `x/y`。

### 激光雷达 — `/sensor/lidar/{front,left,rear,right}_lidar`

当前 ORSM 订阅 4 路 LiDAR：`front_lidar`、`left_lidar`、`rear_lidar`、`right_lidar`。车端当前上报点云已是车辆坐标系，订阅侧不再依赖 LiDAR 外参标定，也不做二次 `lidar->car_center` 变换。哨兵模式仍禁止使用 LiDAR。

视觉-LiDAR 融合采用 cluster-first：先把当前相机可见点云投影并在车辆坐标系按 0.6m 邻域做轻量欧式聚类，再用 cluster 投影框/中心与视觉 bbox 贪心唯一分配；一个视觉目标最多匹配一个 LiDAR cluster，同一 cluster 不会在同帧分给多个目标。融合结果写入 `position_3d / distance_m / velocity_3d / lidar_cluster_id / lidar_association_cost`。

消息类型：AdsfiLidarPointCloud，点云单点结构（12字节 packed）：

```cpp
struct LidarPointField {
    int16_t x;         // 0.01m
    int16_t y;         // 0.01m
    int16_t z;         // 0.01m
    int32_t time;      // 与 header.stamp 差值（nsec）
    int8_t  intensity; // 点云强度
    int8_t  ring;      // 线束编号（从1开始）
};
```

### 摄像头 — 多路输入模式

`CameraSubscriber` 通过 `camera_input_mode` 支持两种车端/台架视频获取方式：

| camera_input_mode | 订阅接口 | 数据格式 | 适用场景 |
|-------------------|----------|----------|----------|
| `encoded_hevc`（默认） | `RsclAccess::subscribe<CameraEncodedStruct>` | HEVC/H.265 编码流，按 I/P 帧送 NVDEC 解码 | bag 回放、编码流 topic，如 `/sensor/camera/*/encode` |
| `cuda_frame` | `RsclAccess::CudaSubscriber` | CUDA `Memblock`：`ImageHeader_t + BGR/NV12 raw frame` | 实车端已解码图像帧，参考车端 `sub.cpp` 获取方式 |

两种模式下游统一输出 `DecodedFrame(BGR cv::Mat)`，因此鱼眼去畸变、检测、跟踪、告警和录像链路无需区分来源。

事件录像触发后，4 路视频按视频文件第一帧时间戳命名；`YYYY-MM-DD_HH-MM-SS` 由对应视频文件第一帧 timestamp 换算得到。目录名使用同一录像窗口最早第一帧 timestamp 加事件序号：

```text
<Sentry-YYYY-MM-DD_HH-MM-SS_INDEX>/
  Sentry-YYYY-MM-DD_HH-MM-SS_front.mp4
  Sentry-YYYY-MM-DD_HH-MM-SS_rear.mp4
  Sentry-YYYY-MM-DD_HH-MM-SS_left.mp4
  Sentry-YYYY-MM-DD_HH-MM-SS_right.mp4
```

| Topic | 说明 |
|-------|------|
| /sensor/camera/center_camera_fov120/encode | 前置中置 120°（当前默认配置不接入） |
| /sensor/camera/front_camera_fov195/encode | 前置鱼眼 195° |
| /sensor/camera/rear_camera_fov195/encode | 后置鱼眼 195° |
| /sensor/camera/left_camera_fov195/encode | 左侧鱼眼 195° |
| /sensor/camera/right_camera_fov195/encode | 右侧鱼眼 195° |

配置示例：

```yaml
# 台架/编码流回放
camera_input_mode: "encoded_hevc"

# 实车 CUDA raw frame
camera_input_mode: "cuda_frame"
```

### 4 路相机时间戳同步

哨兵/堡垒感知链路按设计要求优先保持 4 路环视相机强同步。当前同步策略为：

- `front_camera_fov195` 作为同步基准帧；
- `camera_sync_offsets_ms` 按 `camera_names` 顺序配置每路固定时间偏移，校正时间戳 = 原始时间戳 + offset；
- offset 校正后的时间戳差值在 `camera_sync_tolerance_ms` 内才认为同步；
- 正常情况下凑齐 4/4 路相机后进入检测/跟踪/状态机；
- 某一路短时缺帧时，可在 `camera_sync_hold_ms` 窗口内使用该路上一帧补齐同步组；
- 超过 `camera_sync_degrade_ms` 仍无法补齐时，可降级为已有相机组继续处理，并记录缺失相机。

配置示例：

```yaml
enable_camera_sync: true
camera_sync_tolerance_ms: 80
camera_sync_timeout_ms: 1000
camera_sync_max_buffered_frames: 30
camera_sync_enable_hold: true
camera_sync_hold_ms: 300
camera_sync_enable_degrade: true
camera_sync_degrade_ms: 2500
# 顺序对应 camera_names：front/rear/left/right
camera_sync_offsets_ms: [0, 0, 0, 0]
```

容器回放中若出现大量 `drop unsynchronized group` 或 `degraded group`，应优先检查 bag 是否为真实 4 路硬同步采集，或根据日志统计 front 与 rear/left/right 的固定时间差后填写 `camera_sync_offsets_ms`。

ORSM 目标维护可临时打开诊断：

```yaml
orsm_debug_track: true
```

也可以用环境变量 `ORSM_DEBUG_TRACK=1` 临时打开，默认关闭。

---

## 编译

```bash
cd sentinel_perception
mkdir build && cd build
cmake ..
make sentinel_perception -j$(nproc)
```

## 运行

首次克隆或更换 TensorRT/CUDA/GPU 平台后，先从 RF-DETR Base 检查点导出 ONNX，再生成本平台 Engine（Engine 不作为跨平台通用文件）：

```bash
/home/aloys/sentinel/runtime-env/rfdetr-export/bin/python \
  scripts/export_rfdetr_onnx.py

source /home/aloys/sentinel/runtime-env/tensorrt-10.13.3.9/activate.sh
LD_LIBRARY_PATH=/home/aloys/sentinel/runtime-env/model-export-8.4.62/lib/python3.10/site-packages/tensorrt_libs:${LD_LIBRARY_PATH:-} \
  /home/aloys/sentinel/runtime-env/model-export-8.4.62/bin/python \
  scripts/build_trt_engine.py --fp16
```

`build/config.yaml` 默认使用 `../models/rfdetr_base_fp16.engine`。模型契约、SHA256 和完整命令见 `models/MODEL_INFO.md`。

推荐使用单节点脚本启动：

```bash
./scripts/run_single_node.sh                 # 使用 build/config.yaml
./scripts/run_single_node.sh /path/to/config.yaml
```

也可以直接运行可执行文件：

```bash
cd build
./sentinel_perception                        # 默认 config.yaml
./sentinel_perception /path/to/config.yaml   # 指定配置
```

> 注意：master 不使用 `imu_node` / `uss_node` / `perception_node` / `sentry_node` / `video_node` / `orsm_node` 的多进程启动方式。development 中这些节点的 topic 和消息定义已整理到 `docs/development-single-node-port.md`，供单节点内部发布/订阅改造参考。

---

## 性能指标（目标）

| 指标 | 目标值 |
|------|--------|
| 目标检测准确率 | ≥ 95% |
| 目标检测召回率 | ≥ 90% |
| 单帧端到端延迟 | < 100ms |
| 支持目标数/帧 | ≥ 32 |
| 状态跳转成功率 | 99% |
| 帧率 | ≥ 30FPS |
| 丢帧率 | < 5% |
| 应用冷启动 | ≤ 1s |

---

## 依赖项

| 依赖 | 版本 | 说明 |
|------|------|------|
| SenseAuto SDK | active | pal / swcfl / framework-sdk / rscl / msgs |
| senseauto-msgs | V2.6.0 | 官方 `l3_msgs`：`sentry_mode` / `orsm_info` / `alarm_ctrl` |
| FFmpeg | SenseAuto active | libavcodec / libavutil（需含 CUVID） |
| OpenCV | SenseAuto active（容器为 3.4.16） | 图像处理与显示 |
| CUDA | 11.4（容器路径） | GPU 解码与推理 |
| yaml-cpp | — | 配置文件解析 |
| TensorRT | 8.x | 目标检测推理加速（容器当前可 fallback 构建） |

---

## 单节点版本说明

master 保持单节点部署，复用 development 分支中的消息契约、topic 定义和验证方法：

- 使用 senseauto-msgs V2.6.0 官方 `l3_msgs/sentry_mode.capnp.h`、`l3_msgs/orsm_info.capnp.h`、`l3_msgs/alarm_ctrl.capnp.h` 作为哨兵/堡垒模式对外通信消息契约；本地 `capnp_msgs/` 桥接目录已移除。
- 采用 development 中整理的 `/sentry/*`、`/orsm/*`、`/canbus/vehicle_report` topic 命名作为 master 单节点对外接口。
- 新增 `scripts/run_single_node.sh`，明确只启动 `sentinel_perception`。
- Docker bag replay 默认包含 `/parking/uss/fused_points` 和 `/sensor/uss/fused_points`，用于同时适配新增接口与实测旧 bag 中的 USS fused topic。
- 详细说明见 `docs/development-single-node-port.md` 与 `docs/docker-bag-replay.md`。

当前集成状态：

1. ✅ `SentinelNode` 已订阅 `/canbus/vehicle_report`，`buildOdd()` 优先使用实时车辆状态快照，缺失/超时时回退到配置默认值。
2. ✅ `SentinelNode` 已订阅 `/network/sentry_req`、`/network/fortress_req_ivi`、`/network/fortress_req_tbox`，闭环更新哨兵用户意图、ORSM 用户设置、队长/队员组队请求和 TBOX/云端组队状态。
3. ✅ `SentinelNode::onAlert()` 已生成哨兵事件/动作/媒体命令 JSONL 骨架，字段按 `/sentry/event`、`/sentry/action_request`、`/sentry/internal/media_cmd` 对齐；开启 `enable_rscl_publish` 后额外发布 `SentinelResStatus` 与 `AlarmCtrl`。
4. ✅ `OrsmManager` 已接入 `SentinelNode::onFrame()`，内部生成 `ORSMAlertOutput`；开启 `enable_rscl_publish` 后发布 `/network/fortress_resp` 的 `OrsmStatus`，并通过 `orsmInfoReserved` 携带组队相关预留字段。`/orsm/action_request` 的 `AlarmCtrl` 映射尚未启用对外发布。
5. ✅ 已实现三模式互斥：单车/多车堡垒优先于哨兵，堡垒内部按 ORSM 设置与组队状态路由。
6. ⏳ 下一步：补齐真实 OIB/VMM Remote/ON 控制闭环、OIB 执行动作反馈、哨兵测距/测速/震动识别增强与 HMI/媒体链路闭环。

### 历史验证记录

以下结果对应当时的代码、容器和依赖版本，不代表当前 HEAD 已在相同环境重新验证。

- 已在 `rscl-u2004` 容器中使用 `senseauto-msgs V2.6.0-2026-04-17-x86-Linux` 构建验证，构建目录为 `/root/sentinel_video_recorder/build-rscl-v26`。
- `cmake .. && cmake --build . -j2` 已通过，输出 `[100%] Built target sentinel_perception`。
- 2026-09-18 已在 GPU 验证机 `223.109.239.30` 的 `/root/sentinel-fortress-min-check/sentinel_perception` 使用 `build-x86-orsm-check` 完成本次 `orsmInfoReserved` 改动编译验证；`sentinel_perception` 链接通过，`ctest` 中 `orsm_response_feedback_test`、`orsm_zone_behavior_test`、`sentry_alert_manager_test` 全部通过。
- `enable_rscl_publish=false` 默认配置 smoke 启动成功，保留原 JSONL / 日志 / 控制台输出。
- 临时开启 `enable_rscl_publish=true` 时，已验证 `/sentry/event`、`/sentry/action_request` 和 `/network/fortress_resp` 的 advertise；`/orsm/action_request` 当前未启用。
- 运行库路径已处理：构建时将 `build-rscl*/compat-libs` 写入 RPATH，并显式链接 `opencv_flann`；裸启动下 `ldd ./sentinel_perception | grep "not found"` 为空。
- 当前容器无 NVIDIA GPU 设备透传，完整视频/NVDEC 回放仍需 GPU 容器或 CPU decode fallback；非视频链路可先通过 IMU / USS fused / 车辆状态 bag replay 验证。

## 开发路线图

- [x] RSCL 摄像头数据接入（NVDEC 硬件解码，多路）
- [x] IMU 数据接入与振动检测
- [x] USS 超声波接入（PDC 扇区距离 + MEB 防碰撞）
- [x] LiDAR 激光雷达接入（点云解析 + 近距离入侵检测）
- [x] 多传感器融合威胁分级（IMU + USS + LiDAR + 视觉）
- [x] 哨兵模式状态机（5状态 + M1-M8切换 + CAN输出信号）
- [x] 越野堡垒状态机（4状态 + T1-T6切换）
- [x] 越野堡垒三区报警（A/B/C区 + I/II/III级）
- [x] 越野堡垒 SOME/IP/RSCL 状态接口（已订阅设置/组队输入，`/network/fortress_resp` 可通过 `enable_rscl_publish` 发布；`/orsm/action_request` 尚未启用）
- [x] 模块化框架搭建
- [x] RF-DETR Base TensorRT 推理集成（本机已完成 checkpoint→ONNX→FP16 Engine→C++ 单帧验证；待 Thor 重建 Engine 与完整 bag 回放）
- [x] ByteTrack 多目标跟踪实现（Kalman 预测、高/低置信度二阶段匹配、匈牙利关联、稳定 track_id、轨迹和像素速度）
- [x] 堡垒视觉运动补轨（近场检测短时丢失补轨；远距小目标补轨起始距离可配置，默认 10m；支持远距图像运动 + LiDAR 关联生成 unknown 新候选）
- [x] 车身底盘信号订阅（ODD 实际值，已接车辆状态/存储/用户配置回退；钥匙/DMS/座椅真实信号待 SDK 字段确认）
- [x] IVI / TBOX / CAN 输出骨架（已生成事件/动作/媒体命令 JSONL 与 CAN 0x3D4 请求值；`/sentry/event`、`/sentry/action_request` 可通过 `enable_rscl_publish` 发布）
- [x] 多车联防状态闭环一期（队长/队员请求区分、TBOX/云端组队状态、ODD、安防侧区域、多车→单车回落）
- [ ] 多车联防车辆间通信协议与完整编队几何
- [ ] 实车联调与性能优化

---

## 功能状态说明

本节记录各模块的实现范围和剩余工作。历史实验过程见 `docs/test_records/`。

### 🔴 高优先级（核心感知链路）

#### 1. RF-DETR Base TensorRT 推理（`detection/detector.cpp`）

当前状态：主链路已替换为 RF-DETR Base TensorRT wrapper。本机 RTX 3070 / TensorRT 10.13.3.9 已完成 Engine 构建与 C++ 单帧验证；仍需在 Thor 平台重建 Engine 并完成四路 bag 回放。

实现清单：
```
1. ✅ 读取 .engine 文件，创建 IRuntime / ICudaEngine / IExecutionContext
2. ✅ 分配 GPU 输入输出 buffer（cudaMalloc）
3. ✅ 预处理：BGR → resize → RGB → ImageNet normalize → HWC→CHW → GPU memcpy
4. ✅ context enqueue 执行推理
5. ✅ 后处理：decode RF-DETR boxes/scores → confidence 过滤 → 映射回原图坐标
6. ✅ 输出 Detection 列表（class_id / label / bbox / confidence）
```

检测目标类别（来自功能说明书）：人、汽车、两轮车、大型动物（马/牛/羊）、球类、未知物体。

#### 2. ByteTrack 多目标跟踪（`detection/tracker.cpp`）

当前状态：已升级为完整 ByteTrack 核心流程：Kalman 预测轨迹位置，按高/低置信度检测框做二阶段关联，使用匈牙利算法完成全局匹配，并维护 Tentative / Tracked / Lost / Removed 轨迹生命周期。ByteTrack 本身不是神经网络模型，不需要额外 `.pt/.onnx/.engine` 文件。

实现内容：
```
1. ✅ Kalman Filter 预测 bbox 状态：[cx, cy, aspect, h, vx, vy, vaspect, vh]
2. ✅ 高置信度检测框与 Tracked/Lost 轨迹做第一阶段 class + IoU 匈牙利匹配
3. ✅ 低置信度检测框与未匹配活跃轨迹做第二阶段关联，用于保留遮挡/低分目标
4. ✅ 未匹配轨迹进入 lost 状态，超过 max_lost_frames_ 后移除
5. ✅ 未匹配高置信度检测框创建新轨迹，并通过 min_hits_ 过滤输出
6. ✅ 更新轨迹历史（trajectory）和像素速度向量（用于威胁分级中的速度判断）
```

速度向量是哨兵模式威胁分级的关键输入（M5 一般威胁条件：速度≥0.1m/s且距离<3m）。

---

### 🟡 中优先级（系统集成）

#### 3. 车身底盘信号订阅（`sentinel_node.cpp::buildOdd()`）

当前状态：已接入 `/canbus/vehicle_report` 车辆状态快照；可解析到的车速、SOC、P档、手刹、电源、传感器故障、车内占用等字段会进入 `buildOdd()`；用户开关、钥匙距离、存储状态提供配置/存储空间检查回退，字段缺失或消息超时时使用配置默认值。

待补充信号：
```
订阅底盘话题，获取以下实际信号：
- 档位状态（P/R/N/D）→ gear_park
- 车速（km/h）→ speed_zero / orsm odd.speed_kmh
- 动力电池 SOC（%）→ power_ok / orsm odd.soc
- 手刹状态 → handbrake_on
- 整车电源状态（ON/OFF）→ power_off
- 传感器故障标志 → no_sensor_fault / sensor_fault
- 钥匙/蓝牙距离（>10m）→ key_far（哨兵模式专用）
- DMS 车内无人检测 → no_occupant
```

#### 4. SOME/IP 接口对接（`sentinel_node.cpp::onAlert()`）

当前状态：已在单节点内生成哨兵事件、动作请求、媒体命令和 CAN 0x3D4 灯光/双闪/喇叭请求值，并写入 `event_log_path` JSONL 便于联调；`enable_rscl_publish=true` 时可发布 `/sentry/event`、`/sentry/action_request` 和 `/network/fortress_resp`。`/orsm/action_request` 仅保留映射代码，尚未 advertise 或接入主流程。真实 OIB/VMM Remote/ON 控制闭环和执行反馈仍需后续接入。

待接入内容：
```
越野堡垒输出（HAD → IVI/TBOX）：
- 注册 SOME/IP Server，周期（100ms）发布正式 OrsmStatus：
  orsmSettingStFb / humansRecogSettingFb / animalsRecogSettingFb
  detecRangeSettingFb / alarmLightSettingFb / alarmHornSettingFb
  alarmSoundSettingFb / alarmSndDuraSettingFb / orsmMessageRem
  orsmWarningSt / intrudersNum / orsmWarnLevel / intrudersInfo

哨兵模式输出（HAD → OIB，CAN 0x3D4）：
- HAD_HornReq：ALARM 状态时发送 1（喇叭请求）
- HAD_HzrdLampReq：ALARM 状态时发送 2（双闪灯开）
- HAD_LightReq：WARNING 状态时可发送位置灯请求

IVI 唤醒通知：
- 检测到一般/严重威胁后，通过信号唤醒 IVI 座舱域哨兵 APP
```

---

### 🟢 低优先级（功能完整性）

#### 5. 告警视频录制落盘

当前状态：已增加最近 30s 帧快照环形缓存，告警时创建事件目录、复制 pre-event 帧并生成 manifest；仍不是最终 4 路 H.265 30FPS 1min 视频包，需对接媒体节点/编码器完善。

待完善内容（来自功能说明书）：
```
- 循环缓冲区：持续缓存最近 30s 的 4 路环视视频流
- 触发录制：检测到威胁时，截取事件前 30s + 事件后 30s，共 1min
- 视频规格：分辨率 1080P，帧率 30FPS，格式 H.265
- 存储策略：每 1min 一个包，保存到外置 U盘/SD卡（≥64GB，>10MB/s）
- 存储空间不足时：依次覆盖最旧的视频数据
- 用户可通过中控屏哨兵 APP 查看告警视频
```

#### 6. 多车联防通信（越野堡垒 ORSM）

当前状态：已按文档完成多车联防一期状态闭环：区分队长 `MultiVehicleTeamSetting` 与队员 `MultiVehicleTeamReq`，以 TBOX/云端 `TboxTeamStatusInfo` 作为 HAD 组队完成/解散判断依据；HAD 不判断实际停放位置、不共享车间入侵物监测数据。

实现与外部集成清单（来自功能说明书）：
```
- ✅ 通过 TBOX/云端接收多车联防队伍状态（未组队/组队中/组队完成）
- ✅ 队长车辆使用 `MultiVehicleTeamSetting`，成员车辆使用 `MultiVehicleTeamReq`
- ✅ 各车辆通过 IVI `MultiVehicleTeamPrepared=1` 确认停放完成后，HAD 才允许激活多车联防
- ✅ 联防监控按文档保持本车独立监测：不共享车间入侵物监测数据及报警信息
- ✅ 退出条件：队伍解散/队员退出、网络断开、车速>20km/h、SOC<20%、故障等退出联防
- ⏳ TBOX/云端实际组队服务、队伍名称、推荐停车位置 UI 由 IVI/TBOX 侧实现，HAD 仅消费结果信号
```

---

## 技术栈

- 语言：C++17
- 构建：CMake 3.16+
- 消息总线：SenseAuto RSCL
- 视频输入：HEVC 编码流 + FFmpeg/NVIDIA NVDEC，或车端 CUDA Memblock raw BGR/NV12
- 目标检测：RF-DETR Base + TensorRT
- 多目标跟踪：ByteTrack（Kalman 预测、二阶段匹配、匈牙利关联）
- 通信协议：CAN（哨兵→OIB）+ SOME/IP（越野堡垒→IVI/TBOX）
