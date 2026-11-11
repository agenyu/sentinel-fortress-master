# 哨兵模式 / 越野堡垒模式设计文档对照核查

核查日期：2026-05-08

依据文档：
- `../哨兵模式功能详细说明书V1.3.docx`
- `../越野堡垒功能详细说明书V1.2.docx`
- `../哨兵模式及越野堡垒功能-需求文档.docx`

当前代码范围：`sentinel_perception`，含最新 `SingleOrsmManager` / `MultiVehicleOrsmManager` / `OrsmManager` 路由架构。

## 总体结论

当前代码已经具备主链路骨架：相机/IMU/USS/LiDAR 接入、哨兵状态机、告警评估、事件录像、ORSM 单车三区报警、多车联防 ODD 门控与模式路由、RSCL 状态发布。但与设计文档相比，仍有多项功能处于配置默认、简化实现或未接真实外部接口状态。

优先级最高的缺口是：
1. IVI/TBOX 请求输入未真实订阅，哨兵/ORSM 软开关和多车组队状态仍主要靠配置默认值。
2. OIB/VMM 控制动作未实现：Remote/ON 模式调用、ORSM 声光/双闪/喇叭动作输出、媒体命令正式 topic 未完成。
3. 多车联防监控区域仍复用单车 A/B/C 圆形区域，与文档“左侧/安防侧外部区、内部区、5m 不报警区”不一致。
4. 哨兵威胁识别仍是近似：视觉距离使用 bbox 比例，速度用像素速度；门把手/剐蹭/碰撞未做专门频域/事件识别。
5. 车辆/环境 ODD 仍不完整：钥匙/DMS/座椅/外部锁车/5G/OTA/车辆模式/天气能见度等很多字段没有真实信号接入。

## 哨兵模式差异

### 已实现或部分实现

- 状态机：UNAVAILABLE / STANDBY / ACTIVE / WARNING / ALARM，对应 M1-M8 基本流转。
- ODD 基础项：SOC、车速 0、P 档、手刹、电源、传感器故障、无人、钥匙远离、存储空间。
- 威胁检测：视觉目标、IMU 震动、USS 近距离辅助。
- 告警事件：一般威胁 / 严重威胁映射到 `SentinelResStatus`，严重威胁触发 `AlarmCtrl` 声光字段。
- 录像：有前后 30s 事件录像异步编码能力。
- 4 路鱼眼同步：已实现多相机同步缓冲。

### 未实现 / 不一致

| 功能点 | 文档要求 | 当前代码状态 | 影响 |
|---|---|---|---|
| IVI/TBOX 哨兵请求 | 单次开启、长期开启、预约开启、立即关闭等 | 未订阅 `SentinelRequest`，使用 `user_enabled/user_shutdown_requested` 配置代替 | 无法实车闭环用户意愿 |
| 钥匙距离 >10m | 真实钥匙距离检测 | `default_key_far` 配置默认值，没有钥匙距离信号 | 激活条件不真实 |
| DMS/座椅无人 | 车内无人检测 | 使用配置或 VehicleReport 简化字段 | 无法区分车内有人场景 |
| 存储卡状态 | 外置卡缺失/格式错误/空间不足提醒 | 只检查目录可用空间，不判断外置卡/格式 | HMI 提醒不完整 |
| 视觉距离/速度 | 目标距离 <3m 且速度 >=0.1m/s | 已改为按类别名义高度+鱼眼等效焦距估距，并按 track_id 时间序列估速；仍需后续标定地面投影/实车校准 | 比原 bbox 比例/像素速度更接近真实值，但仍非最终高精度测距 |
| 门把手震动 | 门把手震动持续/幅值判定 | 用 IMU accel/gyro 比值近似，无频域/位置区分 | 拉门把手场景可靠性不足 |
| 剐蹭/碰撞 | 车辆剐蹭、碰撞严重威胁 | 归并为 IMU 大幅震动 | 事件类型不细分 |
| 同一威胁超时 | 严重威胁 >3min 回 ACTIVE | 已按 track_id/传感器事件ID 区分同一威胁，威胁ID变化会重新计时 | 多目标连续报警误停风险降低 |
| IVI 唤醒/媒体命令 | 唤醒哨兵 APP、传递视频事件信息 | 仅 JSONL/日志骨架，`/sentry/internal/media_cmd` 非正式 publish | 座舱保存/查看闭环未完成 |
| 车端 HMI | 中控黑屏/点亮/闪烁/提示语 | 无真实 HMI 控制，仅状态发布 | 显示交互未闭环 |
| 功耗控制 | HAD 关闭不相关传感器/节点 | 未实现功耗管理 | 不满足功耗需求 |
| 云/APP | 文档 V1.3 中多处标注不支持 | 当前也未实现 | 与 V1.3 基本一致 |

## 越野堡垒模式差异

### 已实现或部分实现

- 架构：`SingleOrsmManager`、`MultiVehicleOrsmManager`、`OrsmManager` 路由已拆分。
- 单车三区报警：A/B/C 半径、目标类型过滤、人/动物配置、入侵物列表、报警等级。
- 安全生物：激活瞬间已有目标标记为安全，越过 A 区触发越线报警。
- 多车 ODD：组队完成、IVI prepared、网络、SOC、车速、静止、锁车等门控已补齐为内部逻辑。
- 多车退出：组队不可用、网络断开、车速过高、SOC 低等退出原因已映射。
- 多车回落：多车退出后可自动尝试回落单车安防。
- ORSM 状态发布：`/orsm/state_broadcast` 已支持 `OrsmStatus` publish。

### 未实现 / 不一致

| 功能点 | 文档要求 | 当前代码状态 | 影响 |
|---|---|---|---|
| IVI/TBOX ORSM 设置输入 | ORSMSettingSt、危险生物、监测范围、报警方式、多车 prepared/team status 等真实输入 | 未订阅 `SetOffRoadMdInfoReq` / `OffRoadSetInfoStruct`，主要靠 config 默认 | 实车无法动态配置/组队 |
| OIB/VMM Remote/ON 调用 | 开启后 1s 周期调用 `VMM_UsageMode.enterUsageMode(Remote)` 和 `VMM_PowerMode.setPowerMode(ON)` | 未实现 | ORSM/多车联防电源模式闭环缺失 |
| ORSM 动作输出 | 单车/多车报警时请求灯光、双闪、喇叭 | `/orsm/action_request` 预留，未单独发布；仅哨兵侧有 `AlarmCtrl` publish | 报警执行链路不完整 |
| 单车退出网络条件 | T2 包含 5G 网络断开退出 | 单车 `OrsmStateMachine` 未检查 `network_ok` | 网络断开时单车可能不退出 |
| 单车故障退出语义 | 文档 T2 同时写故障退出，T6 写 Failure | 代码故障进入 FAILURE 并反馈功能受限 | 与部分表述不一致，需要产品确认 |
| 单车低电退出动作 | SOC <20% 退出并双闪 5s/喇叭 1s | 状态退出有，动作输出无 | 用户提醒不完整 |
| 单车报警动作细节 | A/B/C 不同等级灯光/双闪/声音/APP 震动/HMI 雷达图 | 仅输出 OrsmStatus 字段，不输出具体 OIB/APP/HMI 动作 | 交互与执行不完整 |
| 多车监控区域 | 左侧/安防侧：外部区、内部区、5m 内不报警 | 当前复用单车 A/B/C 圆形区域，且 C 区 <=5m 为 III 级报警 | 与多车设计明显不一致 |
| 多车外部/内部区识别 | 外部区雷达可检测但不可区分人/动物；内部区摄像头可区分 | 当前融合视频+LiDAR，目标类型仍按视觉 label | 外部区类型处理不符合文档 |
| 多车报警等级字段 | 文档有“多车联防报警等级”和“多车联防状态文字提醒” | 当前 `OrsmStatus` 内复用 `OrsmWarnLevel` 和 `OrsmMessageReminder`；本地 proto 未独立字段 | 接口可能需与最新 IDL 再核对 |
| 联防队伍创建交互 | 附近队伍、加入队伍、车数量不足、队伍创建完成/解散提示 | 没有 TBOX 云端组队交互，仅状态枚举默认值 | 多车云端/APP 流程未闭环 |
| 停放位置引导 | 推荐停放位置、确认弹窗、IVI prepared | 只消费 prepared 默认值，不生成引导/HMI | 车机体验缺失 |
| 多车不共享数据 | 各车仅显示本车雷达监测范围 | 当前本车实现不共享；但也无雷达图/HMI 输出 | 共享约束满足，展示未完成 |
| 恢复出厂默认开启 | 首次下线/恢复出厂 ORSM 软开关默认开启 | 当前 `default_orsm_setting_st` 配置默认仍为 0 | 与文档默认值不一致 |
| 环境 ODD | 温度、能见度、天气、传感器遮挡/脏污/未校准 | 未接入真实环境/遮挡检测 | ODD 不完整 |
| 优先级互斥 | 越野堡垒 > 露营 > 哨兵，三者互斥 | 代码内哨兵和 ORSM 可同节点并行处理，没有全局互斥调度 | 模式冲突风险 |

## 接口/发布层差异

- 已发布：`/sentry/event`、`/sentry/action_request`、`/orsm/state_broadcast`。
- 未发布或未订阅：
  - `/sentry/state_broadcast` 未看到实际 publish。
  - `/sentry/internal/media_cmd` 仍为 JSONL/日志骨架。
  - `/orsm/action_request` 仍预留。
  - IVI/TBOX → HAD 的 Sentry/ORSM 设置请求未真实接收。
  - VMM/OIB 使用模式、电源模式服务调用未实现。

## 建议开发顺序

1. **先补输入闭环**：订阅并解析 IVI/TBOX 的 `SentinelRequest`、`SetOffRoadMdInfoReq`、TBOX team status，将 config 默认值降级为 fallback。
2. **补 ORSM 动作输出**：实现 `/orsm/action_request` 或统一 `AlarmCtrl` 的 ORSM 声光/双闪/喇叭请求映射。
3. **补多车监控区域模型**：从复用单车 A/B/C 改为多车“安防侧/左侧 + 外部区/内部区/5m 不报警区”。
4. **补 VMM/OIB 模式调用**：Remote/ON 周期调用与退出恢复。
5. **继续提升哨兵威胁精度**：距离/速度已从 bbox 比例/像素速度升级为类别名义高度估距 + track_id 时间序列估速；同一威胁已按 ID 跟踪。后续重点是相机标定地面投影、门把手震动频域识别与实车阈值标定。
6. **补全 HMI/媒体链路**：正式媒体命令、视频保存结果、HMI/APP 提示状态。
7. **补互斥调度**：越野堡垒/露营/哨兵优先级与互斥状态机。
