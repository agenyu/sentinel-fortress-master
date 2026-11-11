# development 分支借鉴清单（保持 master 单节点）

目标：吸收 `development` 分支里已经整理好的 RSCL 工程化约定，但不把 master 改成多进程/多节点。master 仍只构建并启动一个可执行文件：`sentinel_perception`。

## 保留的单节点边界

- 入口：`main.cpp`
- 主节点：`SentinelNode`
- 可执行文件：`sentinel_perception`
- 内部模块：IMU、USS、LiDAR、Camera、Detector、Tracker、AlertManager、SentryStateMachine、OrsmManager 都作为同一进程内模块装配。
- 不引入 `imu_node`、`uss_node`、`perception_node`、`sentry_node`、`video_node`、`orsm_node` 这种多进程启动方式。

## 从 development 借鉴并纳入 master 的内容

### 1. RSCL/Cap'n Proto 消息契约

master 单节点当前已切到 `senseauto-msgs V2.6.0` 官方 `ad_msg_idl/l3_msgs/*.capnp.h`，不再维护本地 `capnp_msgs/` 桥接消息目录。对外接口统一使用官方消息类型：

- `sentry_mode.capnp.h`：`SentinelRequest` / `SentinelResStatus`，用于 IVI/TBOX ↔ HAD 哨兵模式请求与状态/事件反馈。
- `orsm_info.capnp.h`：`SetOffRoadMdInfoReq` / `OffRoadSetInfoStruct` / `OrsmStatus` / `IntrudersInfo`，用于越野堡垒设置请求、设置反馈、状态广播和入侵物信息。
- `alarm_ctrl.capnp.h`：`AlarmCtrl`，用于声光/双闪/鸣笛调测请求。

当前由 `SentinelNode` 内部直接装配并通过 `publish/RsclPublisher` 发布，避免拆成多个 RSCL 节点。

### 2. Topic 命名约定

对外 topic 建议沿用 development 的命名，方便与既有测试工具、`rsclbag` 和 `rscl_monitor` 对齐：

| 类型 | Topic | 单节点落点 |
| --- | --- | --- |
| 车辆状态输入 | `/canbus/vehicle_report` | `SentinelNode` 订阅并更新 ODD |
| 哨兵状态输出 | `/sentry/state_broadcast` | `SentinelNode` 发布状态机状态 |
| 哨兵事件输出 | `/sentry/event` | `SentinelNode::onAlert()` 通过 `SentinelResStatus` 发布 |
| 哨兵动作输出 | `/sentry/action_request` | `SentinelNode::onAlert()` 通过 `AlarmCtrl` 发布 |
| 录像命令输出 | `/sentry/internal/media_cmd` | 当前保留为进程内/JSONL 骨架，未发布正式 RSCL topic |
| 录像结果输入 | `/sentry/internal/media_result` | 后续接入录像模块结果 |
| ORSM 状态输出 | `/orsm/state_broadcast` | `SentinelNode`/`OrsmManager` 通过 `OrsmStatus` 发布 |
| ORSM 动作输出 | `/orsm/action_request` | 预留，当前未单独发布 |

> development 中的 `/sentry_orsm/internal/*` 是多节点内部链路；单节点 master 不需要作为进程间 topic 使用，等价数据应保留为进程内结构体传递。

### 3. 验证方式

可以借鉴 development 的验证方法，但启动对象应只有一个：

```bash
./scripts/run_single_node.sh [config.yaml]
rscl_monitor /sentry/state_broadcast
rscl_monitor /sentry/event
rscl_monitor /orsm/state_broadcast
```

## 后续落地顺序建议

1. ✅ 已把 `/canbus/vehicle_report` 接入 `SentinelNode`，`buildOdd()` 会优先使用实时车辆状态快照；字段缺失或消息超时时回退到配置默认值。
2. ✅ `onAlert()` 已生成哨兵事件、动作、媒体命令和 CAN 0x3D4 请求值，并写入 JSONL 作为单节点联调出口；开启 `enable_rscl_publish` 后额外发布 `/sentry/event` 的 `SentinelResStatus` 与 `/sentry/action_request` 的 `AlarmCtrl`。
3. ✅ ORSM 内部结构已对齐 `OrsmInfo.proto`：`OrsmSettings` ↔ `SetOffRoadMdInfoReq`，`ORSMAlertOutput` ↔ `OrsmStatus`，入侵物字段 ↔ `IntrudersInfo`；开启 `enable_rscl_publish` 后额外发布 `/orsm/state_broadcast` 的 `OrsmStatus`。
4. 后续补 IVI/TBOX 输入订阅（`SentinelRequest`、`SetOffRoadMdInfoReq`）、OIB 执行动作反馈和 `/orsm/action_request` 独立动作 topic。

这样能最大化复用 development 的工程接口，同时保持 master 的单节点部署形态。
