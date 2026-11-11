# 2026-05-19 固定间隔处理 + LiDAR Fusion 优化验证记录

- 测试时间：2026-05-19 00:20-00:38 CST
- 测试环境：GPU 服务器 `101.37.131.214`
- 项目目录：`/root/sentinel_video_recorder`
- 构建目录：`build-gpu`
- 远端验证目录：`/root/sentinel_perf_runs/20260519_final_verify_002039`
- 代码状态：本地优化改动临时同步到 GPU 服务器验证；验证后远端源码与 binary 已恢复
- 回放倍率：`play_rate=0.25`
- 说明：`rsclbag` 在非 TTY 自动化中仍可能返回 `rc=1`，但回放完成并产出完整节点日志，本记录按有效回放统计

## 本次改动范围

1. 纯哨兵模式不执行 LiDAR fusion；只有 ORSM/堡垒模式请求时才允许 LiDAR merge/fusion。
2. 新增 ORSM LiDAR fusion 限频：`orsm.lidar_fusion_period_ms`，默认 100ms（10Hz）。
3. LiDAR fusion 增加 ROI + voxel 下采样缓存，按 LiDAR timestamp 复用。
4. 将相机外参矩阵求逆移出 per-point 投影循环。
5. 先用 expanded bbox gate 过滤投影点，再聚类，避免全量点云聚类。
6. 新增固定间隔处理同步图像组：
   - `sentry_process_interval_ms` 默认 100ms（10Hz）
   - `orsm_process_interval_ms` 默认 200ms（5Hz）
   - 未达到间隔的完整同步组直接丢弃，不进入 FrameProcessing queue。

## 编译验证

GPU 服务器编译通过：

```text
[100%] Built target sentinel_perception
```

验证结束后恢复远端源码并重建原 binary，确认无本次埋点/临时代码残留：

```text
perf_count=0
```

## 纯哨兵 6 bag 验证

配置：

- ORSM OFF
- 排除 LiDAR topic
- `sentry_process_interval_ms=100`（默认 10Hz）
- `enable_save=false`
- `enable_display=false`

| Bag | main_frames | sync_ok | sync_timeout | queue_overflow | interval_drop_groups | interval_drop_frames | total_p90(ms) | queue_wait_p90(ms) |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| 0429_2051_record_data | 688 | 301 | 20 | 0 | 138 | 552 | 63.02 | 151.34 |
| 入侵场景-拉车门 | 700 | 327 | 19 | 0 | 152 | 608 | 63.79 | 152.45 |
| 斜坡 | 708 | 328 | 4 | 0 | 153 | 612 | 63.14 | 147.96 |
| 移动3 | 716 | 323 | 19 | 0 | 148 | 592 | 62.76 | 151.27 |
| 绕圈 | 1440 | 657 | 5 | 0 | 307 | 1228 | 62.74 | 145.20 |
| 蒙脸走动 | 692 | 312 | 2 | 0 | 149 | 596 | 61.93 | 152.67 |

结论：

- 6 个纯哨兵 bag 均 `queue_overflow=0`。
- 固定间隔丢帧生效，30s bag 丢弃约 138~153 个同步组，60s 绕圈丢弃 307 个同步组。
- 主处理耗时 `total_p90≈62~64ms`，与 10Hz 固定处理策略匹配。
- sync timeout 仍与前序分析一致，主要来自 front/left 与 rear/right 原始 FPS 不一致。

## ORSM/LiDAR smoke 验证

配置：

- 使用 `config.bag_replay_orsm_025.yaml`
- ORSM ON
- 包含 `/sensor/lidar/*`
- 默认 `orsm_process_interval_ms=200`（5Hz）
- 默认 `orsm.lidar_fusion_period_ms=100`（10Hz）
- 测试 bag：`/root/bag/入侵场景-拉车门.rsclbag`

| rc | main_frames | queue_overflow | interval_drop_groups | interval_drop_frames | total_p90(ms) | lidar_fusion_p90(ms) | lidar_merge_p90(ms) |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 1 | 448 | 0 | 193 | 772 | 107.25 | 37.01 | 26.28 |

对比优化前同类结果：

- 优化前 `lidar_fusion_ms p90≈2539.90ms`，queue overflow 约 1196 行。
- 优化后 `lidar_fusion_ms p90≈37.01ms`，`queue_overflow=0`。

结论：

- LiDAR fusion 秒级耗时已降至几十毫秒级。
- ORSM/LiDAR 场景在 0.25x 回放下无 FrameProcessing queue overflow。
- 若实车实时运行仍有延迟，可继续降低 ORSM 处理频率或加大 ROI/voxel 参数。

## 注意事项

- 固定间隔处理策略适合在线感知告警，目标是低延迟和新鲜帧，不适合离线逐帧分析。
- 当前测试日志目录 `/root/sentinel_perf_runs` 累计较大，验证后服务器磁盘使用率约 96%；建议后续清理历史测试日志或扩展清理 cron 范围。
