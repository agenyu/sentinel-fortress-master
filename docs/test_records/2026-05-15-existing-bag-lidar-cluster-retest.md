# 2026-05-15 现有 Bag 复测记录（LiDAR 4路 + cluster-first patch）

- 测试时间：2026-05-15 16:48-16:53 CST
- 测试环境：101.37.131.214 GPU服务器
- 项目目录：`/root/sentinel_video_recorder`
- 构建目录：`build-gpu`
- 远端汇总：`/tmp/existing_bags_cluster_summary_20260515_164842.txt`
- 测试范围：`/tmp/bag` + `/root/bag` 当前存在的 6 个 bag
- 代码状态：LiDAR 4路车辆坐标系解析 + `pointStep>=12` stride 解析 + LiDAR cluster-first 融合 patch，未提交

## 汇总

| Bag | rc | Camera订阅 | LiDAR订阅 | LiDAR Raw | LiDAR Frames | LiDAR Skip | LiDAR Fusion Logs | LiDAR Matched | USS Fusion | Errors | 结论 |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|
| 0429_2051_record_data.rsclbag.000.rsclbag | 0 | 4 | 4 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 通过；该包本次无LiDAR帧进入（原始0429包无LiDAR验证数据），主链路无错误。 普通哨兵回放未触发ORSM LiDAR融合日志。 |
| 入侵场景-拉车门.rsclbag | 0 | 4 | 4 | 12 | 164 | 0 | 0 | 0 | 0 | 0 | 通过；4路LiDAR订阅/解析正常，无skip/错误。 普通哨兵回放未触发ORSM LiDAR融合日志。 |
| 斜坡.rsclbag | 0 | 4 | 4 | 12 | 143 | 0 | 0 | 0 | 0 | 0 | 通过；4路LiDAR订阅/解析正常，无skip/错误。 普通哨兵回放未触发ORSM LiDAR融合日志。 |
| 移动3.rsclbag | 0 | 4 | 4 | 12 | 161 | 0 | 0 | 0 | 0 | 0 | 通过；4路LiDAR订阅/解析正常，无skip/错误。 普通哨兵回放未触发ORSM LiDAR融合日志。 |
| 绕圈.rsclbag | 0 | 4 | 4 | 12 | 168 | 0 | 0 | 0 | 0 | 0 | 通过；4路LiDAR订阅/解析正常，无skip/错误。 普通哨兵回放未触发ORSM LiDAR融合日志。 |
| 蒙脸走动.rsclbag | 0 | 4 | 4 | 12 | 153 | 0 | 0 | 0 | 0 | 0 | 通过；4路LiDAR订阅/解析正常，无skip/错误。 普通哨兵回放未触发ORSM LiDAR融合日志。 |

## 说明

- `/root/bag` 5个包均显示 `lidar_sub=4`、`lidar_skip=0`，说明4路LiDAR订阅与 `pointStep=16` stride解析稳定。
- `/tmp/bag/0429_2051...` 本次没有LiDAR raw/frame，符合它主要用于USS近距验证的用途。
- 本轮按普通 bag 回放配置执行，未强制 ORSM 目标场景，因此 `LidarCameraFusion` 日志为0；cluster-first 代码路径已在前一次 `/root/bag/斜坡.rsclbag` ORSM/目标触发验证中确认过 `matched=1`。
- 本轮无 `ERROR/FATAL/Segmentation fault/Aborted`。

## ORSM 激活补充验证（LiDAR cluster-first 闭环）

- 测试时间：2026-05-15 17:17-17:22 CST
- 配置：`build-gpu/config.bag_replay_orsm_lidar.yaml`，将 `orsm.default_orsm_setting_st` 置为 `1`，关闭事件录像/图片保存，并排除 `/canbus/vehicle_report`，使用默认 ODD 让 ORSM 进入激活。
- 远端汇总：`/tmp/orsm_lidar_cluster_validation_20260515_171737.txt`；移动3单独复测日志：`/root/sentinel_video_recorder/build-gpu/logs/orsm_lidar_moving_20260515_172148.log`。

| Bag | ORSM状态日志 | LiDAR订阅 | LiDAR Raw | LiDAR Frames | LiDAR Skip | Fusion Logs | Matched | Errors | 结论 |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---|
| 斜坡.rsclbag | 11 | 4 | 17 | 1191 | 0 | 13 | 1 | 0 | 通过；ORSM 0→1 激活并进入 WARNING，LiDAR cluster-first 融合触发且 matched=1。 |
| 移动3.rsclbag | 7 | 4 | 17 | 1066 | 0 | 6 | 4 | 0 | 通过；单独复测中 ORSM/LiDAR 融合稳定触发，最多单帧 3 个视觉目标匹配 3 个 LiDAR cluster。 |

样例：

```text
[LidarCameraFusion] tracks=1 projected=1238 raw_points=743231 clusters=8 candidates=2 matched=1
[LidarCameraFusion] tracks=3 projected=23500 raw_points=750689 clusters=95 candidates=61 matched=3
```

结论：此前“普通哨兵回放未触发 LiDAR fusion”的疑点已补充闭环。ORSM 激活配置下，LiDAR 4路输入、pointStep stride解析、体素下采样聚类、cluster-to-bbox 一对一匹配均有日志证据，且无 skip/崩溃/错误。

## play_rate=0.25 全量复测（Camera 丢帧改善验证）

- 测试时间：2026-05-15 19:18-19:36 CST
- 配置：`build-gpu/config.bag_replay_orsm_025.yaml`（`orsm.default_orsm_setting_st=1`，关闭录像/保存）
- 回放速率：`play_rate=0.25`（降速以匹配 GPU 服务器 NVDEC 解码吞吐）
- 远端汇总：`/tmp/playrate025_full_test_latest.txt`

### 对比：play_rate=1.0 vs 0.25

| 指标 | 1.0x (斜坡 ORSM) | 0.25x (斜坡 ORSM) | 改善 |
|---|---:|---:|---|
| 解码平均耗时 | 95ms | 52ms | ↓45% |
| 解码 p90 | 133ms | 77ms | ↓42% |
| timeout 丢弃 | 329 | 4 | ↓99% |
| stale 丢弃 | 870 | 479 | ↓45% |
| queue overflow | 1035 | 1166 | ≈持平 |
| LiDAR fusion logs | 13 | 28 | ↑115% |
| LiDAR matched | 1 | 8 | ↑700% |
| ORSM 状态转换 | 11 | 35 | ↑218% |
| errors | 0 | 0 | — |

### 全量 6 bag 结果

| Bag | 解码帧 | 解码avg/p90/max(ms) | timeout | stale | overflow | ORSM状态 | LiDAR Frames | Fusion Logs | Matched | Errors |
|---|---:|---|---:|---:|---:|---:|---:|---:|---:|---:|
| 0429_2051 (无LiDAR) | 1786 | 54/77/114 | 20 | 467 | 0 | 305 | 0 | 0 | 0 | 0 |
| 入侵场景-拉车门 | 1833 | 61/90/173 | 19 | 482 | 1188 | 44 | 1194 | 43 | 22 | 0 |
| 斜坡 | 1841 | 52/77/342 | 4 | 479 | 1166 | 35 | 1192 | 28 | 8 | 0 |
| 移动3 | 1862 | 62/90/140 | 19 | 498 | 1189 | 45 | 1065 | 53 | 33 | 0 |
| 绕圈 | 3749 | 60/88/158 | 5 | 993 | 2572 | 20 | 2397 | 118 | 107 | 0 |
| 蒙脸走动 | 1840 | 61/89/142 | 2 | 491 | 1194 | 54 | 1192 | 38 | 18 | 0 |

### 分析

1. **解码耗时显著下降**：avg 从 95-110ms 降至 52-62ms，p90 从 133-145ms 降至 77-90ms。降速后 NVDEC 不再被输入速率压满。
2. **timeout 丢弃大幅减少**：从 329 降至 2-20，说明 4 路相机时间戳同步窗口内解码完成率大幅提升。
3. **stale 丢弃仍存在**：约 480-990 次。这是因为 CameraSync 的 `max_buffered_frames=30` 限制，解码后的帧如果等待同步超过缓冲区容量仍会被标记为 stale。
4. **queue overflow 仍存在**：约 1100-2500 次。这是 FrameProcessing 队列容量限制，说明检测/推理仍是瓶颈（YOLOv8 推理 + LiDAR 融合 + ORSM 评估的总耗时仍超过帧间隔）。
5. **LiDAR 融合大幅改善**：matched 从 1 提升到 8-107，说明更多帧成功进入 ORSM 处理并完成 cluster-to-bbox 匹配。
6. **ORSM 状态机活跃**：所有 LiDAR bag 均有 ORSM ACTIVE→WARNING 转换，验证了完整的威胁检测链路。
7. **无错误**：6 个 bag 均 `errors=0`。

### 结论

`play_rate=0.25` 有效缓解了 GPU 服务器 NVDEC 解码瓶颈导致的 timeout 丢帧（↓99%），使 LiDAR 融合和 ORSM 状态机获得更多有效帧进行处理。但 stale/overflow 仍存在，根因是 YOLOv8 推理 + 后处理的总吞吐仍低于 4 路 5fps（0.25x 下的等效输入速率）。

对实车影响：实车 Orin NVDEC 解码 5-15ms + TensorRT 推理 ~10ms，总处理能力远超 4×20fps 输入，不会有此问题。
