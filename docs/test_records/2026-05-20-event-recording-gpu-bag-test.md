# 报警事件录像 GPU Bag 专项测试记录（2026-05-20）

## 1. 测试目标

验证哨兵模式报警事件触发后，事件录像链路是否能够完成：

1. 告警事件生成；
2. `VideoRecorder::triggerRecord()` 触发；
3. 生成 `event_<id>/alert_<id>.mp4`；
4. 生成 `manifest.json`；
5. 录像完成后回填 `sentry_events.jsonl` 的 `video_paths`。

## 2. 测试环境

- GPU 服务器：`101.37.131.214`
- 测试目录：`/root/sentinel_recording_test_20260520/src`
- 构建目录：`/root/sentinel_recording_test_20260520/src/build-gpu-test`
- 测试代码来源：本地当前最新代码同步到 GPU 临时目录
- 本地分支：`fix/cuda-frame-malloc-guard-20260520`
- 最新提交：`cfb3eca Guard CUDA frame subscriber against invalid headers`
- 构建结果：`[100%] Built target sentinel_perception`
- 测试 Bag：`/root/bag/入侵场景-拉车门.rsclbag`

> 说明：本次为录像专项验证，测试配置将 `vehicle_report_topic` 临时指向无效 topic，并设置 `default_key_far: true`，使用 ODD 调试默认值让哨兵状态进入 ACTIVE/WARNING，避免 bag 内车辆状态把状态机拉回 UNAVAILABLE。

## 3. 配置

### 3.1 正式录像窗口配置

配置：

```yaml
enable_event_recording: true
event_pre_seconds: 30
event_post_seconds: 30
event_record_dir: <独立测试目录>
event_log_path: <独立测试目录>/sentry_events.jsonl
camera_input_mode: encoded_hevc
```

回放：

```bash
/root/sentinel_recording_test_20260520/run_recording_test.sh /root/bag/入侵场景-拉车门.rsclbag 0.5
```

### 3.2 缩短窗口诊断配置

为区分“编码器不可用”和“30s 前后缓存/多事件压力问题”，补充执行缩短窗口诊断：

```yaml
event_pre_seconds: 3
event_post_seconds: 3
```

回放：

```bash
REC_PRE_SECONDS=3 REC_POST_SECONDS=3 PLAY_TIMEOUT_SEC=120 \
  /root/sentinel_recording_test_20260520/run_recording_test.sh \
  /root/bag/入侵场景-拉车门.rsclbag 1.0
```

## 4. 测试结果

### 4.1 正式 30s + 30s 录像窗口测试

结果：**未通过**。

关键现象：

- 程序成功启动：`[SentinelNode] All modules started.`
- 状态机进入 ACTIVE/WARNING：
  - `SentryState 1 → 2 | M3: 钥匙离车>10m，激活哨兵模式`
  - `SentryState 2 → 3 | M5: 检测到一般威胁，进入警戒状态`
- 告警事件生成：`sentry_events.jsonl` 共 39 行。
- 但未生成 mp4：
  - `RESULT_MP4_COUNT=0`
  - `RESULT_MANIFEST_COUNT=0`
- `sentry_events.jsonl` 中 `video_paths` 均为空。
- 进程最终被系统 kill，日志中出现：

```text
Killed env LD_LIBRARY_PATH=... sentinel_perception ...
```

结论：正式 30s pre + 30s post 配置下，当前录像链路在 bag 回放中会因短时间多次告警触发大量录像任务/帧 clone，导致内存压力过大，录像未能完成。

### 4.2 缩短 3s + 3s 录像窗口诊断测试

结果：**部分通过 / 编码链路可用**。

输出目录：

```text
/root/sentinel_recording_test_20260520/src/build-gpu-test/events_recording_20260520_212050
```

关键统计：

```text
RESULT_MP4_COUNT=2
RESULT_MANIFEST_COUNT=2
RESULT_JSONL_COUNT=1
sentry_events.jsonl lines=37
with_video_count=2
```

生成文件：

```text
event_1/alert_1.mp4  340K
event_2/alert_2.mp4  883K
event_1/manifest.json
event_2/manifest.json
sentry_events.jsonl
```

录像日志：

```text
[VideoRecorder] Encoding event_1 frames=10 pre=0 post=10 → .../event_1/alert_1.mp4
[VideoRecorder] Event 1 saved: .../event_1/alert_1.mp4
[SentinelNode] Video recorded: event_1 → .../event_1/alert_1.mp4

[VideoRecorder] Encoding event_2 frames=33 pre=5 post=28 → .../event_2/alert_2.mp4
[VideoRecorder] Event 2 saved: .../event_2/alert_2.mp4
[SentinelNode] Video recorded: event_2 → .../event_2/alert_2.mp4
```

`video_paths` 回填验证：

```json
{"event_id":1,"video_paths":[".../event_1/alert_1.mp4"]}
{"event_id":2,"video_paths":[".../event_2/alert_2.mp4"]}
```

结论：FFmpeg 编码器、mp4 文件生成、manifest 写入、录像完成回调、`video_paths` 回填链路是可工作的；问题集中在正式 30s + 30s 配置下的内存/并发/重复触发控制。

## 5. 当前结论

报警事件录像功能当前状态：**功能链路具备，但正式配置下尚不能认为验证通过。**

- ✅ 告警事件可以触发；
- ✅ 缩短窗口下可以生成 mp4；
- ✅ 缩短窗口下可以生成 manifest；
- ✅ 缩短窗口下可以回填 `video_paths`；
- ❌ 正式 `30s pre + 30s post` 配置下未生成 mp4；
- ❌ 正式配置下多次告警会造成录像任务/帧缓存压力，进程被 kill。

## 6. 问题分析

当前 `VideoRecorder::triggerRecord()` 每次告警都会：

1. 加锁读取 ring buffer；
2. clone 最近 `pre_seconds` 秒图像帧；
3. 将完整 pre-frame 副本放入 `pending_tasks_`；
4. worker 再按 `post_seconds` 采集 post-frame；
5. 对每个事件单独编码。

当告警在短时间内连续触发时，多个录像任务同时持有大量 1920×1536 图像 clone。正式 `pre=30s/post=30s/fps=10` 的配置下，单事件理论帧量约 600 帧；多事件连续触发时内存会快速放大。

此外，当前逻辑没有明显的录像任务去重、合并、限流或同一威胁只保留一个录像任务的策略。

## 7. 建议修复方向

优先级建议：

1. **同一告警窗口合并录像任务**  
   当 `VideoRecorder::isRecording()` 为 true 或已有 pending task 时，不应为每个 AlertEvent 新建完整录像任务；可延长当前录像窗口或只记录 event metadata。

2. **限制 pending_tasks_ 队列长度**  
   例如最多 1~2 个录像任务；超出后丢弃或合并。

3. **pre-frame 不要在 triggerRecord() 时整批 clone**  
   可只保存 timestamp/index，编码时从 ring buffer/磁盘缓存读取；或降低存储分辨率。

4. **按威胁 ID / 状态转换触发录像**  
   优先在 `ACTIVE → WARNING`、`WARNING → ALARM` 状态转换时触发一次录像，而不是每条告警输出都触发。

5. **增加录像性能保护指标**  
   输出 pending task 数、ring buffer 帧数、估算缓存内存、drop/merge 次数，方便实车调试。

## 8. 后续验证建议

修复后重新执行：

1. `30s + 30s` 正式配置；
2. 同一 bag 全量回放；
3. 检查：
   - mp4 数量；
   - manifest 数量；
   - `video_paths` 回填数量；
   - 进程是否被 kill；
   - 峰值内存；
   - 重复告警是否合并；
   - 主链路 `frame_queue_overflow` 是否受录像影响。

## 9. 追加核查：报警过多来源（USS vs 视频）

用户反馈“视频侧应该没这么多报警”，要求核查报警过多是否由 USS 逻辑导致。已基于 GPU 服务器本次两轮日志统计确认：报警风暴主要来自 `AlertManager` 的 USS raw fallback，而不是视频侧本身。

### 9.1 代码路径

`alert/alert_manager.cpp::assessLevel()` 当前评估顺序为：

1. IMU severe/general；
2. 计算 `raw_uss_near`；
3. 视觉目标 near + moving；
4. general vibration fallback；
5. **raw USS near fallback**：

```cpp
if (raw_uss_near) {
    std::cout << "[AlertManager][USS-RawFallback] min_dist=" << raw_uss_min_dist
              << "mm threshold=" << kUssAlertDistMm
              << " visual_targets=" << tracked.size() << std::endl;
    return {ThreatLevel::MEDIUM, kThreatIdUss};  // kThreatIdUss = -200
}
```

因此只要 USS min distance 小于阈值，即使没有视觉目标融合确认，也会返回 MEDIUM 威胁并触发“中控警戒 + 录像”。

### 9.2 日志统计

正式 30s+30s 测试 run：`20260520_211505`

```text
USS-RawFallback: 44
USS-Fusion: 0
IMU-GeneralFallback: 0
[SentinelNode][ALERT]: 40
sentry_events.jsonl: 39 行
level=2 MEDIUM: 34
level=1 LOW: 5
threat_id=-200: 18
```

缩短 3s+3s 诊断 run：`20260520_212050`

```text
USS-RawFallback: 179
USS-Fusion: 0
IMU-GeneralFallback: 0
[SentinelNode][ALERT]: 35
sentry_events.jsonl: 37 行
level=2 MEDIUM: 37
threat_id=-200: 27
video_paths 回填: 2
```

### 9.3 结论

- 日志中没有 `USS-Fusion`，说明不是“视觉目标 + USS cluster 融合”确认后触发。
- 大量 `USS-RawFallback` 出现，且事件 `threat_id=-200` 占比很高，说明主要由 USS 原始距离兜底触发。
- 当前 onAlert 对每个 MEDIUM alert 都调用 `video_recorder_->triggerRecord()`，没有按状态转换/同一威胁/录像中状态做合并，因此 USS raw fallback 的持续近距离会进一步放大为大量录像任务。

### 9.4 建议

1. 哨兵录像触发建议绑定状态转换：优先只在 `ACTIVE → WARNING`、`WARNING → ALARM` 触发录像，而不是每条 MEDIUM alert 都触发。
2. USS raw fallback 不建议直接触发录像；可改为：
   - 仅作为状态/低速近距提示；
   - 或要求连续 N 帧、持续 T 秒；
   - 或要求视觉目标存在 / USS cluster 与视觉目标融合确认后才升级 MEDIUM。
3. `VideoRecorder` 应增加任务合并/限流：录像中或已有 pending task 时，不再为每条 alert 新建完整 pre/post 任务。
4. 测试文档中的“正式 30s+30s 不通过”应理解为录像模块被上游 USS raw fallback 高频触发放大后的系统性问题，不是单纯 FFmpeg 编码不可用。

## 10. 改动验证：以视频检测为触发前提，USS 只做位置精度/危险等级增强

根据用户确认的策略，已修改 `AlertManager`：

- 删除 USS raw/PDC 近距直接触发 MEDIUM/录像的 fallback；
- 保留 `UssVisionFusion` 对视觉 track 的增强：只有视频目标存在，且 USS 点云 cluster 在 1.3m 内匹配到该视觉目标时，才用 USS 距离提升位置精度并升级为一般威胁；
- 一般威胁描述更新为“视频目标接近/USS融合近距/小幅震动”。

关键代码原则：

```text
无视频目标：USS near 不单独触发 MEDIUM/录像
有视频目标：视觉 near/moving 或 USS 1.3m 内融合确认 → MEDIUM
```

### 10.1 GPU 编译验证

在 GPU 服务器临时测试目录中替换 `alert/alert_manager.cpp/.h` 后重新构建：

```text
[100%] Built target sentinel_perception
```

### 10.2 Bag 快速回归

测试命令：

```bash
REC_PRE_SECONDS=3 REC_POST_SECONDS=3 PLAY_TIMEOUT_SEC=120 \
  /root/sentinel_recording_test_20260520/run_recording_test.sh \
  /root/bag/入侵场景-拉车门.rsclbag 1.0
```

测试输出目录：

```text
/root/sentinel_recording_test_20260520/src/build-gpu-test/events_recording_20260520_230247
```

关键统计：

```text
USS-RawFallback: 0
USS-Fusion(AlertManager): 0
[SentinelNode][ALERT]: 15
VideoRecorder Encoding: 2
sentry_events.jsonl: 17 行
levels: LOW=10, MEDIUM=6, HIGH=1
video_paths 回填: 2
```

对比改动前缩短窗口诊断 run：

```text
改动前：USS-RawFallback=179, ALERT=35, jsonl=37 行，MEDIUM=37
改动后：USS-RawFallback=0,   ALERT=15, jsonl=17 行，MEDIUM=6
```

结论：USS raw fallback 报警风暴已消除；报警数量明显下降，事件主要回到视频目标/IMU触发路径。该 bag 中本轮没有出现有效 `AlertManager][USS-Fusion]`，说明该段数据没有形成 1.3m 内可匹配视觉目标的 USS cluster，但增强路径仍保留在 `UssVisionFusion` + `AlertManager` 中。

## 11. 录像触发限流与内存占用修复（2026-05-21）

针对第 4～7 节确认的正式 `30s + 30s` 配置下录像任务/帧缓存压力问题，已补充录像触发与 `VideoRecorder` 侧保护。

### 11.1 代码改动

修改文件：

- `recording/video_recorder.h`
- `recording/video_recorder.cpp`
- `sentinel_node.cpp`

主要策略：

1. **按状态窗口触发录像**
   - 仅在即将 `ACTIVE → WARNING` 的一般/严重威胁触发录像；
   - 仅在即将 `WARNING → ALARM` 的严重威胁触发录像；
   - `WARNING` 状态内连续 `MEDIUM` 告警或 `threat_id` 抖动只发布事件，不再重复创建录像任务。

2. **录像任务合并/限流**
   - `VideoRecorder::triggerRecord()` 返回值从 `event_id` 改为 `bool`；
   - 当已有录制任务正在执行或队列中已有 pending task 时，直接跳过新录像任务，认为当前 `pre + post` 窗口已覆盖该告警窗口；
   - 避免告警风暴下多个任务同时持有大量 `pre_frames` clone。

3. **修复 post-frame 重复采集**
   - post-event 采集从 `task.trigger_timestamp_ns` 之后的新帧开始；
   - 修复旧逻辑从 `last_captured_ts = 0` 开始导致 ring buffer 里的 pre-frame 再次 clone 到 post_frames 的问题。

4. **按配置创建录像器**
   - 仅当 `enable_event_recording=true` 时创建并启动 `VideoRecorder`。

### 11.2 GPU 构建验证

已将修改同步到 GPU 服务器测试目录：

```text
/root/sentinel_recording_test_20260520/src
```

在已有 GPU build 环境中重新构建：

```bash
cd /root/sentinel_recording_test_20260520/src/build-gpu-test
cmake --build . -j2
```

构建结果：

```text
[100%] Built target sentinel_perception
```

### 11.3 状态

本次改动已解决“每条 MEDIUM alert 都新建完整录像任务”和“post 采集重复 clone pre-frame”两类高内存风险。后续仍建议重新执行正式 `REC_PRE_SECONDS=30 REC_POST_SECONDS=30` 全量 bag 回归，确认 mp4/manifest/video_paths 与峰值内存表现。

## 12. 正式 30s + 30s 录像窗口回归（2026-05-21）

基于第 11 节录像触发限流与 post-frame 重复采集修复后，重新执行正式 `30s pre + 30s post` bag 回归。

### 12.1 测试命令

```bash
PLAY_TIMEOUT_SEC=240 REC_PRE_SECONDS=30 REC_POST_SECONDS=30 \
  /root/sentinel_recording_test_20260520/run_recording_test.sh \
  /root/bag/入侵场景-拉车门.rsclbag 1.0
```

测试输出：

```text
RESULT_STAMP=20260521_093351
RESULT_LOG_DIR=/root/sentinel_recording_test_20260520/src/build-gpu-test/logs_recording_20260521_093351
RESULT_EVENT_DIR=/root/sentinel_recording_test_20260520/src/build-gpu-test/events_recording_20260521_093351
RESULT_PLAY_EXIT=1
RESULT_MP4_COUNT=2
RESULT_MANIFEST_COUNT=2
RESULT_JSONL_COUNT=1
```

> `RESULT_PLAY_EXIT=1` 出现在 `rsclbag` 输出 `Play End` 后，测试脚本已标记为可继续；本轮应用进程未被系统 kill。

### 12.2 结果统计

生成录像文件：

```text
event_2/alert_2.mp4    2,388,585 bytes
event_26/alert_26.mp4  5,521,583 bytes
```

`sentry_events.jsonl`：

```text
JSONL_LINES=28
WITH_VIDEO=2
LEVELS={LOW:11, MEDIUM:15, HIGH:2}
THREATS={0:21, 1:4, 9:1, -100:2}
VIDEO_EVENT_IDS=[2, 26]
```

录像链路日志：

```text
[VideoRecorder] Encoding event_2 frames=98 pre=5 post=93
[VideoRecorder] Event 2 saved: .../event_2/alert_2.mp4
[SentinelNode] Video recorded: event_2 → .../event_2/alert_2.mp4

[VideoRecorder] Encoding event_26 frames=230 pre=137 post=93
[VideoRecorder] Event 26 saved: .../event_26/alert_26.mp4
[SentinelNode] Video recorded: event_26 → .../event_26/alert_26.mp4
```

异常关键字统计：

```text
encoding FAILED: 0
Killed: 0
bad_alloc: 0
USS-RawFallback: 0
IMU-GeneralFallback: 0
[AlertManager][USS-Fusion]: 0
```

仍需关注的性能统计：

```text
[PerfSummary] frames=250 sync_ok=89 sync_timeout=149 sync_buffer_overflow=0 frame_queue_overflow=6 interval_drop_groups=24 interval_drop_frames=96
```

另日志开头仍有 4 条 NVDEC 初始化告警：

```text
[NVDEC WARN] Failed to create CUDA device: Cannot allocate memory
```

但本轮后续仍完成解码/告警/录像输出，未导致应用被 kill。

### 12.3 结论

本轮正式 `30s + 30s` 回归 **通过录像专项核心验证**：

- ✅ 生成 mp4；
- ✅ 生成 manifest；
- ✅ `sentry_events.jsonl` 回填 `video_paths`；
- ✅ 未出现编码失败；
- ✅ 未被系统 kill；
- ✅ USS raw fallback 报警风暴保持为 0。

需要继续跟踪但不阻塞录像专项结论的问题：

1. `frame_queue_overflow=6`、`sync_timeout=149`，说明感知主链路仍有同步/处理性能压力；
2. NVDEC 初始化仍有 `Cannot allocate memory` 告警，需要结合 CUDA/NVDEC 资源状态继续排查；
3. 本 bag 中 `event_2` 的 pre 帧只有 5 帧，说明首次触发时环形缓冲尚未积满 30s；这是测试开始早期触发导致，不等同于 recorder 不能保留 30s。

## 13. overflow / timeout / NVDEC memory 排查（2026-05-21）

基于第 12 节正式回归日志继续排查。

### 13.1 关键日志证据

第 12 节回归最终性能统计：

```text
[PerfSummary] frames=250 sync_ok=89 sync_timeout=149 sync_buffer_overflow=0 frame_queue_overflow=6 interval_drop_groups=24 interval_drop_frames=96
```

最终性能分布：

```text
[PerfMetric] name=process.total_ms count=250 avg=101.14 p50=84.50 p90=117.15 p99=168.34 min=55.95 max=2718.51 ms
[PerfMetric] name=frame_queue_wait_ms count=250 avg=1296.53 p50=1307.63 p90=2275.16 p99=2818.67 min=0.02 max=3029.36 ms
[PerfMetric] name=frame_queue_size_before_push count=260 avg=12.65 p50=13.00 p90=20.00 p99=31.00 min=0.00 max=31.00
[PerfMetric] name=camera_sync_timeout_wait_ms count=149 avg=1025.32 p50=1022.42 p90=1050.11 p99=1088.37 min=1000.07 max=1088.75 ms
[PerfMetric] name=camera_sync_group_delta_ms count=89 avg=48.32 p50=50.01 p90=50.03 p99=100.01 min=0.01 max=100.01 ms
```

同步超时缺失路统计：

```text
[PerfCounter] sync_timeout_missing camera=right_camera_fov195 count=79
[PerfCounter] sync_timeout_missing camera=left_camera_fov195 count=85
[PerfCounter] sync_timeout_missing camera=rear_camera_fov195 count=77
```

队列溢出实际 drop 明细：

```text
[FrameProcessing] queue overflow, drop camera=left_camera_fov195 seq=31297
[FrameProcessing] queue overflow, drop camera=right_camera_fov195 seq=52756
[FrameProcessing] queue overflow, drop camera=front_camera_fov195 seq=31327
[FrameProcessing] queue overflow, drop camera=rear_camera_fov195 seq=52798
[FrameProcessing] queue overflow, drop camera=left_camera_fov195 seq=31307
[FrameProcessing] queue overflow, drop camera=right_camera_fov195 seq=52773
```

NVDEC 初始化：

```text
[NVDEC WARN] Failed to create CUDA device: Cannot allocate memory
[NVDEC] CUDA/NVDEC unavailable, falling back to software HEVC decoder
[NVDEC] Initialized software HEVC decoder (GPU=0, 1920x1536)
```

共 4 路相机均出现上述 fallback。

### 13.2 初步结论

1. **overflow 直接原因**：处理队列消费速度低于入队速度。
   - 当前 `sentry_process_interval_ms=100`，同步组按 10Hz 进入。
   - 但处理线程是单线程逐帧处理，每个同步组 4 路相机，`process.total_ms avg=101.14ms/帧`，约等于 `400ms/四路组`。
   - 因此 10Hz 四路全处理超出单线程能力，队列等待升至 `avg=1296ms`，`p99=2818ms`，最终触发 32 帧队列上限溢出。

2. **timeout 直接原因**：四路相机到达/解码不同步，front 作为基准帧等待 1s 后仍常缺 rear/left/right。
   - 日志多为 `got=1/4`、`got=2/4`、`got=3/4`。
   - 缺失统计集中在 rear/left/right，而不是单一路固定异常，说明更像解码/调度滞后，而不是某一路 topic 缺失。
   - 同步成功组的时间戳差 `avg=48.32ms`，大部分在 tolerance 内；问题主要发生在 arrival/processing 延迟超过 `camera_sync_timeout_ms=1000`。

3. **memory 告警不是当前系统显存耗尽**：
   - 回归后 `nvidia-smi` 显示 T4 显存 `0MiB / 15360MiB`，无残留进程。
   - `free -h` 显示可用内存约 `13GiB`。
   - Python 直接调用 CUDA Driver API：`cuInit`、`cuDeviceGet`、`cuCtxCreate`、`cuDevicePrimaryCtxRetain` 均返回 0。
   - `LD_DEBUG` 确认加载的是 `/lib/x86_64-linux-gnu/libcuda.so.1` 真驱动库。
   - 因此 `av_hwdevice_ctx_create(AV_HWDEVICE_TYPE_CUDA)` 返回的 `Cannot allocate memory` 更像 SenseAuto 3rdparty FFmpeg 与当前 CUDA/Driver/运行环境的 hwdevice 初始化兼容问题，而不是当前机器真的没有 GPU/内存。

4. **fallback 到软件 HEVC 是 overflow/timeout 的上游放大因素**：
   - 4 路 1920x1536 HEVC 软件解码平均约 `100ms/帧` 量级。
   - 软件解码导致四路帧 arrival 抖动增大，并使单线程感知处理无法跟上 10Hz 四路输入。

### 13.3 已修复的小问题

`frame_queue_drop` 的最终统计中曾出现乱码 camera 名称，原因是日志统计在 `pop_front()` 后继续引用已失效的 `dropped` 引用。已修正为 pop 前复制 camera 名称。

### 13.4 建议修复顺序

1. **优先修 NVDEC/hw decode 初始化**：这是 timeout 与 overflow 的共同上游。建议用独立最小 FFmpeg hwdevice 程序或替换/重建 3rdparty FFmpeg 验证 `AV_HWDEVICE_TYPE_CUDA`，不要只看 CUDA Driver API。
2. **短期规避 overflow**：在软件解码环境下，将 `sentry_process_interval_ms` 从 `100` 提高到 `300~400`，或改成“每个同步组只保留最新组、丢旧组”，避免历史帧排队造成秒级延迟。
3. **短期减少 timeout**：在软件解码环境下将 `camera_sync_timeout_ms` 从 `1000` 提高到 `2500~3000`，并适当提高 `camera_sync_max_buffered_frames`，但这只是容忍慢解码，会增加端到端延迟；不应作为最终方案。
4. **中期结构优化**：四路同步组应作为 group 入队，处理策略可改为：队列满时丢整组旧帧；或 detector 线程池并行处理 4 路；否则 10Hz 四路全量检测单线程不可持续。

## 14. 建议 1：切换系统 FFmpeg/NVDEC 后复测（2026-05-21）

按“建议 1”优先处理 NVDEC / FFmpeg hw decode 初始化问题，并重新执行同一 bag 回归。

### 14.1 修复方式

排查发现 SenseAuto bundled FFmpeg 存在两个问题：

```text
av_hwdevice_ctx_create(AV_HWDEVICE_TYPE_CUDA, null/0/cuda:0) -> -12 Cannot allocate memory
libavcodec-ffmpeg.so.58 decoders: only software hevc, no hevc_cuvid
```

因此 `Cannot allocate memory` 不是 CUDA Driver 本身失败，而是 bundled FFmpeg 不具备可用 HEVC NVDEC 路径。

本次在测试机安装/修复系统 FFmpeg dev 依赖，并新增 CMake 开关：

```cmake
option(SENTINEL_USE_SYSTEM_FFMPEG "Use system FFmpeg instead of SenseAuto bundled FFmpeg" OFF)
```

隔离构建目录：

```text
/root/sentinel_recording_test_20260520/src/build-system-ffmpeg
```

构建命令：

```bash
cmake -S . -B build-system-ffmpeg \
  -DSENTINEL_USE_SYSTEM_FFMPEG=ON \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_PREFIX_PATH="/opt/senseauto_active/senseauto-framework-sdk/cmake;/opt/senseauto_active/senseauto-msgs/cmake" \
  -DTENSORRT_NVINFER_LIB=/root/yolo_export_venv/lib/python3.10/site-packages/tensorrt_libs/libnvinfer.so
cmake --build build-system-ffmpeg -j2
```

smoke 验证：

```text
[NVDEC] Opened hevc_cuvid decoder successfully
[NVDEC] Initialized NVDEC HEVC decoder (GPU=0, 1920x1536)
```

4 路相机均初始化为 NVDEC，不再 fallback 到 software HEVC。

### 14.2 回归结果

回归戳：

```text
RESULT_STAMP=20260521_103501_system_ffmpeg_nvdec_tty
RESULT_LOG_DIR=/root/sentinel_recording_test_20260520/src/build-system-ffmpeg/logs_recording_20260521_103501_system_ffmpeg_nvdec_tty
RESULT_EVENT_DIR=/root/sentinel_recording_test_20260520/src/build-system-ffmpeg/events_recording_20260521_103501_system_ffmpeg_nvdec_tty
```

产物：

```text
MP4_COUNT=2
MANIFEST_COUNT=2
JSONL_COUNT=1
JSONL_LINES=160
WITH_VIDEO=2
VIDEO_EVENT_IDS=[1, 158]
```

录像文件：

```text
event_1/alert_1.mp4      1,081,473 bytes
event_158/alert_158.mp4 10,932,925 bytes
```

关键异常统计：

```text
NVDEC Cannot allocate memory: 0
Initialized NVDEC: 4
software HEVC fallback: 0
CameraSync timeout: 19
frame queue overflow: 0
Killed/bad_alloc: 0
encoding FAILED: 0
```

最终性能统计：

```text
[PerfSummary] frames=650 sync_ok=304 sync_timeout=19 sync_buffer_overflow=0 frame_queue_overflow=0 interval_drop_groups=141 interval_drop_frames=564
```

关键性能指标：

```text
process.total_ms count=650 avg=47.32 p50=39.51 p90=53.43 p99=80.07 max=3133.78 ms
frame_queue_wait_ms count=650 avg=92.99 p50=69.04 p90=133.24 p99=1571.37 max=3220.12 ms
frame_queue_size_before_push count=652 avg=1.50 p50=1.00 p90=3.00 p99=3.00 max=10.00
camera_sync_timeout_wait_ms count=19 avg=1189.19 p50=1181.23 p90=1357.77 p99=1398.56
```

同步缺失路统计：

```text
left_camera_fov195=19
rear_camera_fov195=8
right_camera_fov195=5
```

### 14.3 对比第 12 节正式回归

| 指标 | bundled FFmpeg/software HEVC | system FFmpeg/NVDEC |
| --- | ---: | ---: |
| NVDEC memory 告警 | 4 | 0 |
| software HEVC fallback | 4 | 0 |
| sync_timeout | 149 | 19 |
| frame_queue_overflow | 6 | 0 |
| process.total_ms avg | 101.14ms | 47.32ms |
| frame_queue_wait_ms avg | 1296.53ms | 92.99ms |
| frame_queue_size p99 | 31 | 3 |
| mp4/manifest/video_paths | 正常 | 正常 |

### 14.4 结论

建议 1 验证有效：切到支持 `hevc_cuvid` 的系统 FFmpeg 后，NVDEC 初始化成功，`Cannot allocate memory` 消失；同步 timeout 从 149 降到 19，队列 overflow 从 6 降到 0。

仍需注意：

1. 本轮 `rsclbag` 在 TTY 下实际显示 `PlayRate: 0.10`，不是目标 `1.0`；因此结果证明 NVDEC 路径和趋势有效，但还应补一轮确认 play_rate 参数/脚本行为。
2. `process.total_ms max=3133.78ms` 和 `frame_queue_wait_ms max=3220.12ms` 仍有个别尖峰，可能来自录像编码/日志/资源调度，需要后续看是否影响实车实时性。
3. 系统 FFmpeg 切换目前只在隔离构建目录验证；若要合入正式分支，需要确认目标部署环境是否允许依赖系统 FFmpeg，或把支持 `hevc_cuvid` 的 FFmpeg 作为明确部署依赖。

## 15. ThreatSession 去重修复与 GPU 回归（2026-05-22）

### 背景

第二轮 GPU Bag 回测发现 `sentry_events.jsonl` 告警行数偏多：6 个 bag 分别约 86、141、249、566、207、191 行。排查确认 `USS-RawFallback=0`，不是 USS 原始距离兜底报警风暴，而是视觉 tracker 的 `track_id` 在遮挡、丢帧、多目标和重识别时抖动，导致 `AlertManager` 以 `threat_id` 变化为新告警反复触发 `SentinelNode::onAlert()`。

### 修复内容

在 `SentinelNode` 增加稳定 `ThreatSession`：

- 将“同一威胁”定义为持续威胁窗口，而不是瞬时视觉 `track_id`；
- `LOW` 仅作为内部观察状态，不写正式外部事件；
- `MEDIUM` 在同一 ThreatSession 内只发布一次警戒事件；
- `HIGH` 在同一 ThreatSession 内只发布一次报警升级事件；
- 录像在同一 ThreatSession 内只触发一次；
- 状态机 M8 “同一威胁 >3min”使用稳定 session id，避免 raw `track_id` 抖动重置计时。

### GPU 构建验证

将本地最新代码同步到 GPU 服务器：

```text
/root/sentinel_recording_test_20260520/src
```

构建目录：

```text
/root/sentinel_recording_test_20260520/src/build-system-ffmpeg
```

构建结果：

```text
[100%] Built target sentinel_perception
```

### 6 bag 回归结果

回归目录：

```text
/root/sentinel_recording_test_20260520/src/build-system-ffmpeg/round2_20260522_113347_round2_latest
```

配置：`play_rate=1.0`，`event_pre_seconds=30`，`event_post_seconds=30`。

| Bag | 修复前 JSONL 行数 | 修复后 JSONL 行数 | mp4 | manifest | video_paths 回填 | 说明 |
| --- | ---: | ---: | ---: | ---: | ---: | --- |
| 入侵场景-拉车门 | 86 | 3 | 1 | 1 | 1 | 1 条 MEDIUM、1 条 HIGH、1 条录像回填 |
| 斜坡 | 141 | 2 | 1 | 1 | 1 | 1 条 MEDIUM、1 条录像回填 |
| 移动3 | 249 | 2 | 1 | 1 | 1 | 1 条 MEDIUM、1 条录像回填 |
| 绕圈 | 566 | 2 | 1 | 1 | 1 | 1 条 MEDIUM、1 条录像回填 |
| 蒙脸走动 | 207 | 2 | 1 | 1 | 1 | 1 条 MEDIUM、1 条录像回填 |
| 0429_2051_record_data | 191 | 2 | 1 | 1 | 1 | 1 条 MEDIUM、1 条录像回填 |

关键统计：

- 6 个 bag 均 `Play End`；
- 6 个 bag 均生成 1 个 mp4、1 个 manifest，并完成 `video_paths` 回填；
- `USS-RawFallback=0`；
- `NVDEC` 初始化失败为 0，software fallback 为 0；
- 控制台仍会打印原始 `[SentinelNode][ALERT]` 调试日志，但重复项已被 `[SentinelNode][ThreatSession] suppress duplicate alert` 抑制，不再写正式事件。

### 结论

方案 C 验证通过：外部告警事件从“每次 `track_id` 抖动都发布”收敛为“每个 ThreatSession 按警戒/报警升级发布”。这符合设计文档中“同一威胁”和状态转换型告警语义，同时保留录像回填链路。
