# Docker RSCL bag 回放联调准备

目标：先不考虑 THOR/aarch64 交付，在 `rscl-u2004` x86 Docker 容器内用录制 bag 对 `sentinel_perception` 单节点做链路联调。

## 1. 容器与工具

```bash
ssh root@120.24.215.24
docker exec -it rscl-u2004 bash
cd /root/sentinel_video_recorder
```

容器内需要：

```bash
command -v rsclbag
rsclbag --help
```

当前已确认 `/usr/bin/rsclbag` 可用。

## 2. 准备回放配置

```bash
./scripts/prepare_bag_replay_env.sh
```

脚本会：

- 检查/构建 `build-rscl/sentinel_perception`
- 生成 `build-rscl/config.bag_replay.yaml`
- 将 `enable_display`、`enable_save` 关闭，适配无 GUI Docker
- 将 `default_key_far` 置为 `true`，便于 bag 缺少钥匙距离信号时激活哨兵 ODD
- 创建 `logs/`、`events/`、`output/` 目录

## 3. 检查 bag

```bash
rsclbag check /path/to/test.bag
rsclbag info /path/to/test.bag --detail
```

重点核对 bag 内是否包含以下 topic：

| 类型 | 当前配置 topic |
| --- | --- |
| 前鱼眼 | `/sensor/camera/front_camera_fov195/encode` |
| 后鱼眼 | `/sensor/camera/rear_camera_fov195/encode` |
| 左鱼眼 | `/sensor/camera/left_camera_fov195/encode` |
| 右鱼眼 | `/sensor/camera/right_camera_fov195/encode` |
| IMU | `/sensor/imu` |
| USS PDC | `/parking/pdc/pdc_info` |
| USS MEB | `/parking/meb/meb_info` |
| USS fused | `/sensor/uss/fused_points` |
| 前/左/后/右激光雷达 | `/sensor/lidar/front_lidar`, `/sensor/lidar/left_lidar`, `/sensor/lidar/rear_lidar`, `/sensor/lidar/right_lidar` |
| 车辆状态 | `/canbus/vehicle_report` |

若 bag topic 不一致，用 `rsclbag simple_play --remap /bag/topic:=/config/topic`。

## 4. 回放联调

当前 `rscl-u2004` 容器没有挂载 NVIDIA 设备，启动日志会出现 `NVDEC ERROR Failed to create CUDA device`。因此建议先做非视频链路回放：

```bash
INCLUDED_TOPICS="/sensor/imu /sensor/uss/fused_points /parking/* /sensor/lidar/* /canbus/vehicle_report" \
./scripts/run_bag_replay.sh /path/to/test.bag build-rscl/config.bag_replay.yaml 1.0
```

如果后续用 `--gpus all` / NVIDIA runtime 启动容器，或实现 CPU 解码 fallback，再打开视频 topic：

```bash
./scripts/run_bag_replay.sh /path/to/test.bag build-rscl/config.bag_replay.yaml 1.0
```

默认只回放相关输入：

```text
/sensor/camera/* /sensor/imu /sensor/uss/fused_points /parking/* /sensor/lidar/* /canbus/vehicle_report
```

可通过环境变量覆盖：

```bash
INCLUDED_TOPICS="/sensor/imu /sensor/uss/fused_points /parking/*" \
./scripts/run_bag_replay.sh /path/to/test.bag
```

topic remap 示例：

```bash
REMAP_ARGS="/imu:=/sensor/imu /pdc:=/parking/pdc/pdc_info" \
./scripts/run_bag_replay.sh /path/to/test.bag
```

截取某段回放：

```bash
RELATIVE_START_SEC=30 ./scripts/run_bag_replay.sh /path/to/test.bag
```

## 5. 结果查看

脚本输出日志位置：

- `build-rscl/logs/sentinel_*.log`
- `build-rscl/logs/rsclbag_*.log`
- `build-rscl/logs/bag_info_*.log`
- `build-rscl/events/sentry_events.jsonl`
- `build-rscl/events/event_*/` 或 `.mp4` 录像结果

重点检查：

```bash
grep -E "All modules started|SentryState|Alert|Output|Detector|Video recorded|ERROR|Failed" build-rscl/logs/sentinel_*.log | tail -100
tail -20 build-rscl/events/sentry_events.jsonl
```

## 6. 当前限制

- TensorRT 尚未补齐，检测器会 fallback，bag 联调优先验证 topic、IMU/USS/LiDAR、状态机、事件输出链路。
- 当前服务器/容器没有 NVIDIA GPU 设备透传，视频 NVDEC 链路不能完整验证；先排除 `/sensor/camera/*` 做非视频联调。
- `rscl_monitor` 当前缺 `libncurses.so.5`，先用日志和 `sentry_events.jsonl` 验证输出；必要时再补系统依赖。
- 若视频解码触发 FFmpeg 缺库，优先确认已运行 `scripts/prepare_bag_replay_env.sh` 并 source 生成的 `bag_replay_env.sh`；构建时也会把 `build-rscl*/compat-libs` 写入 RPATH，避免裸启动时 FFmpeg/OpenCV/Cap'n Proto 的间接依赖找不到。仍异常时再按日志补齐对应 `.so` 或先缩小 `INCLUDED_TOPICS` 做 IMU/USS/LiDAR 分项测试。
