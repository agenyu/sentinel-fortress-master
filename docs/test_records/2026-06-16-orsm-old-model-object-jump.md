# 2026-06-16 ORSM 旧模型报警对象跳变排查记录

## 背景

用户反馈：旧模型堡垒模式 bag 包仍有报警对象跳变问题。

本次按“先固定旧模型变量，再验证 ORSM 主对象稳定化”的策略处理：

- 已回退 `detection/detector.cpp` 中为新模型增加的 `wolf/tiger/leopard` 类别映射。
- 暂不替换新模型，因为新模型存在漏报警风险。
- 本轮只保留 ORSM 输出对象稳定化补丁，避免模型类别变化干扰旧 bag 回归。

## 根因判断

ORSM 输出层通常把 `intruders_info[0]` 作为当前报警对象展示。原逻辑已经做了同 `track_id` 保持，但旧模型/旧 bag 下 tracker 容易在遮挡、丢帧、多目标重识别时重建 ID。

当上一帧主报警对象的 `track_id` 消失后，原逻辑会重新按“报警区域等级、距离、ID”选择主对象，导致前端看到报警对象跳变。

此外，旧 COCO 模型会输出大量非安防类别；如果 `INTRUDER_UNKNOWN` 参与 ORSM 候选，也会把杂物框混入报警对象列表，放大跳变。

## 本次代码处理

文件：`orsm/single_orsm_manager.cpp`

1. 主报警对象稳定化增强
   - 优先保持上一帧同 `intruder_id`。
   - 若 ID 已重建，则按车体坐标寻找与上一帧主对象空间最近、且在锁定距离内的候选，继续作为 `intruders_info[0]`。
   - 只有上一主对象确实不存在/距离过远时，才回退到“更高区域、更近距离、更小 ID”的重选逻辑。

2. UNKNOWN 类别过滤
   - ORSM 安防目标限定为人/动物。
   - `INTRUDER_UNKNOWN` 不再进入报警候选，避免旧模型杂物框影响报警对象。

## 已执行验证

- `git diff --check`：通过。
- 本地最小 C++ 回归：通过。
  - 覆盖同 ID 保持。
  - 覆盖 ID 重建但空间接近时保持同一物理主目标。
  - 覆盖原目标消失后按最高等级/最近距离重选。

## 未完成验证 / 当前阻塞

- 本机完整 CMake 编译受限：缺少 SenseAuto `ADBuildTools`。
- GPU 服务器 `101.37.131.214` 当前 SSH 连接超时/不可用。
- 120 服务器有 RSCL SDK，但系统 CMake 为 3.10，低于项目要求 3.16；本轮未修改服务器环境。
- 120 服务器未发现可直接用于本问题的完整旧模型 ORSM bag 包。

## 后续建议

1. GPU 服务器恢复后，优先同步当前源码到临时目录，使用旧模型和问题 bag 跑 ORSM 回归。
2. 回归重点统计连续 WARNING 期间 `intruders_info[0].intruder_id` 与 `intruder_pose_x/y`，确认主报警对象不再跨物理目标跳变。
3. 新模型漏报警单独建项评估，不与旧模型对象跳变问题混测。

## 新模型漏报警后续项

暂不替换新模型。待本轮旧模型问题闭环后，单独评估：

- 新模型漏报警场景清单；
- 类别映射是否仍需要 `wolf/tiger/leopard` 等扩展；
- 新旧模型在同一批 bag 下的召回率、误报率、对象稳定性对比。

## 120 RSCL 容器补充验证（11:25-11:40）

GPU 服务器仍然 SSH 超时，按降级策略使用 120 服务器的 `rscl-u2004` 容器验证。

### 编译验证

- 本地当前源码同步到容器临时目录：`/root/sentinel_orsm_jump_verify_20260616_112544/src`
- 容器环境：Ubuntu 20.04，CMake 3.16.3，RSCL/SenseAuto SDK 可用。
- CMake 配置通过。
- 编译通过：`[100%] Built target sentinel_perception`
- 链接阶段出现 FFmpeg 旧依赖 warning（`libvpx.so.3` 等），运行时已用既有 `compat-libs` 补齐。

### Bag 回放尝试

容器仅找到一个可用 bag：

- `/root/sentinel_bags/0429_2051_record_data.rsclbag.000.rsclbag`

该 bag 包含四路相机与 USS，但不包含本问题所需的 ORSM/LiDAR 入侵目标验证数据，也没有可用 `yolov8n.engine` 检测模型。

执行两轮回放：

1. 包含 `/canbus/vehicle_report`
   - 回放完成。
   - ORSM 被车辆状态拉回 `state=0`，无报警。
2. 排除 `/canbus/vehicle_report`，使用默认 ODD 强制 ORSM
   - 回放完成。
   - ORSM 进入 `state=1`，但 `intruders=0 / warn_level=0`。
   - 原因：容器内无 TensorRT `.engine` 模型，检测器无法产生视觉目标；该 bag 也不是问题入侵 bag。

### 结论

- 本次 ORSM 稳定化补丁已完成 RSCL 容器级编译验证。
- 120 容器现有 bag 可验证运行链路，但不能覆盖“旧模型堡垒模式报警对象跳变”。
- 最终问题闭环仍需要 GPU 服务器恢复后，使用旧 `yolov8n.engine`/问题 ORSM bag 复测。

## 科研 GPU 旧模型 fortress bag 复测（12:10-12:25）

按用户要求改用科研 GPU 服务器验证。

### 环境

- 服务器：科研论文1 GPU服务器，RTX 4090，Ubuntu 22.04
- 临时目录：`/root/sentinel_orsm_jump_verify_20260616_114345/src`
- 模型：`/root/sentinel_model_eval_20260615_2303/src/models/yolov8n.engine`
- Bag：`/root/sentinel_model_eval_20260615_2303/bags/fortress/0613_1043_fortress.rsclbag.000.rsclbag`
- 回放 topics：`/sensor/camera/* /sensor/lidar/*`
- CMake/GPU 编译：通过，`[100%] Built target sentinel_perception`

### 关键发现

1. 最初加入的 `INTRUDER_UNKNOWN` 过滤不可保留。
   - 未打补丁历史源码在该 fortress bag 上能触发 ORSM warning；报警目标大量为 `type=0`（`INTRUDER_UNKNOWN`）。
   - 如果过滤 UNKNOWN，当前旧模型 fortress bag 会变成 `state=1 / warn_level=0 / intruders=0`，等价于漏报警。
   - 因此最终补丁撤销 UNKNOWN 过滤，保留旧模型报警能力。

2. `track_id` 在旧模型/多相机链路下不能单独作为主目标稳定依据。
   - 日志中同一个 `#0` 会对应不同车辆坐标。
   - 因此最终补丁要求“同 ID 也必须空间接近”才视为同一物理主目标。

3. 原跳变模式复现：
   - 未稳定/中间版本会在近处 `#0 zone3 pos≈(2.9,0.16)` 和远处 `#5 zone2 pos≈(5.7,-1.6)` 间切换。
   - 这符合前端报警对象跳变现象。

### 最终补丁效果

最终补丁保留 UNKNOWN 报警，并在 WARNING hold 窗口内锁定上一主目标：

- 重新编译通过。
- 旧模型 fortress bag 回放完成。
- 10Hz ORSM warning 输出中，远处 `#5 zone2 pos≈(5.7,-1.6)` 不再作为主报警对象输出。
- 剩余主对象坐标均在近处 zone3 约 1～2m 范围内，属于同一近场目标/检测框抖动，不再出现近/远目标来回跳。

最新日志：

- `/root/sentinel_orsm_jump_verify_20260616_114345/src/build-gpu/logs/sentinel_20260616_122317.log`
