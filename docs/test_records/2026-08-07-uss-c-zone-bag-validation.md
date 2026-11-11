# 0806 摇晃 bag：USS 实数与 ORSM C 区测距验证

## 数据包

- 文件：`/home/aloys/文档/bag'/0806_1613_yaohuang.rsclbag.000.rsclbag`
- 时长：30.186949 s
- USS topic：`/sensor/uss/fused_points`
- 消息类型：`UssFusedStaticMap`
- USS 帧数：2997（约 100 Hz）

## 原始 USS 核查

使用 `tools/uss_data_inspector.cpp` 对 USS 消息逐槽统计：

- 每帧 `fusedPointsSize=0`，但 Cap'n Proto list 长度固定为 150；生产端没有填写有效数量字段。
- 449550 个槽中，446338 个为全零，2997 个为 `(-2000,-2000)` 填充值。
- 215 个有效槽均为 `existProb=91`；其余概率字段及 point timestamp/age/type 为 0。
- 有效原始坐标集中在 `(-1487,-1972)` 附近，原始径向范围 2466.62～2472.20。
- 若按旧代码的 cm 比例解析，距离为 24.67～24.72 m；按 mm 解析为 2.4666～2.4722 m。结合 `int16` 坐标、±2000 填充边界及同类历史样本，本包按 mm 解释。
- 有效点仅出现在 215 帧中，呈多个约 80 ms 的短时间段，不是配置中的固定 1.3 m 数值。

## 解析修正

- `uss_fused_coordinate_scale_m` 默认设为 `0.001`，允许按数据源配置。
- `fusedPointsSize>0` 时只扫描声明范围；等于 0 时兼容扫描固定 list。
- 仅接收 `existProb >= uss_fused_min_exist_prob`（默认 50）的槽，避免将全零和 `(-2000,-2000)` 解析成障碍物。
- 空帧会更新为空 USS 快照，不延用无效填充值。

## ORSM C 区处理

- `enable_orsm_uss_distance=true` 时，USS 只与已有视觉目标关联，不独立生成入侵物。
- 只使用时间差不超过 `uss_fusion_max_age_ms` 且距离不超过 `orsm.zone_c_radius` 的 USS 点。
- 命中后以 USS 径向距离作为 C 区距离，目标方位仍沿用视觉/LiDAR 方位；输出标记为 `distance_source=orsm_c_zone_uss`。

## 整包回放结果

配置：`build-diagnostic/config.replay.yaml`，1.0 倍速完整回放。

- 1300 个图像处理帧，整包播放完成，无崩溃。
- USS-视觉融合日志 101 条；最终 ORSM USS 距离输出 99 帧，均落到稳定入侵物 ID 1。
- ORSM USS 输出距离均为 2.47 m，输出位置径向误差最大约 0.009 m。
- 对应状态均为 C 区 `zone=3`、`warn_level=3`。
- 未再出现旧解析产生的 24.7 m 或填充值产生的 28.284 m USS 距离。
- 单元测试 `orsm_response_feedback_test`、`orsm_zone_behavior_test` 均通过。

回放日志：`build-diagnostic/logs/sentinel_20260807_234436.log`。
