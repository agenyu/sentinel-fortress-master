# NvMedia 解码清晰度车端调测指导

## 目标

核查车端 NvMedia 硬解码链路是否正确，区分问题来自 HEVC 源码流、NvMedia 解码输出、NV12 到 BGR 转换、鱼眼去畸变或调试保存。

## 适用环境

- Thor/DRIVE OS 车端或等价 NvMedia 运行环境。
- 已编译 `libsentinel_nvmedia_decoder.so`，优先使用 `bridge/nvmedia_bridge_thor`。
- x86 GPU 云服务器只能验证 `ffmpeg_nvdec`，不能作为 NvMedia 正确性结论。

## 配置

在车端配置文件中显式开启 NvMedia 后端和诊断输出：

```yaml
camera_input_mode: "encoded_hevc"
camera_decoder_backend: "nvmedia"
camera_width: 1920
camera_height: 1536

camera_decode_debug_enabled: true
camera_decode_debug_dir: "/ota/nvmedia_decode_debug"
camera_decode_debug_max_frames_per_camera: 5
camera_decode_debug_save_hevc: true
camera_decode_debug_save_decoded_bgr: true
camera_decode_debug_save_node_images: true
```

设置 bridge 路径：

```bash
export SENTINEL_NVMEDIA_DECODER_LIB=/path/to/libsentinel_nvmedia_decoder.so
export LD_LIBRARY_PATH=/path/to/bridge/lib:$LD_LIBRARY_PATH
```

如车端码流需要特殊 payload 类型，按实际环境设置：

```bash
export SENTINEL_NVMEDIA_CODEC=h265
# 旧 ADHAL bridge 如需 J5 H265 payload：
export SENTINEL_NVMEDIA_PAYLOAD_TYPE=h265_j5
```

## 启动后必须确认的日志

日志中应出现：

```text
[SentinelNode] Camera input mode: encoded_hevc decoder_backend=nvmedia
[SentinelNode][CameraDecodeDebug] enabled dir=...
[NvMediaDecoder] Initialized via bridge: ...
[CameraSubscriber] HEVC decoder backend=nvmedia
```

Thor direct bridge 还应看到类似：

```text
[sentinel_nvmedia_thor] readback surface layout=pitch-linear ... pitch=... offset=...
[NvMediaDecoder][DecodeReturn] ... frame=1920x1536 stride=5760 channels=3
```

如果只看到 `ffmpeg_nvdec` 或 `hevc_cuvid`，说明没有走 NvMedia。

## 诊断输出文件

每路相机会在 `camera_decode_debug_dir/<camera_name>/` 下保存：

- `NNN_seq_<seq>_ts_<ts>_<I|P>.h265`：送入解码器的原始 HEVC 包。
- `NNN_seq_<seq>_ts_<ts>_<I|P>_decoded_bgr.png`：解码器/bridge 输出 BGR。
- `NNN_seq_<seq>_ts_<ts>_node_raw_bgr.png`：进入 SentinelNode 的原始 BGR。
- `NNN_seq_<seq>_ts_<ts>_undistorted_bgr.png`：鱼眼去畸变后的 BGR。

优先取同一路相机的同一 `seq/ts` 做对比。

## 对比方法

1. 用 ffmpeg 离线解码同一个 `.h265` 包，生成参考图。

```bash
ffmpeg -y -i 001_seq_xxx_ts_xxx_I.h265 -frames:v 1 ref.png
```

如果单个包不能独立解码，按时间顺序拼接同一路从首个 I 帧开始的若干包：

```bash
cat 001_*.h265 002_*.h265 003_*.h265 > sample.h265
ffmpeg -y -i sample.h265 -frames:v 5 ref_%03d.png
```

2. 对比 `ref.png` 与 `*_decoded_bgr.png`。

- 二者都模糊：优先查采集/编码码率、QP、镜头焦距和源图质量。
- ref 清晰但 decoded_bgr 模糊或错位：优先查 NvMedia readback、pitch/offset、NV12/NV21 和色彩转换。
- decoded_bgr 清晰但 undistorted_bgr 变糊：优先查鱼眼标定、`cam_K_new` 和 remap 插值。
- undistorted_bgr 清晰但最终截图糊：查画框、resize、JPEG 保存质量或显示链路。

3. 可用 ImageMagick/OpenCV 做局部裁剪和指标对比。

```bash
magick ref.png -crop 400x300+760+560 ref_crop.png
magick 001_seq_xxx_decoded_bgr.png -crop 400x300+760+560 nvmedia_crop.png
```

重点看车牌、树叶、路面颗粒、行人轮廓。

## 常见问题定位

### 没有走 NvMedia

现象：

```text
decoder_backend=ffmpeg_nvdec
hevc_cuvid
```

处理：

- 配置显式设置 `camera_decoder_backend: "nvmedia"`。
- 检查 `SENTINEL_NVMEDIA_DECODER_LIB` 指向的 so 是否存在。
- 检查 `LD_LIBRARY_PATH` 是否包含 bridge 及 NvMedia 依赖库。

### NvMedia 初始化失败

现象：

```text
[NvMediaDecoder] Failed to load bridge ...
[NvMediaDecoder] Bridge failed to create decoder
```

处理：

- `ldd libsentinel_nvmedia_decoder.so` 检查缺失依赖。
- 确认 bridge 和系统架构一致，Thor 车端应为 aarch64。
- 确认 Thor direct bridge 可访问 NvMedia/NvSci 运行库。

### ref 清晰，decoded_bgr 不清晰

重点核查：

- NvMedia 输出是否 pitch-linear。
- Y/UV plane offset 和 pitch 是否按实际值读取。
- NV12/NV21 是否弄反。
- 色彩标准是否匹配，BT.601/BT.709、limited/full range。
- coded size 与 display size/crop 是否一致。

代码风险点：

- `bridge/nvmedia_bridge_adhal/sentinel_nvmedia_adhal_bridge.cpp` 的 `nv12ToBgr()` 假设 NV12 紧密排列，没有 pitch/offset 参数。
- Thor bridge 已有 pitch-linear readback 和 pitch 日志，优先使用 Thor bridge。

### I 帧前后大量丢帧或 P 帧无法解码

现象：

```text
[CameraSubscriber][DecoderWarmup] drop_non_key_before_first_I
[CameraSubscriber][EncodedGap] ... reset_decoder_wait_I=1
[CameraSubscriber][DecodeFail]
```

处理：

- 从首个 I 帧开始采样。
- 确认相机 topic 没有解码队列溢出。
- 降低回放速率或提高解码线程实时性。

## 采集交付物

一次有效调测至少保留：

- Sentinel 日志完整文件。
- `camera_decode_debug_dir` 下每路相机前 5 帧输出。
- 使用的配置文件。
- bridge so 路径和 `ldd` 输出。
- 若怀疑源流问题，保留同一时间段原始 bag 或相机编码 topic 导出。

## 调测完成后的关闭项

诊断输出会保存 PNG 和 HEVC 包，调测完成后务必关闭：

```yaml
camera_decode_debug_enabled: false
```

