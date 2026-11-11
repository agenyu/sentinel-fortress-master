# RF-DETR Base 模型与 TensorRT 产物说明

`feature/rfdetr-medium-direct` 当前实际使用 RF-DETR Base，而不是 Medium。哨兵模式和单车/多车堡垒模式共用同一检测器和 Engine。

| 文件 | SHA256 | 用途 |
| --- | --- | --- |
| `rfdetr_base.pt` | `d8f70210e425a4a4234d547737f57500bcc4ac24a333b99e33d5a371e0b80f` | RF-DETR Base PyTorch 检查点，用于重新导出 ONNX |
| `rfdetr_base.onnx` | `a1ecefdd0490db89639cde1d23e91bfb8d95c9aa831683c8c4dacade5e4a2ada` | 静态 FP32 ONNX，用于在目标平台生成 Engine |
| `rfdetr_base_labels.txt` | — | 91 个 raw logit 槽位到 COCO/下游语义的映射 |
| `rfdetr_base_fp16.engine` | 不提交 | GPU/CUDA/TensorRT 平台本地生成的运行文件 |

## 模型契约

- 输入：`input`, FP32, `[1,3,560,560]`
- 输出框：`dets`, FP32, `[1,300,4]`，归一化 `cx,cy,w,h`
- 输出分类：`labels`, FP32, `[1,300,91]`，raw logits
- 后处理：sigmoid 后在全部 query/class 对上做全局 top-300
- 类别：COCO 稀疏 category ID 显式映射为下游连续 ID，保持 `person=0`、`car/vehicle=2`

RF-DETR Base 不包含原 YOLO 自定义的 `wolf/tiger/leopard` 三类。如需区分这些类别，必须训练 RF-DETR 定制检查点。

## 导出 ONNX

```bash
/home/aloys/sentinel/runtime-env/rfdetr-export/bin/python \
  scripts/export_rfdetr_onnx.py
```

导出脚本会校验 560 输入分辨率、300 query、91 分类槽位和 ONNX 张量契约。

## 生成 Engine

```bash
source /home/aloys/sentinel/runtime-env/tensorrt-10.13.3.9/activate.sh
LD_LIBRARY_PATH=/home/aloys/sentinel/runtime-env/model-export-8.4.62/lib/python3.10/site-packages/tensorrt_libs:${LD_LIBRARY_PATH:-} \
  /home/aloys/sentinel/runtime-env/model-export-8.4.62/bin/python \
  scripts/build_trt_engine.py --fp16
```

`build/config.yaml` 默认引用 `../models/rfdetr_base_fp16.engine`。RTX 3070 本机生成的 Engine 不能直接用在 Thor；Thor 必须在车端 TensorRT/CUDA 环境重建。
