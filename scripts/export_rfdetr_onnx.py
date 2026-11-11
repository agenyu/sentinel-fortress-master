#!/usr/bin/env python3
"""Export the trusted RF-DETR Base checkpoint and validate its ONNX contract."""

from __future__ import annotations

import argparse
from pathlib import Path
import shutil

import onnx
from rfdetr import RFDETRBase


def tensor_shape(value_info: onnx.ValueInfoProto) -> list[int | str]:
    dims: list[int | str] = []
    for dim in value_info.type.tensor_type.shape.dim:
        if dim.HasField("dim_value"):
            dims.append(dim.dim_value)
        elif dim.HasField("dim_param"):
            dims.append(dim.dim_param)
        else:
            dims.append("?")
    return dims


def main() -> int:
    repo_root = Path(__file__).resolve().parent.parent
    parser = argparse.ArgumentParser(description="Export RF-DETR Base checkpoint to static ONNX")
    parser.add_argument("--checkpoint", type=Path, default=repo_root / "models" / "rfdetr_base.pt")
    parser.add_argument("--output", type=Path, default=repo_root / "models" / "rfdetr_base.onnx")
    parser.add_argument("--opset", type=int, default=17)
    args = parser.parse_args()

    if not args.checkpoint.is_file():
        raise FileNotFoundError(f"RF-DETR checkpoint not found: {args.checkpoint}")
    args.output.parent.mkdir(parents=True, exist_ok=True)

    model = RFDETRBase(
        pretrain_weights=str(args.checkpoint),
        device="cpu",
        trust_checkpoint=True,
    )
    if model.model_config.resolution != 560 or model.model_config.num_queries != 300:
        raise RuntimeError(
            "Unexpected RF-DETR Base contract: "
            f"resolution={model.model_config.resolution}, queries={model.model_config.num_queries}"
        )
    head_shape = tuple(model.model.model.class_embed.weight.shape)
    if head_shape[0] != 91:
        raise RuntimeError(f"Expected 91 RF-DETR Base COCO logit slots, got head={head_shape}")

    exported = Path(
        model.export(
            output_dir=str(args.output.parent),
            opset_version=args.opset,
            verbose=False,
            shape=(560, 560),
            batch_size=1,
            dynamic_batch=False,
        )
    )
    if exported.resolve() != args.output.resolve():
        shutil.move(str(exported), str(args.output))

    graph = onnx.load(str(args.output), load_external_data=True)
    onnx.checker.check_model(graph)
    inputs = [(value.name, tensor_shape(value)) for value in graph.graph.input]
    outputs = [(value.name, tensor_shape(value)) for value in graph.graph.output]
    if inputs != [("input", [1, 3, 560, 560])]:
        raise RuntimeError(f"Unexpected ONNX inputs: {inputs}")
    output_shapes = {name: shape for name, shape in outputs}
    if output_shapes.get("dets") != [1, 300, 4] or output_shapes.get("labels") != [1, 300, 91]:
        raise RuntimeError(f"Unexpected ONNX outputs: {outputs}")

    print(f"Saved and validated ONNX: {args.output}")
    print(f"Input: {inputs}")
    print(f"Outputs: {outputs}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
