#!/usr/bin/env python3
"""Build a static TensorRT engine from an ONNX model."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys


def import_tensorrt():
    try:
        import tensorrt as trt  # type: ignore
        return trt
    except ModuleNotFoundError:
        import tensorrt_bindings as trt  # type: ignore
        return trt


def main() -> int:
    repo_root = Path(__file__).resolve().parent.parent
    default_onnx = repo_root / "models" / "rfdetr_base.onnx"
    default_engine = repo_root / "models" / "rfdetr_base_fp16.engine"
    parser = argparse.ArgumentParser(description="Build TensorRT engine from ONNX")
    parser.add_argument(
        "--onnx",
        default=default_onnx,
        type=Path,
        help=f"Input ONNX file (default: {default_onnx})",
    )
    parser.add_argument(
        "--engine",
        default=default_engine,
        type=Path,
        help=f"Output engine file (default: {default_engine})",
    )
    parser.add_argument(
        "--workspace-mb",
        type=int,
        default=4096,
        help="TensorRT workspace memory pool size in MiB",
    )
    parser.add_argument(
        "--fp16",
        action="store_true",
        help="Enable FP16 kernels while keeping FP32 input/output bindings.",
    )
    parser.add_argument(
        "--input-shape",
        default=None,
        help="Optional static input shape override, for example images:1x3x560x560.",
    )
    args = parser.parse_args()

    if not args.onnx.is_file():
        print(f"ONNX file not found: {args.onnx}", file=sys.stderr)
        return 2

    trt = import_tensorrt()
    logger = trt.Logger(trt.Logger.INFO)
    builder = trt.Builder(logger)
    flags = 1 << int(trt.NetworkDefinitionCreationFlag.EXPLICIT_BATCH)
    network = builder.create_network(flags)
    parser = trt.OnnxParser(network, logger)

    if not parser.parse(args.onnx.read_bytes()):
        print("TensorRT ONNX parse failed:", file=sys.stderr)
        for idx in range(parser.num_errors):
            print(parser.get_error(idx), file=sys.stderr)
        return 3

    if args.input_shape:
        name, _, shape_text = args.input_shape.partition(":")
        if not name or not shape_text:
            print("--input-shape must look like name:1x3x560x560", file=sys.stderr)
            return 2
        dims = tuple(int(part) for part in shape_text.lower().split("x"))
        profile = builder.create_optimization_profile()
        profile.set_shape(name, dims, dims, dims)
        config_profile = profile
    else:
        config_profile = None

    config = builder.create_builder_config()
    config.set_memory_pool_limit(
        trt.MemoryPoolType.WORKSPACE,
        int(args.workspace_mb) * 1024 * 1024,
    )
    if args.fp16 and builder.platform_has_fast_fp16:
        config.set_flag(trt.BuilderFlag.FP16)
    elif args.fp16:
        print("Warning: requested FP16 but platform_has_fast_fp16 is false", file=sys.stderr)
    if config_profile is not None:
        config.add_optimization_profile(config_profile)

    print(f"TensorRT version: {trt.__version__}")
    print(f"Network inputs: {network.num_inputs}, outputs: {network.num_outputs}")
    for idx in range(network.num_inputs):
        tensor = network.get_input(idx)
        print(f"  input[{idx}] {tensor.name}: shape={tensor.shape} dtype={tensor.dtype}")
    for idx in range(network.num_outputs):
        tensor = network.get_output(idx)
        print(f"  output[{idx}] {tensor.name}: shape={tensor.shape} dtype={tensor.dtype}")

    serialized = builder.build_serialized_network(network, config)
    if serialized is None:
        print("TensorRT engine build failed", file=sys.stderr)
        return 4

    args.engine.parent.mkdir(parents=True, exist_ok=True)
    args.engine.write_bytes(bytes(serialized))
    print(f"Saved engine: {args.engine} ({args.engine.stat().st_size / (1024 * 1024):.1f} MiB)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
