#!/usr/bin/env python3
"""Export complete encoded camera streams from an RSCL bag."""

import argparse
import json
import os
import statistics
from pathlib import Path

RSCL_BASE = Path("/opt/senseauto_active/senseauto-rscl")
THIRD_PARTY = Path("/opt/senseauto_active/senseauto-3rdparty/3rdparty")
CAMERAS = ("front_camera_fov195", "rear_camera_fov195",
           "left_camera_fov195", "right_camera_fov195")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag", type=Path)
    parser.add_argument("output_dir", type=Path)
    args = parser.parse_args()

    os.environ["PYTHONPATH"] = f"{RSCL_BASE}/python:" + os.environ.get("PYTHONPATH", "")
    os.environ["LD_LIBRARY_PATH"] = ":".join([
        str(RSCL_BASE / "lib"), str(THIRD_PARTY / "protobuf/lib"),
        str(THIRD_PARTY / "lib"), os.environ.get("LD_LIBRARY_PATH", "")])
    import sys
    sys.path.insert(0, str(RSCL_BASE / "python"))
    import ad_bag_py

    topics = {f"/sensor/camera/{name}/encode": name for name in CAMERAS}
    args.output_dir.mkdir(parents=True, exist_ok=True)
    streams = {name: (args.output_dir / f"{name}.h265").open("wb") for name in CAMERAS}
    stats = {name: {"timestamps_ns": [], "frames": 0, "bytes": 0,
                    "skipped_before_keyframe": 0, "started": False}
             for name in CAMERAS}

    try:
        reader = ad_bag_py.BagReader(str(args.bag), included_channels=list(topics))
        while True:
            message = reader.read_next_message()
            if message is None:
                break
            camera = topics[message.channel_name]
            obj = message.message_obj
            item = stats[camera]
            if not item["started"]:
                if int(obj.frameType) != 2:
                    item["skipped_before_keyframe"] += 1
                    continue
                item["started"] = True
            payload = bytes(obj.data)
            streams[camera].write(payload)
            item["timestamps_ns"].append(int(obj.header.time.nanoSec))
            item["frames"] += 1
            item["bytes"] += len(payload)
    finally:
        for stream in streams.values():
            stream.close()

    manifest = {"bag": str(args.bag), "cameras": {}}
    for camera, item in stats.items():
        timestamps = item.pop("timestamps_ns")
        deltas = [(b - a) / 1e9 for a, b in zip(timestamps, timestamps[1:]) if b > a]
        median_delta = statistics.median(deltas) if deltas else 0.0
        duration = (timestamps[-1] - timestamps[0]) / 1e9 if len(timestamps) > 1 else 0.0
        manifest["cameras"][camera] = {
            **item,
            "first_timestamp_ns": timestamps[0] if timestamps else 0,
            "last_timestamp_ns": timestamps[-1] if timestamps else 0,
            "duration_s": round(duration, 6),
            "median_delta_ms": round(median_delta * 1000, 3),
            "nominal_fps": round(1.0 / median_delta, 3) if median_delta > 0 else 0.0,
        }
    (args.output_dir / "manifest.json").write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(manifest, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
