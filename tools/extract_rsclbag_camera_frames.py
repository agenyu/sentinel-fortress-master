#!/usr/bin/env python3
"""Extract nearest camera frames from an RSCL bag without replay/network/GPU."""

import argparse
import json
import os
import subprocess
from pathlib import Path

RSCL_BASE = Path("/opt/senseauto_active/senseauto-rscl")
THIRD_PARTY = Path("/opt/senseauto_active/senseauto-3rdparty/3rdparty")
CAMERAS = ("front_camera_fov195", "rear_camera_fov195",
           "left_camera_fov195", "right_camera_fov195")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag", type=Path)
    parser.add_argument("output_dir", type=Path)
    parser.add_argument("timestamps_ns", nargs="+", type=int)
    parser.add_argument("--decoder", type=Path, required=True)
    args = parser.parse_args()

    os.environ["PYTHONPATH"] = f"{RSCL_BASE}/python:" + os.environ.get("PYTHONPATH", "")
    os.environ["LD_LIBRARY_PATH"] = ":".join([
        str(RSCL_BASE / "lib"), str(THIRD_PARTY / "protobuf/lib"),
        str(THIRD_PARTY / "lib"), os.environ.get("LD_LIBRARY_PATH", "")])
    import sys
    sys.path.insert(0, str(RSCL_BASE / "python"))
    import ad_bag_py

    topics = {f"/sensor/camera/{name}/encode": name for name in CAMERAS}
    reader = ad_bag_py.BagReader(str(args.bag), included_channels=list(topics))
    frames = {name: [] for name in CAMERAS}
    while True:
        message = reader.read_next_message()
        if message is None:
            break
        obj = message.message_obj
        name = topics[message.channel_name]
        frames[name].append({
            "bag_timestamp_ns": int(message.timestamp),
            "camera_timestamp_ns": int(obj.header.time.nanoSec),
            "seq": int(obj.header.seq),
            "frame_type": int(obj.frameType),
            "data": bytes(obj.data),
        })

    args.output_dir.mkdir(parents=True, exist_ok=True)
    manifest = []
    for scene_index, requested_ts in enumerate(args.timestamps_ns, 1):
        scene_dir = args.output_dir / f"scene_{scene_index:02d}_{requested_ts}"
        scene_dir.mkdir(exist_ok=True)
        for camera, camera_frames in frames.items():
            target_index = min(range(len(camera_frames)),
                               key=lambda i: abs(camera_frames[i]["camera_timestamp_ns"] - requested_ts))
            key_index = target_index
            while key_index > 0 and camera_frames[key_index]["frame_type"] != 2:
                key_index -= 1
            if camera_frames[key_index]["frame_type"] != 2:
                raise RuntimeError(f"no preceding key frame: {camera} target={requested_ts}")
            segment = scene_dir / f"{camera}.h265"
            with segment.open("wb") as stream:
                for item in camera_frames[key_index:target_index + 1]:
                    stream.write(item["data"])
            output = scene_dir / f"{camera}.jpg"
            # BagReader 使用 SenseAuto 库，独立解码器固定使用 Ubuntu FFmpeg/OpenCV。
            decoder_env = os.environ.copy()
            decoder_env["LD_LIBRARY_PATH"] = "/usr/lib/x86_64-linux-gnu:/lib/x86_64-linux-gnu"
            subprocess.run([str(args.decoder), str(segment), str(output)],
                           check=True, env=decoder_env)
            target = camera_frames[target_index]
            manifest.append({
                "scene": scene_index, "requested_timestamp_ns": requested_ts,
                "camera": camera, "seq": target["seq"],
                "camera_timestamp_ns": target["camera_timestamp_ns"],
                "delta_ms": round((target["camera_timestamp_ns"] - requested_ts) / 1e6, 3),
                "image": str(output.relative_to(args.output_dir)),
            })
            segment.unlink()
    (args.output_dir / "manifest.json").write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
