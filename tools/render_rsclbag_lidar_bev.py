#!/usr/bin/env python3
"""Render four raw RSCL LiDAR point clouds into a vehicle-coordinate BEV MP4."""
import argparse
import os
import sys
from collections import deque
from pathlib import Path

import numpy as np

RSCL_PYTHON = "/opt/senseauto_active/senseauto-rscl/python"
TOPICS = {
    "/sensor/lidar/front_lidar": ("FRONT", (80, 210, 255)),
    "/sensor/lidar/left_lidar": ("LEFT", (90, 255, 90)),
    "/sensor/lidar/rear_lidar": ("REAR", (255, 110, 80)),
    "/sensor/lidar/right_lidar": ("RIGHT", (255, 90, 255)),
}


def points(raw: bytes, point_step: int) -> np.ndarray:
    """Parse robosense_aligned16 x/y/z int16 centimetres from a point cloud."""
    if point_step < 16:
        return np.empty((0, 3), np.float32)
    count = len(raw) // point_step
    records = np.frombuffer(raw, dtype=np.uint8, count=count * point_step).reshape(count, point_step)
    xyz_cm = records[:, :6].copy().view('<i2').reshape(count, 3)
    xyz = xyz_cm.astype(np.float32) * 0.01
    finite = np.isfinite(xyz).all(axis=1)
    distance = np.hypot(xyz[:, 0], xyz[:, 1])
    return xyz[finite & (distance >= 0.5) & (distance <= 50.0) &
               (xyz[:, 2] >= -2.5) & (xyz[:, 2] <= 3.0)]


def render_frame(clouds, timestamp_ns: int, origin_ns: int, width: int, height: int, metres: float):
    import cv2
    image = np.full((height, width, 3), 18, dtype=np.uint8)
    cx, cy = width // 2, height // 2
    scale = min(width, height) / (2.0 * metres)
    for radius in (5, 10, 20, 30, 40):
        if radius > metres:
            continue
        px = int(radius * scale)
        cv2.circle(image, (cx, cy), px, (58, 58, 58), 1, cv2.LINE_AA)
        cv2.putText(image, f"{radius}m", (cx + 4, cy - px - 3), cv2.FONT_HERSHEY_SIMPLEX,
                    0.36, (150, 150, 150), 1, cv2.LINE_AA)
    cv2.line(image, (cx, 0), (cx, height), (65, 65, 65), 1)
    cv2.line(image, (0, cy), (width, cy), (65, 65, 65), 1)

    for name, colour, xyz in clouds:
        # Vehicle convention: x forward, y left.  Image: up is forward.
        u = np.rint(cx - xyz[:, 1] * scale).astype(np.int32)
        v = np.rint(cy - xyz[:, 0] * scale).astype(np.int32)
        keep = (u >= 0) & (u < width) & (v >= 0) & (v < height)
        image[v[keep], u[keep]] = colour
    car = np.array([[cx, cy - 25], [cx - 13, cy + 22], [cx + 13, cy + 22]], np.int32)
    cv2.fillConvexPoly(image, car, (230, 230, 230))
    cv2.putText(image, "RAW LiDAR point cloud | x forward, y left", (18, 28),
                cv2.FONT_HERSHEY_SIMPLEX, 0.65, (240, 240, 240), 2, cv2.LINE_AA)
    labels = "  ".join(f"{name}" for name, _, _ in clouds)
    cv2.putText(image, labels, (18, height - 20), cv2.FONT_HERSHEY_SIMPLEX,
                0.55, (220, 220, 220), 1, cv2.LINE_AA)
    elapsed = (timestamp_ns - origin_ns) / 1e9
    cv2.putText(image, f"t={elapsed:05.2f}s", (width - 155, 28), cv2.FONT_HERSHEY_SIMPLEX,
                0.55, (240, 240, 240), 1, cv2.LINE_AA)
    return image


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--range-m", type=float, default=40.0)
    args = parser.parse_args()
    sys.path.insert(0, RSCL_PYTHON)
    import ad_bag_py
    import cv2

    reader = ad_bag_py.BagReader(str(args.bag), included_channels=list(TOPICS))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    writer = cv2.VideoWriter(str(args.output), cv2.VideoWriter_fourcc(*"mp4v"), 10.0, (1024, 1024))
    if not writer.isOpened():
        raise RuntimeError(f"cannot open video writer: {args.output}")
    latest, frames, origin = {}, 0, None
    try:
        while True:
            message = reader.read_next_message()
            if message is None:
                break
            topic = message.channel_name
            obj = message.message_obj
            ts = int(obj.header.time.nanoSec)
            cloud = points(bytes(obj.data), int(obj.pointStep))
            latest[topic] = (ts, cloud)
            # Front LiDAR is the timeline clock, one raw BEV frame per scan.
            if topic != "/sensor/lidar/front_lidar" or not cloud.size:
                continue
            if origin is None:
                origin = ts
            coloured = []
            for current_topic, (name, colour) in TOPICS.items():
                sample = latest.get(current_topic)
                if sample is not None and abs(sample[0] - ts) <= 150_000_000 and sample[1].size:
                    coloured.append((name, colour, sample[1]))
            writer.write(render_frame(coloured, ts, origin, 1024, 1024, args.range_m))
            frames += 1
    finally:
        writer.release()
    if frames == 0:
        raise RuntimeError("no front-LiDAR frames were rendered")
    print(f"rendered_frames={frames} output={args.output}")


if __name__ == "__main__":
    main()
