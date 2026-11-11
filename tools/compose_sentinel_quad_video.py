#!/usr/bin/env python3
"""Compose four timestamp-aligned sentry-camera overlays into one MP4."""

from __future__ import annotations

import argparse
import re
import subprocess
from pathlib import Path


CAMERAS = (
    ("front_camera_fov195", "FRONT"),
    ("rear_camera_fov195", "REAR"),
    ("left_camera_fov195", "LEFT"),
    ("right_camera_fov195", "RIGHT"),
)

CAMERA_RE = re.compile(r"\[Camera\] camera=(\S+) seq=(\d+) ts=(\d+)")
IMAGE_RE = re.compile(r"(.+)_seq_(\d+)(?:_ALERT)?\.jpg$")


def concat_quote(path: Path) -> str:
    return str(path.resolve()).replace("'", "'\\''")


def load_timestamps(log_path: Path) -> dict[tuple[str, int], int]:
    result: dict[tuple[str, int], int] = {}
    with log_path.open("r", encoding="utf-8", errors="replace") as stream:
        for line in stream:
            for match in CAMERA_RE.finditer(line):
                sequence = int(match.group(2))
                timestamp = int(match.group(3))
                # 线程交错的坏行会把 ts 数字串进 seq、把残缺值留给 ts；
                # 只接受 19 位 epoch ns（约 1e18~2e18）与合理 seq。
                if (not (1_000_000_000_000_000_000 <= timestamp < 2_000_000_000_000_000_000)
                        or sequence > 10_000_000):
                    continue
                result[(match.group(1), sequence)] = timestamp
    return result


def load_images(frames_dir: Path) -> dict[tuple[str, int], Path]:
    # When two diagnostic runs produced both normal and _ALERT versions of the
    # same sequence, use the most recently generated file.
    result: dict[tuple[str, int], Path] = {}
    for path in frames_dir.glob("*.jpg"):
        match = IMAGE_RE.fullmatch(path.name)
        if not match:
            continue
        key = (match.group(1), int(match.group(2)))
        previous = result.get(key)
        if previous is None or path.stat().st_mtime_ns > previous.stat().st_mtime_ns:
            result[key] = path
    return result


def write_concat_list(
    path: Path,
    frames: list[tuple[int, Path]],
    global_start: int,
    global_end: int,
) -> None:
    if not frames:
        raise RuntimeError(f"No matched frames for {path.stem}")

    lines = ["ffconcat version 1.0"]
    for index, (timestamp, image_path) in enumerate(frames):
        if index == 0:
            next_timestamp = frames[index + 1][0] if len(frames) > 1 else global_end
            duration_ns = next_timestamp - global_start
        elif index + 1 < len(frames):
            duration_ns = frames[index + 1][0] - timestamp
        else:
            duration_ns = global_end - timestamp
        duration_s = max(duration_ns / 1_000_000_000, 0.001)
        lines.append(f"file '{concat_quote(image_path)}'")
        lines.append(f"duration {duration_s:.9f}")

    # The concat demuxer ignores the final duration unless the final file is
    # repeated. Repeating it also holds the last panel until the global end.
    lines.append(f"file '{concat_quote(frames[-1][1])}'")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def write_direction_labels(path: Path, duration_s: float) -> None:
    labels = (
        ("FRONT", 620, 14),
        ("REAR", 1260, 14),
        ("LEFT", 620, 526),
        ("RIGHT", 1260, 526),
    )
    end_seconds = max(int(duration_s) + 1, 1)
    events = [
        f"Dialogue: 0,0:00:00.00,0:00:{end_seconds:02d}.00,Direction,,0,0,0,,"
        f"{{\\an9\\pos({x},{y})}}{label}"
        for label, x, y in labels
    ]
    content = """[Script Info]
ScriptType: v4.00+
PlayResX: 1280
PlayResY: 1024
ScaledBorderAndShadow: yes

[V4+ Styles]
Format: Name,Fontname,Fontsize,PrimaryColour,SecondaryColour,OutlineColour,BackColour,Bold,Italic,Underline,StrikeOut,ScaleX,ScaleY,Spacing,Angle,BorderStyle,Outline,Shadow,Alignment,MarginL,MarginR,MarginV,Encoding
Style: Direction,DejaVu Sans,27,&H00FFFFFF,&H000000FF,&H90000000,&H90000000,-1,0,0,0,100,100,0,0,3,1,0,9,10,10,10,1

[Events]
Format: Layer,Start,End,Style,Name,MarginL,MarginR,MarginV,Effect,Text
""" + "\n".join(events) + "\n"
    path.write_text(content, encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--log", required=True, type=Path)
    parser.add_argument("--frames", required=True, type=Path)
    parser.add_argument("--ffmpeg", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--fps", type=int, default=20)
    parser.add_argument("--timeline-start-ns", type=int,
                        help="Optional bag timeline start; preserves leading hold time.")
    parser.add_argument("--timeline-duration-ms", type=int,
                        help="Optional bag timeline duration; preserves trailing hold time.")
    args = parser.parse_args()

    timestamps = load_timestamps(args.log)
    images = load_images(args.frames)
    matched: dict[str, list[tuple[int, Path]]] = {}
    for camera, _ in CAMERAS:
        camera_frames = [
            (timestamp, images[(camera, sequence)])
            for (logged_camera, sequence), timestamp in timestamps.items()
            if logged_camera == camera and (camera, sequence) in images
        ]
        matched[camera] = sorted(camera_frames)

    all_frames = [frame for frames in matched.values() for frame in frames]
    if not all_frames:
        raise RuntimeError("No logged camera frames matched the saved images")
    observed_start = min(timestamp for timestamp, _ in all_frames)
    observed_end = max(timestamp for timestamp, _ in all_frames)
    global_start = args.timeline_start_ns if args.timeline_start_ns is not None else observed_start
    global_end = (
        global_start + args.timeline_duration_ms * 1_000_000
        if args.timeline_duration_ms is not None
        else observed_end
    )
    if global_start > observed_start or global_end < observed_end:
        raise RuntimeError("Requested timeline does not cover all matched frames")

    args.output.parent.mkdir(parents=True, exist_ok=True)
    list_dir = args.output.parent / ".sentinel_quad_concat"
    list_dir.mkdir(parents=True, exist_ok=True)
    list_paths: list[Path] = []
    for camera, _ in CAMERAS:
        list_path = list_dir / f"{camera}.ffconcat"
        write_concat_list(
            list_path, matched[camera], global_start, global_end
        )
        list_paths.append(list_path)

    duration_s = (global_end - global_start) / 1_000_000_000
    label_path = list_dir / "direction_labels.ass"
    write_direction_labels(label_path, duration_s)

    filters: list[str] = []
    for index, _ in enumerate(CAMERAS):
        filters.append(
            f"[{index}:v]scale=640:512:force_original_aspect_ratio=decrease,"
            f"pad=640:512:(ow-iw)/2:(oh-ih)/2:black[v{index}]"
        )
    filters.append(
        "[v0][v1][v2][v3]xstack=inputs=4:"
        "layout=0_0|640_0|0_512|640_512:fill=black,"
        f"fps={args.fps},ass='{label_path.resolve()}',format=yuv420p[out]"
    )

    command = [str(args.ffmpeg), "-y", "-hide_banner", "-loglevel", "warning"]
    for list_path in list_paths:
        command.extend(["-f", "concat", "-safe", "0", "-i", str(list_path)])
    command.extend(
        [
            "-filter_complex",
            ";".join(filters),
            "-map",
            "[out]",
            "-t",
            f"{duration_s:.6f}",
            "-an",
            "-c:v",
            "libx264",
            "-preset",
            "medium",
            "-crf",
            "23",
            "-movflags",
            "+faststart",
            str(args.output),
        ]
    )
    print(
        "matched frames: "
        + ", ".join(f"{camera}={len(matched[camera])}" for camera, _ in CAMERAS)
    )
    print(f"timeline: {(global_end - global_start) / 1_000_000_000:.3f}s")
    subprocess.run(command, check=True)


if __name__ == "__main__":
    main()
