#!/usr/bin/env python3
"""Export every left-camera encoded frame from an RSCL bag."""
import argparse, json, os, sys
from pathlib import Path

RSCL_BASE = Path('/opt/senseauto_active/senseauto-rscl')
THIRD_PARTY = Path('/opt/senseauto_active/senseauto-3rdparty/3rdparty')
TOPIC = '/sensor/camera/left_camera_fov195/encode'

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bag', type=Path)
    ap.add_argument('output_dir', type=Path)
    args = ap.parse_args()
    os.environ['PYTHONPATH'] = f'{RSCL_BASE}/python:' + os.environ.get('PYTHONPATH', '')
    os.environ['LD_LIBRARY_PATH'] = ':'.join([str(RSCL_BASE/'lib'), str(THIRD_PARTY/'protobuf/lib'), str(THIRD_PARTY/'lib'), os.environ.get('LD_LIBRARY_PATH','')])
    sys.path.insert(0, str(RSCL_BASE/'python'))
    import ad_bag_py
    args.output_dir.mkdir(parents=True, exist_ok=True)
    stream_path = args.output_dir / 'left_camera_fov195.h265'
    manifest = []
    started = False
    with stream_path.open('wb') as out:
        reader = ad_bag_py.BagReader(str(args.bag), included_channels=[TOPIC])
        while True:
            msg = reader.read_next_message()
            if msg is None:
                break
            obj = msg.message_obj
            frame_type = int(obj.frameType)
            if not started:
                if frame_type != 2:
                    continue
                started = True
            out.write(bytes(obj.data))
            manifest.append({'index': len(manifest), 'bag_timestamp_ns': int(msg.timestamp), 'camera_timestamp_ns': int(obj.header.time.nanoSec), 'seq': int(obj.header.seq), 'frame_type': frame_type})
    (args.output_dir/'left_camera_manifest.json').write_text(json.dumps(manifest, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    print(json.dumps({'frames': len(manifest), 'stream': str(stream_path)}, ensure_ascii=False))

if __name__ == '__main__':
    main()
