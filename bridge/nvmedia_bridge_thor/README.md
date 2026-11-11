# Thor direct NvMedia bridge

This bridge keeps the existing `nvmedia_decoder_bridge.h` C ABI used by
`NvMediaHevcDecoder`, but its implementation is intended to run directly on
Thor/DRIVE OS with `NvMediaParser`, `NvMediaIDE`, `NvSciBuf`, and `NvSciSync`.

The older `nvmedia_bridge_adhal` path depends on the SenseAuto adHal video
decoder wrapper. This direct bridge is the preferred vehicle-side direction
when the target environment already provides Thor Camera Framework / DRIVE OS
NvMedia runtime components.

## Build

On the validated Thor setup, the library is cross-compiled on the Sunlogin
laptop or in the Thor SDK Docker image. The script auto-detects the SDK under
`~/thor_test_container` when that layout is used:

```bash
cd sentinel_perception/bridge/nvmedia_bridge_thor
./build_aarch64.sh
```

For other environments, set the SDK roots if they are not under the defaults
used by the script:

```bash
export THOR_TEST_CONTAINER_ROOT=/path/to/thor_test_container
export DRIVEOS_INCLUDE_DIR=/usr/include
export DRIVEOS_LIB_DIR=/usr/lib/aarch64-linux-gnu
./build_aarch64.sh
```

The output is:

```text
build/libsentinel_nvmedia_decoder.so
```

Point the application at it:

```bash
export SENTINEL_NVMEDIA_DECODER_LIB=/path/to/libsentinel_nvmedia_decoder.so
```

## Verified Thor smoke test

Version v16 was verified through Aliyun Sunlogin on Thor `172.16.0.11` with a
real HEVC Annex-B sample:

```bash
python3 tests/test_nvmedia_decoder.py \
  --lib /ota/0729bag/l16.so \
  --input /ota/0729bag/s.h265
```

Expected success includes a decoded BGR frame such as:

```text
[frame] index=4 width=1920 height=1080 stride=5760 channels=3
```

The working path keeps IDE decode targets block-linear, then uses NvMedia2D to
copy into a CPU-readable pitch-linear NV12 readback surface before converting
to BGR. Direct pitch-linear decode targets failed on this stack with
`NvMediaIDERegisterNvSciBufObj failed status=7`.

## Current integration policy

- Public ABI remains `sentinel_nvmedia_create/destroy/decode/release_frame/flush`.
- Upper application code continues to receive BGR frames.
- NvMedia details stay inside this shared library so `sub_encode_example` does
  not need Thor SDK headers.
- `nvmedia_bridge_adhal` remains available only as a legacy fallback.
