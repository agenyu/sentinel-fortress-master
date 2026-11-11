# AD HAL NvMedia bridge

Legacy bridge implementation for `libsentinel_nvmedia_decoder.so` using the
SenseAuto adHal `ADVideoDecoder` wrapper.

The preferred vehicle-side implementation is `../nvmedia_bridge_thor`, which
uses Thor/DRIVE OS NvMedia/NvSci APIs directly. Keep this bridge as a fallback
for environments where the adHal video decoder path is the only available
decode entry.

## Build

Use the Thor/aarch64 cross compiler environment:

```bash
./build_aarch64.sh
```

The output is:

```text
build/libsentinel_nvmedia_decoder.so
```
