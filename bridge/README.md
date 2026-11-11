# NvMedia bridge

This directory contains the vehicle-side HEVC hardware decode bridge used by
`NvMediaHevcDecoder`.

The application does not link Thor NvMedia headers or libraries directly.
Instead it loads `libsentinel_nvmedia_decoder.so` at runtime through the C ABI
declared in `../nvmedia_decoder_bridge.h`.

## Layout

- `nvmedia_bridge_thor/` - preferred Thor/DRIVE OS implementation using
  NvMedia/NvSci directly.
- `nvmedia_bridge_adhal/` - legacy fallback using the SenseAuto adHal video
  decoder wrapper.

## Runtime

Build one bridge implementation, deploy the resulting shared library beside the
application, and point the application at it:

```bash
export SENTINEL_NVMEDIA_DECODER_LIB=/path/to/libsentinel_nvmedia_decoder.so
```

The runtime package only needs the compiled shared library. These source
directories are needed for rebuilding and diagnosing the bridge.
