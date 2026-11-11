#!/usr/bin/env bash
set -euo pipefail

# Prepare a Docker/container-side bag replay workspace for sentinel_perception.
# Run inside rscl-u2004 from the source root, for example:
#   cd /root/sentinel_video_recorder && ./scripts/prepare_bag_replay_env.sh

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_DIR="${BUILD_DIR:-${ROOT_DIR}/build-rscl}"
BASE_CONFIG="${1:-${BUILD_DIR}/config.yaml}"
REPLAY_CONFIG="${2:-${BUILD_DIR}/config.bag_replay.yaml}"
BIN="${BUILD_DIR}/sentinel_perception"

need() {
  command -v "$1" >/dev/null 2>&1 || { echo "[FAIL] missing command: $1" >&2; exit 1; }
}

need rsclbag
need cmake

if [[ ! -x "${BIN}" ]]; then
  echo "[INFO] sentinel_perception not found at ${BIN}; building..."
  mkdir -p "${BUILD_DIR}"
  cd "${BUILD_DIR}"
  # x86 回放环境使用系统 FFmpeg，避免 SDK 内置 FFmpeg 依赖车端才有的
  # libvpx.so.3/libx264.so.148 等旧版转码库。-U 避免旧 CMake 缓存保留错误库路径。
  cmake -U 'FFMPEG_*_LIB' .. \
    -DADFramework_DIR=/opt/senseauto_active/senseauto-framework-sdk/cmake \
    -DSENTINEL_USE_SYSTEM_FFMPEG=ON
  cmake --build . -j"$(nproc 2>/dev/null || echo 2)"
fi

if [[ ! -f "${BASE_CONFIG}" ]]; then
  if [[ -f "${ROOT_DIR}/build/config.yaml" ]]; then
    cp "${ROOT_DIR}/build/config.yaml" "${BASE_CONFIG}"
  else
    echo "[FAIL] config not found: ${BASE_CONFIG}" >&2
    exit 1
  fi
fi

cp "${BASE_CONFIG}" "${REPLAY_CONFIG}"

# 离线验证需要视觉推理。相对模型路径按运行目录 BUILD_DIR 转为绝对路径。
ENGINE_CONFIG_VALUE="$(sed -n 's/^[[:space:]]*engine_path:[[:space:]]*["'\'']\{0,1\}\([^"'\'']*\)["'\'']\{0,1\}[[:space:]]*$/\1/p' "${REPLAY_CONFIG}" | head -1)"
if [[ -z "${ENGINE_CONFIG_VALUE}" ]]; then
  echo "[FAIL] engine_path is missing from ${REPLAY_CONFIG}" >&2
  exit 1
fi
if [[ "${ENGINE_CONFIG_VALUE}" = /* ]]; then
  REPLAY_ENGINE="${ENGINE_CONFIG_VALUE}"
else
  REPLAY_ENGINE="${BUILD_DIR}/${ENGINE_CONFIG_VALUE}"
fi
if [[ ! -s "${REPLAY_ENGINE}" ]]; then
  echo "[FAIL] TensorRT engine not found: ${REPLAY_ENGINE}" >&2
  echo "       Visual inference is required for sentry bag validation." >&2
  exit 1
fi

# Bag replay defaults: avoid GUI in headless Docker, avoid large debug image dumps,
# and explicitly satisfy sentry ODD so a bag with low SOC/missing key or user-enable
# signals still enters ACTIVE during an offline sentry regression test.
sed -i \
  -e 's/^enable_display:.*/enable_display: false/' \
  -e 's/^enable_save:.*/enable_save: false/' \
  -e 's/^camera_decoder_backend:.*/camera_decoder_backend: "ffmpeg_nvdec"/' \
  -e 's/^override_power_ok:.*/override_power_ok: true/' \
  -e 's/^override_speed_zero:.*/override_speed_zero: true/' \
  -e 's/^override_gear_park:.*/override_gear_park: true/' \
  -e 's/^override_handbrake_on:.*/override_handbrake_on: true/' \
  -e 's/^override_power_off:.*/override_power_off: true/' \
  -e 's/^override_no_sensor_fault:.*/override_no_sensor_fault: true/' \
  -e 's/^override_no_occupant:.*/override_no_occupant: true/' \
  -e 's/^override_vehicle_locked:.*/override_vehicle_locked: true/' \
  -e 's/^override_temp_ok:.*/override_temp_ok: true/' \
  -e 's/^override_key_far:.*/override_key_far: true/' \
  -e 's/^override_storage_ok:.*/override_storage_ok: true/' \
  -e 's/^default_key_far:.*/default_key_far: true/' \
  -e 's/^user_enabled:.*/user_enabled: true/' \
  -e "s#^engine_path:.*#engine_path: \"${REPLAY_ENGINE}\"#" \
  -e "s#^calib_base_dir:.*#calib_base_dir: \"${ROOT_DIR}/param/calib\"#" \
  "${REPLAY_CONFIG}"

mkdir -p "${BUILD_DIR}/output" "${BUILD_DIR}/events" "${BUILD_DIR}/logs" "${BUILD_DIR}/compat-libs"

# Runtime loader path for SenseAuto indirect FFmpeg/OpenCV/Cap'n Proto deps.
cat > "${BUILD_DIR}/bag_replay_env.sh" <<ENVEOF
#!/usr/bin/env bash
export SENTINEL_BAG_REPLAY_COMPAT_DIR="\${SENTINEL_BAG_REPLAY_COMPAT_DIR:-\$(cd "\$(dirname "\${BASH_SOURCE[0]}")" && pwd)/compat-libs}"
export SENTINEL_RUNTIME_ENV="\${SENTINEL_RUNTIME_ENV:-${ROOT_DIR}/../runtime-env}"
export LD_LIBRARY_PATH="\${SENTINEL_BAG_REPLAY_COMPAT_DIR}:/opt/senseauto_active/senseauto-3rdparty/3rdparty/ffmpeg/lib:/opt/senseauto_active/senseauto-3rdparty/3rdparty/opencv/lib:/opt/senseauto_active/senseauto-3rdparty/3rdparty/lib:/opt/senseauto_active/senseauto-msgs/lib:/opt/senseauto_active/senseauto-rscl/lib:/opt/senseauto_active/senseauto-pal/lib:/opt/senseauto_active/senseauto-framework-sdk/lib:/opt/senseauto_active/senseauto-swcfl/lib:\${SENTINEL_RUNTIME_ENV}/tensorrt-10.13.3.9/lib:\${SENTINEL_RUNTIME_ENV}/cuda-11.4/lib64:\${LD_LIBRARY_PATH:-}"
ENVEOF
chmod +x "${BUILD_DIR}/bag_replay_env.sh"

cat <<EOF
[OK] Bag replay env prepared
  root          : ${ROOT_DIR}
  build         : ${BUILD_DIR}
  binary        : ${BIN}
  replay config : ${REPLAY_CONFIG}
  rsclbag       : $(command -v rsclbag)

Next:
  rsclbag info /path/to/test.bag --detail
  ${ROOT_DIR}/scripts/run_bag_replay.sh /path/to/test.bag ${REPLAY_CONFIG}
EOF
