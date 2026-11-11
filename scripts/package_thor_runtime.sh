#!/usr/bin/env bash
set -euo pipefail
export LC_ALL=C

BUILD_DIR="${1:?usage: $0 BUILD_DIR OUT_DIR [CONFIG] [NVMEDIA_BRIDGE]}"
OUT_DIR="${2:?usage: $0 BUILD_DIR OUT_DIR [CONFIG] [NVMEDIA_BRIDGE]}"
CONFIG="${3:-${BUILD_DIR}/config.yaml}"
NVMEDIA_BRIDGE="${4:-}"
BIN="${BUILD_DIR}/sentinel_perception"
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if [[ -z "${NVMEDIA_BRIDGE}" ]]; then
  default_bridge="${PROJECT_DIR}/bridge/nvmedia_bridge_thor/build/libsentinel_nvmedia_decoder.so"
  if [[ -f "${default_bridge}" ]]; then
    NVMEDIA_BRIDGE="${default_bridge}"
  fi
fi

if [[ ! -x "${BIN}" ]]; then
  echo "[FAIL] binary not found or not executable: ${BIN}" >&2
  exit 1
fi
if [[ ! -f "${CONFIG}" ]]; then
  echo "[FAIL] config not found: ${CONFIG}" >&2
  exit 1
fi
if ! command -v readelf >/dev/null 2>&1; then
  echo "[FAIL] readelf not found" >&2
  exit 1
fi

mkdir -p "${OUT_DIR}/lib"
cp -f "${BIN}" "${OUT_DIR}/sentinel_perception"
cp -f "${CONFIG}" "${OUT_DIR}/config.yaml"
if [[ -n "${NVMEDIA_BRIDGE}" ]]; then
  if [[ ! -f "${NVMEDIA_BRIDGE}" ]]; then
    echo "[FAIL] NvMedia bridge not found: ${NVMEDIA_BRIDGE}" >&2
    exit 1
  fi
  cp -f "${NVMEDIA_BRIDGE}" "${OUT_DIR}/lib/libsentinel_nvmedia_decoder.so"
fi

IFS=':' read -r -a SEARCH_DIRS <<< "${THOR_RUNTIME_LIB_DIRS:-}"
DEFAULT_DIRS=(
  "${BUILD_DIR}/compat-libs"
  "${SENTINEL_THOR_ROOT:-}/drive-linux/lib"
  "${SENTINEL_THOR_ROOT:-}/aarch64-linux-gnu/lib"
  "${SENTINEL_THOR_ROOT:-}/cuda-12.8/targets/aarch64-linux/lib"
  "${SENTINEL_THOR_ROOT:-}/aarch64--glibc--bleeding-edge-2024.02-1/aarch64-buildroot-linux-gnu/sysroot/usr/lib"
  "${SENTINEL_THOR_ROOT:-}/aarch64--glibc--bleeding-edge-2024.02-1/aarch64-buildroot-linux-gnu/sysroot/lib"
  "${SENSEAUTO_ACTIVE_ROOT:-/opt/senseauto_active}/senseauto-msgs/lib"
  "${SENSEAUTO_ACTIVE_ROOT:-/opt/senseauto_active}/senseauto-rscl/lib"
  "${SENSEAUTO_ACTIVE_ROOT:-/opt/senseauto_active}/senseauto-framework-sdk/lib"
  "${SENSEAUTO_ACTIVE_ROOT:-/opt/senseauto_active}/senseauto-3rdparty/3rdparty/lib"
  "${SENSEAUTO_ROOT:-/opt/senseauto}/senseauto-3rdparty/0.251127.1344+/3rdparty/lib"
)
SEARCH_DIRS+=("${DEFAULT_DIRS[@]}")

declare -A COPIED=()
declare -A MISSING=()
declare -A SKIPPED_SYSTEM=()
queue=("${OUT_DIR}/sentinel_perception")
if [[ -n "${NVMEDIA_BRIDGE}" ]]; then
  queue+=("${OUT_DIR}/lib/libsentinel_nvmedia_decoder.so")
fi

resolve_lib() {
  local name="$1"
  local dir candidate
  for dir in "${SEARCH_DIRS[@]}"; do
    [[ -n "${dir}" && -d "${dir}" ]] || continue
    candidate="${dir}/${name}"
    if [[ -f "${candidate}" ]]; then
      printf '%s\n' "${candidate}"
      return 0
    fi
  done
  return 1
}

is_system_runtime_lib() {
  case "$1" in
    ld-linux-aarch64.so.*|libc.so.*|libm.so.*|libpthread.so.*|libdl.so.*|librt.so.*|libresolv.so.*|libnsl.so.*|libutil.so.*|libanl.so.*)
      return 0
      ;;
  esac
  return 1
}

copy_runtime_lib() {
  local src="$1"
  local name="$2"
  local real_src

  real_src="$(readlink -f "${src}")"
  if [[ ! -f "${real_src}" ]]; then
    echo "[FAIL] dependency resolves to a missing target: ${src}" >&2
    return 1
  fi
  if ! readelf -h "${real_src}" >/dev/null 2>&1; then
    echo "[FAIL] dependency is not a valid ELF shared object: ${src} -> ${real_src}" >&2
    return 1
  fi
  cp -Lf "${real_src}" "${OUT_DIR}/lib/${name}"
}

while ((${#queue[@]})); do
  current="${queue[0]}"
  queue=("${queue[@]:1}")
  while read -r needed; do
    [[ -n "${needed}" ]] || continue
    if is_system_runtime_lib "${needed}"; then
      SKIPPED_SYSTEM["${needed}"]=1
      continue
    fi
    if [[ -n "${COPIED[${needed}]:-}" ]]; then
      continue
    fi
    if path="$(resolve_lib "${needed}")"; then
      copy_runtime_lib "${path}" "${needed}"
      COPIED["${needed}"]=1
      queue+=("${OUT_DIR}/lib/${needed}")
    else
      echo "[WARN] dependency not bundled: ${needed}" >&2
      MISSING["${needed}"]=1
    fi
  done < <(readelf -d "${current}" 2>/dev/null | sed -n 's/.*Shared library: \[\(.*\)\].*/\1/p')
done

if ((${#MISSING[@]})); then
  echo "[FAIL] runtime package is incomplete; missing dependencies:" >&2
  printf '  %s\n' "${!MISSING[@]}" | sort >&2
  echo "[HINT] Set THOR_RUNTIME_LIB_DIRS, SENSEAUTO_ACTIVE_ROOT, SENSEAUTO_ROOT, and SENTINEL_THOR_ROOT to the aarch64 Thor/SenseAuto SDK paths, then rerun." >&2
  exit 1
fi

echo "[OK] Thor runtime package: ${OUT_DIR}"
if ((${#SKIPPED_SYSTEM[@]})); then
  echo "[OK] skipped Thor system runtime libraries:"
  printf '  %s\n' "${!SKIPPED_SYSTEM[@]}" | sort
fi
find "${OUT_DIR}" -maxdepth 2 -type f | sort
