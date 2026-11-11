#!/usr/bin/env bash
set -euo pipefail

# Build the standalone offline renderer against Ubuntu OpenCV. The project
# activation script prepends SenseAuto OpenCV 3.4 and BLAS/LAPACK directories;
# leaving those variables active can mix OpenCV 4.5 with the old LAPACK stack
# and fail on the unavailable GFORTRAN_1.x ABI.
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUTPUT="${1:-${ROOT_DIR}/tools/bin/sup_orsm_offline}"
SOURCE="${ROOT_DIR}/sup_orsm.cpp"
SYSTEM_LIB_DIR=/usr/lib/x86_64-linux-gnu

for required in \
  /usr/bin/g++ \
  /usr/include/opencv4/opencv2/core.hpp \
  "${SYSTEM_LIB_DIR}/libopencv_core.so" \
  "${SYSTEM_LIB_DIR}/libopencv_imgproc.so" \
  "${SYSTEM_LIB_DIR}/libopencv_videoio.so"; do
  if [[ ! -e "${required}" ]]; then
    echo "[FAIL] missing build dependency: ${required}" >&2
    exit 1
  fi
done

mkdir -p "$(dirname "${OUTPUT}")"
env -u LD_LIBRARY_PATH -u LIBRARY_PATH -u CPATH -u CPLUS_INCLUDE_PATH \
  /usr/bin/g++ -std=c++17 -O2 -pthread -DSUP_ORSM_OFFLINE_LOG \
  -I/usr/include/opencv4 \
  "${SOURCE}" \
  -o "${OUTPUT}" \
  "${SYSTEM_LIB_DIR}/libopencv_imgproc.so" \
  "${SYSTEM_LIB_DIR}/libopencv_videoio.so" \
  "${SYSTEM_LIB_DIR}/libopencv_core.so"

if env -u LD_LIBRARY_PATH /usr/bin/ldd "${OUTPUT}" | grep -q 'not found'; then
  echo "[FAIL] offline renderer has unresolved libraries" >&2
  env -u LD_LIBRARY_PATH /usr/bin/ldd "${OUTPUT}" >&2
  exit 1
fi

echo "[OK] built ${OUTPUT}"
sha256sum "${OUTPUT}"
