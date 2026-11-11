#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BINARY="${SUP_ORSM_OFFLINE_BIN:-${ROOT_DIR}/tools/bin/sup_orsm_offline}"

if [[ ! -x "${BINARY}" ]]; then
  "${ROOT_DIR}/tools/build_sup_orsm_offline.sh" "${BINARY}"
fi

# 独立渲染器固定使用 Ubuntu OpenCV，避免受已激活运行环境影响。
exec env LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:/lib/x86_64-linux-gnu \
  "${BINARY}" "$@"
