#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_DIR="${ROOT_DIR}/build"
BIN="${BUILD_DIR}/sentinel_perception"
CONFIG="${1:-${BUILD_DIR}/config.yaml}"

if [[ ! -x "${BIN}" ]]; then
  echo "未找到可执行文件: ${BIN}" >&2
  echo "请先执行: mkdir -p build && cd build && cmake .. && make sentinel_perception -j\$(nproc)" >&2
  exit 1
fi

cd "${BUILD_DIR}"
echo "启动单节点 sentinel_perception"
echo "配置文件: ${CONFIG}"
exec "${BIN}" "${CONFIG}"
