#!/usr/bin/env bash
set -euo pipefail

pkill -TERM -f 'sentinel_perception' || true
echo "已请求停止 sentinel_perception 单节点进程"
