#!/usr/bin/env bash
set -euo pipefail

# Run sentinel_perception and replay one RSCL bag into it.
# Run inside rscl-u2004 from any path after prepare_bag_replay_env.sh.
# Usage:
#   ./scripts/run_bag_replay.sh /path/to/test.bag [config] [play_rate]
# Optional env:
#   BUILD_DIR=/home/aloys/orsm/0904/build-local
#   INCLUDED_TOPICS="/sensor/camera/* /sensor/imu /sensor/uss/fused_points /parking/uss/* /parking/* /sensor/lidar/* /canbus/vehicle_report"
#   REMAP_ARGS="/old/topic:=/new/topic ..."
#   RELATIVE_START_SEC=10 END_TIME_NS=...
#   RSCLBAG_FULL_LOG=1       # 调试播放器本身时才保存完整终端进度
#   RSCLBAG_LOG_MAX_LINES=2000 # 默认模式最多保留多少条播放器错误/警告

BAG="${1:?usage: $0 /path/to/test.bag [config] [play_rate]}"
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# 离线 ORSM 验证固定使用当前本地构建，避免误用旧 build-rscl 二进制。
BUILD_DIR="${BUILD_DIR:-${ROOT_DIR}/build-local}"
CONFIG="${2:-${BUILD_DIR}/config.bag_replay.yaml}"
PLAY_RATE="${3:-1.0}"
BIN="${BUILD_DIR}/sentinel_perception"
LOG_DIR="${ORSM_REPLAY_LOG_DIR:-${BUILD_DIR}/logs}"
EVENTS_DIR="${ORSM_REPLAY_EVENTS_DIR:-${BUILD_DIR}/events}"
RUNTIME_OUTPUT_DIR="${ORSM_REPLAY_OUTPUT_DIR:-${BUILD_DIR}/output}"
STAMP="$(date +%Y%m%d_%H%M%S)"
APP_LOG="${LOG_DIR}/sentinel_${STAMP}.log"
BAG_LOG="${LOG_DIR}/rsclbag_${STAMP}.log"

if [[ ! -f "${BAG}" ]]; then echo "[FAIL] bag not found: ${BAG}" >&2; exit 1; fi
if [[ ! -x "${BIN}" ]]; then echo "[FAIL] binary not found: ${BIN}" >&2; exit 1; fi
if [[ ! -f "${CONFIG}" ]]; then echo "[FAIL] config not found: ${CONFIG}" >&2; exit 1; fi
command -v rsclbag >/dev/null 2>&1 || { echo "[FAIL] rsclbag not found" >&2; exit 1; }
mkdir -p "${LOG_DIR}" "${EVENTS_DIR}" "${RUNTIME_OUTPUT_DIR}"
if [[ -f "${BUILD_DIR}/bag_replay_env.sh" ]]; then
  # shellcheck disable=SC1091
  source "${BUILD_DIR}/bag_replay_env.sh"
fi

cleanup() {
  local code=$?
  if [[ -n "${APP_PID:-}" ]] && kill -0 "${APP_PID}" 2>/dev/null; then
    kill -TERM "${APP_PID}" 2>/dev/null || true
    wait "${APP_PID}" 2>/dev/null || true
  fi
  echo "[INFO] logs:"
  echo "  sentinel: ${APP_LOG}"
  echo "  rsclbag : ${BAG_LOG}"
  exit "${code}"
}
trap cleanup EXIT INT TERM

echo "[INFO] bag summary"
rsclbag info "${BAG}" --detail | tee "${LOG_DIR}/bag_info_${STAMP}.log" >/dev/null || true

cd "${BUILD_DIR}"
echo "[INFO] starting sentinel_perception..."
env LD_LIBRARY_PATH="${LD_LIBRARY_PATH:-}" "${BIN}" "${CONFIG}" >"${APP_LOG}" 2>&1 &
APP_PID=$!
sleep 3
if ! kill -0 "${APP_PID}" 2>/dev/null; then
  echo "[FAIL] sentinel_perception exited early. Tail:" >&2
  tail -80 "${APP_LOG}" >&2 || true
  exit 1
fi

read -r -a included <<< "${INCLUDED_TOPICS:-/sensor/camera/* /sensor/imu /sensor/uss/fused_points /parking/uss/* /parking/* /sensor/lidar/* /canbus/vehicle_report}"
read -r -a remaps <<< "${REMAP_ARGS:-}"
PLAY_CMD=(rsclbag simple_play "${BAG}" --play_rate "${PLAY_RATE}" --included "${included[@]}")
if ((${#remaps[@]})); then
  PLAY_CMD+=(--remap "${remaps[@]}")
fi
if [[ -n "${RELATIVE_START_SEC:-}" ]]; then PLAY_CMD+=(--relative_start_sec "${RELATIVE_START_SEC}"); fi
if [[ -n "${BEGIN_TIME_NS:-}" ]]; then PLAY_CMD+=(--begin_time_ns "${BEGIN_TIME_NS}"); fi
if [[ -n "${END_TIME_NS:-}" ]]; then PLAY_CMD+=(--end_time_ns "${END_TIME_NS}"); fi

echo "[INFO] replay command: ${PLAY_CMD[*]}"
set +e
if [[ "${RSCLBAG_FULL_LOG:-0}" == "1" ]]; then
  "${PLAY_CMD[@]}" >"${BAG_LOG}" 2>&1
  PLAY_EXIT=$?
else
  # rsclbag 使用 '\r' 高频刷新同一行进度。直接重定向会把一次短回放膨胀成
  # 数 GB 日志。非 TTY 下 tcsetattr 还可能以极高频率重复，因此只记录一次
  # 抑制提示，并为其他错误/警告设置行数上限；Play End 始终保留。
  RSCLBAG_LOG_MAX_LINES="${RSCLBAG_LOG_MAX_LINES:-2000}"
  "${PLAY_CMD[@]}" 2>&1 |
    tr '\r' '\n' |
    awk -v max_lines="${RSCLBAG_LOG_MAX_LINES}" '
      /tcsetattr/ {
        if (!terminal_warning_seen++) {
          print "[WARN] repeated rsclbag tcsetattr messages suppressed"
          fflush()
        }
        next
      }
      /Play End/ {
        print
        fflush()
        next
      }
      /ERROR|WARN|Error|error|FAIL|failed/ {
        if (kept_lines < max_lines) {
          print
          fflush()
        } else if (kept_lines == max_lines) {
          print "[WARN] rsclbag log line limit reached; further diagnostics suppressed"
          fflush()
        }
        kept_lines++
      }
    ' \
      >"${BAG_LOG}"
  PLAY_EXIT=${PIPESTATUS[0]}
fi
set -e

# rsclbag writes terminal-control warnings (tcsetattr ... Inappropriate ioctl)
# and may return non-zero when launched from non-interactive automation even
# after the bag reaches "Play End". Treat that known non-TTY condition as a
# successful replay so CI/chat-driven validation is not reported as a false
# failure. Other non-zero exits still fail normally.
if (( PLAY_EXIT != 0 )); then
  if grep -q -- "------ Play End -------" "${BAG_LOG}"; then
    echo "[WARN] rsclbag exited ${PLAY_EXIT} after a confirmed Play End; treating replay as success"
  else
    echo "[FAIL] rsclbag exited ${PLAY_EXIT}. Tail:" >&2
    tail -80 "${BAG_LOG}" >&2 || true
    exit "${PLAY_EXIT}"
  fi
fi

echo "[OK] replay finished"
echo "[INFO] sentinel tail:"
tail -80 "${APP_LOG}" || true
if [[ -f "${EVENTS_DIR}/sentry_events.jsonl" ]]; then
  echo "[INFO] event tail:"
  tail -20 "${EVENTS_DIR}/sentry_events.jsonl" || true
fi
