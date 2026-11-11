#!/usr/bin/env bash
set -euo pipefail

# Run with sudo. This does not reload the display driver or reboot the host.
if [[ ${EUID} -ne 0 ]]; then
  echo "Run: sudo bash scripts/repair_cuda_runtime_pm.sh" >&2
  exit 1
fi
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TARGET=/etc/modprobe.d/sentinel-nvidia-runtime-pm.conf
if [[ -e "$TARGET" ]] && ! cmp -s "$SCRIPT_DIR/nvidia-no-runtime-d3.conf" "$TARGET"; then
  echo "Refusing to overwrite a different existing $TARGET" >&2
  exit 1
fi
install -m 0644 "$SCRIPT_DIR/nvidia-no-runtime-d3.conf" "$TARGET"
update-initramfs -u
echo "Installed $TARGET. Shut down and power on the host, then verify nvidia-smi."
echo "To undo: remove $TARGET and run sudo update-initramfs -u."
