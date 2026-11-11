#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_dir="$(cd "${script_dir}/../.." && pwd)"
build_dir="${script_dir}/build"
stub_dir="${build_dir}/stub"
out="${build_dir}/libsentinel_nvmedia_decoder.so"

mkdir -p "${stub_dir}"

cat > "${stub_dir}/adhal_stub.cpp" <<'EOF'
extern "C" void sentinel_adhal_link_stub() {}
EOF

aarch64-linux-gnu-g++ -shared -fPIC "${stub_dir}/adhal_stub.cpp" \
  -Wl,-soname,libad_hal.so \
  -o "${stub_dir}/libad_hal.so"

aarch64-linux-gnu-g++ -std=c++17 -O2 -Wall -Wextra -Wpedantic -shared -fPIC \
  -I"${repo_dir}" \
  "${script_dir}/sentinel_nvmedia_adhal_bridge.cpp" \
  -L"${stub_dir}" -Wl,--no-as-needed -lad_hal -Wl,--as-needed \
  -Wl,-rpath,/opt/senseauto_active/senseauto-framework-sdk/lib \
  -o "${out}"

xz -zkf -9 "${out}"
sha256sum "${out}" "${out}.xz"
readelf -d "${out}" | grep -E 'NEEDED|RUNPATH'
