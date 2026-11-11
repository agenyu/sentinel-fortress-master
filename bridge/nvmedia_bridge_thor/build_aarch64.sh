#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_dir="$(cd "${script_dir}/../.." && pwd)"
build_dir="${script_dir}/build"
out="${build_dir}/libsentinel_nvmedia_decoder.so"

mkdir -p "${build_dir}"

find_first_existing() {
  for path in "$@"; do
    if [[ -e "${path}" ]]; then
      printf '%s\n' "${path}"
      return 0
    fi
  done
  return 1
}

find_first_dir_with_file() {
  local filename="$1"
  shift
  for dir in "$@"; do
    if [[ -f "${dir}/${filename}" ]]; then
      printf '%s\n' "${dir}"
      return 0
    fi
  done
  return 1
}

thor_root="${THOR_TEST_CONTAINER_ROOT:-}"
if [[ -z "${thor_root}" && -d "${HOME}/thor_test_container" ]]; then
  thor_root="${HOME}/thor_test_container"
fi

if [[ -z "${CXX:-}" && -n "${thor_root}" ]]; then
  cxx="$(find_first_existing \
    "${thor_root}"/usr/local/thor/aarch64--glibc--*/bin/aarch64-linux-g++ \
    "${thor_root}"/usr/local/thor/aarch64--glibc--*/bin/aarch64-buildroot-linux-gnu-g++ \
    "${thor_root}/usr/local/mdc_sdk/dp_gea/mdc_cross_compiler/bin/aarch64-linux-gnu-g++" \
    "${thor_root}/usr/local/mdc_sdk/dp_gea/mdc_cross_compiler/bin/aarch64-none-linux-gnu-g++" \
    || true)"
else
  cxx="${CXX:-}"
fi
cxx="${cxx:-aarch64-linux-gnu-g++}"

include_dirs=("${repo_dir}")
lib_dirs=()
cxx_flags=()

if [[ -n "${DRIVEOS_INCLUDE_DIR:-}" ]]; then
  include_dirs+=("${DRIVEOS_INCLUDE_DIR}")
fi
if [[ -n "${DRIVEOS_LIB_DIR:-}" ]]; then
  lib_dirs+=("${DRIVEOS_LIB_DIR}")
fi

if [[ -n "${thor_root}" ]]; then
  sysroot="$(find_first_existing \
    "${thor_root}"/usr/local/thor/aarch64--glibc--*/aarch64-buildroot-linux-gnu/sysroot \
    "${thor_root}"/usr/local/mdc_sdk/dp_gea/mdc_cross_compiler/sysroot \
    || true)"
  if [[ -n "${sysroot}" ]]; then
    cxx_flags+=("--sysroot=${sysroot}")
    include_dirs+=("${sysroot}/usr/include")
    lib_dirs+=("${sysroot}/usr/lib" "${sysroot}/lib")
  fi

  shopt -s nullglob
  include_dirs+=(
    "${thor_root}"/opt/senseauto/senseauto-3rdparty/*/3rdparty/nvmedia-thor/include
    "${thor_root}"/opt/senseauto/senseauto-3rdparty/*/3rdparty/include
    "${thor_root}"/opt/senseauto/senseauto-framework-sdk/*/include/internal/implementation/thor/nvmedia_6x
    "${thor_root}"/usr/local/thor/drive-linux/include
  )
  lib_dirs+=(
    "${thor_root}"/opt/senseauto/senseauto-3rdparty/*/3rdparty/nvmedia-thor/lib
    "${thor_root}"/usr/local/thor/drive-linux/lib
    "${thor_root}"/usr/local/thor/aarch64--glibc--*/aarch64-buildroot-linux-gnu/sysroot/usr/lib
    "${thor_root}"/usr/local/thor/aarch64--glibc--*/aarch64-buildroot-linux-gnu/sysroot/lib
    "${thor_root}"/usr/local/mdc_sdk/dp_gea/mdc_cross_compiler/sysroot/usr/lib/aarch64-linux-gnu
    "${thor_root}"/usr/local/mdc_sdk/dp_gea/mdc_cross_compiler/sysroot/lib/aarch64-linux-gnu
  )
  shopt -u nullglob
fi

nvmedia_include="$(find_first_dir_with_file nvmedia_ide.h "${include_dirs[@]}" || true)"
nvmedia2d_include="$(find_first_dir_with_file nvmedia_2d.h "${include_dirs[@]}" || true)"
nvmedia2d_sci_include="$(find_first_dir_with_file nvmedia_2d_sci.h "${include_dirs[@]}" || true)"
parser_include="$(find_first_dir_with_file nvmedia_parser.h "${include_dirs[@]}" || true)"
nvscibuf_include="$(find_first_dir_with_file nvscibuf.h "${include_dirs[@]}" || true)"
nvscisync_include="$(find_first_dir_with_file nvscisync.h "${include_dirs[@]}" || true)"

missing=0
for item in \
  "nvmedia_ide.h:${nvmedia_include}" \
  "nvmedia_2d.h:${nvmedia2d_include}" \
  "nvmedia_2d_sci.h:${nvmedia2d_sci_include}" \
  "nvmedia_parser.h:${parser_include}" \
  "nvscibuf.h:${nvscibuf_include}" \
  "nvscisync.h:${nvscisync_include}"; do
  name="${item%%:*}"
  value="${item#*:}"
  if [[ -z "${value}" ]]; then
    echo "Missing required header: ${name}" >&2
    missing=1
  fi
done
if [[ "${missing}" -ne 0 ]]; then
  echo "Set THOR_TEST_CONTAINER_ROOT, DRIVEOS_INCLUDE_DIR, or install the DRIVE OS/NvMedia dev headers." >&2
  exit 1
fi

dedup_flags() {
  awk '!seen[$0]++'
}

mapfile -t include_flags < <(
  printf '%s\n' "${include_dirs[@]}" | while read -r dir; do
    [[ -d "${dir}" ]] && printf -- '-I%s\n' "${dir}"
  done | dedup_flags
)
mapfile -t lib_flags < <(
  printf '%s\n' "${lib_dirs[@]}" | while read -r dir; do
    [[ -d "${dir}" ]] && printf -- '-L%s\n' "${dir}"
  done | dedup_flags
)
mapfile -t rpath_flags < <(
  printf '%s\n' "${lib_dirs[@]}" | while read -r dir; do
    [[ -d "${dir}" ]] && printf -- '-Wl,-rpath,%s\n' "${dir}"
  done | dedup_flags
)

has_library() {
  local libname="$1"
  shift
  for dir in "$@"; do
    if [[ -f "${dir}/lib${libname}.so" || -f "${dir}/lib${libname}.a" ]]; then
      return 0
    fi
  done
  return 1
}

nvmedia_link_libs=(-lnvmedia_ide -lnvmedia_parser)
if ! has_library nvmedia_ide "${lib_dirs[@]}" || ! has_library nvmedia_parser "${lib_dirs[@]}"; then
  if has_library nvmedia_ide_parser_prod "${lib_dirs[@]}"; then
    nvmedia_link_libs=(-lnvmedia_ide_parser_prod)
    if has_library nvmedia_ide_sci_prod "${lib_dirs[@]}"; then
      nvmedia_link_libs+=(-lnvmedia_ide_sci_prod)
    fi
  fi
fi
if has_library nvmedia2d "${lib_dirs[@]}"; then
  nvmedia_link_libs+=(-lnvmedia2d)
elif has_library nvmedia_2d "${lib_dirs[@]}"; then
  nvmedia_link_libs+=(-lnvmedia_2d)
elif has_library nvmedia_2d_sci "${lib_dirs[@]}"; then
  nvmedia_link_libs+=(-lnvmedia_2d_sci)
elif has_library nvmedia_2d_sci_prod "${lib_dirs[@]}"; then
  nvmedia_link_libs+=(-lnvmedia_2d_sci_prod)
else
  echo "Missing required library: libnvmedia2d.so, libnvmedia_2d.so, or compatible 2D NvMedia library" >&2
  exit 1
fi

echo "CXX=${cxx}"
echo "THOR_TEST_CONTAINER_ROOT=${thor_root:-<unset>}"
echo "SYSROOT=${sysroot:-<unset>}"
echo "Using NvMedia include: ${nvmedia_include}"
echo "Using NvMedia2D include: ${nvmedia2d_include}"
echo "Using NvMedia2D Sci include: ${nvmedia2d_sci_include}"
echo "Using parser include: ${parser_include}"
echo "Using NvSciBuf include: ${nvscibuf_include}"
echo "Using NvSciSync include: ${nvscisync_include}"
echo "Using NvMedia link libs: ${nvmedia_link_libs[*]}"

"${cxx}" "${cxx_flags[@]}" -std=c++17 -O2 -Wall -Wextra -Wpedantic -shared -fPIC \
  "${include_flags[@]}" \
  "${script_dir}/sentinel_nvmedia_thor_bridge.cpp" \
  "${lib_flags[@]}" \
  "${nvmedia_link_libs[@]}" -lnvscibuf -lnvscisync \
  "${rpath_flags[@]}" \
  -o "${out}"

xz -zkf -9 "${out}"
sha256sum "${out}" "${out}.xz"
readelf -d "${out}" | grep -E 'NEEDED|RUNPATH'
