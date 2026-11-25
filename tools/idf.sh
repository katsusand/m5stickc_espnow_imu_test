#!/usr/bin/env bash
# Run idf.py for one M5Stick board with its own build directory and sdkconfig.
#
#   tools/idf.sh <plus|plus2|s3> <idf.py arguments...>
#   tools/idf.sh plus2 build
#   tools/idf.sh s3 -p /dev/cu.usbmodem1101 flash monitor
#
# Each board gets build-<board>/ (build output and sdkconfig), the matching
# sdkconfig.defaults.<board> and the matching IDF_TARGET, so switching boards
# never reuses another board's configuration. The repo-root sdkconfig and
# build/ are not touched.

set -euo pipefail

IDF_ACTIVATE_SH="${HOME}/.espressif/tools/activate_idf_v5.4.3.sh"
PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

usage() {
    echo "usage: tools/idf.sh <plus|plus2|s3> <idf.py arguments...>" >&2
    exit 2
}

[[ $# -ge 2 ]] || usage
board="$1"
shift

case "${board}" in
    plus)  target="esp32";   board_symbol="CONFIG_M5STICKC_BOARD_PLUS";  label="M5StickC Plus" ;;
    plus2) target="esp32";   board_symbol="CONFIG_M5STICKC_BOARD_PLUS2"; label="M5StickC Plus2" ;;
    s3)    target="esp32s3"; board_symbol="CONFIG_M5STICK_BOARD_S3";     label="M5StickS3" ;;
    *) echo "error: unknown board '${board}' (plus, plus2, s3)" >&2; usage ;;
esac

if [[ ! -f "${IDF_ACTIVATE_SH}" ]]; then
    echo "error: ESP-IDF activation script not found: ${IDF_ACTIVATE_SH}" >&2
    exit 1
fi

build_dir="build-${board}"
sdkconfig="${build_dir}/sdkconfig"
defaults="sdkconfig.defaults.${board}"

cd "${PROJECT_ROOT}"

# An existing sdkconfig must belong to this board; idf.py would silently keep it.
if [[ -f "${sdkconfig}" ]] && ! grep -q "^${board_symbol}=y" "${sdkconfig}"; then
    echo "error: ${sdkconfig} is not configured for ${label}." >&2
    echo "hint: delete ${build_dir}/ and run again." >&2
    exit 1
fi

echo "== ${label}: target=${target} build=${build_dir} defaults=${defaults}"

bash -lc '
set -euo pipefail
cd "$1"
set +u
# shellcheck disable=SC1090
source "$2" || true
set -u
if [[ -z "${IDF_PATH:-}" ]] || ! command -v python >/dev/null 2>&1; then
    echo "error: ESP-IDF v5.4.3 is not usable after activation." >&2
    exit 1
fi
exec python "${IDF_PATH}/tools/idf.py" \
    -DIDF_TARGET="$3" -D SDKCONFIG="$4" -D SDKCONFIG_DEFAULTS="$5" -B "$6" "${@:7}"
' bash "${PROJECT_ROOT}" "${IDF_ACTIVATE_SH}" "${target}" "${sdkconfig}" "${defaults}" "${build_dir}" "$@"

if [[ -f "${sdkconfig}" ]]; then
    flash_size="$(sed -n 's/^CONFIG_ESPTOOLPY_FLASHSIZE="\(.*\)"/\1/p' "${sdkconfig}")"
    echo "== done: ${label} (flash header ${flash_size}), sdkconfig=${sdkconfig}"
fi
