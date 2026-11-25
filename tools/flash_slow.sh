#!/usr/bin/env bash
# Flash and monitor one board at a low baud rate (default 115200).
# M5StickC Plus fails at idf.py's default 460800 bps with
# "Unable to verify flash chip connection". (tools/idf.sh already defaults the
# plus board to 115200; this script adds port auto-detection and monitor.)
#
#   tools/flash_slow.sh <plus|plus2|s3> [port] [baud]
#
# Builds with tools/idf.sh, so the board's own build-<board>/ is used. When port
# is omitted, the first USB serial device found is used.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

board="${1:-}"
port="${2:-}"
baud="${3:-115200}"

if [[ -z "${board}" ]]; then
    echo "Usage: tools/flash_slow.sh <plus|plus2|s3> [port] [baud]" >&2
    exit 2
fi

if [[ -z "${port}" ]]; then
    shopt -s nullglob
    for candidate in /dev/cu.usbserial-* /dev/cu.SLAB_USBtoUART* /dev/cu.wchusbserial* /dev/cu.usbmodem*; do
        if [[ -e "${candidate}" ]]; then
            port="${candidate}"
            break
        fi
    done
fi

if [[ -z "${port}" ]]; then
    echo "USB serial port was not found." >&2
    echo "Usage: tools/flash_slow.sh <plus|plus2|s3> /dev/cu.usbserial-XXXX [baud]" >&2
    exit 1
fi

exec "${SCRIPT_DIR}/idf.sh" "${board}" -p "${port}" -b "${baud}" flash monitor
