#!/usr/bin/env zsh
set -euo pipefail

project_root="${0:A:h:h}"
idf_export="${HOME}/.espressif/v5.4.3/esp-idf/export.sh"

port="${1:-}"
baud="${2:-115200}"

if [[ -z "${port}" ]]; then
    candidates=(/dev/cu.usbserial-*(N) /dev/cu.SLAB_USBtoUART*(N) /dev/cu.wchusbserial*(N))
    for candidate in "${candidates[@]}"; do
        if [[ -e "${candidate}" ]]; then
            port="${candidate}"
            break
        fi
    done
fi

if [[ -z "${port}" ]]; then
    echo "USB serial port was not found."
    echo "Usage: tools/flash_slow.sh /dev/cu.usbserial-XXXX [baud]"
    exit 1
fi

source "${idf_export}"
cd "${project_root}"

idf.py -p "${port}" -b "${baud}" flash monitor
