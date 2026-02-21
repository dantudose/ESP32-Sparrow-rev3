#!/usr/bin/env bash
set -euo pipefail

if [[ $# -ne 1 ]]; then
  echo "Usage: $0 <serial-port>"
  echo "Example: $0 /dev/cu.usbmodem1101"
  exit 2
fi

PORT="$1"
BOARD="${BOARD:-esp32c6_devkitc/esp32c6/hpcore}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

require_cmd() {
  if ! command -v "$1" >/dev/null 2>&1; then
    echo "Missing required tool: $1"
    exit 1
  fi
}

require_cmd west
require_cmd python3
require_cmd screen
if [[ -z "${MKLITTLEFS:-}" ]] && ! command -v mklittlefs >/dev/null 2>&1; then
  echo "Missing mklittlefs in PATH (or set MKLITTLEFS=/path/to/mklittlefs)"
  exit 1
fi

export ESPTOOL_PORT="${PORT}"

cd "${ROOT_DIR}"

reset
rm -rf build
west build -b "${BOARD}" -p auto .
west build -t littlefs_image
python3 scripts/flash_littlefs_image.py --build-dir build
west flash -d build --runner esp32

echo "Opening serial terminal on ${PORT} (115200)..."
exec screen "${PORT}" 115200
