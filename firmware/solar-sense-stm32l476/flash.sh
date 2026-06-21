#!/usr/bin/env bash
#
# Flash the SolarSense v1 firmware to the STM32L476RGT6 via ST-Link.
# Usage:
#   ./flash.sh            build (if needed) then flash the .bin at 0x08000000
#   ./flash.sh --build    force a clean rebuild before flashing
#   ./flash.sh --erase    mass-erase the chip, then flash
#   ./flash.sh --reset    just reset the target (no flashing)
#   ./flash.sh --cur      connect under reset (use if SWD won't attach)
#
set -euo pipefail

CONNECT_OPT=""

cd "$(dirname "$0")"

TARGET="solar-sense-v1-fw"
BUILD_DIR="build"
BIN="${BUILD_DIR}/${TARGET}.bin"
FLASH_ADDR="0x08000000"

# --- checks -----------------------------------------------------------------
command -v st-flash >/dev/null 2>&1 || {
  echo "error: st-flash not found (install 'stlink-tools')." >&2
  exit 1
}

# --- options ----------------------------------------------------------------
case "${1:-}" in
  --reset)
    echo ">> resetting target"
    st-flash reset
    exit 0
    ;;
  --build)
    echo ">> clean rebuild"
    make clean
    make -j"$(nproc)"
    ;;
  --erase)
    DO_ERASE=1
    ;;
  --cur)
    CONNECT_OPT="--connect-under-reset"
    ;;
esac

# --- ensure a fresh binary exists -------------------------------------------
echo ">> building (incremental)"
make -j"$(nproc)"

[ -f "$BIN" ] || { echo "error: $BIN not found after build." >&2; exit 1; }

# --- probe the ST-Link / target ---------------------------------------------
echo ">> probing target"
st-info --probe

# --- optional mass erase ----------------------------------------------------
if [ "${DO_ERASE:-0}" = "1" ]; then
  echo ">> mass erase"
  st-flash erase
fi

# --- flash and reset --------------------------------------------------------
echo ">> flashing $BIN -> $FLASH_ADDR"
st-flash $CONNECT_OPT --reset write "$BIN" "$FLASH_ADDR"

echo ">> done. LED_STATUS should blink at 1 Hz; heartbeat on USART2 (PA2/PA3, 115200 8N1)."
