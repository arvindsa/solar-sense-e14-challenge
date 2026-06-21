#!/usr/bin/env bash
#
# Attach a debugger to the STM32L476 via ST-Link and report where the CPU is.
# Useful to confirm whether the firmware is stuck in Error_Handler() (e.g. a
# crystal that won't oscillate hanging SystemClock_Config).
#
# Usage:
#   ./debug.sh            halt, print PC/LR/SP + backtrace, then detach (one-shot)
#   ./debug.sh --interactive   halt and drop into an interactive gdb session
#
# Requires an ARM-aware gdb: gdb-multiarch  (sudo apt install gdb-multiarch)
#                        or  arm-none-eabi-gdb
# The native x86 `gdb` CANNOT debug Cortex-M and is intentionally not used.
#
set -euo pipefail

cd "$(dirname "$0")"

TARGET="solar-sense-v1-fw"
ELF="build/${TARGET}.elf"
PORT=4242

# --- locate an ARM-capable gdb ----------------------------------------------
GDB=""
for cand in gdb-multiarch arm-none-eabi-gdb; do
  if command -v "$cand" >/dev/null 2>&1; then GDB="$cand"; break; fi
done
[ -n "$GDB" ] || {
  echo "error: no ARM gdb found. Install one:" >&2
  echo "       sudo apt install gdb-multiarch" >&2
  exit 1
}

command -v st-util >/dev/null 2>&1 || { echo "error: st-util not found (stlink-tools)." >&2; exit 1; }
[ -f "$ELF" ] || { echo "error: $ELF not found - run ./flash.sh or make first." >&2; exit 1; }

# --- clear any wedged st-util, then start a fresh gdbserver -----------------
pkill -9 -f st-util 2>/dev/null || true
sleep 1

echo ">> starting st-util gdbserver on :$PORT"
st-util > /tmp/stutil.log 2>&1 &
STUTIL_PID=$!
# make sure we always reap the gdbserver on exit
trap 'kill "$STUTIL_PID" 2>/dev/null || true' EXIT
sleep 2

if ! kill -0 "$STUTIL_PID" 2>/dev/null; then
  echo "error: st-util failed to start. Log:" >&2
  cat /tmp/stutil.log >&2
  exit 1
fi

# --- common gdb connect commands --------------------------------------------
COMMON=(-q "$ELF"
  -ex "set architecture armv7e-m"
  -ex "set confirm off"
  -ex "target extended-remote :$PORT"
  -ex "monitor halt")

if [ "${1:-}" = "--interactive" ]; then
  echo ">> interactive gdb (type 'continue' to resume, 'quit' to exit)"
  "$GDB" "${COMMON[@]}"
else
  echo ">> halting and reading CPU state"
  "$GDB" "${COMMON[@]}" \
    -ex "info registers pc lr sp xpsr" \
    -ex "echo \n== backtrace ==\n" \
    -ex "backtrace" \
    -ex "echo \n(If the backtrace shows Error_Handler <- SystemClock_Config,\n a crystal/clock source is not coming up.)\n" \
    -ex "detach" \
    -ex "quit"
fi
