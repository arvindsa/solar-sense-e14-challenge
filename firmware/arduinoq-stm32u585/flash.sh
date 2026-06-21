#!/bin/bash
set -e

TARGET=solar-sense-qstm-fw
BUILD_DIR=build
BIN=$BUILD_DIR/$TARGET.bin
DEVICE_TMP=/tmp/$TARGET.bin
OPENOCD_DIR=/opt/openocd
OPENOCD=$OPENOCD_DIR/bin/openocd
FLASH_ADDR=0x08000000

echo "==> Building..."
make -j$(nproc)

echo "==> Pushing binary to UNO Q..."
adb push "$BIN" "$DEVICE_TMP"

# Read initial MSP (vector[0]) and reset vector (vector[1]) from the binary.
# We launch the app manually after flashing, so we must restore BOTH the stack
# pointer and the PC -- the hardware never does a vector fetch for us here.
read MSP PC < <(python3 - "$BIN" <<'PY'
import sys, struct
sp, pc = struct.unpack('<II', open(sys.argv[1], 'rb').read(8))
print(f'0x{sp:08x} 0x{pc & ~1:08x}')
PY
)
echo "==> Initial MSP=$MSP  Reset vector=$PC"

# Flash with a ONE-SHOT OpenOCD running on the device (no persistent instance,
# no telnet tunnel -- those silently no-op'd when OpenOCD wasn't running).
#
#   reset; halt        -> clean, writable state. This lands in the factory
#                         bootloader (0x0BF9xxxx), which is fine: we only need
#                         the core halted to program flash.
#   flash write_image  -> program our firmware at 0x08000000.
#   reg msp/pc; resume -> launch the app WITHOUT a hardware reset. A reset would
#                         re-route the STM32U585 into the factory bootloader
#                         (boot option bytes) instead of our flash, so we set
#                         the vector manually and resume in place.
#   shutdown           -> exit OpenOCD, releasing the SWD GPIO lines cleanly.
OCMD="reset_config srst_only srst_push_pull; init; reset; halt;"
OCMD+=" flash write_image erase $DEVICE_TMP $FLASH_ADDR bin;"
OCMD+=" reg msp $MSP; reg pc $PC; resume; shutdown"

echo "==> Flashing via on-device OpenOCD..."
adb shell "$OPENOCD -s $OPENOCD_DIR -f openocd_gpiod.cfg -c '$OCMD'"

echo "==> Done!"
echo "    App launched at $PC for THIS power session only. A power cycle or"
echo "    hardware reset boots the factory bootloader, not your flash."
