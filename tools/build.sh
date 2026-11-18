#!/bin/bash
# Build the scanner with arduino-cli and merge it into one .bin that flashes at 0x0
# (bootloader + partition table + boot_app0 + app). Same toolchain as H@N-CYD-GBC:
#   arduino-cli, core esp32:esp32 2.0.17, LovyanGFX 1.1.16, plus RadioLib 7.8.1
#
#   tools/build.sh              build docs/firmware/HAN-433-Scanner.bin
#   tools/build.sh flash COM4   build, then write that same image to a board
#   LANDSCAPE=1 tools/build.sh  the landscape build (-DHAN_LANDSCAPE), docs/firmware/HAN-433-Scanner-Landscape.bin
#
# Used by CI (.github/workflows/build.yml) and locally. Override if needed:
#   ACLI=/path/to/arduino-cli         (default: arduino-cli on PATH)
#   ACLI_CONFIG=/path/to/arduino-cli.yaml   (a separate data folder, so an Arduino
#                                             IDE with a newer ESP32 core is not touched)
#   ESPTOOL="python -m esptool"       (default: that if it works, else the core's own)
set -e

ROOT=$(cd "$(dirname "$0")/.." && pwd)
ACLI=${ACLI:-arduino-cli}
CFG=()
[ -n "$ACLI_CONFIG" ] && CFG=(--config-file "$ACLI_CONFIG")
WORK=${WORK:-$ROOT/build}
FQBN=esp32:esp32:esp32:FlashMode=dio,FlashFreq=40,CPUFreq=240,PartitionScheme=default,DebugLevel=none
NAME=HAN-433-Scanner; EXTRA=()
if [ -n "$LANDSCAPE" ]; then
  NAME=HAN-433-Scanner-Landscape
  EXTRA=(--build-property "compiler.cpp.extra_flags=-DHAN_LANDSCAPE")
  WORK=$WORK/landscape
fi
OUT="$ROOT/docs/firmware/$NAME.bin"

command -v "$ACLI" >/dev/null 2>&1 || [ -x "$ACLI" ] || { echo "arduino-cli not found at: $ACLI (set ACLI=...)" >&2; exit 1; }

# Where arduino-cli keeps its cores, for boot_app0.bin and its esptool.
#   arduino-cli 1.x: "config get"; 0.35 (inside Arduino IDE 2): only "config dump" has it.
DATA=$("$ACLI" "${CFG[@]}" config get directories.data 2>/dev/null) || DATA=""
[ -d "$DATA" ] || DATA=$("$ACLI" "${CFG[@]}" config dump | sed -n 's/^ *data: *//p' | head -1)
CORE="$DATA/packages/esp32/hardware/esp32/2.0.17"
[ -d "$CORE" ] || { echo "ESP32 core 2.0.17 not installed in $DATA (arduino-cli core install esp32:esp32@2.0.17)" >&2; exit 1; }

if [ -z "$ESPTOOL" ]; then
  if python -m esptool version >/dev/null 2>&1; then ESPTOOL="python -m esptool"
  else ESPTOOL=$(ls "$DATA"/packages/esp32/tools/esptool_py/*/esptool* 2>/dev/null | grep -E 'esptool(\.exe|\.py)?$' | head -1)
  fi
fi
[ -n "$ESPTOOL" ] || { echo "esptool not found (set ESPTOOL=...)" >&2; exit 1; }

rm -rf "$WORK/out"
echo "== compile"
"$ACLI" "${CFG[@]}" compile --fqbn "$FQBN" "${EXTRA[@]}" --build-path "$WORK/cache" --output-dir "$WORK/out" "$ROOT" 2>&1 \
  | grep -E "Sketch uses|Global variables|error|Error" || true
APP="$WORK/out/$(basename "$ROOT").ino.bin"
[ -f "$APP" ] || { echo "FAILED: no app image produced" >&2; exit 1; }

echo "== merge -> docs/firmware/$NAME.bin"
$ESPTOOL --chip esp32 merge_bin -o "$OUT" \
  --flash_mode dio --flash_freq 40m --flash_size 4MB \
  0x1000 "$WORK/out/$(basename "$ROOT").ino.bootloader.bin" \
  0x8000 "$WORK/out/$(basename "$ROOT").ino.partitions.bin" \
  0xe000 "$CORE/tools/partitions/boot_app0.bin" \
  0x10000 "$APP" >/dev/null
ls -l "$OUT"

if [ "$1" = "flash" ]; then
  PORT=${2:?usage: tools/build.sh flash COM4}
  echo "== flash $PORT"
  $ESPTOOL --chip esp32 --port "$PORT" --baud 460800 write_flash 0x0 "$OUT"
fi
