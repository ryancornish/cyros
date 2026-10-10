#!/usr/bin/env bash
# Build the Orbit demo and put it on the EK-TM4C123GXL, where it stays.
#
#   ./run_demo.sh [--mhz 16|80] [--build-only] [--watch SECONDS]
#
#   1. the builder builds libcyros.a for the demo's profile (orbit_demo.toml):
#      the cortex_m port, tickless SysTick, sync, chrono and channel.
#   2. the demo is compiled against that include tree with the M4 toolchain's
#      flags, and linked with the board's own files from .., the same ones the
#      tests use (startup, clock, console, linker script).
#   3. OpenOCD programs flash and lets the board run, then detaches. Unlike a
#      test, the demo never ends, and it survives a power cycle.
#
# --watch prints the console for that many seconds after programming: the
# logger thread's boot line, every input change, and a status line every five
# seconds. Or watch it yourself: picocom -b 921600 /dev/ttyACM0.
#
# The image is ROM + RAM like a test's (../tm4c123.ld). --mhz is the board
# clock (../board.h), 80 by default here because the demo has no reason to be
# slow.
set -euo pipefail

here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
board="$(cd "$here/.." && pwd)"
cyros_root="$(cd "$here/../../../.." && pwd)"

usage="usage: run_demo.sh [--mhz 16|80] [--build-only] [--watch SECONDS]"
mode=run mhz=80 watch=0
while [ $# -gt 0 ]; do
   case "$1" in
      --build-only) mode=build ;;
      --mhz)        mhz="${2:?--mhz needs 16 or 80}"; shift ;;
      --watch)      watch="${2:?--watch needs a number of seconds}"; shift ;;
      *)            echo "$usage" >&2; exit 2 ;;
   esac
   shift
done
case "$mhz" in 16|80) ;; *) echo "--mhz must be 16 or 80" >&2; exit 2 ;; esac

build_root="$cyros_root/out/tm4c123_orbit_demo/arm-none-eabi-debug-m4"
out="$cyros_root/out/hardware/tm4c123/orbit_demo/${mhz}mhz"
mkdir -p "$out"

# 1.
echo "== libcyros.a (orbit_demo.toml)"
( cd "$cyros_root" && cyros-builder build -p "$here/orbit_demo.toml" ) | tail -n 1
[ -f "$build_root/lib/libcyros.a" ] || { echo "the builder produced no archive" >&2; exit 1; }

# 2. Must match build/toolchains/arm-none-eabi-base.toml plus -debug and
# -debug-m4, as ../run_test.sh does.
common=(-mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16
        -ffunction-sections -fdata-sections -Og -g3 "-DBOARD_SYSCLK_HZ=$((mhz * 1000000))u")
cflags=("${common[@]}" -std=gnu23 -Wall -Werror -Wextra -Wpedantic -fno-exceptions)
cxxflags=("${common[@]}" -std=gnu++26 -Wall -Werror -Wextra -Wpedantic -fno-exceptions -fno-rtti)
includes=(-I "$build_root/include" -I "$board")

objects=()
compile() {  # <source path>
   local object="$out/$(basename "$1").o"
   case "$1" in
      *.c) arm-none-eabi-gcc "${cflags[@]}" "${includes[@]}" -c "$1" -o "$object" ;;
      *)   arm-none-eabi-g++ "${cxxflags[@]}" "${includes[@]}" -c "$1" -o "$object" ;;
   esac
   objects+=("$object")
}
compile "$here/main.cpp"
compile "$here/orbit.cpp"
compile "$here/screen.cpp"
compile "$board/startup_tm4c123.c"
compile "$board/board_clock.c"
compile "$board/console.c"
compile "$cyros_root/tests/unit/common/syscall_stubs.c"

elf="$out/orbit_demo.elf"
arm-none-eabi-g++ "${common[@]}" -nostartfiles -nostdlib++ -Wl,--gc-sections \
   -T "$board/tm4c123.ld" "${objects[@]}" "$build_root/lib/libcyros.a" \
   -o "$elf" -Wl,-Map="${elf%.elf}.map"
echo "== built $elf (${mhz} MHz)"
arm-none-eabi-size "$elf" | tail -n 1

[ "$mode" = "build" ] && exit 0

# 3.
command -v openocd >/dev/null || { echo "openocd not installed" >&2; exit 1; }
tty="${TM4C_TTY:-$(ls /dev/serial/by-id/usb-Texas_Instruments_In-Circuit_Debug_Interface_*-if00 2>/dev/null | head -n 1)}"
baud="$(sed -n 's/^#define BOARD_CONSOLE_BAUD \([0-9]*\)u$/\1/p' "$board/board.h")"
log="$out/console.log"

# The console is opened, and configured while held open, before the board is
# programmed, so the boot line cannot be lost (../run_test.sh explains).
reader=""
if [ "$watch" -gt 0 ]; then
   [ -n "$tty" ] && [ -e "$tty" ] || { echo "no ICDI virtual COM port found, set TM4C_TTY" >&2; exit 1; }
   cat "$tty" > "$log" &
   reader=$!
   trap 'kill "$reader" 2>/dev/null || true' EXIT
   stty -F "$tty" "$baud" raw -echo
fi

echo "== programming the EK-TM4C123GXL"
openocd -f "$board/openocd.cfg" -c init -c "program $elf verify" -c "reset run" \
   -c "mww 0xE000EDF0 0xA05F0000" -c shutdown \
   > "$out/openocd.log" 2>&1 || { cat "$out/openocd.log" >&2; exit 1; }
echo "== running"

if [ -n "$reader" ]; then
   sleep "$watch"
   kill "$reader" 2>/dev/null || true
   wait 2>/dev/null || true
   # The ICDI turns the reset's break on PA1 into NUL bytes (../run_test.sh),
   # and now and then a byte above 0x7E. The demo prints neither.
   LC_ALL=C tr -d '\000\177-\377' < "$log"
fi
