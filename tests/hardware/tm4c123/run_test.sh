#!/usr/bin/env bash
# Run one of the on-target unit tests on the EK-TM4C123GXL instead of QEMU.
#
#   ./run_test.sh <test name> [--mhz 16|80] [--build-only]
#   ./run_test.sh test_cortex_m_systick --mhz 80
#
# The builder builds every on-target test for QEMU's mps2-an386, the Cortex-M4F
# bench, and runs it there. The same test runs on this board unchanged, the way
# ../u575/run_test.sh does it:
#
#   1. the builder builds the test, and runs it under QEMU, a free baseline;
#   2. the builder's per-test libcyros.a and include tree are reused as they
#      are, since the test's config header is baked into both;
#   3. the test's own sources are recompiled with this board's files and the
#      M4 toolchain's flags, and linked for the TM4C123;
#   4. OpenOCD programs the image into flash, and the test's output is read
#      from the ICDI's virtual COM port until the board prints its final
#      "EXIT: n" line (console.c, startup_tm4c123.c).
#
# The image is ROM + RAM, the model for a memory-constrained part: code and
# read-only data execute in place from the 256 KB of flash, and only .data,
# .bss and the main stack take the 32 KB of SRAM (tm4c123.ld).
#
# OpenOCD stays attached while the test runs, because the port's panic report
# still goes through semihosting (src/port/arm/armv7m_armv8m/cortex_m.hpp). A
# panic therefore arrives on OpenOCD's output, and this script shows it.
#
# --mhz is the board clock (board.h): 16, the crystal and the default, or 80
# from the PLL. It changes the SysTick rate cyros is told and the console's
# baud divisor together, from the one constant.
#
# Single-core tests only (the cortex_m port). The TM4C123 has one core.
set -euo pipefail

here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cyros_root="$(cd "$here/../../.." && pwd)"

usage="usage: run_test.sh <test name> [--mhz 16|80] [--build-only]"
test="" mode=run mhz=16
while [ $# -gt 0 ]; do
   case "$1" in
      --build-only) mode=build ;;
      --mhz)        mhz="${2:?--mhz needs 16 or 80}"; shift ;;
      -*)           echo "$usage" >&2; exit 2 ;;
      *)            test="$1" ;;
   esac
   shift
done
[ -n "$test" ] || { echo "$usage" >&2; exit 2; }
case "$mhz" in 16|80) ;; *) echo "--mhz must be 16 or 80" >&2; exit 2 ;; esac

test_dir="$(find "$cyros_root/tests/unit" -type d -name "$test" | head -n 1)"
[ -n "$test_dir" ] && [ -f "$test_dir/test.toml" ] || { echo "no test directory named $test" >&2; exit 2; }

profile=unit_test_cortex_m4
build_root="$cyros_root/out/tests/$test/$profile/arm-none-eabi-debug-m4"
out="$cyros_root/out/hardware/tm4c123/tests/$test/${mhz}mhz"
mkdir -p "$out"

# 1. The builder's own build of the test. Its QEMU verdict is reported, not
# used as a gate: a test that fails on the bench is still worth running here.
echo "== QEMU baseline (mps2-an386)"
( cd "$cyros_root" && cyros-builder test -p "build/profiles/$profile.toml" --filter "$test" ) \
   | grep -E "Results|\[(PASS|FAIL|BUILD|BLOCK|SKIP) *\]" || true
[ -f "$build_root/lib/libcyros.a" ] || { echo "the builder produced no archive for $test" >&2; exit 1; }

# 2 and 3. Must match build/toolchains/arm-none-eabi-base.toml plus -debug and
# -debug-m4.
common=(-mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16
        -ffunction-sections -fdata-sections -Og -g3 "-DBOARD_SYSCLK_HZ=$((mhz * 1000000))u")
cflags=("${common[@]}" -std=gnu23 -Wall -Werror -Wextra -Wpedantic -fno-exceptions)
cxxflags=("${common[@]}" -std=gnu++26 -Wall -Werror -Wextra -Wpedantic -fno-exceptions -fno-rtti)
includes=(-I "$build_root/include" -I "$cyros_root/src/port/internal_headers"
          -I "$cyros_root/tests/unit" -I "$test_dir")

# The test's own sources. A test.toml lists no board files: the builder takes
# the bench's from its toolchain, and this board's replace them below.
mapfile -t sources < <(python3 - "$test_dir/test.toml" <<'PY'
import sys, tomllib
with open(sys.argv[1], "rb") as f:
    src = tomllib.load(f)["test"]["source"]
for s in ([src] if isinstance(src, str) else src):
    print(s)
PY
)

objects=()
compile() {  # <source path> <object name>
   case "$1" in
      *.c) arm-none-eabi-gcc "${cflags[@]}" "${includes[@]}" -c "$1" -o "$out/$2" ;;
      *)   arm-none-eabi-g++ "${cxxflags[@]}" "${includes[@]}" -c "$1" -o "$out/$2" ;;
   esac
   objects+=("$out/$2")
}
for s in "${sources[@]}"; do
   compile "$test_dir/$s" "$(basename "$s").o"
done
compile "$here/startup_tm4c123.c" startup_tm4c123.o
compile "$here/board_clock.c"     board_clock.o
compile "$here/console.c"         console.o
compile "$cyros_root/tests/unit/port/arm_bench/syscall_stubs.c" syscall_stubs.o

link() {  # <linker script> <elf>
   arm-none-eabi-g++ "${common[@]}" -nostartfiles -nostdlib++ -Wl,--gc-sections \
      -T "$here/$1" "${objects[@]}" "$build_root/lib/libcyros.a" \
      -o "$2" -Wl,-Map="${2%.elf}.map"
}

elf="$out/$test.elf"
link tm4c123.ld "$elf"
echo "== built $elf (${mhz} MHz)"
arm-none-eabi-size "$elf" | tail -n 1

[ "$mode" = "build" ] && exit 0

# 4. Put it on the board, run, and read the console until the board reports its
# exit.
command -v openocd >/dev/null || { echo "openocd not installed" >&2; exit 1; }
tty="${TM4C_TTY:-$(ls /dev/serial/by-id/usb-Texas_Instruments_In-Circuit_Debug_Interface_*-if00 2>/dev/null | head -n 1)}"
[ -n "$tty" ] && [ -e "$tty" ] || { echo "no ICDI virtual COM port found, set TM4C_TTY" >&2; exit 1; }
baud="$(sed -n 's/^#define BOARD_CONSOLE_BAUD \([0-9]*\)u$/\1/p' "$here/board.h")"
[ -n "$baud" ] || { echo "cannot read BOARD_CONSOLE_BAUD from board.h" >&2; exit 1; }

log="$out/$test.log"
ocd_log="$out/$test.openocd.log"
echo "== running on the EK-TM4C123GXL at ${mhz} MHz (console: $tty, log: $log)"

# Bytes above 0x7E, which no bench output contains. A console full of them
# means the board's clock is not what board.h says, since the baud divisor is
# computed from it: an 80 MHz image that booted at 12.5 MHz arrived as one 0xE0
# per falling edge (board_clock.c). That is a board defect, so it fails, and
# fails at once rather than after the timeout.
unreadable() {
   [ "$(LC_ALL=C tr -d '\000-\176' < "$log" | wc -c)" -ge 16 ]
}

# The port is opened, and configured while held open, BEFORE the board is
# programmed, so the first line cannot be lost. Raw, no echo: an echo would send
# every received byte back to the board's RX.
cat "$tty" > "$log" &
reader=$!
stty -F "$tty" "$baud" raw -echo
openocd -f "$here/openocd.cfg" -c init -c "program $elf verify" -c "reset run" > "$ocd_log" 2>&1 &
ocd=$!
trap 'kill "$ocd" "$reader" 2>/dev/null || true' EXIT

deadline=$((SECONDS + ${TM4C_TEST_TIMEOUT:-180}))
verdict=""
until grep -q "^EXIT: " "$log" 2>/dev/null; do
   # OpenOCD ends itself on a semihosting exit, which on this board means the
   # port panicked. Its report is in OpenOCD's log.
   if ! kill -0 "$ocd" 2>/dev/null; then verdict="openocd exited (a port panic reports this way)"; break; fi
   if unreadable; then verdict="console unreadable: the board is not at ${mhz} MHz (board_clock.c)"; break; fi
   if [ "$SECONDS" -ge "$deadline" ]; then verdict="timed out waiting for EXIT"; break; fi
   sleep 0.2
done
kill "$reader" "$ocd" 2>/dev/null || true
wait 2>/dev/null || true

# A reset tristates PA1, the line to the ICDI idles low until the image hands
# the pin to the UART, and the ICDI turns that break into 0x00 data bytes: two
# per run, out of step with the real data, one often a few bytes in. Every real
# byte arrives (measured 2026-10-03), and the bench can never print a NUL, so
# they go.
tr -d '\000' < "$log" > "$log.tmp" && mv "$log.tmp" "$log"

cat "$log"
# OpenOCD's log matters only when something went wrong: a port panic arrives
# there through semihosting, and so does a failure to program.
if [ -n "$verdict" ] || grep -q -E "PANIC|Error: [^S]" "$ocd_log"; then
   echo "== OpenOCD ($ocd_log):"
   grep -v -E "^(Info|Open On-Chip|Licensed|For bug|	http)" "$ocd_log" || true
fi
[ -n "$verdict" ] && echo "$verdict" >&2

grep -q "^RESULT: PASS" "$log" && grep -q "^EXIT: 0$" "$log"
