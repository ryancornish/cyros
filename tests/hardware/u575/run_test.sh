#!/usr/bin/env bash
# Run one of the on-target unit tests on the NUCLEO-U575ZI-Q instead of QEMU.
#
#   ./run_test.sh <test name> [--mhz 4|160] [--build-only]
#   ./run_test.sh test_cortex_m_systick --mhz 160
#
# The builder builds every on-target test for QEMU's mps2-an505 and runs it
# there. The same test runs on the real part unchanged, because a test is
# written against bench.hpp, whose console and exit the BOARD supplies, and
# enters at cyros_bench_main, which the U575 startup calls exactly as the
# bench's does. Only the board differs: startup, clock, console and linker
# script. So:
#
#   1. the builder builds the test, and runs it under QEMU, a free baseline;
#   2. the builder's per-test libcyros.a and include tree are reused as they
#      are, since the test's config header is baked into both;
#   3. the test's own sources are recompiled with the U575 board files instead
#      of the bench's, with the ARM toolchain's flags, and linked for the U575;
#   4. OpenOCD programs the board, and the test's output is read from the
#      ST-LINK's virtual COM port until the board prints its final "EXIT: n"
#      line (console.c, startup_stm32u575.c).
#
# --mhz is the board clock (board.h): 4, the reset clock and the default, or
# 160 from PLL1. It changes the SysTick rate cyros is told and the console's
# baud divisor together, from the one constant.
#
# OpenOCD stays attached while the test runs, although no test output goes
# through it, because two things still use semihosting: the PORT's panic report
# (src/port/arm/armv7m_armv8m/cortex_m.hpp) and bench.hpp's elapsed-time reference. A
# panic therefore still arrives, on OpenOCD's output, and this script shows it.
#
# Single-core tests only (the cortex_m port). The U575 has one core.
set -euo pipefail

here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cyros_root="$(cd "$here/../../.." && pwd)"

usage="usage: run_test.sh <test name> [--mhz 4|160] [--build-only]"
test="" mode=run mhz=4
while [ $# -gt 0 ]; do
   case "$1" in
      --build-only) mode=build ;;
      --mhz)        mhz="${2:?--mhz needs 4 or 160}"; shift ;;
      -*)           echo "$usage" >&2; exit 2 ;;
      *)            test="$1" ;;
   esac
   shift
done
[ -n "$test" ] || { echo "$usage" >&2; exit 2; }
case "$mhz" in 4|160) ;; *) echo "--mhz must be 4 or 160" >&2; exit 2 ;; esac

test_dir="$(find "$cyros_root/tests/unit" -type d -name "$test" | head -n 1)"
[ -n "$test_dir" ] && [ -f "$test_dir/test.toml" ] || { echo "no test directory named $test" >&2; exit 2; }

profile=unit_test_cortex_m33
build_root="$cyros_root/out/tests/$test/$profile/arm-none-eabi-debug"
out="$cyros_root/out/hardware/u575/tests/$test/${mhz}mhz"
mkdir -p "$out"

# 1. The builder's own build of the test. Its QEMU verdict is reported, not
# used as a gate: a test that fails on the bench is still worth running here.
echo "== QEMU baseline"
( cd "$cyros_root" && cyros-builder test -p "build/profiles/$profile.toml" --filter "$test" ) \
   | grep -E "Results|\[(PASS|FAIL|BUILD|BLOCK|SKIP) *\]" || true
[ -f "$build_root/lib/libcyros.a" ] || { echo "the builder produced no archive for $test" >&2; exit 1; }

# 2 and 3. Must match build/toolchains/arm-none-eabi-base.toml plus -debug.
common=(-mcpu=cortex-m33 -mthumb -mfloat-abi=hard -mfpu=fpv5-sp-d16
        -ffunction-sections -fdata-sections -Og -g3 "-DBOARD_SYSCLK_HZ=$((mhz * 1000000))u")
cflags=("${common[@]}" -std=gnu23 -Wall -Werror -Wextra -Wpedantic -fno-exceptions)
cxxflags=("${common[@]}" -std=gnu++26 -Wall -Werror -Wextra -Wpedantic -fno-exceptions -fno-rtti)
includes=(-I "$build_root/include" -I "$cyros_root/src/port/internal_headers"
          -I "$cyros_root/tests/unit" -I "$test_dir")

# The test's own sources. A test.toml lists no board files: the builder takes
# the bench's from its toolchain, and the U575's replace them below.
mapfile -t sources < <(python3 - "$test_dir/test.toml" <<'PY'
import sys, tomllib
with open(sys.argv[1], "rb") as f:
    src = tomllib.load(f)["test"]["source"]
for s in ([src] if isinstance(src, str) else src):
    print(s)
PY
)

# And its own link flags ([link].flags), which the builder passes to the link
# too. test_cortex_m_idle observes idle through a --wrap there.
mapfile -t link_flags < <(python3 - "$test_dir/test.toml" <<'PY'
import sys, tomllib
with open(sys.argv[1], "rb") as f:
    flags = tomllib.load(f).get("link", {}).get("flags", [])
for flag in flags:
    print(flag)
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
compile "$here/startup_stm32u575.c" startup_stm32u575.o
compile "$here/board_clock.c"       board_clock.o
compile "$here/console.c"           console.o
compile "$cyros_root/tests/unit/port/arm_bench/syscall_stubs.c" syscall_stubs.o

elf="$out/$test.elf"
arm-none-eabi-g++ "${common[@]}" -nostartfiles -nostdlib++ -Wl,--gc-sections \
   -T "$here/stm32u575.ld" "${objects[@]}" "$build_root/lib/libcyros.a" "${link_flags[@]}" \
   -o "$elf" -Wl,-Map="$out/$test.map"
echo "== built $elf (${mhz} MHz)"
arm-none-eabi-size "$elf" | tail -n 1

[ "$mode" = "build" ] && exit 0

# 4. Program, run, and read the console until the board reports its exit.
command -v openocd >/dev/null || { echo "openocd not installed" >&2; exit 1; }
tty="${U575_TTY:-$(ls /dev/serial/by-id/usb-STMicroelectronics_STLINK-V3_*-if02 2>/dev/null | head -n 1)}"
[ -n "$tty" ] && [ -e "$tty" ] || { echo "no ST-LINK virtual COM port found, set U575_TTY" >&2; exit 1; }
baud="$(sed -n 's/^#define BOARD_CONSOLE_BAUD \([0-9]*\)u$/\1/p' "$here/board.h")"
[ -n "$baud" ] || { echo "cannot read BOARD_CONSOLE_BAUD from board.h" >&2; exit 1; }

log="$out/$test.log"
ocd_log="$out/$test.openocd.log"
echo "== running on the NUCLEO-U575ZI-Q at ${mhz} MHz (console: $tty, log: $log)"

# The port is opened, and configured while held open, BEFORE the board is
# programmed, so the first line cannot be lost. Raw, no echo: an echo would
# send every received byte back to the board's RX.
cat "$tty" > "$log" &
reader=$!
stty -F "$tty" "$baud" raw -echo
openocd -f "$here/openocd.cfg" \
   -c "init" -c "program $elf verify" -c "reset run" > "$ocd_log" 2>&1 &
ocd=$!
trap 'kill "$ocd" "$reader" 2>/dev/null || true' EXIT

deadline=$((SECONDS + ${U575_TEST_TIMEOUT:-180}))
verdict=""
until grep -q "^EXIT: " "$log" 2>/dev/null; do
   # OpenOCD ends itself on a semihosting exit, which on this board means the
   # port panicked. Its report is in OpenOCD's log.
   if ! kill -0 "$ocd" 2>/dev/null; then verdict="openocd exited (a port panic reports this way)"; break; fi
   if [ "$SECONDS" -ge "$deadline" ]; then verdict="timed out waiting for EXIT"; break; fi
   sleep 0.2
done
kill "$reader" "$ocd" 2>/dev/null || true
wait 2>/dev/null || true

cat "$log"
# OpenOCD's log matters only when something went wrong: a port panic arrives
# there through semihosting, and so does a failure to program.
if [ -n "$verdict" ] || grep -q -E "PANIC|Error" "$ocd_log"; then
   echo "== OpenOCD ($ocd_log):"
   grep -v -E "^(Info|Open On-Chip|Licensed|For bug|	http)" "$ocd_log" || true
fi
[ -n "$verdict" ] && echo "$verdict" >&2

grep -q "^RESULT: PASS" "$log" && grep -q "^EXIT: 0$" "$log"
