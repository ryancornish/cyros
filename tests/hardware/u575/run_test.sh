#!/usr/bin/env bash
# Run one of the on-target unit tests on the NUCLEO-U575ZI-Q instead of QEMU.
#
#   ./run_test.sh <test name> [--build-only]
#   ./run_test.sh test_cortex_m33_systick
#
# The builder builds every on-target test for QEMU's mps2-an505 and runs it
# there. The same test runs on the real part unchanged, because a test is
# written against bench.hpp (semihosting, which works over SWD too) and enters
# at cyros_bench_main, which the U575 startup calls exactly as the bench's does.
# Only the BOARD differs: startup, linker script and the SysTick clock. So:
#
#   1. the builder builds the test, and runs it under QEMU, a free baseline;
#   2. the builder's per-test libcyros.a and include tree are reused as they
#      are, since the test's config header is baked into both;
#   3. the test's own sources are recompiled with the U575 board files instead
#      of the bench's, with the ARM toolchain's flags, and linked for the U575;
#   4. OpenOCD programs the board and stays up to service semihosting until the
#      test prints its RESULT line.
#
# Single-core tests only (the cortex_m33 port). The U575 has one core.
set -euo pipefail

here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cyros_root="$(cd "$here/../../.." && pwd)"

test="${1:?usage: run_test.sh <test name> [--build-only]}"
mode="${2:-run}"

test_dir="$(find "$cyros_root/tests/unit" -type d -name "$test" | head -n 1)"
[ -n "$test_dir" ] && [ -f "$test_dir/test.toml" ] || { echo "no test directory named $test" >&2; exit 2; }

profile=unit_test_cortex_m33
build_root="$cyros_root/out/tests/$test/$profile/arm-none-eabi-debug"
out="$cyros_root/out/hardware/u575/tests/$test"
mkdir -p "$out"

# 1. The builder's own build of the test. Its QEMU verdict is reported, not
# used as a gate: a test that fails on the bench is still worth running here.
echo "== QEMU baseline"
( cd "$cyros_root" && cyros-builder test -p "build/profiles/$profile.toml" --filter "$test" ) \
   | grep -E "Results|\[(PASS|FAIL|BUILD|BLOCK|SKIP) *\]" || true
[ -f "$build_root/lib/libcyros.a" ] || { echo "the builder produced no archive for $test" >&2; exit 1; }

# 2 and 3. Must match build/toolchains/arm-none-eabi-base.toml plus -debug.
common=(-mcpu=cortex-m33 -mthumb -mfloat-abi=hard -mfpu=fpv5-sp-d16
        -ffunction-sections -fdata-sections -Og -g3)
cflags=("${common[@]}" -std=gnu23 -Wall -Werror -Wextra -Wpedantic -fno-exceptions)
cxxflags=("${common[@]}" -std=gnu++26 -Wall -Werror -Wextra -Wpedantic -fno-exceptions -fno-rtti)
includes=(-I "$build_root/include" -I "$cyros_root/src/port/internal_headers"
          -I "$cyros_root/tests/unit" -I "$test_dir")

# The test's sources, minus the bench's startup, which the U575's replaces.
mapfile -t sources < <(python3 - "$test_dir/test.toml" <<'PY'
import sys, tomllib
with open(sys.argv[1], "rb") as f:
    src = tomllib.load(f)["test"]["source"]
for s in ([src] if isinstance(src, str) else src):
    if "arm_bench/startup" not in s:
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
compile "$here/startup_stm32u575.c" startup_stm32u575.o
compile "$here/board_clock.c"       board_clock.o

elf="$out/$test.elf"
arm-none-eabi-g++ "${common[@]}" -nostartfiles -nostdlib++ -Wl,--gc-sections \
   -T "$here/stm32u575.ld" "${objects[@]}" "$build_root/lib/libcyros.a" \
   -o "$elf" -Wl,-Map="$out/$test.map"
echo "== built $elf"
arm-none-eabi-size "$elf" | tail -n 1

[ "$mode" = "--build-only" ] && exit 0

# 4. Program, run, and service semihosting until the verdict appears. OpenOCD
# prints the target's semihosting output on its own output.
command -v openocd >/dev/null || { echo "openocd not installed" >&2; exit 1; }
log="$out/$test.log"
echo "== running on the NUCLEO-U575ZI-Q (log: $log)"
openocd -f "$here/openocd.cfg" \
   -c "init" -c "arm semihosting enable" \
   -c "program $elf verify" -c "reset run" > "$log" 2>&1 &
ocd=$!
trap 'kill "$ocd" 2>/dev/null || true' EXIT

deadline=$((SECONDS + ${U575_TEST_TIMEOUT:-180}))
until grep -q "RESULT:" "$log" 2>/dev/null; do
   if ! kill -0 "$ocd" 2>/dev/null; then echo "openocd exited early" >&2; break; fi
   if [ "$SECONDS" -ge "$deadline" ]; then echo "timed out waiting for RESULT" >&2; break; fi
   sleep 1
done
sleep 1   # let the checks summary that precedes RESULT finish arriving

grep -v -E "^(Info|Warn|Open On-Chip|Licensed|For bug|debug_level|clock_config|srst_only)" "$log" | sed -n '/cortex_m33\|===\|checks\|RESULT\|FAIL\|PANIC/p'
grep -q "RESULT: PASS" "$log"
