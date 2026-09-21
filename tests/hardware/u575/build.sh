#!/usr/bin/env bash
# Build the NUCLEO-U575ZI-Q image.
#
# A script on top of cyros-builder rather than a builder feature, because this
# produces an APPLICATION and the builder has never linked one. It uses the
# builder for what the builder is for (assembling libcyros.a from the manifests)
# and does the final link itself.
set -euo pipefail

here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cyros_root="$(cd "$here/../../.." && pwd)"
profile="$cyros_root/build/profiles/cortex_m33_bringup.toml"

# Build output goes to out/, like everything else in this project. Writing into
# the source tree meant .elf, .bin and .map sat next to the sources: .o is
# gitignored but those three are not, so they would have been committed.
out="$cyros_root/out/hardware/u575"

# The archive and the generated include tree, from the manifests.
( cd "$cyros_root" && cyros-builder build -p "$profile" )

lib_dir="$cyros_root/out/cortex_m33_bringup/arm-none-eabi-debug"

mkdir -p "$out"

# -Og not -O0: the port's context switch is hand-written assembly whose
# correctness does not depend on the optimisation level, and -O0 on a 4 MHz
# part is needlessly slow. -g3 for the debugger, which is the entire point.
# Must match build/toolchains/arm-none-eabi-base.toml exactly. Hard float
# changes the ABI, so an image whose application objects disagree with
# libcyros.a will not link, and if it did it would pass arguments in the wrong
# registers. The STM32U575's FPU is the same FPv5-SP-D16 the bench models.
common=(-mcpu=cortex-m33 -mthumb -mfloat-abi=hard -mfpu=fpv5-sp-d16
        -ffunction-sections -fdata-sections -Og -g3)
cxx=("${common[@]}" -std=gnu++26 -fno-exceptions -fno-rtti
     -Wall -Werror -Wextra -Wpedantic)

arm-none-eabi-gcc "${common[@]}" -c "$here/startup_stm32u575.c" -o "$out/startup.o"
arm-none-eabi-gcc "${common[@]}" -c "$here/board_clock.c"       -o "$out/board_clock.o"
# The bench's newlib stubs, referenced rather than copied. They are board
# independent (they exist to PANIC if anything reaches a heap or a file
# descriptor) and a second copy here would drift from the one the QEMU tests use.
arm-none-eabi-gcc "${common[@]}" \
   -c "$cyros_root/tests/unit/port/arm_bench/syscall_stubs.c" -o "$out/syscall_stubs.o"
arm-none-eabi-g++ "${cxx[@]}" \
   -I "$lib_dir/include" -I "$cyros_root/tests/unit" \
   -c "$here/main.cpp" -o "$out/main.o"

# -nostdlib++ enforces the rule that cyros uses no libstdc++ SYMBOL. Same flag
# the QEMU toolchain carries, for the same reason.
arm-none-eabi-g++ "${common[@]}" -nostartfiles -nostdlib++ -Wl,--gc-sections \
   -T "$here/stm32u575.ld" \
   "$out/startup.o" "$out/board_clock.o" "$out/syscall_stubs.o" "$out/main.o" \
   "$lib_dir/lib/libcyros.a" \
   -o "$out/cyros-u575.elf" -Wl,-Map="$out/cyros-u575.map"

arm-none-eabi-objcopy -O binary "$out/cyros-u575.elf" "$out/cyros-u575.bin"

echo
arm-none-eabi-size "$out/cyros-u575.elf"
echo
echo "image: $out/cyros-u575.elf"
