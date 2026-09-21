#!/usr/bin/env bash
# Flash, then drop into gdb stopped at the reset handler.
#
# Runs OpenOCD as a child and kills it on exit, so there is no stray gdbserver
# holding the probe afterwards. That is the usual way to end up with a
# "device busy" on the next attempt.
set -euo pipefail
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cyros_root="$(cd "$here/../../.." && pwd)"
elf="$cyros_root/out/hardware/u575/cyros-u575.elf"

command -v openocd >/dev/null || { echo "openocd not installed. See ~/starch-env toolchain role." >&2; exit 1; }
[ -f "$elf" ] || { echo "no image at $elf. Run ./build.sh first." >&2; exit 1; }

openocd -f "$here/openocd.cfg" -c "init" -c "arm semihosting enable" -c "reset halt" &
ocd=$!
trap 'kill $ocd 2>/dev/null || true' EXIT

# Give the gdbserver a moment to bind :3333.
for _ in $(seq 1 40); do
   (exec 3<>/dev/tcp/127.0.0.1/3333) 2>/dev/null && break
   sleep 0.25
done

arm-none-eabi-gdb "$elf" -x "$here/cyros.gdb"
