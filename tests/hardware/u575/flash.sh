#!/usr/bin/env bash
# Flash the NUCLEO-U575ZI-Q and leave it running.
set -euo pipefail
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cyros_root="$(cd "$here/../../.." && pwd)"
elf="$cyros_root/out/hardware/u575/cyros-u575.elf"

command -v openocd >/dev/null || { echo "openocd not installed. See ~/starch-env toolchain role." >&2; exit 1; }
[ -f "$elf" ] || { echo "no image at $elf. Run ./build.sh first." >&2; exit 1; }

exec openocd -f "$here/openocd.cfg" \
   -c "init" \
   -c "arm semihosting enable" \
   -c "program $elf verify reset" \
   -c "shutdown"
