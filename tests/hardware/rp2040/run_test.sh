#!/usr/bin/env bash
# The builder's runner for the RP2040 Pico: runs one test image and exits with
# its status, as QEMU does for the benches.
#
#   run_test.sh <elf>
#   cyros-builder test -p build/profiles/unit_test_rp2040.toml
#
# OpenOCD loads the image into SRAM over the Debug Probe after a reset halt and
# starts it, the Pico 2 W runner's RP2350_BOOT=swd path (../rp2350). Each run
# ends with a watchdog reset of the whole chip, which with nothing in flash
# leaves the board in BOOTSEL.
#
# Needs the Debug Probe on the board's SWD pads ("D"), with nothing else
# holding OpenOCD's Tcl port. RP2040_TEST_TIMEOUT (30 s) bounds the image once
# it has started. With the driver's 8 s wait for the start and one retry it
# stays under the builder's default --test-timeout of 60 s, so the driver
# always gets to reset the board. The test's console arrives over RTT on
# stdout, and a port panic, which is semihosting, arrives in OpenOCD's log,
# shown on failure.
set -euo pipefail

here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
elf="${1:?usage: run_test.sh <elf>}"
timeout_s="${RP2040_TEST_TIMEOUT:-30}"
port=6666
log="$elf.openocd.log"

lsusb -d 2e8a:000c >/dev/null 2>&1 || { echo "rp2040: no Raspberry Pi Debug Probe on USB" >&2; exit 125; }
if [[ -n $(ss -Hltn "sport = :$port") ]]; then
   echo "rp2040: port $port is taken, another OpenOCD is running" >&2
   exit 125
fi

# The two cores as separate OpenOCD targets, not rp2040.cfg's SMP group: on
# the RP2350 the group left core 0 halted for good after a semihosting call,
# because core 1 never halted (rp2350-notes.md 7b). Any USE_CORE but 0, 1 and
# SMP configures both cores apart.
openocd_start() {
   # --foreground keeps OpenOCD in this script's process group, so the
   # builder's kill on a timeout reaches it. Bounded either way.
   timeout --foreground $((timeout_s + 30)) openocd-rp2350 -f interface/cmsis-dap.cfg \
      -c "adapter speed 5000" -c "set USE_CORE BOTH" -f target/rp2040.cfg -c "gdb port disabled" \
      -c "telnet port disabled" -c "tcl port $port" ${RP2040_OPENOCD_DEBUG:+-d3} >>"$log" 2>&1 &
   ocd=$!
}
ocd=""
trap '[[ -n $ocd ]] && kill $ocd 2>/dev/null || true' EXIT
: >"$log"

# One run of the image. 125 is the bench failing (no board, an image the
# runner never saw start), anything else is the image's own status.
attempt() {
   openocd_start
   local rc=0
   python3 -B "$here/drive_test.py" "$elf" --timeout "$timeout_s" --port "$port" || rc=$?
   wait "$ocd" || true
   ocd=""
   if (( rc != 0 )); then
      echo "--- OpenOCD ($log):" >&2
      # Without the driver's own Tcl replies (bare numbers and memory words), and
      # without the watchdog trigger's write, which fails because the chip resets
      # under it.
      grep -vE '^(Info|Open On-Chip|Licensed|For bug|	http)|failed to read memory|^(-?[0-9]+|0x[0-9a-f]+( 0x[0-9a-f]+)*)$|^$|[Ff]ailed to write memory|SWD DPIDR|shutdown command' "$log" | tail -20 >&2 || true
   fi
   return $rc
}

# An image that never visibly started is retried once from the top, as on the
# Pico 2 W board. A test that started and hangs is never retried.
rc=0
attempt || rc=$?
if (( rc == 125 )); then
   echo "rp2040: retrying once, from the top" >&2
   : >"$log"
   rc=0
   attempt || rc=$?
fi
exit $rc
