#!/usr/bin/env bash
# The builder's runner for the Pico 2 W's Hazard3 core 0: runs one test image
# and exits with its status, as qemu-system-riscv32 does for riscv_virt.
#
#   run_test.sh <elf>
#   cyros-builder test -p build/profiles/unit_test_rp2350_hazard3.toml
#
# RP2350_BOOT=picotool (the default) has the boot ROM load the image into SRAM
# and enter it, the path a product takes (rp2350-notes.md 10). RP2350_BOOT=swd
# loads it over the Debug Probe after a reset halt instead, which needs no
# USB but skips the boot ROM. Nothing is written to flash or to OTP.
#
# Needs the Debug Probe on the board's SWD port ("D"), with nothing else
# holding OpenOCD's Tcl port, and for picotool the board on USB. Each run ends
# with a watchdog reset of the board, which with an empty flash is BOOTSEL
# again, and a board found running something else is reset there first.
#
# RP2350_TEST_TIMEOUT (40 s) bounds the image. It is under the builder's own
# default --test-timeout of 60 s, so the driver always gets to reset the board.
# The test's console arrives over RTT on stdout, and a port panic, which is
# semihosting, arrives in OpenOCD's log, shown on failure.
set -euo pipefail

here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
elf="${1:?usage: run_test.sh <elf>}"
boot="${RP2350_BOOT:-picotool}"
timeout_s="${RP2350_TEST_TIMEOUT:-40}"
port=6666
log="$elf.openocd.log"

lsusb -d 2e8a:000c >/dev/null 2>&1 || { echo "rp2350: no Raspberry Pi Debug Probe on USB" >&2; exit 125; }
if [[ -n $(ss -Hltn "sport = :$port") ]]; then
   echo "rp2350: port $port is taken, another OpenOCD is running" >&2
   exit 125
fi

openocd_start() {
   # --foreground keeps OpenOCD in this script's process group, so the
   # builder's kill on a timeout reaches it. Bounded either way.
   timeout --foreground $((timeout_s + 30)) openocd-rp2350 -f interface/cmsis-dap.cfg \
      -c "adapter speed 5000" -f target/rp2350-riscv.cfg -c "gdb port disabled" \
      -c "telnet port disabled" -c "tcl port $port" ${RP2350_OPENOCD_DEBUG:+-d3} "$@" >>"$log" 2>&1 &
   ocd=$!
}
ocd=""
trap '[[ -n $ocd ]] && kill $ocd 2>/dev/null || true' EXIT
: >"$log"

in_bootsel() { timeout 10 picotool info >/dev/null 2>&1; }

# Up to $1 tries, about 0.3 s apart: picotool fails fast when nothing answers.
wait_for_bootsel() {
   for _ in $(seq "${1:-50}"); do in_bootsel && return 0; sleep 0.2; done
   return 1
}

# A board left running an image (a run the builder killed, or an image loaded
# by hand) is reset through SWD by the watchdog, as drive_test.py ends every
# run (its watchdog_reboot says why not `reset run`). Whichever architecture
# the cores are in answers, so both configurations are tried.
recover() {
   local cfg target
   for cfg in target/rp2350-riscv.cfg target/rp2350.cfg; do
      case $cfg in *riscv*) target=rp2350.rv0 ;; *) target=rp2350.cm0 ;; esac
      timeout --foreground 30 openocd-rp2350 -f interface/cmsis-dap.cfg -c "adapter speed 5000" \
         -f "$cfg" -c "gdb port disabled" -c "telnet port disabled" -c "tcl port disabled" \
         -c init -c "catch {riscv set_mem_access sysbus}" \
         -c "catch {$target write_memory 0x40018008 32 0x1fffff3}" \
         -c "catch {$target write_memory 0x400d8000 32 0x80000000}" \
         -c shutdown >>"$log" 2>&1 || true
      wait_for_bootsel && return 0
   done
   return 1
}

if [[ $boot == picotool ]]; then
   # The last run's watchdog reset brings the board back in about 2 s.
   if ! wait_for_bootsel 12 && ! recover; then
      echo "rp2350: the board is not in BOOTSEL and a reset did not bring it there" >&2
      exit 125
   fi
   # The cores' architecture is chosen at each reboot. The boot ROM would
   # switch it to boot a RISC-V image too, but by rebooting again after the
   # load, which OpenOCD then races. So it is chosen first.
   timeout 20 picotool reboot -u -c riscv >>"$log" 2>&1 || true
   if ! wait_for_bootsel; then
      echo "rp2350: the board did not come back in RISC-V BOOTSEL" >&2
      exit 125
   fi
   # The boot ROM must enter the image before OpenOCD first examines the
   # harts, so OpenOCD starts with noinit and the driver runs init.
   if ! timeout 60 picotool load -x -t elf "$elf" >>"$log" 2>&1; then
      echo "rp2350: picotool could not load the image" >&2
      tail -5 "$log" >&2
      exit 125
   fi
   sleep 0.3
   openocd_start -c noinit
else
   openocd_start
fi

rc=0
python3 -B "$here/drive_test.py" "$elf" --boot "$boot" --timeout "$timeout_s" --port "$port" --ocd-log "$log" || rc=$?
wait "$ocd" || true
ocd=""
if (( rc != 0 )); then
   echo "--- OpenOCD ($log):" >&2
   grep -vE '^(Info|Open On-Chip|Licensed|For bug|	http)|failed to read memory|not examined yet' "$log" | tail -20 >&2 || true
fi
exit $rc
