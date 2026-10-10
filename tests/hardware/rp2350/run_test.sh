#!/usr/bin/env bash
# The builder's runner for the Pico 2 W, on either ISA: runs one test image
# and exits with its status, as QEMU does for the benches. The image's ELF
# header says which cores it is for, the Hazard3s or the Cortex-M33s, and the
# board is switched to that architecture before it boots.
#
#   run_test.sh <elf>
#   cyros-builder test -p build/profiles/unit_test_rp2350_hazard3.toml   # and _smp
#   cyros-builder test -p build/profiles/unit_test_rp2350_m33.toml       # and _smp
#
# RP2350_BOOT=picotool (the default) has the boot ROM load the image into SRAM
# and enter it, the path a product takes (rp2350-notes.md 10), and an image
# linked for flash (the -flash toolchains) is written to flash and booted from
# there. RP2350_BOOT=swd loads an SRAM image over the Debug Probe after a reset
# halt instead, which needs no USB but skips the boot ROM. Never OTP.
#
# Needs the Debug Probe on the board's SWD port ("D"), with nothing else
# holding OpenOCD's Tcl port, and for picotool the board on USB. Each run ends
# with a watchdog reset of the board into BOOTSEL, whatever flash holds, and a
# board found running something else is reset there first.
#
# RP2350_TEST_TIMEOUT (30 s) bounds the image once it has started. With the
# driver's 8 s wait for the start and one retry it stays under the builder's
# default --test-timeout of 60 s, so the driver always gets to reset the board.
# The test's console arrives over RTT on stdout, and a port panic, which is
# semihosting, arrives in OpenOCD's log, shown on failure.
set -euo pipefail

here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
elf="${1:?usage: run_test.sh <elf>}"
boot="${RP2350_BOOT:-picotool}"
timeout_s="${RP2350_TEST_TIMEOUT:-30}"
port=6666
log="$elf.openocd.log"

# On Arm the two cores are separate OpenOCD targets (USE_SMP 0). Grouped, as
# rp2350.cfg has them by default, the resume after a semihosting call on core
# 0 fails because core 1 never halted, and core 0 is left halted for good
# (rp2350-notes.md 7b).
case $(readelf -h "$elf" 2>/dev/null | sed -n 's/^ *Machine: *//p') in
   *RISC-V*) arch=riscv; cfg=target/rp2350-riscv.cfg; cfg_pre=() ;;
   *ARM*)    arch=arm;   cfg=target/rp2350.cfg; cfg_pre=(-c "set USE_SMP 0") ;;
   *)        echo "rp2350: $elf is neither a RISC-V nor an Arm image" >&2; exit 125 ;;
esac

# An image whose entry lies in flash's window (0x10000000 up) is a flash image,
# which only the boot ROM's flash boot runs.
entry=$(readelf -h "$elf" | sed -n 's/^ *Entry point address: *//p')
if (( entry >= 0x10000000 && entry < 0x20000000 )) && [[ $boot == swd ]]; then
   echo "rp2350: $elf is a flash image, and RP2350_BOOT=swd loads SRAM images only" >&2
   exit 125
fi

lsusb -d 2e8a:000c >/dev/null 2>&1 || { echo "rp2350: no Raspberry Pi Debug Probe on USB" >&2; exit 125; }
if [[ -n $(ss -Hltn "sport = :$port") ]]; then
   echo "rp2350: port $port is taken, another OpenOCD is running" >&2
   exit 125
fi

openocd_start() {
   # --foreground keeps OpenOCD in this script's process group, so the
   # builder's kill on a timeout reaches it. Bounded either way.
   timeout --foreground $((timeout_s + 30)) openocd-rp2350 -f interface/cmsis-dap.cfg \
      -c "adapter speed 5000" "${cfg_pre[@]}" -f "$cfg" -c "gdb port disabled" \
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
# by hand) is reset through SWD by the watchdog into BOOTSEL, as drive_test.py
# ends every run (its watchdog_reboot says why not `reset run`, and what the
# scratch words are). Whichever architecture the cores are in answers, so both
# configurations are tried.
recover() {
   local cfg target
   for cfg in target/rp2350-riscv.cfg target/rp2350.cfg; do
      case $cfg in *riscv*) target=rp2350.rv0 ;; *) target=rp2350.cm0 ;; esac
      timeout --foreground 30 openocd-rp2350 -f interface/cmsis-dap.cfg -c "adapter speed 5000" \
         -f "$cfg" -c "gdb port disabled" -c "telnet port disabled" -c "tcl port disabled" \
         -c init -c "catch {riscv set_mem_access sysbus}" \
         -c "catch {$target write_memory 0x400d8014 32 {0 0 0xb007c0d3 0xfffffffe 2 0xb007c0d3}}" \
         -c "catch {$target write_memory 0x40018008 32 0x1fffff3}" \
         -c "catch {$target write_memory 0x400d8000 32 0x80000000}" \
         -c shutdown >>"$log" 2>&1 || true
      wait_for_bootsel && return 0
   done
   return 1
}

# One boot of the image and one run of it. 125 is the bench failing (no
# board, no BOOTSEL, an image the runner never saw start), anything else is
# the image's own status.
attempt() {
   if [[ $boot == picotool ]]; then
      # The last run's watchdog reset brings the board back in about 2 s.
      if ! wait_for_bootsel 12 && ! recover; then
         echo "rp2350: the board is not in BOOTSEL and a reset did not bring it there" >&2
         return 125
      fi
      # The cores' architecture is chosen at each reboot. The boot ROM would
      # switch it to boot an image of the other one too, but by rebooting again
      # after the load, which OpenOCD then races. So it is chosen first.
      timeout 20 picotool reboot -u -c "$arch" >>"$log" 2>&1 || true
      if ! wait_for_bootsel; then
         echo "rp2350: the board did not come back in $arch BOOTSEL" >&2
         return 125
      fi
      # On RISC-V, OpenOCD's examination halts the harts, so it must come after
      # the boot ROM has entered the image: OpenOCD starts with noinit and the
      # driver runs init. On Arm examining halts nothing, and rp2350.cfg's
      # availability check fails under noinit, so it starts as usual.
      if ! timeout 60 picotool load -x -t elf "$elf" >>"$log" 2>&1; then
         echo "rp2350: picotool could not load the image" >&2
         tail -5 "$log" >&2
         return 125
      fi
      sleep 0.3
      if [[ $arch == riscv ]]; then openocd_start -c noinit; else openocd_start; fi
   else
      # A debugger start runs whichever architecture the last reboot chose, so
      # a board in BOOTSEL is switched first, through the boot ROM. One not on
      # USB keeps its own, and OpenOCD then finds the cores unavailable if it
      # is the other.
      if in_bootsel; then
         timeout 20 picotool reboot -u -c "$arch" >>"$log" 2>&1 || true
         # The old device can still answer for a moment, so give the reboot
         # time to take it away before looking for the new one.
         sleep 1
         wait_for_bootsel || true
      fi
      openocd_start
   fi

   local rc=0
   python3 -B "$here/drive_test.py" "$elf" --arch "$arch" --boot "$boot" --timeout "$timeout_s" --port "$port" --ocd-log "$log" || rc=$?
   wait "$ocd" || true
   ocd=""
   if (( rc != 0 )); then
      echo "--- OpenOCD ($log):" >&2
      # Without the driver's own Tcl replies (bare numbers and memory words), and
      # without the watchdog trigger's write, which fails because the chip resets
      # under it.
      grep -vE '^(Info|Open On-Chip|Licensed|For bug|	http)|failed to read memory|not examined yet|^(-?[0-9]+|0x[0-9a-f]+( 0x[0-9a-f]+)*)$|^$|[Ff]ailed to write memory|SWD DPIDR|shutdown command' "$log" | tail -20 >&2 || true
   fi
   return $rc
}

# An image that never visibly started is retried once from the top: it was
# seen twice in about 300 runs, both under heavy host load, once with OpenOCD
# failing to examine hart 0 (2026-10-10). A test that started and hangs is
# never retried.
rc=0
attempt || rc=$?
if (( rc == 125 )); then
   echo "rp2350: retrying once, from the top" >&2
   : >"$log"
   rc=0
   attempt || rc=$?
fi
exit $rc
