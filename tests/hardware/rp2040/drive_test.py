#!/usr/bin/env python3
"""Run one cyros test image on the RP2040 Pico over OpenOCD's Tcl port, for
run_test.sh. The Pico 2 W board's drive_test.py (../rp2350) for this chip's
one ISA and its one boot path.

drive_test.py <elf> --timeout <s>

OpenOCD loads the image after a reset halt and starts core 0 at _entry, with
core 1 sent back to the boot ROM, where the rp2040_smp target launches it
from. The image waits for the go word
(startup_rp2040.c), its console is read out of its RTT ring and copied to
stdout, and its exit status is read from bench_exit_code once bench_done is
set. The chip is reset by the watchdog at the end, which with nothing in
flash leaves it in BOOTSEL.

Exit status: the image's, or 124 if it started and never finished, or 125 if
the board could not be reached or never visibly started the image."""
import argparse
import os
import socket
import subprocess
import sys
import time

HOST_GO = 0x60606060
DEBUG = bool(os.environ.get("RP2040_DEBUG"))

CORES = ("rp2040.core0", "rp2040.core1")
CORE0 = CORES[0]
REGS = ("pc", "lr", "sp", "xpsr", "msp", "psp")


class Ocd:
    """OpenOCD's Tcl port."""

    def __init__(self, port):
        deadline = time.monotonic() + 10
        while True:
            try:
                self.sock = socket.create_connection(("127.0.0.1", port), timeout=60)
                return
            except OSError:
                if time.monotonic() > deadline:
                    raise
                time.sleep(0.1)

    def cmd(self, c):
        self.sock.sendall(c.encode() + b"\x1a")
        buf = b""
        while not buf.endswith(b"\x1a"):
            chunk = self.sock.recv(4096)
            if not chunk:
                raise ConnectionError("OpenOCD closed the Tcl port")
            buf += chunk
        out = buf[:-1].decode()
        if DEBUG and "read_memory" not in c:
            print(f"ocd> {c}\n{out}", file=sys.stderr)
        return out

    def word(self, addr):
        out = self.cmd(f"{CORE0} read_memory {addr:#x} 32 1").strip()
        try:
            return int(out, 0)
        except ValueError:
            return None


PSM_WDSEL = 0x40010008
WATCHDOG_CTRL = 0x40058000
WATCHDOG_SCRATCH4 = 0x4005801c
WDSEL_ALL_BUT_OSCILLATORS = 0x1fffc   # the SDK's watchdog_reboot choice
WATCHDOG_TRIGGER = 1 << 31


def watchdog_reboot(ocd):
    """Reset the whole chip but its oscillators, as the SDK's watchdog_reboot
    does. A debugger `reset run` resets the cores alone and would leave the
    peripherals as the image left them. SCRATCH4 is cleared first, so the boot
    ROM takes no stale request to enter an address, and with nothing in flash
    it settles in BOOTSEL. The trigger's write reports an error, because the
    chip resets under it."""
    ocd.cmd(f"capture {{{CORE0} write_memory {WATCHDOG_SCRATCH4:#x} 32 0}}")
    ocd.cmd(f"capture {{{CORE0} write_memory {PSM_WDSEL:#x} 32 {WDSEL_ALL_BUT_OSCILLATORS:#x}}}")
    ocd.cmd(f"capture {{{CORE0} write_memory {WATCHDOG_CTRL:#x} 32 {WATCHDOG_TRIGGER:#x}}}")


def symbols(elf):
    out = subprocess.run(["arm-none-eabi-nm", elf], capture_output=True, text=True, check=True).stdout
    table = {}
    for line in out.splitlines():
        parts = line.split()
        if len(parts) == 3:
            table[parts[2]] = int(parts[0], 16)
    return table


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("elf")
    ap.add_argument("--timeout", type=float, default=120)
    ap.add_argument("--port", type=int, default=6666)
    a = ap.parse_args()
    sym = symbols(a.elf)

    ocd = Ocd(a.port)
    ocd.cmd("capture init")
    ocd.cmd("capture {reset halt}")
    # load_image and verify_image act on the CURRENT target, which is the last
    # one configured, core 1, unless chosen.
    ocd.cmd(f"capture {{targets {CORE0}}}")
    out = ocd.cmd(f"capture {{load_image {a.elf}}}")
    if "rror" in out:
        print(out.strip(), file=sys.stderr)
        ocd.cmd("shutdown")
        return 125
    out = ocd.cmd(f"capture {{verify_image {a.elf}}}")
    if "verified" not in out:
        print(f"rp2040: verify_image failed: {out.strip()[-300:]}", file=sys.stderr)
        ocd.cmd("shutdown")
        return 125
    # The words the driver polls live in .bss, which load_image never writes,
    # so until the image zeroes them they hold whatever the last image left:
    # a stack's fill once read as "ready" and "done", with the image not yet
    # running. Zeroed here, so only the image can set them.
    for name in ("rtt_ready", "host_go", "bench_done", "bench_exit_code"):
        ocd.cmd(f"capture {{{CORE0} write_memory {sym[name]:#x} 32 0}}")
    # A port panic reports through semihosting.
    for t in reversed(CORES):
        ocd.cmd(f"capture {{targets {t}; arm semihosting enable}}")
    # Core 0 into the image (a Thumb symbol's bit 0 is not the PC's), and core
    # 1 back to the boot ROM, where it waits to be launched.
    ocd.cmd(f"capture {{targets {CORE0}; reg pc {sym['_entry'] & ~1:#x}}}")
    ocd.cmd(f"capture {{targets {CORES[1]}; resume}}")
    ocd.cmd(f"capture {{targets {CORE0}; resume}}")
    if DEBUG:
        for t in CORES:
            print(f"{t}: {ocd.cmd(f'{t} curstate').strip()}", file=sys.stderr)

    # The console: rtt_cb's up buffer 0, read by the host itself over the
    # memory AP, which never halts the core, as on the Pico 2 W board.
    # Offsets are console_rtt.c's layout.
    up = sym["rtt_cb"] + 24
    up_buf, up_size, up_wr, up_rd = up + 4, up + 8, up + 12, up + 16

    def read_bytes(addr, count):
        first, last = addr & ~3, (addr + count + 3) & ~3
        raw = bytearray()
        for at in range(first, last, 256):
            n = min(64, (last - at) // 4)
            out = ocd.cmd(f"{CORE0} read_memory {at:#x} 32 {n}")
            for x in out.split():
                raw += int(x, 0).to_bytes(4, "little")
        return raw[addr - first:addr - first + count]

    def drain_console():
        wr, rd = ocd.word(up_wr), ocd.word(up_rd)
        if wr is None or rd is None or wr == rd:
            return 0
        buf, size = ocd.word(up_buf), ocd.word(up_size)
        if not buf or not size or wr >= size or rd >= size:
            return 0
        spans = [(rd, wr)] if wr > rd else [(rd, size), (0, wr)]
        data = bytearray()
        for lo, hi in spans:
            if hi > lo:
                data += read_bytes(buf + lo, hi - lo)
        ocd.cmd(f"{CORE0} write_memory {up_rd:#x} 32 {wr:#x}")
        sys.stdout.write(data.decode(errors="replace"))
        sys.stdout.flush()
        return len(data)

    verdict = 124
    ready_by = time.monotonic() + min(8.0, a.timeout)
    try:
        while time.monotonic() < ready_by:
            if ocd.word(sym["rtt_ready"]):
                break
            time.sleep(0.02)
        else:
            # The board never ran the image visibly, which is the bench's
            # failure and not the test's: run_test.sh retries it once.
            print("rp2040: the image never set rtt_ready", file=sys.stderr)
            for t in CORES:
                print(f"rp2040: {t} {ocd.cmd(f'{t} curstate').strip()}", file=sys.stderr)
            ocd.cmd(f"capture {{targets {CORE0}; halt 1000}}")
            pc = ocd.cmd(f"capture {{targets {CORE0}; reg pc}}").strip().splitlines()
            print(f"rp2040: {CORE0} {pc[-1] if pc else 'pc ?'}", file=sys.stderr)
            return 125
        ocd.cmd(f"capture {{{CORE0} write_memory {sym['host_go']:#x} 32 {HOST_GO:#x}}}")
        deadline = time.monotonic() + a.timeout

        while time.monotonic() < deadline:
            moved = drain_console()
            if ocd.word(sym["bench_done"]):
                drain_console()
                verdict = ocd.word(sym["bench_exit_code"])
                break
            if not moved:
                time.sleep(0.01)
        else:
            drain_console()
            print(f"\nrp2040: no exit after {a.timeout:.0f} s", file=sys.stderr)
            print(f"rp2040: console wr {ocd.word(up_wr)} rd {ocd.word(up_rd)}, "
                  f"bench_done {ocd.word(sym['bench_done'])}", file=sys.stderr)
            # Where each core is, for the hang report. Core 1 is in the boot
            # ROM unless the image launched it. `halt` waits for the halt,
            # where arp_halt only asks, and a register read before it lands
            # fails.
            for t in CORES:
                ocd.cmd(f"capture {{targets {t}; halt 1000}}")
                regs = []
                for r in REGS:
                    out = ocd.cmd(f"capture {{targets {t}; reg {r}}}").strip().splitlines()
                    regs.append(out[-1] if out else f"{r} ?")
                print(f"rp2040: {t}: " + ", ".join(regs), file=sys.stderr)
    finally:
        # RP2040_KEEP leaves the board as the image left it, for a debugger.
        if not os.environ.get("RP2040_KEEP"):
            watchdog_reboot(ocd)
        ocd.cmd("shutdown")
    return verdict if verdict is not None else 125


if __name__ == "__main__":
    sys.exit(main())
