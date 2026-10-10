#!/usr/bin/env python3
"""Run one cyros test image on the Pico 2 W's Hazard3 core 0 over OpenOCD's
Tcl port, for run_test.sh.

drive_test.py <elf> --boot picotool|swd --timeout <s>

picotool: the boot ROM loads the image into SRAM and enters it (`picotool load
-x`), and OpenOCD attaches only afterwards. swd: OpenOCD loads it after a reset
halt and starts it at _entry. Either way the image waits for the go word
(startup_rp2350_hazard3.c), its console is read out of its RTT ring and
copied to stdout, and its exit status is read from bench_exit_code once
bench_done is set. The board is reset by the watchdog at the end, which with nothing in
flash is BOOTSEL, ready for the next image.

Exit status: the image's, or 124 if it never finished, or 125 if the board
could not be reached."""
import argparse
import os
import socket
import subprocess
import sys
import time

HOST_GO = 0x60606060
HART = "rp2350.rv0"
DEBUG = bool(os.environ.get("RP2350_DEBUG"))


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
        out = self.cmd(f"{HART} read_memory {addr:#x} 32 1").strip()
        try:
            return int(out, 0)
        except ValueError:
            return None


def settle(ocd, log_path):
    """Wait until OpenOCD has taken the boot's reset report from both harts.

    It notices each hart's reset on a poll some 170 ms after init and handles
    it by halting the hart, and the resume writes a stale value into s0
    (rp2350-notes.md 7b). Under a running test that corrupts whatever s0 held.
    So it is drawn out here, while the image waits for go on temporaries
    only. The report lands in a poll's reply or, from the background poll, in
    OpenOCD's log, so both are watched. False if it never came."""
    seen = set()
    for _ in range(40):
        text = ocd.cmd("capture poll")
        try:
            with open(log_path, errors="replace") as log:
                text += log.read()
        except OSError:
            pass
        for hart in ("rv0", "rv1"):
            if f"[rp2350.{hart}] Hart unexpectedly reset" in text:
                seen.add(hart)
        if len(seen) == 2:
            return True
        time.sleep(0.05)
    return False


PSM_WDSEL = 0x40018008
WATCHDOG_CTRL = 0x400d8000
WDSEL_ALL_BUT_OSCILLATORS = 0x1fffff3   # the SDK's watchdog_reboot choice
WATCHDOG_TRIGGER = 1 << 31


def watchdog_reboot(ocd):
    """Reset the whole chip but its oscillators, as the SDK's watchdog_reboot
    does. A debugger `reset run` resets the cores alone, which left the boot
    ROM short of BOOTSEL once core 1 had been launched out of its pen. With
    nothing in flash, and the boot ROM's own reboot request still saying
    BOOTSEL, the board comes back on USB in about 2 s. The trigger's write
    reports an error, because the chip resets under it."""
    ocd.cmd(f"capture {{{HART} write_memory {PSM_WDSEL:#x} 32 {WDSEL_ALL_BUT_OSCILLATORS:#x}}}")
    ocd.cmd(f"capture {{{HART} write_memory {WATCHDOG_CTRL:#x} 32 {WATCHDOG_TRIGGER:#x}}}")


def symbols(elf):
    out = subprocess.run(["riscv64-elf-nm", elf], capture_output=True, text=True, check=True).stdout
    table = {}
    for line in out.splitlines():
        parts = line.split()
        if len(parts) == 3:
            table[parts[2]] = int(parts[0], 16)
    return table


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("elf")
    ap.add_argument("--boot", default="picotool", choices=["picotool", "swd"])
    ap.add_argument("--timeout", type=float, default=120)
    ap.add_argument("--port", type=int, default=6666)
    ap.add_argument("--ocd-log", default="")
    a = ap.parse_args()
    sym = symbols(a.elf)

    ocd = Ocd(a.port)
    if a.boot == "swd":
        ocd.cmd("capture init")
        ocd.cmd("capture {reset halt}")
        out = ocd.cmd(f"capture {{load_image {a.elf}}}")
        if "rror" in out:
            print(out.strip(), file=sys.stderr)
            return 125
        if "verified" not in ocd.cmd(f"capture {{verify_image {a.elf}}}"):
            print("rp2350: verify_image failed", file=sys.stderr)
            return 125
    else:
        # The boot ROM has already entered the image (run_test.sh), which is
        # waiting for the go word, so OpenOCD's examination halts nothing timed.
        out = ocd.cmd("capture init")
        if "rror" in out:
            print(out.strip(), file=sys.stderr)
    ocd.cmd("capture {riscv set_mem_access sysbus}")
    ocd.cmd("capture {arm semihosting enable}")
    if DEBUG:
        for t in ("rp2350.rv0", "rp2350.rv1"):
            print(f"{t}: {ocd.cmd(f'{t} curstate').strip()}", file=sys.stderr)
        print(f"MTIME_CTRL {ocd.word(0xd00001a4)}", file=sys.stderr)
    if a.boot == "swd":
        ocd.cmd(f"capture {{{HART} arp_halt}}")
        ocd.cmd(f"capture {{reg pc {sym['_entry']:#x}}}")
        ocd.cmd("capture resume")

    # The console. This OpenOCD has RTT commands for its Arm targets only, so
    # the host reads the ring itself: rtt_cb's up buffer 0, over the system
    # bus, which never halts the hart. Offsets are console_rtt.c's layout.
    up = sym["rtt_cb"] + 24
    up_buf, up_size, up_wr, up_rd = up + 4, up + 8, up + 12, up + 16

    def read_bytes(addr, count):
        # Whole aligned words, 64 at a time. A long byte-wide read over the
        # system bus trips an assertion in this OpenOCD's RISC-V batching
        # (riscv_batch_add_dm_read) and kills it.
        first, last = addr & ~3, (addr + count + 3) & ~3
        raw = bytearray()
        for at in range(first, last, 256):
            n = min(64, (last - at) // 4)
            out = ocd.cmd(f"{HART} read_memory {at:#x} 32 {n}")
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
        ocd.cmd(f"{HART} write_memory {up_rd:#x} 32 {wr:#x}")
        sys.stdout.write(data.decode(errors="replace"))
        sys.stdout.flush()
        return len(data)

    verdict = 124
    deadline = time.monotonic() + a.timeout
    try:
        while time.monotonic() < deadline:
            if ocd.word(sym["rtt_ready"]):
                break
            time.sleep(0.02)
        else:
            print("rp2350: the image never set rtt_ready", file=sys.stderr)
            return 124
        # A debugger start's reset halt takes the report itself.
        if a.boot == "picotool" and not settle(ocd, a.ocd_log):
            print("rp2350: OpenOCD never reported the boot's reset, going on", file=sys.stderr)
        ocd.cmd(f"capture {{{HART} write_memory {sym['host_go']:#x} 32 {HOST_GO:#x}}}")

        while time.monotonic() < deadline:
            moved = drain_console()
            if ocd.word(sym["bench_done"]):
                drain_console()
                if DEBUG:
                    for t in ("rp2350.rv0", "rp2350.rv1"):
                        print(f"{t}: {ocd.cmd(f'{t} curstate').strip()}", file=sys.stderr)
                verdict = ocd.word(sym["bench_exit_code"])
                break
            if not moved:
                time.sleep(0.01)
        else:
            drain_console()
            print(f"\nrp2350: no exit after {a.timeout:.0f} s", file=sys.stderr)
            # Where each hart is, for the hang report. The halt writes a stale
            # s0 (rp2350-notes.md 7b), which no longer matters.
            for hart in ("rp2350.rv0", "rp2350.rv1"):
                ocd.cmd(f"capture {{{hart} arp_halt}}")
                regs = []
                for r in ("pc", "ra", "sp", "mcause", "mepc"):
                    out = ocd.cmd(f"capture {{targets {hart}; reg {r}}}").strip().splitlines()
                    regs.append(out[-1] if out else f"{r} ?")
                print(f"rp2350: {hart}: " + ", ".join(regs), file=sys.stderr)
    finally:
        watchdog_reboot(ocd)
        ocd.cmd("shutdown")
    return verdict if verdict is not None else 125


if __name__ == "__main__":
    sys.exit(main())
