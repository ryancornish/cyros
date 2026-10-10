#!/usr/bin/env python3
"""Run one cyros test image on the Pico 2 W over OpenOCD's Tcl port, for
run_test.sh, on either ISA: the Hazard3 cores or the Cortex-M33s.

drive_test.py <elf> --arch riscv|arm --boot picotool|swd --timeout <s>

picotool: the boot ROM loads the image into SRAM and enters it (`picotool load
-x`), and OpenOCD attaches only afterwards. swd: OpenOCD loads it after a reset
halt and starts it at _entry. Either way the image waits for the go word
(startup_rp2350_hazard3.c, startup_rp2350_m33.c), its console is read out of
its RTT ring and copied to stdout, and its exit status is read from
bench_exit_code once bench_done is set. The board is reset by the watchdog at
the end, which with nothing in flash is BOOTSEL, ready for the next image.

Exit status: the image's, or 124 if it started and never finished, or 125 if
the board could not be reached or never visibly started the image."""
import argparse
import os
import socket
import subprocess
import sys
import time

HOST_GO = 0x60606060
DEBUG = bool(os.environ.get("RP2350_DEBUG"))

# Per ISA: OpenOCD's names for the two cores, the nm that reads the image, and
# the registers worth printing when an image hangs.
ARCHES = {
    "riscv": {"cores": ("rp2350.rv0", "rp2350.rv1"), "nm": "riscv64-elf-nm",
              "regs": ("pc", "ra", "sp", "mcause", "mepc")},
    "arm": {"cores": ("rp2350.cm0", "rp2350.cm1"), "nm": "arm-none-eabi-nm",
            "regs": ("pc", "lr", "sp", "xpsr", "msp", "psp")},
}
HART = ARCHES["riscv"]["cores"][0]   # core 0's name, set by main from --arch


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


def symbols(elf, nm):
    out = subprocess.run([nm, elf], capture_output=True, text=True, check=True).stdout
    table = {}
    for line in out.splitlines():
        parts = line.split()
        if len(parts) == 3:
            table[parts[2]] = int(parts[0], 16)
    return table


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("elf")
    ap.add_argument("--arch", default="riscv", choices=sorted(ARCHES))
    ap.add_argument("--boot", default="picotool", choices=["picotool", "swd"])
    ap.add_argument("--timeout", type=float, default=120)
    ap.add_argument("--port", type=int, default=6666)
    ap.add_argument("--ocd-log", default="")
    a = ap.parse_args()
    global HART
    arch = ARCHES[a.arch]
    cores = arch["cores"]
    HART = cores[0]
    riscv = a.arch == "riscv"
    sym = symbols(a.elf, arch["nm"])

    ocd = Ocd(a.port)
    if a.boot == "swd":
        ocd.cmd("capture init")
        ocd.cmd("capture {reset halt}")
        # load_image and verify_image act on the CURRENT target, which is the
        # last one configured, core 1, unless chosen.
        ocd.cmd(f"capture {{targets {HART}}}")
        out = ocd.cmd(f"capture {{load_image {a.elf}}}")
        if "rror" in out:
            print(out.strip(), file=sys.stderr)
            ocd.cmd("shutdown")
            return 125
        out = ocd.cmd(f"capture {{verify_image {a.elf}}}")
        if "verified" not in out:
            print(f"rp2350: verify_image failed: {out.strip()[-300:]}", file=sys.stderr)
            ocd.cmd("shutdown")
            return 125
    else:
        # The boot ROM has already entered the image (run_test.sh), which is
        # waiting for the go word, so OpenOCD's examination halts nothing timed.
        out = ocd.cmd("capture init")
        if "rror" in out:
            print(out.strip(), file=sys.stderr)
    if riscv:
        # Under host load OpenOCD sometimes fails to examine a hart at init
        # ("target rp2350.rv0 examination failed") and never tries again, and
        # nothing then reads through it. Examining it now halts it, which is
        # harmless while the image waits for the go word on temporaries.
        for t in cores:
            for _ in range(10):
                if ocd.cmd(f"{t} was_examined").strip() == "1":
                    break
                print(f"rp2350: {t} not examined, examining it again", file=sys.stderr)
                time.sleep(0.2)
                ocd.cmd(f"capture {{{t} arp_examine}}")
        ocd.cmd("capture {riscv set_mem_access sysbus}")
    # A port panic reports through semihosting, from either core.
    for t in reversed(cores):
        ocd.cmd(f"capture {{targets {t}; arm semihosting enable}}")
    if DEBUG:
        for t in cores:
            print(f"{t}: {ocd.cmd(f'{t} curstate').strip()}", file=sys.stderr)
        print(f"MTIME_CTRL {ocd.word(0xd00001a4)}", file=sys.stderr)
    if a.boot == "swd":
        # Core 0 into the image (a Thumb symbol's bit 0 is not the PC's), and
        # core 1 back to the boot ROM, whose holding pen it must be in before
        # a two-core image can launch it.
        ocd.cmd(f"capture {{{HART} arp_halt}}")
        ocd.cmd(f"capture {{targets {HART}; reg pc {sym['_entry'] & ~1:#x}}}")
        ocd.cmd(f"capture {{targets {cores[1]}; resume}}")
        ocd.cmd(f"capture {{targets {HART}; resume}}")

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
    # An image sets rtt_ready within milliseconds of the boot ROM entering it,
    # so a short wait is enough, and it leaves room in the builder's own
    # timeout for run_test.sh's one retry.
    ready_by = time.monotonic() + min(8.0, a.timeout)
    try:
        while time.monotonic() < ready_by:
            if ocd.word(sym["rtt_ready"]):
                break
            time.sleep(0.02)
        else:
            # The board never ran the image visibly, which is the bench's
            # failure and not the test's: run_test.sh retries it once.
            print("rp2350: the image never set rtt_ready", file=sys.stderr)
            return 125
        # A debugger start's reset halt takes the report itself, and the Arm
        # cores make none.
        if riscv and a.boot == "picotool" and not settle(ocd, a.ocd_log):
            print("rp2350: OpenOCD never reported the boot's reset, going on", file=sys.stderr)
        ocd.cmd(f"capture {{{HART} write_memory {sym['host_go']:#x} 32 {HOST_GO:#x}}}")
        deadline = time.monotonic() + a.timeout

        while time.monotonic() < deadline:
            moved = drain_console()
            if ocd.word(sym["bench_done"]):
                drain_console()
                if DEBUG:
                    for t in cores:
                        print(f"{t}: {ocd.cmd(f'{t} curstate').strip()}", file=sys.stderr)
                verdict = ocd.word(sym["bench_exit_code"])
                break
            if not moved:
                time.sleep(0.01)
        else:
            drain_console()
            print(f"\nrp2350: no exit after {a.timeout:.0f} s", file=sys.stderr)
            print(f"rp2350: console wr {ocd.word(up_wr)} rd {ocd.word(up_rd)}, "
                  f"bench_done {ocd.word(sym['bench_done'])}", file=sys.stderr)
            # Where each core is, for the hang report. On RISC-V the halt
            # writes a stale s0 (rp2350-notes.md 7b), which no longer matters.
            for hart in cores:
                ocd.cmd(f"capture {{{hart} arp_halt}}")
                regs = []
                for r in arch["regs"]:
                    out = ocd.cmd(f"capture {{targets {hart}; reg {r}}}").strip().splitlines()
                    regs.append(out[-1] if out else f"{r} ?")
                print(f"rp2350: {hart}: " + ", ".join(regs), file=sys.stderr)
    finally:
        # RP2350_KEEP leaves the board as the image left it, for a debugger.
        if not os.environ.get("RP2350_KEEP"):
            watchdog_reboot(ocd)
        ocd.cmd("shutdown")
    return verdict if verdict is not None else 125


if __name__ == "__main__":
    sys.exit(main())
