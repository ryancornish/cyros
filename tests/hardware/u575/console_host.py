#!/usr/bin/env python3
# SPDX-License-Identifier: MPL-2.0
# Copyright (C) 2026 Ryan Cornish

"""Program the NUCLEO-U575ZI-Q, read its console, and answer its host-time
requests, for run_test.sh.

console_host.py --elf <elf> --cfg <openocd.cfg> --tty <tty> --baud <n>
                --log <file> --ocd-log <file> --timeout <s>

The console is opened and configured BEFORE OpenOCD programs the board, so the
first line cannot be lost. Raw, no echo: an echo would send every received byte
back to the board's RX.

OpenOCD has neither of the two elapsed-time semihosting calls, so the board
asks here instead (console.c), the LEON3 board's protocol: a line
"@@cyros-host tickfreq" gets 1000000000, and "@@cyros-host elapsed" the
nanoseconds since OpenOCD was started, each as a decimal line on the board's
RX. Those lines are answered, not logged. Every other line goes to the log.

Exit status: 0 once the board prints its "EXIT: n" line, 1 if OpenOCD ended
first (a port panic reports that way, through semihosting), 2 on the timeout,
3 if the console passed 1 MB without an exit."""
import argparse
import os
import select
import subprocess
import sys
import termios
import time
import tty

REQUEST = b"@@cyros-host "
EXIT = b"EXIT: "
BYTE_CAP = 1_000_000


def open_console(path, baud):
    fd = os.open(path, os.O_RDWR | os.O_NOCTTY)
    tty.setraw(fd)
    attrs = termios.tcgetattr(fd)
    speed = getattr(termios, f"B{baud}")
    attrs[4] = attrs[5] = speed
    attrs[3] &= ~termios.ECHO
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    termios.tcflush(fd, termios.TCIOFLUSH)
    return fd


def main():
    parser = argparse.ArgumentParser()
    for name in ("--elf", "--cfg", "--tty", "--log", "--ocd-log"):
        parser.add_argument(name, required=True)
    parser.add_argument("--baud", type=int, required=True)
    parser.add_argument("--timeout", type=float, required=True)
    args = parser.parse_args()

    fd = open_console(args.tty, args.baud)
    with open(args.ocd_log, "wb") as ocd_log, open(args.log, "wb") as log:
        ocd = subprocess.Popen(
            ["openocd", "-f", args.cfg, "-c", "init",
             "-c", f"program {args.elf} verify", "-c", "reset run"],
            stdout=ocd_log, stderr=subprocess.STDOUT)
        started = time.monotonic_ns()
        deadline = time.monotonic() + args.timeout
        pending = b""
        received = 0
        status = None
        try:
            while status is None:
                if ocd.poll() is not None:
                    status = 1
                    break
                if time.monotonic() >= deadline:
                    status = 2
                    break
                if received > BYTE_CAP:
                    status = 3
                    break
                ready, _, _ = select.select([fd], [], [], 0.2)
                if not ready:
                    continue
                chunk = os.read(fd, 4096)
                received += len(chunk)
                pending += chunk
                while b"\n" in pending:
                    line, pending = pending.split(b"\n", 1)
                    if line.startswith(REQUEST):
                        what = line[len(REQUEST):].strip()
                        if what == b"tickfreq":
                            os.write(fd, b"1000000000\n")
                        elif what == b"elapsed":
                            os.write(fd, b"%d\n" % (time.monotonic_ns() - started))
                        continue
                    log.write(line + b"\n")
                    log.flush()
                    if line.startswith(EXIT):
                        status = 0
                        break
        finally:
            if pending:
                log.write(pending + b"\n")
            ocd.terminate()
            try:
                ocd.wait(timeout=10)
            except subprocess.TimeoutExpired:
                ocd.kill()
                ocd.wait()
            os.close(fd)

    messages = {1: "openocd exited (a port panic reports this way)",
                2: "timed out waiting for EXIT",
                3: "the console passed 1 MB without an EXIT"}
    if status in messages:
        print(messages[status], file=sys.stderr)
    return status


if __name__ == "__main__":
    sys.exit(main())
