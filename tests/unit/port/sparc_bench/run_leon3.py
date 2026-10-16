#!/usr/bin/env python3
# SPDX-License-Identifier: MPL-2.0
# Copyright (C) 2026 Ryan Cornish

"""Run a cyros test image on QEMU's leon3_generic and exit with its status.

The builder's [runner] for the SPARC toolchains. QEMU has no semihosting on
SPARC and exits with 0 whatever the guest did, so the status comes from the
console: the board prints "EXIT: n" and halts, and this exits with n. A run
that ends without that line, a hang the builder's timeout kills, or a QEMU
that aborts (a trap with traps disabled is the processor's error state, which
QEMU reports and aborts on) is a failure.

It also answers the board's host-time requests, which stand in for the
semihosting calls bench.hpp makes elsewhere: a line "@@cyros-host tickfreq"
gets 1000000000, and "@@cyros-host elapsed" the nanoseconds since QEMU was
started, each as a decimal line on the UART's receive side. Those lines are
not echoed.

Usage: run_leon3.py [--smp N] image.elf
"""

import argparse
import os
import resource
import subprocess
import sys
import time

REQUEST = b"@@cyros-host "
EXIT = b"EXIT: "


def no_core_dumps():
   # QEMU aborts on the error state, and a core per failing test would fill
   # systemd-coredump. RLIMIT_CORE 0 is honoured by it.
   resource.setrlimit(resource.RLIMIT_CORE, (0, 0))


def main():
   parser = argparse.ArgumentParser()
   parser.add_argument("--smp", type=int, default=1)
   parser.add_argument("image")
   args = parser.parse_args()

   command = [
      "qemu-system-sparc", "-M", "leon3_generic", "-smp", str(args.smp),
      "-display", "none", "-monitor", "none", "-serial", "stdio",
      "-kernel", args.image,
   ]
   started = time.monotonic_ns()
   qemu = subprocess.Popen(command, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                           bufsize=0, preexec_fn=no_core_dumps)

   status = None
   pending = b""
   out = sys.stdout.buffer
   while True:
      chunk = qemu.stdout.read(4096)
      if not chunk:
         break
      pending += chunk
      while b"\n" in pending:
         line, pending = pending.split(b"\n", 1)
         if line.startswith(REQUEST):
            what = line[len(REQUEST):].strip()
            if what == b"tickfreq":
               reply = 1_000_000_000
            elif what == b"elapsed":
               reply = time.monotonic_ns() - started
            else:
               reply = None
            if reply is not None:
               qemu.stdin.write(b"%d\n" % reply)
               qemu.stdin.flush()
            continue
         out.write(line + b"\n")
         out.flush()
         if line.startswith(EXIT):
            try:
               status = int(line[len(EXIT):].strip())
            except ValueError:
               status = None
   if pending:
      out.write(pending + b"\n")
      out.flush()

   qemu.wait()
   if status is None:
      print(f"run_leon3: no EXIT line (QEMU exited with {qemu.returncode})", flush=True)
      return 100
   return status


if __name__ == "__main__":
   sys.exit(main())
