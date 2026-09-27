#!/usr/bin/env bash
# Watch the board's console: USART1 through the ST-LINK's virtual COM port.
#
# The rate comes from board.h, so this never needs telling which clock the
# image was built for. Leave with C-a C-x.
set -euo pipefail

here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

command -v picocom >/dev/null || { echo "picocom not installed. See ~/starch-env toolchain role." >&2; exit 1; }
tty="${U575_TTY:-$(ls /dev/serial/by-id/usb-STMicroelectronics_STLINK-V3_*-if02 2>/dev/null | head -n 1)}"
[ -n "$tty" ] && [ -e "$tty" ] || { echo "no ST-LINK virtual COM port found, set U575_TTY" >&2; exit 1; }
baud="$(sed -n 's/^#define BOARD_CONSOLE_BAUD \([0-9]*\)u$/\1/p' "$here/board.h")"

# The board writes bare newlines. --imap lfcrlf turns them into line breaks.
exec picocom -b "$baud" --imap lfcrlf "$tty"
