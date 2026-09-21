# gdb setup for stepping cyros on the U575.
target extended-remote :3333

set confirm off
set pagination off
set print pretty on

# Load the image, then start from reset so the first thing seen is the very
# first instruction rather than wherever the part happened to be.
load
monitor reset halt

# The interesting places. PendSV_Handler is the context switch itself: its
# naked prologue stacks r4-r11 onto the OUTGOING thread's PSP and its epilogue
# pops them from a DIFFERENT stack, which is the one thing that cannot be seen
# on a host port at all.
break cyros_bench_main
break cyros_port_start_first
break PendSV_Handler
break cyros_port_switch

define hw
  printf "baton=%u  switches=%u  ping=%u  pong=%u  marker=0x%x\n", \
         baton, switch_count, ping_loops, pong_loops, ping_marker_seen
  printf "psp=0x%08x  msp=0x%08x  psplim=0x%08x\n", $psp, $msp, $psplim
  printf "control=0x%x  primask=0x%x  basepri=0x%x  ipsr=%d\n", \
         $control, $primask, $basepri, ($xpsr & 0x1ff)
end
document hw
Show cyros's live state: the baton, switch counts, and the masking registers.
end

echo \n=== cyros on STM32U575 ===\n
echo Breakpoints set. `c` to run, `hw` for live state.\n
echo PendSV_Handler is the context switch. `layout asm` then `si` through it.\n\n
