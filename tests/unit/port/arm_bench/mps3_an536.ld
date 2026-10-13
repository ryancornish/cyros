/* QEMU's mps3-an536: the image in DDR at 0x20000000, where QEMU loads the ELF
 * and starts every core at its entry. One image in RAM, so .data needs no copy.
 * The vector table is the board's (startup_mps3_an536.c), 32-byte aligned for
 * VBAR. */
OUTPUT_ARCH(arm)
ENTRY(_start)

MEMORY
{
   DDR (rwx) : ORIGIN = 0x20000000, LENGTH = 64M
}

SECTIONS
{
   .text : {
      KEEP(*(.text.reset))
      . = ALIGN(32);
      KEEP(*(.vectors))
      *(.text .text.*)
      *(.glue_7 .glue_7t .vfp11_veneer .v4_bx)
   } > DDR

   .ARM.extab : { *(.ARM.extab* .gnu.linkonce.armextab.*) } > DDR
   .ARM.exidx : {
      __exidx_start = .;
      *(.ARM.exidx* .gnu.linkonce.armexidx.*)
      __exidx_end = .;
   } > DDR

   .rodata : {
      *(.rodata .rodata.*)
   } > DDR

   .init_array : {
      . = ALIGN(4);
      __init_array_start = .;
      KEEP(*(SORT(.init_array.*)))
      KEEP(*(.init_array*))
      __init_array_end = .;
   } > DDR

   .data : {
      *(.data .data.*)
   } > DDR

   .bss (NOLOAD) : {
      . = ALIGN(4);
      __bss_start__ = .;
      *(.bss .bss.* COMMON)
      . = ALIGN(4);
      __bss_end__ = .;
   } > DDR

   /* Core 0's boot stack, which becomes its interrupt stack once cyros starts
    * its first thread (port_core_armv8r.cpp). */
   .stack (NOLOAD) : {
      . = ALIGN(8);
      . += 64K;
      __stack_top = .;
   } > DDR

   /DISCARD/ : { *(.note.GNU-stack) }
}
