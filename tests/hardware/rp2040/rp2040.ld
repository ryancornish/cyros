/* SPDX-License-Identifier: MPL-2.0 */
/* Copyright (C) 2026 Ryan Cornish */

/* The RP2040's striped SRAM0 to SRAM3 as one image: code, data and stack,
 * entered at _entry by the debugger. Nothing is in flash, so .data needs no
 * copy. SRAM4 and SRAM5 (0x20040000) are left to the boot ROM, whose stack is
 * there. Core 0's handler stack is this image's stack, the one _entry sets.
 * ../rp2350/rp2350_m33.ld's shape. */
MEMORY
{
   RAM (rwx) : ORIGIN = 0x20000000, LENGTH = 256K
}

ENTRY(_entry)

SECTIONS
{
   .text : {
      KEEP(*(.vectors))
      *(.text*)
      *(.rodata*)
      . = ALIGN(4);
   } > RAM

   .ARM.extab : { *(.ARM.extab* .gnu.linkonce.armextab.*) } > RAM
   .ARM.exidx : {
      __exidx_start = .;
      *(.ARM.exidx* .gnu.linkonce.armexidx.*)
      __exidx_end = .;
   } > RAM

   .init_array : {
      . = ALIGN(4);
      __init_array_start = .;
      KEEP(*(SORT(.init_array.*)))
      KEEP(*(.init_array*))
      __init_array_end = .;
      . = ALIGN(4);
   } > RAM

   .data : {
      . = ALIGN(4);
      *(.data*)
      . = ALIGN(4);
   } > RAM

   .bss (NOLOAD) : {
      . = ALIGN(4);
      __bss_start__ = .;
      *(.bss*)
      *(COMMON)
      . = ALIGN(4);
      __bss_end__ = .;
   } > RAM

   .stack (NOLOAD) : {
      . = ALIGN(8);
      . += 16K;
      __stack_top = .;
   } > RAM

   /DISCARD/ : { *(.note.GNU-stack) }
}
