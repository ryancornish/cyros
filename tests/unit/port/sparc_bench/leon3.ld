/* SPDX-License-Identifier: MPL-2.0 */
/* Copyright (C) 2026 Ryan Cornish */

/* QEMU's leon3_generic: the image in RAM at 0x40000000, where QEMU loads the
 * ELF and its PROM code jumps to the entry. One image in RAM, so .data needs
 * no copy. The port's trap table is 4 KB aligned by its own section. */
OUTPUT_ARCH(sparc)
ENTRY(_start)

/* A sparc-elf GCC links no C library unless told (its LIB_SPEC is empty,
 * where arm-none-eabi's is -lc), and the builder's link puts toolchain flags
 * before the objects, where -lc would resolve nothing. A GROUP in the script
 * is searched after them. The board's stubs and libstdc++'s headers rely on
 * newlib, as on the other boards. */
GROUP(-lc -lgcc)

MEMORY
{
   RAM (rwx) : ORIGIN = 0x40000000, LENGTH = 64M
}

SECTIONS
{
   .text : {
      KEEP(*(.text.reset))
      *(.text .text.*)
   } > RAM

   .rodata : {
      *(.rodata .rodata.*)
   } > RAM

   .init_array : {
      . = ALIGN(4);
      __init_array_start = .;
      KEEP(*(SORT(.init_array.*)))
      KEEP(*(.init_array*))
      __init_array_end = .;
   } > RAM

   .data : {
      *(.data .data.*)
   } > RAM

   .bss (NOLOAD) : {
      . = ALIGN(8);
      __bss_start__ = .;
      *(.bss .bss.* COMMON)
      . = ALIGN(8);
      __bss_end__ = .;
   } > RAM

   /* Core 0's boot stack, which becomes its interrupt stack once cyros starts
    * its first thread (port_core_sparcv8.cpp). */
   .stack (NOLOAD) : {
      . = ALIGN(8);
      . += 64K;
      __stack_top = .;
   } > RAM

   /DISCARD/ : { *(.note.GNU-stack) }
}
