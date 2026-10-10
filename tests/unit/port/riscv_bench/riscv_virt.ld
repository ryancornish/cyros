/* QEMU's RISC-V virt: 128 MB of RAM at 0x80000000, where `-bios none` starts
 * every hart. One image in RAM, so .data needs no copy. */
OUTPUT_ARCH(riscv)
ENTRY(_start)

MEMORY
{
   RAM (rwx) : ORIGIN = 0x80000000, LENGTH = 128M
}

SECTIONS
{
   .text : {
      KEEP(*(.text.reset))
      *(.text .text.*)
   } > RAM

   .rodata : {
      *(.rodata .rodata.* .srodata .srodata.*)
   } > RAM

   .init_array : {
      . = ALIGN(4);
      __init_array_start = .;
      KEEP(*(SORT(.init_array.*)))
      KEEP(*(.init_array*))
      __init_array_end = .;
   } > RAM

   .data : {
      *(.data .data.* .sdata .sdata.*)
   } > RAM

   .bss (NOLOAD) : {
      . = ALIGN(4);
      __bss_start__ = .;
      *(.bss .bss.* .sbss .sbss.* COMMON)
      . = ALIGN(4);
      __bss_end__ = .;
   } > RAM

   /* The boot stack, which becomes hart 0's interrupt stack once cyros starts
    * its first thread (port_core_rv32.cpp). */
   .stack (NOLOAD) : {
      . = ALIGN(16);
      . += 64K;
      __stack_top = .;
   } > RAM

   /DISCARD/ : { *(.note.GNU-stack) }
}
