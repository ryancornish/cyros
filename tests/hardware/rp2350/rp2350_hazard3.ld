/* The Pico 2 W's SRAM as one RISC-V image: code, data and stack, entered at
 * _entry by the boot ROM (through the IMAGE_DEF block, which must lie in the
 * first 4 kB) or by the debugger. Nothing is in flash, so .data needs no
 * copy (rp2350_hazard3_flash.ld is the image that does). The core layer's
 * interrupt stack is this image's stack, the one _entry sets. */
OUTPUT_ARCH(riscv)

MEMORY
{
   RAM (rwx) : ORIGIN = 0x20000000, LENGTH = 512K
}

ENTRY(_entry)

SECTIONS
{
   .text : {
      KEEP(*(.text.board_entry))
      KEEP(*(.embedded_block))
      *(.text*)
      *(.rodata*)
      *(.srodata*)
      . = ALIGN(4);
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
      __data_start__ = .;
      *(.data*)
      *(.sdata*)
      . = ALIGN(4);
      __data_end__ = .;
   } > RAM
   __data_load__ = LOADADDR(.data);   /* the same address: no copy */

   .bss (NOLOAD) : {
      . = ALIGN(4);
      __bss_start__ = .;
      *(.bss*)
      *(.sbss*)
      *(COMMON)
      . = ALIGN(4);
      __bss_end__ = .;
   } > RAM

   .stack (NOLOAD) : {
      . = ALIGN(16);
      . += 16K;
      __stack_top = .;
   } > RAM

   /DISCARD/ : { *(.note.GNU-stack) }
}
