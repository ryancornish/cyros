/* The Pico 2 W's SRAM as one Arm image: code, data and stack, entered at
 * _entry by the boot ROM (through the IMAGE_DEF block, which must lie in the
 * first 4 kB, after the vector table it names) or by the debugger. Nothing is
 * in flash, so .data needs no copy (rp2350_m33_flash.ld is the image that
 * does). Core 0's handler stack is this image's stack, the one _entry sets. */
MEMORY
{
   RAM (rwx) : ORIGIN = 0x20000000, LENGTH = 512K
}

ENTRY(_entry)

SECTIONS
{
   .text : {
      KEEP(*(.vectors))
      KEEP(*(.embedded_block))
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
      __etext = .;
      __data_start__ = .;
      *(.data*)
      . = ALIGN(4);
      __data_end__ = .;
   } > RAM
   __data_load__ = LOADADDR(.data);   /* the same address: no copy */

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
