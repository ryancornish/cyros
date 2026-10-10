/* The Pico 2 W's flash as one Arm image, executed in place, the sibling of
 * rp2350_m33.ld (SRAM): code and constants in flash, .data copied to SRAM by
 * board_early, .bss and the stack in SRAM. The boot ROM's flash boot finds the
 * IMAGE_DEF block in the first 4 kB, after the vector table it names, and
 * enters through that table. Core 0's handler stack is this image's stack, the
 * one _entry sets. */
MEMORY
{
   FLASH (rx) : ORIGIN = 0x10000000, LENGTH = 4M
   RAM (rwx)  : ORIGIN = 0x20000000, LENGTH = 512K
}

ENTRY(_entry)

SECTIONS
{
   .text : {
      KEEP(*(.vectors))
      KEEP(*(.embedded_block))
      __embedded_block_end = .;
      *(.text*)
      *(.rodata*)
      . = ALIGN(4);
   } > FLASH

   .ARM.extab : { *(.ARM.extab* .gnu.linkonce.armextab.*) } > FLASH
   .ARM.exidx : {
      __exidx_start = .;
      *(.ARM.exidx* .gnu.linkonce.armexidx.*)
      __exidx_end = .;
   } > FLASH

   .init_array : {
      . = ALIGN(4);
      __init_array_start = .;
      KEEP(*(SORT(.init_array.*)))
      KEEP(*(.init_array*))
      __init_array_end = .;
      . = ALIGN(4);
   } > FLASH

   .data : {
      . = ALIGN(4);
      __data_start__ = .;
      *(.data*)
      . = ALIGN(4);
      __data_end__ = .;
   } > RAM AT > FLASH
   __data_load__ = LOADADDR(.data);

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

ASSERT(__embedded_block_end <= ORIGIN(FLASH) + 4K, "the IMAGE_DEF must lie in flash's first 4 kB")
