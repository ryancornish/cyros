/* Linker script for QEMU's microbit, a Cortex-M0 (nRF51), the ARMv6-M bench.
 *
 * Code in the 256 kB flash at 0, which QEMU loads from the ELF and the core
 * cannot write, so .data is copied out of it by the reset path. RAM at
 * 0x20000000 is 256 kB only because the runner asks for it
 * (-global nrf51-soc.sram-size=262144): the machine's default 16 kB fits no
 * test image. Otherwise mps2_an386.ld's shape. */

MEMORY
{
   FLASH (rx) : ORIGIN = 0x00000000, LENGTH = 256K
   RAM   (rw) : ORIGIN = 0x20000000, LENGTH = 256K
}

ENTRY(Reset_Handler)

PHDRS
{
   code PT_LOAD FLAGS(5);   /* r-x */
   data PT_LOAD FLAGS(6);   /* rw- */
}

SECTIONS
{
   .text : {
      KEEP(*(.vectors))
      *(.text*)
      *(.rodata*)
      . = ALIGN(4);
   } > FLASH :code

   .ARM.extab : { *(.ARM.extab* .gnu.linkonce.armextab.*) } > FLASH :code
   .ARM.exidx : {
      __exidx_start = .;
      *(.ARM.exidx* .gnu.linkonce.armexidx.*)
      __exidx_end = .;
   } > FLASH :code

   .init_array : {
      . = ALIGN(4);
      __init_array_start = .;
      KEEP(*(SORT(.init_array.*)))
      KEEP(*(.init_array*))
      __init_array_end = .;
      . = ALIGN(4);
   } > FLASH :code

   .data : {
      __data_start__ = .;
      *(.data*)
      . = ALIGN(4);
      __data_end__ = .;
   } > RAM AT> FLASH :data
   __data_load__ = LOADADDR(.data);

   .bss (NOLOAD) : {
      __bss_start__ = .;
      *(.bss*)
      *(COMMON)
      . = ALIGN(4);
      __bss_end__ = .;
   } > RAM :NONE

   __stack_top = ORIGIN(RAM) + LENGTH(RAM);

   /DISCARD/ : { *(.note.GNU-stack) }
}
