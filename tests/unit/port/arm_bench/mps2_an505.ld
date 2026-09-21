/* Linker script for QEMU's mps2-an505 (Cortex-M33), SECURE aliases.
 *
 * The AN505 image exposes the same SRAM twice: the non-secure view at
 * 0x00000000 / 0x20000000 and the secure view at 0x10000000 / 0x38000000.
 * QEMU boots the board in Secure state and nothing in cyros wants TrustZone,
 * so everything is linked into the secure aliases and the SAU is left alone.
 *
 * The STM32U575 will want its own script with the same shape and different
 * numbers. Nothing in the port depends on either. */

MEMORY
{
   CODE (rx)  : ORIGIN = 0x10000000, LENGTH = 4M
   RAM  (rw)  : ORIGIN = 0x38000000, LENGTH = 4M
}

ENTRY(Reset_Handler)

/* Explicit program headers. Without them ld folds the read-only CODE sections
 * and .data's load image into a single LOAD segment and gives it RWX, which it
 * warns about. Naming the two segments keeps the executable region
 * non-writable, which is worth having on a part with an MPU even though
 * nothing enables one yet. */
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
   } > CODE :code

   /* ARM unwind tables. Nothing in cyros unwinds, but libgcc's unwinder
    * references __exidx_start and __exidx_end unconditionally once anything
    * drags it in, and an unplaced .ARM.exidx silently OVERLAPS .data rather
    * than erroring. The C++26 contracts `enforce` semantic is one route in,
    * through std::terminate. Placing them keeps that failure loud. */
   .ARM.extab : { *(.ARM.extab* .gnu.linkonce.armextab.*) } > CODE :code
   .ARM.exidx : {
      __exidx_start = .;
      *(.ARM.exidx* .gnu.linkonce.armexidx.*)
      __exidx_end = .;
   } > CODE :code

   /* C++ static constructors, run by Reset_Handler. */
   .init_array : {
      . = ALIGN(4);
      __init_array_start = .;
      KEEP(*(SORT(.init_array.*)))
      KEEP(*(.init_array*))
      __init_array_end = .;
      . = ALIGN(4);
   } > CODE :code

   __etext = .;

   .data : {
      __data_start__ = .;
      *(.data*)
      . = ALIGN(4);
      __data_end__ = .;
   } > RAM AT> CODE :data

   .bss (NOLOAD) : {
      __bss_start__ = .;
      *(.bss*)
      *(COMMON)
      . = ALIGN(4);
      __bss_end__ = .;
   } > RAM :NONE

   /* The main stack lives at the very top of RAM and grows down. Thread stacks
    * are NOT here: cyros hands every thread a caller-owned buffer, so they are
    * ordinary objects in .bss. Only the boot path, exception handlers and the
    * scheduler run on this one. */
   __stack_top = ORIGIN(RAM) + LENGTH(RAM);

   /DISCARD/ : { *(.note.GNU-stack) }
}
