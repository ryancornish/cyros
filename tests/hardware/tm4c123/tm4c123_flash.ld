/* Linker script for the TM4C123GH6PM on an EK-TM4C123GXL, from flash.
 *
 * 256 KB of flash at 0 and 32 KB of SRAM at 0x20000000 (SPMS376E, table 2-4).
 * Same shape as the mps2-an386 bench's script, whose core this is.
 *
 * Programming flash erases what is there, which on a LaunchPad straight from
 * the box is TI's demo. run_test.sh uses tm4c123_sram.ld whenever the image
 * fits, and only falls back to this when asked to. */

MEMORY
{
   CODE (rx)  : ORIGIN = 0x00000000, LENGTH = 256K
   RAM  (rw)  : ORIGIN = 0x20000000, LENGTH = 32K
}

ENTRY(Reset_Handler)

/* Explicit program headers. Without them ld folds the read-only CODE sections
 * and .data's load image into a single LOAD segment and gives it RWX, which it
 * warns about. Naming the two segments keeps the executable region
 * non-writable. (These are ELF permissions only. The port's one MPU region is
 * the stack guard, over the default memory map.) */
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

   /* The main stack takes what is left above .bss. Boot, the exception
    * handlers and the scheduler run on it, and the deepest any test took it on
    * the mps2-an386 bench was 376 bytes. A test that leaves less than 1 KB
    * fails here, at link time, instead of overwriting .bss on the board. */
   ASSERT(__stack_top - __bss_end__ >= 1024, "less than 1 KB of RAM left for the main stack")

   /DISCARD/ : { *(.note.GNU-stack) }
}
