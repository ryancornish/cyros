/* Linker script for the STM32U575ZIT6Q on a NUCLEO-U575ZI-Q.
 *
 * Same shape as the QEMU bench's mps2_an505.ld, different numbers, which is
 * the whole point of the port being core-level: nothing in libcyros.a changes
 * between the two.
 *
 * Memory map, from RM0456. The part has 2 MB of flash and 786 KB of SRAM, and
 * the first three SRAM banks are CONTIGUOUS, so they are declared as one
 * region rather than three:
 *
 *   SRAM1  192 KB  0x20000000
 *   SRAM2   64 KB  0x20030000   -> 768 KB contiguous from 0x20000000
 *   SRAM3  512 KB  0x20040000
 *   SRAM4   16 KB  0x28000000   (separate, not used here)
 *
 * TrustZone is NOT enabled. A factory NUCLEO-U575 ships with the TZEN option
 * byte clear, so the part boots with the Security Extension off and flash is
 * addressed at 0x08000000. Nothing here touches the option bytes, and nothing
 * should: setting TZEN is hard to undo and is not needed by anything cyros
 * does. PSPLIM is a Main Extension feature and works regardless.
 */

MEMORY
{
   FLASH (rx) : ORIGIN = 0x08000000, LENGTH = 2048K
   RAM   (rw) : ORIGIN = 0x20000000, LENGTH = 768K
}

ENTRY(Reset_Handler)

/* Explicit program headers, so the executable region is not marked writable.
 * See the bench script for the reasoning. */
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

   /* ARM unwind tables. Nothing in cyros unwinds, but an unplaced .ARM.exidx
    * silently OVERLAPS .data rather than erroring, so both are placed. */
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

   __etext = .;

   .data : {
      __data_start__ = .;
      *(.data*)
      . = ALIGN(4);
      __data_end__ = .;
   } > RAM AT> FLASH :data

   .bss (NOLOAD) : {
      __bss_start__ = .;
      *(.bss*)
      *(COMMON)
      . = ALIGN(4);
      __bss_end__ = .;
   } > RAM :NONE

   /* Main stack at the top of the contiguous SRAM, growing down. Thread stacks
    * are NOT here: cyros hands every thread a caller-owned buffer, so they are
    * ordinary objects in .bss. Only the boot path, exception handlers and the
    * scheduler run on this one. */
   __stack_top = ORIGIN(RAM) + LENGTH(RAM);

   /DISCARD/ : { *(.note.GNU-stack) }
}
