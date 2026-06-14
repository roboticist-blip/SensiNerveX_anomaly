/*
 * STM32F405RGT6 — Custom Linker Script for SensiNerveX v3.0
 *
 * Memory map:
 *   FLASH   : 0x08000000 — 1 MB  (program code + read-only data)
 *   SRAM    : 0x20000000 — 128 KB (main RAM: stack, heap, BSS)
 *   CCM     : 0x10000000 — 64 KB  (Core-Coupled Memory — CPU only, no DMA)
 *                                   → AE_Model, NormStats, SNX_Supervisor
 *
 * CCM SRAM:
 *   - Directly connected to CPU D-bus (zero wait state)
 *   - NOT accessible by DMA → safe for FPU-heavy weight arrays
 *   - Variables go in .ccmram section (explicitly placed in code)
 *
 * Usage in C:
 *   __attribute__((section(".ccmram"))) AE_Model g_ae;
 */

ENTRY(Reset_Handler)

_estack = 0x20020000;   /* End of SRAM (128 KB above 0x20000000) */
_Min_Heap_Size  = 0x400;   /* 1 KB minimum heap  */
_Min_Stack_Size = 0x800;   /* 2 KB minimum stack */

MEMORY
{
  FLASH  (rx)  : ORIGIN = 0x08000000, LENGTH = 1024K
  CCM    (rwx) : ORIGIN = 0x10000000, LENGTH = 64K
  SRAM   (rwx) : ORIGIN = 0x20000000, LENGTH = 128K
}

SECTIONS
{
  .isr_vector :
  {
    . = ALIGN(4);
    KEEP(*(.isr_vector))
    . = ALIGN(4);
  } >FLASH

  .text :
  {
    . = ALIGN(4);
    *(.text)
    *(.text*)
    *(.glue_7)
    *(.glue_7t)
    *(.eh_frame)
    KEEP(*(.init))
    KEEP(*(.fini))
    . = ALIGN(4);
    _etext = .;
  } >FLASH

  .rodata :
  {
    . = ALIGN(4);
    *(.rodata)
    *(.rodata*)
    . = ALIGN(4);
  } >FLASH

  .ARM.extab : { *(.ARM.extab* .gnu.linkonce.armextab.*) } >FLASH
  .ARM        : {
    __exidx_start = .;
    *(.ARM.exidx*)
    __exidx_end = .;
  } >FLASH

  .preinit_array :
  {
    PROVIDE_HIDDEN(__preinit_array_start = .);
    KEEP(*(.preinit_array*))
    PROVIDE_HIDDEN(__preinit_array_end = .);
  } >FLASH

  .init_array :
  {
    PROVIDE_HIDDEN(__init_array_start = .);
    KEEP(*(SORT(.init_array.*)))
    KEEP(*(.init_array*))
    PROVIDE_HIDDEN(__init_array_end = .);
  } >FLASH

  .fini_array :
  {
    PROVIDE_HIDDEN(__fini_array_start = .);
    KEEP(*(SORT(.fini_array.*)))
    KEEP(*(.fini_array*))
    PROVIDE_HIDDEN(__fini_array_end = .);
  } >FLASH

  /* ── CCM SRAM section — weights, activations, supervisor state ── */
  /* Initialised data copied from flash LMA to CCM VMA at startup    */
  _siccmram = LOADADDR(.ccmram);
  .ccmram :
  {
    . = ALIGN(4);
    _sccmram = .;
    *(.ccmram)
    *(.ccmram*)
    . = ALIGN(4);
    _eccmram = .;
  } >CCM AT> FLASH

  _sidata = LOADADDR(.data);
  .data :
  {
    . = ALIGN(4);
    _sdata = .;
    *(.data)
    *(.data*)
    . = ALIGN(4);
    _edata = .;
  } >SRAM AT> FLASH

  .bss :
  {
    . = ALIGN(4);
    _sbss = .;
    __bss_start__ = _sbss;
    *(.bss)
    *(.bss*)
    *(COMMON)
    . = ALIGN(4);
    _ebss = .;
    __bss_end__ = _ebss;
  } >SRAM

  ._user_heap_stack :
  {
    . = ALIGN(8);
    PROVIDE(end = .);
    PROVIDE(_end = .);
    . = . + _Min_Heap_Size;
    . = . + _Min_Stack_Size;
    . = ALIGN(8);
  } >SRAM

  /DISCARD/ :
  {
    libc.a(*)
    libm.a(*)
    libgcc.a(*)
  }

  .ARM.attributes 0 : { *(.ARM.attributes) }
}
