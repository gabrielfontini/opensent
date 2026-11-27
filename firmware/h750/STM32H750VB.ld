/* STM32H750VB (WeAct core board): 128 KB flash.
 * .data/.bss live in AXI SRAM (DMA reachable, no D-cache used).
 * The stack and .noinit live in DTCM (not reachable by DMA1/2). */
ENTRY(Reset_Handler)

MEMORY
{
  FLASH  (rx)  : ORIGIN = 0x08000000, LENGTH = 128K
  DTCM   (rw)  : ORIGIN = 0x20000000, LENGTH = 128K
  AXISRAM (rw) : ORIGIN = 0x24000000, LENGTH = 512K
  SRAM12 (rw)  : ORIGIN = 0x30000000, LENGTH = 256K   /* D2: logic-analyzer buffer, next to DMA2 */
}

_estack = ORIGIN(DTCM) + LENGTH(DTCM);
_Min_Heap_Size  = 0x1000;
_Min_Stack_Size = 0x2000;

SECTIONS
{
  .isr_vector : { . = ALIGN(4); KEEP(*(.isr_vector)) . = ALIGN(4); } >FLASH

  .text :
  {
    . = ALIGN(4);
    *(.text) *(.text*) *(.glue_7) *(.glue_7t) *(.eh_frame)
    KEEP(*(.init)) KEEP(*(.fini))
    . = ALIGN(4);
    _etext = .;
  } >FLASH

  .rodata : { . = ALIGN(4); *(.rodata) *(.rodata*) . = ALIGN(4); } >FLASH
  .ARM.extab : { *(.ARM.extab* .gnu.linkonce.armextab.*) } >FLASH
  .ARM : { __exidx_start = .; *(.ARM.exidx*) __exidx_end = .; } >FLASH

  .preinit_array : { PROVIDE_HIDDEN(__preinit_array_start = .); KEEP(*(.preinit_array*)) PROVIDE_HIDDEN(__preinit_array_end = .); } >FLASH
  .init_array : { PROVIDE_HIDDEN(__init_array_start = .); KEEP(*(SORT(.init_array.*))) KEEP(*(.init_array*)) PROVIDE_HIDDEN(__init_array_end = .); } >FLASH
  .fini_array : { PROVIDE_HIDDEN(__fini_array_start = .); KEEP(*(SORT(.fini_array.*))) KEEP(*(.fini_array*)) PROVIDE_HIDDEN(__fini_array_end = .); } >FLASH

  _sidata = LOADADDR(.data);
  .data :
  {
    . = ALIGN(4);
    _sdata = .;
    *(.data) *(.data*)
    . = ALIGN(4);
    _edata = .;
  } >AXISRAM AT> FLASH

  .bss (NOLOAD) :
  {
    . = ALIGN(4);
    _sbss = .;
    __bss_start__ = _sbss;
    *(.bss) *(.bss*) *(COMMON)
    . = ALIGN(4);
    _ebss = .;
    __bss_end__ = _ebss;
  } >AXISRAM

  ._user_heap (NOLOAD) :
  {
    . = ALIGN(8);
    PROVIDE(end = .);
    PROVIDE(_end = .);
    . = . + _Min_Heap_Size;
    . = ALIGN(8);
  } >AXISRAM

  .sram12 (NOLOAD) : { . = ALIGN(4); *(.sram12) *(.sram12*) } >SRAM12

  .noinit (NOLOAD) : { . = ALIGN(4); *(.noinit) *(.noinit*) . = ALIGN(4); } >DTCM
  ._stack_check (NOLOAD) : { . = ALIGN(8); . = . + _Min_Stack_Size; } >DTCM

  .ARM.attributes 0 : { *(.ARM.attributes) }
}
