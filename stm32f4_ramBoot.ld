/*
*****************************************************************************
**
**  File        : stm32f4_flash.ld
**
**  Abstract    : Linker script for STM32F407 Device with
**                0.5MByte FLASH, 128KByte SRAM, 64KByte CCM
**
**                Set heap size, stack size and stack location according
**                to application requirements.
**
**                Set memory bank area and size if external memory is used.
**
**  Target      : STMicroelectronics STM32
**
*****************************************************************************
*/

/* Entry Point */
ENTRY(Reset_Handler)

/* Highest address of the user mode stack */
_estack = 0x20030000;    /* end of 192K SRAM */

/* Generate a link error if heap and stack don't fit into RAM */
_Min_Heap_Size = 0;      /* required amount of heap  */
_Min_Stack_Size = 0x200 ; /* required amount of stack for interrupt stack (Main stack) */

/* Specify the memory areas */
MEMORY
{
  FLASH (rx)      : ORIGIN = 0x08000000, LENGTH = 2048K
  RAM (xrw)       : ORIGIN = 0x20000000, LENGTH = 192K
  CCM (xrw)       : ORIGIN = 0x10000000, LENGTH = 64K
  MEMORY_B1 (rx)  : ORIGIN = 0x60000000, LENGTH = 0K
  SDRAM(xrw)      : ORIGIN = 0xD0000000, LENGTH = 8192K
}

/* Define output sections */
SECTIONS
{
  /* The startup code goes first into FLASH */

  /* The program code and other data goes into FLASH */
  .text :
  {
	 . = ALIGN(4);
    KEEP(*(.isr_vector)) /* Startup code */
	 *(.version)
    . = ALIGN(4);      /* Align the start of the text part */
    *(.text)           /* .text sections (code) */
    *(.text*)          /* .text* sections (code) */
    *(.rodata)         /* .rodata sections (constants, strings, etc.) */
    *(.rodata*)        /* .rodata* sections (constants, strings, etc.) */
    *(.glue_7)         /* glue arm to thumb code */
    *(.glue_7t)        /* glue thumb to arm code */
	*(.eh_frame)

    KEEP (*(.init))
    KEEP (*(.fini))

    . = ALIGN(4);
    _etext = .;        /* define a global symbols at end of code */
  } >RAM


   .ARM.extab   : { *(.ARM.extab* .gnu.linkonce.armextab.*) } >FLASH
    .ARM : {
    __exidx_start = .;
      *(.ARM.exidx*)
      __exidx_end = .;
    } >RAM

  .preinit_array     :
  {
    PROVIDE_HIDDEN (__preinit_array_start = .);
    KEEP (*(.preinit_array*))
    PROVIDE_HIDDEN (__preinit_array_end = .);
  } >RAM
  .init_array :
  {
    PROVIDE_HIDDEN (__init_array_start = .);
    KEEP (*(SORT(.init_array.*)))
    KEEP (*(.init_array*))
    PROVIDE_HIDDEN (__init_array_end = .);
  } >RAM
  .fini_array :
  {
    PROVIDE_HIDDEN (__fini_array_start = .);
    KEEP (*(.fini_array*))
    KEEP (*(SORT(.fini_array.*)))
    PROVIDE_HIDDEN (__fini_array_end = .);
  } >RAM

  /* used by the startup to initialize data */
  _sidata = .;

  /* Initialized data sections goes into RAM, load LMA copy after code */
  .data : AT ( _sidata )
  {
    . = ALIGN(4);
    _sdata = .;        /* create a global symbol at data start */
    *(.data)           /* .data sections */
    *(.data*)          /* .data* sections */

    . = ALIGN(4);
    _edata = .;        /* define a global symbol at data end */
  } >RAM

  /* Uninitialized data section */
  . = ALIGN(4);
  .bss :
  {
    /* This is used by the startup in order to initialize the .bss secion */
    _sbss = .;         /* define a global symbol at bss start */
    __bss_start__ = _sbss;
    *(.bss)
    *(.bss*)
    *(COMMON)

    . = ALIGN(4);
    _ebss = .;         /* define a global symbol at bss end */
    __bss_end__ = _ebss;
  } >RAM

  PROVIDE ( end = _ebss );
  PROVIDE ( _end = _ebss );

  /* User_heap_stack section, used to check that there is enough RAM left */
  ._user_heap_stack :
  {
    . = ALIGN(4);
    . = . + _Min_Heap_Size;
    . = . + _Min_Stack_Size;
    . = ALIGN(4);
  } >RAM

  /* MEMORY_bank1 section, code must be located here explicitly            */
  /* Example: extern int foo(void) __attribute__ ((section (".mb1text"))); */
  .memory_b1_text :
  {
    *(.mb1text)        /* .mb1text sections (code) */
    *(.mb1text*)       /* .mb1text* sections (code)  */
    *(.mb1rodata)      /* read-only data (constants) */
    *(.mb1rodata*)
  } >MEMORY_B1
  
  .sdram (NOLOAD) :
  {
    *(.sdram)
    *(.sdram*)
    *(.sdram_rodata)
    *(.sdram_rodata*)
  } >SDRAM
  
  /* Remove information from the standard libraries */
  /DISCARD/ :
  {
    libc.a ( * )
    libm.a ( * )
    libgcc.a ( * )
  }

  .ARM.attributes 0 : { *(.ARM.attributes) }
}
