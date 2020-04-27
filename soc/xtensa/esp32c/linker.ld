/* ESP32 Linker Script Memory Layout

   This file describes the memory layout (memory blocks) as virtual
   memory addresses.

   esp32.common.ld contains output sections to link compiler output
   into these memory blocks.

   ***

   This linker script is passed through the C preprocessor to include
   configuration options.

   Please use preprocessor features sparingly! Restrict
   to simple macros with numeric values, and/or #if/#endif blocks.
*/

#include <devicetree.h>
#include <autoconf.h>
#include <linker/sections.h>
#include <linker/linker-defs.h>
#include <linker/linker-tool.h>
#include "esp32.rom.ld"

#define RAMABLE_REGION dram0_0_seg :dram0_0_phdr
#define ROMABLE_REGION iram0_0_seg :iram0_0_phdr

PROVIDE ( __stack = 0x3fceb310 );

PROVIDE ( esp32c_rom_uart_tx_one_char = 0x40013f48 );
PROVIDE ( esp32c_rom_uart_rx_one_char = 0x40014040 );
PROVIDE ( esp32c_rom_uart_attach = 0x40013bc8 );
PROVIDE ( esp32c_rom_intr_matrix_set = 0x4000ffdc );
PROVIDE ( esp32c_rom_gpio_matrix_in = 0x4001bb0c );
PROVIDE ( esp32c_rom_gpio_matrix_out = 0x4001bb3c );
PROVIDE ( esp32c_rom_ets_set_appcpu_boot_addr = 0x40010390 );

MEMORY
{
  /* All these values assume the flash cache is on, and have the blocks this uses subtracted from the length
  of the various regions. The 'data access port' dram/drom regions map to the same iram/irom regions but
  are connected to the data port of the CPU and eg allow bytewise access. */

  /* IRAM for PRO cpu. Not sure if happy with this, this is MMU area... */
#if CONFIG_INSTRUCTION_CACHE_16KB
  iram0_0_seg (RX) :                 org = 0x40374000, len = 0x1C000
#else
  iram0_0_seg (RX) :                 org = 0x40378000, len = 0x18000
#endif

  /* Even though the segment name is iram, it is actually mapped to flash
  */
  iram0_2_seg (RX) :                 org = 0x42000018, len = 0x4000000-0x18



  /* Shared data RAM, excluding memory reserved for ROM bss/data/stack.

     Enabling Bluetooth & Trace Memory features in menuconfig will decrease
     the amount of RAM available.

     Note: Length of this section *should* be 0x50000, and this extra DRAM is available
     in heap at runtime. However due to static ROM memory usage at this 176KB mark, the
     additional static memory temporarily cannot be used.
  */
  dram0_0_seg (RW) :                 org = 0x3FCA0000,
                                     len = 0x38000

  /* Flash mapped constant data */
  drom0_0_seg (R) :                  org = 0x3D000018, len = 0x4000000-0x18

  /* (See iram0_2_seg for meaning of 0x18 offset in the above.) */

  /* RTC fast memory (executable). Persists over deep sleep.
   */
  rtc_iram_seg(RWX) :                org = 0x600fe000, len = 0x2000
  rtc_slow_seg(RW): org = 0x50000000, len = 0x1000
#ifdef CONFIG_GEN_ISR_TABLES
  IDT_LIST(RW): org = 0x3ebfe010, len = 0x2000
#endif
}

/* Heap ends at top of dram0_0_seg */
_heap_end = 0x40000000;


PHDRS
{
  iram0_0_phdr PT_LOAD;
  dram0_0_phdr PT_LOAD;
}

/*  Default entry point:  */
PROVIDE ( _ResetVector = 0x40000400 );
ENTRY(CONFIG_KERNEL_ENTRY)

_rom_store_table = 0;

PROVIDE(_memmap_vecbase_reset = 0x40000454);
PROVIDE(_memmap_reset_vector = 0x40000400);

SECTIONS
{

#include <linker/rel-sections.ld>
#if CONFIG_SOC_ESP32C

#else
  /* RTC fast memory holds RTC wake stub code,
     including from any source file named rtc_wake_stub*.c
  */
  .rtc.text :
  {
    . = ALIGN(4);
    *(.rtc.literal .rtc.text)
    *rtc_wake_stub*.o(.literal .text .literal.* .text.*)
  } >rtc_iram_seg

  /* RTC slow memory holds RTC wake stub
     data/rodata, including from any source file
     named rtc_wake_stub*.c
  */
  .rtc.data :
  {
    _rtc_data_start = ABSOLUTE(.);
    *(.rtc.data)
    *(.rtc.rodata)
    *rtc_wake_stub*.o(.data .rodata .data.* .rodata.* .bss .bss.*)
    _rtc_data_end = ABSOLUTE(.);
  } > rtc_slow_seg

  /* RTC bss, from any source file named rtc_wake_stub*.c */
  .rtc.bss (NOLOAD) :
  {
    _rtc_bss_start = ABSOLUTE(.);
    *rtc_wake_stub*.o(.bss .bss.*)
    *rtc_wake_stub*.o(COMMON)
    _rtc_bss_end = ABSOLUTE(.);
  } > rtc_slow_seg
#endif
  /* Send .iram0 code to iram */
  .iram0.vectors : ALIGN(4)
  {
    /* Vectors go to IRAM */
    _init_start = ABSOLUTE(.);
    /* Vectors according to builds/RF-2015.2-win32/esp108_v1_2_s5_512int_2/config.html */
    . = 0x0;
    KEEP(*(.WindowVectors.text));
    . = 0x180;
    KEEP(*(.Level2InterruptVector.text));
    . = 0x1c0;
    KEEP(*(.Level3InterruptVector.text));
    . = 0x200;
    KEEP(*(.Level4InterruptVector.text));
    . = 0x240;
    KEEP(*(.Level5InterruptVector.text));
    . = 0x280;
    KEEP(*(.DebugExceptionVector.text));
    . = 0x2c0;
    KEEP(*(.NMIExceptionVector.text));
    . = 0x300;
    KEEP(*(.KernelExceptionVector.text));
    . = 0x340;
    KEEP(*(.UserExceptionVector.text));
    . = 0x3C0;
    KEEP(*(.DoubleExceptionVector.text));
    . = 0x400;
    *(.*Vector.literal)

    *(.UserEnter.literal);
    *(.UserEnter.text);
    . = ALIGN (16);
    *(.entry.text)
    *(.init.literal)
    *(.init)
    _init_end = ABSOLUTE(.);

    /* This goes here, not at top of linker script, so addr2line finds it last,
       and uses it in preference to the first symbol in IRAM */
    _iram_start = ABSOLUTE(0);
  } GROUP_LINK_IN(ROMABLE_REGION)

#include <linker/common-ram.ld>
#include <linker/common-rom.ld>

  SECTION_PROLOGUE(_TEXT_SECTION_NAME, , ALIGN(4))
  {
    /* Code marked as running out of IRAM */
    _iram_text_start = ABSOLUTE(.);
    *(.iram1 .iram1.*)
    *(.iram0.literal .iram.literal .iram.text.literal .iram0.text .iram.text)
    *(.literal .text .literal.* .text.*)
    _iram_text_end = ABSOLUTE(.);
  } GROUP_LINK_IN(ROMABLE_REGION)

  .dram0.data :
  {
    _data_start = ABSOLUTE(.);
    *(.data)
    *(.data.*)
    *(.gnu.linkonce.d.*)
    *(.data1)
    *(.sdata)
    *(.sdata.*)
    *(.gnu.linkonce.s.*)
    *(.sdata2)
    *(.sdata2.*)
    *(.gnu.linkonce.s2.*)
    KEEP(*(.jcr))
    *(.dram1 .dram1.*)
    _data_end = ABSOLUTE(.);
    . = ALIGN(4);
  } GROUP_LINK_IN(RAMABLE_REGION)

  SECTION_PROLOGUE(_RODATA_SECTION_NAME,,ALIGN(4))
  {
    _rodata_start = ABSOLUTE(.);
    *(.rodata)
    *(.rodata.*)
    *(.gnu.linkonce.r.*)
    *(.rodata1)
    __XT_EXCEPTION_TABLE__ = ABSOLUTE(.);
    KEEP (*(.xt_except_table))
    KEEP (*(.gcc_except_table .gcc_except_table.*))
    *(.gnu.linkonce.e.*)
    *(.gnu.version_r)
    KEEP (*(.eh_frame))
    /*  C++ constructor and destructor tables, properly ordered:  */
    KEEP (*crtbegin.o(.ctors))
    KEEP (*(EXCLUDE_FILE (*crtend.o) .ctors))
    KEEP (*(SORT(.ctors.*)))
    KEEP (*(.ctors))
    KEEP (*crtbegin.o(.dtors))
    KEEP (*(EXCLUDE_FILE (*crtend.o) .dtors))
    KEEP (*(SORT(.dtors.*)))
    KEEP (*(.dtors))
    /*  C++ exception handlers table:  */
    __XT_EXCEPTION_DESCS__ = ABSOLUTE(.);
    *(.xt_except_desc)
    *(.gnu.linkonce.h.*)
    __XT_EXCEPTION_DESCS_END__ = ABSOLUTE(.);
    *(.xt_except_desc_end)
    *(.dynamic)
    *(.gnu.version_d)
    . = ALIGN(4);               /* this table MUST be 4-byte aligned */
    _rodata_end = ABSOLUTE(.);
  } GROUP_LINK_IN(RAMABLE_REGION)


  /* Shared RAM */
  SECTION_DATA_PROLOGUE(_BSS_SECTION_NAME,(NOLOAD),)
  {
    . = ALIGN (8);
    _bss_start = ABSOLUTE(.);
    *(.dynsbss)
    *(.sbss)
    *(.sbss.*)
    *(.gnu.linkonce.sb.*)
    *(.scommon)
    *(.sbss2)
    *(.sbss2.*)
    *(.gnu.linkonce.sb2.*)
    *(.dynbss)
    *(.bss)
    *(.bss.*)
    *(.share.mem)
    *(.gnu.linkonce.b.*)
    *(COMMON)
    . = ALIGN (8);
    _bss_end = ABSOLUTE(.);
  } GROUP_LINK_IN(RAMABLE_REGION)


   SECTION_DATA_PROLOGUE(_APP_NOINIT_SECTION_NAME, (NOLOAD),)
   {
     . = ALIGN (8);
     *(.app_noinit)
     *("app_noinit.*")
     . = ALIGN (8);
    _app_end = ABSOLUTE(.);
  } GROUP_LINK_IN(RAMABLE_REGION)


  SECTION_DATA_PROLOGUE(_NOINIT_SECTION_NAME, (NOLOAD),)
  {
    . = ALIGN (8);
    *(.noinit)
    *(".noinit.*")
    . = ALIGN (8);
    _heap_start = ABSOLUTE(.);
  } GROUP_LINK_IN(RAMABLE_REGION)

#ifdef CONFIG_GEN_ISR_TABLES
#include <linker/intlist.ld>
#endif

#include <linker/debug-sections.ld>

  SECTION_PROLOGUE(.xtensa.info, 0,)
  {
    *(.xtensa.info)
  }

}
