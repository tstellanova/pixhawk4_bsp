/* Memory layout for stm32f765 */
MEMORY
{
  /* NOTE K = 1024 bytes */
  /* FLASH and RAM are mandatory memory regions */

  /* NOTE: the PX4 bootloader is stored below this address */
  /* AXIM interface flash */
  FLASH  :    ORIGIN = 0x08008000, LENGTH = 2000K
  /* AKA SRAM1 */
  RAM   :     ORIGIN = 0x20020000, LENGTH = 368K


  ITCM_RAM (rwx) : ORIGIN = 0x00000000, LENGTH = 16K
  DTCM_RAM (rwx) : ORIGIN = 0x20000000, LENGTH = 128K
  /* SRAM1    (rwx) : ORIGIN = 0x20020000, LENGTH = 368K */
  SRAM2    (rwx) : ORIGIN = 0x2007c000, LENGTH = 16K
  /* FLASH_ITCM (rx) : ORIGIN = 0x00208000, LENGTH = 2016K */
  /* FLASH_AXIM (rx) : ORIGIN = 0x08008000, LENGTH = 2016K */

}


/*
This is where the call stack will be allocated.
The stack is of the full descending type.
Place the stack at the end of RAM.
*/
_stack_start = ORIGIN(RAM) + LENGTH(RAM);

/* The location of the .text section can be overridden using the
   `_stext` symbol.  By default it will place after .vector_table */
/* _stext = ORIGIN(FLASH) + 0x40c; */

