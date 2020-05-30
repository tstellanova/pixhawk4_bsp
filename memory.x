/* Memory layout for stm32f765 */
MEMORY
{
  /* NOTE K = 1024 bytes */
  /* FLASH and RAM are mandatory memory regions */

  /* NOTE: the PX4 bootloader is stored below our app  at 0x08000000 and is 16K long (up to 32K)*/
  /* BL_FLASH  :    ORIGIN = 0x08000000, LENGTH = 16K */

  /* AXIM interface to flash aka FLASH_AXIM 0x08008000  - 2016K */
  FLASH  :    ORIGIN = 0x08008000, LENGTH = 2016K
  /* AKA DTCM_RAM+SRAM1+SRAM2  0x20000000 - 512K */
  RAM   :     ORIGIN = 0x20000000, LENGTH = 512K


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

