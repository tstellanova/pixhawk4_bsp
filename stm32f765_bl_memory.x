/*
 For STM32F765 device with offset for PX4 bootloader 
*/
MEMORY
{
  /* NOTE K = KiBi = 1024 bytes */
  /* reserve up to 32K for the PX4 bootloader: */
  BL_FLASH : ORIGIN = 0x08000000, LENGTH = 32K
  FLASH : ORIGIN = 0x8008000, LENGTH = 2M - 32K
  RAM : ORIGIN = 0x20000000, LENGTH = 368K + 16K
}

/* This is where the call stack will be allocated. */
/* The stack is of the full descending type. */
/* NOTE Do NOT modify `_stack_start` unless you know what you are doing */
_stack_start = ORIGIN(RAM) + LENGTH(RAM);
