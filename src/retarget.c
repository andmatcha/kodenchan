#include "drivers/uart_async.h"

#include <stdio.h>
#include <unistd.h>

int _write(int file, char *ptr, int len)
{
  (void)file;
  return uart_async_write((const uint8_t *)ptr, (uint16_t)len);
}
