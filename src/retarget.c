#include <stdio.h>
#include <unistd.h>

int _write(int file, char *ptr, int len)
{
  (void)file;
  (void)ptr;
  return len;
}
