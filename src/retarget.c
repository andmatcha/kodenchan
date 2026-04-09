#include "main.h"
#include <stdio.h>
#include <unistd.h>

// UARTハンドラ（CubeMXで有効化されている必要あり）
extern UART_HandleTypeDef huart2;

// printfをシリアルモニタに表示するための関数
// _write: printfが内部で呼ぶシステム関数の一部
int _write(int file, char *ptr, int len)
{
  (void)file;
  return UART_AsyncWrite((const uint8_t *)ptr, (uint16_t)len);
}
