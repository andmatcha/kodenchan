#include "stm32f3xx_hal.h"
#include <stdio.h>
#include <unistd.h>

// UARTハンドラ（CubeMXで有効化されている必要あり）
extern UART_HandleTypeDef huart2;

// printfをシリアルモニタに表示するための関数
// _write: printfが内部で呼ぶシステム関数の一部
int _write(int file, char *ptr, int len)
{
  HAL_UART_Transmit(&huart2, (uint8_t *)ptr, len, HAL_MAX_DELAY);
  return len;
}
