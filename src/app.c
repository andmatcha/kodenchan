#include "app.h"

#include "main.h"
#include "modules/can_bridge.h"
#include "modules/uart_async.h"

extern CAN_HandleTypeDef hcan;
extern UART_HandleTypeDef huart2;

void app_init(void)
{
  uart_async_init(&huart2);
  can_bridge_init(&hcan, &huart2);
}

void app_poll(void)
{
  can_bridge_poll();
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  uart_async_on_tx_complete(huart);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  uart_async_on_error(huart);
}
