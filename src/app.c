#include "app.h"

#include "drivers/uart_async.h"
#include "main.h"
#include "services/uart_can_bridge_service.h"

extern CAN_HandleTypeDef hcan;
extern UART_HandleTypeDef huart2;

void app_init(void)
{
  uart_can_bridge_init(&hcan, &huart2);
}

void app_poll(void)
{
  uart_can_bridge_poll();
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  uart_async_on_tx_complete(huart);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  uart_async_on_error(huart);
}
