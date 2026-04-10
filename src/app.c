#include "app.h"

#include "drivers/can_bus.h"
#include "drivers/uart_async.h"
#include "main.h"
#include "services/button_can_tx_service.h"
#include "services/uart_packet_to_can_service.h"

#include <stdbool.h>

extern CAN_HandleTypeDef hcan;
extern UART_HandleTypeDef huart2;

static bool s_can_rx_mode = false;

static void set_can_rx_mode(bool enabled)
{
  if (enabled == s_can_rx_mode)
  {
    return;
  }

  if (enabled)
  {
    can_bus_set_tx_enabled(false);
    uart_packet_to_can_service_reset_input();
    can_bus_discard_rx();

    if (can_bus_set_rx_all_ids(true) != HAL_OK)
    {
      Error_Handler();
    }

    can_bus_discard_rx();
  }
  else
  {
    uart_packet_to_can_service_reset_input();
    can_bus_discard_rx();

    if (can_bus_set_rx_all_ids(false) != HAL_OK)
    {
      Error_Handler();
    }

    can_bus_discard_rx();
    can_bus_set_tx_enabled(true);
  }

  s_can_rx_mode = enabled;
}

void app_init(void)
{
  uart_packet_to_can_service_init(&hcan, &huart2);
  button_can_tx_service_init();
  s_can_rx_mode = false;
}

void app_poll(void)
{
  bool toggle_requested = false;

  if (s_can_rx_mode)
  {
    uart_packet_to_can_service_reset_input();
  }

  toggle_requested = button_can_tx_service_poll(!s_can_rx_mode);
  if (toggle_requested)
  {
    set_can_rx_mode(!s_can_rx_mode);
    return;
  }

  if (s_can_rx_mode)
  {
    can_bus_log_rx();
    return;
  }

  uart_packet_to_can_service_poll();
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  uart_async_on_tx_complete(huart);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  uart_async_on_error(huart);
}
