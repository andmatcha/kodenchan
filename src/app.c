#include "app.h"

#include "drivers/can_bus.h"
#include "drivers/uart_async.h"
#include "main.h"
#include "services/button_can_tx_service.h"
#include "services/uart_packet_to_can_service.h"

#include <stdbool.h>
#include <stdint.h>

extern CAN_HandleTypeDef hcan;
extern UART_HandleTypeDef huart2;

#define CAN_RX_LED_BLINK_PERIOD_MS 50U
#define CAN_RX_LED_ACTIVITY_HOLD_MS 200U

static bool s_can_rx_mode = false;
static uint32_t s_can_rx_last_activity_ms = 0U;
static uint32_t s_can_rx_led_last_toggle_ms = 0U;
static bool s_can_rx_led_activity_active = false;
static GPIO_PinState s_can_rx_led_state = GPIO_PIN_RESET;

static void set_can_rx_led(GPIO_PinState state)
{
  if (state == s_can_rx_led_state)
  {
    return;
  }

  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, state);
  s_can_rx_led_state = state;
}

static void reset_can_rx_led(bool enabled, uint32_t now_ms)
{
  s_can_rx_last_activity_ms = now_ms;
  s_can_rx_led_last_toggle_ms = now_ms;
  s_can_rx_led_activity_active = false;
  set_can_rx_led(enabled ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void update_can_rx_led(uint32_t now_ms, bool received)
{
  if (received)
  {
    if (!s_can_rx_led_activity_active)
    {
      s_can_rx_led_last_toggle_ms = now_ms - CAN_RX_LED_BLINK_PERIOD_MS;
    }

    s_can_rx_led_activity_active = true;
    s_can_rx_last_activity_ms = now_ms;
  }

  if (s_can_rx_led_activity_active &&
      ((now_ms - s_can_rx_last_activity_ms) < CAN_RX_LED_ACTIVITY_HOLD_MS))
  {
    if ((now_ms - s_can_rx_led_last_toggle_ms) >= CAN_RX_LED_BLINK_PERIOD_MS)
    {
      s_can_rx_led_last_toggle_ms = now_ms;
      set_can_rx_led((s_can_rx_led_state == GPIO_PIN_SET) ? GPIO_PIN_RESET : GPIO_PIN_SET);
    }

    return;
  }

  s_can_rx_led_activity_active = false;
  s_can_rx_led_last_toggle_ms = now_ms;
  set_can_rx_led(GPIO_PIN_SET);
}

static void set_can_rx_mode(bool enabled)
{
  uint32_t now_ms = HAL_GetTick();

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
  reset_can_rx_led(enabled, now_ms);
}

void app_init(void)
{
  uart_packet_to_can_service_init(&hcan, &huart2);
  button_can_tx_service_init();
  s_can_rx_mode = false;
  reset_can_rx_led(false, HAL_GetTick());
}

void app_poll(void)
{
  uint32_t now_ms = HAL_GetTick();
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
    update_can_rx_led(now_ms, can_bus_log_rx() > 0U);
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
