/*
 * 責務: UART DMA circular RX と DMA TX ring を提供する。
 * 依存関係: HAL UART/DMA を下位層として使い、services/uart_packet_to_can_service へ受信byte列、retarget から送信byte列を受け渡す。
 */

#include "drivers/uart_async.h"

#include "main.h"

#include <stdbool.h>
#include <string.h>

#define UART_DMA_RX_BUFFER_LEN 256U
#define UART_TX_BUFFER_LEN 1024U
#define UART_TX_WRITE_TIMEOUT_MS 50U

static UART_HandleTypeDef *s_huart = NULL;
static uint8_t s_uart_dma_rx_buffer[UART_DMA_RX_BUFFER_LEN];
static volatile uint16_t s_uart_dma_rx_read_index = 0U;
static uint8_t s_uart_tx_buffer[UART_TX_BUFFER_LEN];
static volatile uint16_t s_uart_tx_head = 0U;
static volatile uint16_t s_uart_tx_tail = 0U;
static volatile uint16_t s_uart_tx_dma_length = 0U;
static volatile uint8_t s_uart_tx_dma_active = 0U;
static volatile uint8_t s_uart_async_ready = 0U;

static uint32_t enter_critical_section(void)
{
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  return primask;
}

static void exit_critical_section(uint32_t primask)
{
  if (primask == 0U)
  {
    __enable_irq();
  }
}

static bool is_target_uart(const UART_HandleTypeDef *huart)
{
  return (s_huart != NULL) && (huart == s_huart);
}

static uint16_t get_uart_tx_free_locked(void)
{
  if (s_uart_tx_head >= s_uart_tx_tail)
  {
    return (uint16_t)((UART_TX_BUFFER_LEN - (s_uart_tx_head - s_uart_tx_tail)) - 1U);
  }

  return (uint16_t)((s_uart_tx_tail - s_uart_tx_head) - 1U);
}

static uint16_t enqueue_uart_tx_locked(const uint8_t *data, uint16_t length)
{
  uint16_t free_space = get_uart_tx_free_locked();
  uint16_t copy_length = (length < free_space) ? length : free_space;
  uint16_t first_chunk = copy_length;

  if (copy_length == 0U)
  {
    return 0U;
  }

  if ((s_uart_tx_head + first_chunk) > UART_TX_BUFFER_LEN)
  {
    first_chunk = (uint16_t)(UART_TX_BUFFER_LEN - s_uart_tx_head);
  }

  memcpy(&s_uart_tx_buffer[s_uart_tx_head], data, first_chunk);

  if (copy_length > first_chunk)
  {
    memcpy(s_uart_tx_buffer, data + first_chunk, copy_length - first_chunk);
  }

  s_uart_tx_head = (uint16_t)((s_uart_tx_head + copy_length) % UART_TX_BUFFER_LEN);
  return copy_length;
}

static void start_uart_tx_dma_locked(void)
{
  uint16_t transfer_length = 0U;

  if ((s_huart == NULL) || (s_uart_async_ready == 0U) || (s_uart_tx_dma_active != 0U))
  {
    return;
  }

  if (s_uart_tx_head == s_uart_tx_tail)
  {
    return;
  }

  if (s_uart_tx_head > s_uart_tx_tail)
  {
    transfer_length = (uint16_t)(s_uart_tx_head - s_uart_tx_tail);
  }
  else
  {
    transfer_length = (uint16_t)(UART_TX_BUFFER_LEN - s_uart_tx_tail);
  }

  s_uart_tx_dma_length = transfer_length;
  s_uart_tx_dma_active = 1U;

  if (HAL_UART_Transmit_DMA(s_huart, &s_uart_tx_buffer[s_uart_tx_tail], transfer_length) != HAL_OK)
  {
    s_uart_tx_dma_length = 0U;
    s_uart_tx_dma_active = 0U;
  }
}

static HAL_StatusTypeDef start_circular_reception(void)
{
  HAL_StatusTypeDef status = HAL_ERROR;

  if (s_huart == NULL)
  {
    return HAL_ERROR;
  }

  status = HAL_UART_Receive_DMA(s_huart, s_uart_dma_rx_buffer, UART_DMA_RX_BUFFER_LEN);
  if (status == HAL_OK)
  {
    __HAL_DMA_DISABLE_IT(s_huart->hdmarx, DMA_IT_HT);
    __HAL_DMA_DISABLE_IT(s_huart->hdmarx, DMA_IT_TC);
  }

  return status;
}

static void restart_circular_reception(void)
{
  if (s_huart == NULL)
  {
    return;
  }

  if (HAL_UART_AbortReceive(s_huart) != HAL_OK)
  {
    return;
  }

  __HAL_UART_CLEAR_OREFLAG(s_huart);
  s_uart_dma_rx_read_index = 0U;

  if (start_circular_reception() != HAL_OK)
  {
    Error_Handler();
  }
}

void uart_async_init(UART_HandleTypeDef *huart)
{
  s_huart = huart;
  s_uart_dma_rx_read_index = 0U;
  s_uart_tx_head = 0U;
  s_uart_tx_tail = 0U;
  s_uart_tx_dma_length = 0U;
  s_uart_tx_dma_active = 0U;
  s_uart_async_ready = 0U;

  if (start_circular_reception() != HAL_OK)
  {
    Error_Handler();
  }

  s_uart_async_ready = 1U;
}

uint16_t uart_async_read(uint8_t *data, uint16_t max_length)
{
  uint16_t write_index = 0U;
  uint16_t count = 0U;

  if ((data == NULL) || (max_length == 0U) || (s_huart == NULL) || (s_huart->hdmarx == NULL))
  {
    return 0U;
  }

  write_index = (uint16_t)(UART_DMA_RX_BUFFER_LEN - __HAL_DMA_GET_COUNTER(s_huart->hdmarx));
  if (write_index >= UART_DMA_RX_BUFFER_LEN)
  {
    write_index = 0U;
  }

  while ((s_uart_dma_rx_read_index != write_index) && (count < max_length))
  {
    data[count++] = s_uart_dma_rx_buffer[s_uart_dma_rx_read_index];
    s_uart_dma_rx_read_index = (uint16_t)((s_uart_dma_rx_read_index + 1U) % UART_DMA_RX_BUFFER_LEN);
  }

  return count;
}

int uart_async_write(const uint8_t *data, uint16_t length)
{
  uint16_t written = 0U;
  uint32_t start_tick = HAL_GetTick();

  if ((data == NULL) || (length == 0U))
  {
    return 0;
  }

  if (s_huart == NULL)
  {
    return 0;
  }

  if (s_uart_async_ready == 0U)
  {
    if (HAL_UART_Transmit(s_huart, (uint8_t *)data, length, HAL_MAX_DELAY) == HAL_OK)
    {
      return (int)length;
    }

    return 0;
  }

  while (written < length)
  {
    uint32_t primask = enter_critical_section();
    written += enqueue_uart_tx_locked(data + written, (uint16_t)(length - written));
    start_uart_tx_dma_locked();
    exit_critical_section(primask);

    if (written >= length)
    {
      break;
    }

    if ((HAL_GetTick() - start_tick) >= UART_TX_WRITE_TIMEOUT_MS)
    {
      break;
    }
  }

  return (int)written;
}

void uart_async_on_tx_complete(UART_HandleTypeDef *huart)
{
  uint32_t primask = 0U;

  if (!is_target_uart(huart))
  {
    return;
  }

  primask = enter_critical_section();
  s_uart_tx_tail = (uint16_t)((s_uart_tx_tail + s_uart_tx_dma_length) % UART_TX_BUFFER_LEN);
  s_uart_tx_dma_length = 0U;
  s_uart_tx_dma_active = 0U;
  start_uart_tx_dma_locked();
  exit_critical_section(primask);
}

void uart_async_on_error(UART_HandleTypeDef *huart)
{
  if (!is_target_uart(huart))
  {
    return;
  }

  restart_circular_reception();
}
