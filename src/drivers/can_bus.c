/*
 * 責務: HAL CAN の filter 設定、送信、feedback polling をまとめる。
 * 依存関係: services から送信frameを受けてHAL CANへ出し、受信feedbackは callback で control/arm_state 側へ渡す。
 */

#include "drivers/can_bus.h"

#include "drivers/uart_async.h"
#include "main.h"

#include <stdbool.h>

#define CAN_TX_TIMEOUT_MS 10U
#define CAN_FILTER_STDID_SHIFT 5U
#define CAN_RX_FILTER_BANK_200 0U
#define CAN_RX_FILTER_BANK_300 1U
#define CAN_RX_FILTER_BASE_ID_200 0x200U
#define CAN_RX_FILTER_BASE_ID_300 0x300U
#define CAN_RX_FILTER_ARM_MASK 0x7F8U
#define CAN_STD_ID_HEX_DIGITS 3U
#define CAN_EXT_ID_HEX_DIGITS 8U
#define CAN_MAX_DLC 8U
#define CAN_ALL_TX_MAILBOXES (CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2)

static CAN_HandleTypeDef *s_hcan = 0;
static bool s_tx_enabled = true;

static char hex_digit(uint8_t value)
{
  return (value < 10U) ? (char)('0' + value) : (char)('A' + (value - 10U));
}

static void write_hex_byte(char *text, uint8_t value)
{
  text[0] = hex_digit((uint8_t)(value >> 4));
  text[1] = hex_digit((uint8_t)(value & 0x0FU));
}

static void write_hex_u32(char *text, uint32_t value, uint8_t digit_count)
{
  for (uint8_t i = 0U; i < digit_count; ++i)
  {
    uint8_t shift = (uint8_t)((digit_count - 1U - i) * 4U);
    text[i] = hex_digit((uint8_t)((value >> shift) & 0x0FU));
  }
}

static void print_can_data_line(const char direction[2], uint32_t id, uint8_t id_digit_count, const uint8_t data[8], uint8_t data_len)
{
  char line[44];
  uint32_t offset = 0U;

  if (data_len > CAN_MAX_DLC)
  {
    data_len = CAN_MAX_DLC;
  }

  line[offset++] = 'C';
  line[offset++] = 'A';
  line[offset++] = 'N';
  line[offset++] = ' ';
  line[offset++] = direction[0];
  line[offset++] = direction[1];
  line[offset++] = ' ';
  line[offset++] = '0';
  line[offset++] = 'x';
  write_hex_u32(&line[offset], id, id_digit_count);
  offset += id_digit_count;
  line[offset++] = ':';

  for (uint32_t i = 0U; i < data_len; ++i)
  {
    line[offset++] = ' ';
    write_hex_byte(&line[offset], data[i]);
    offset += 2U;
  }

  line[offset++] = '\r';
  line[offset++] = '\n';
  (void)uart_async_write((const uint8_t *)line, (uint16_t)offset);
}

static void print_can_tx_line(uint16_t std_id, const uint8_t data[8])
{
  print_can_data_line("TX", std_id, CAN_STD_ID_HEX_DIGITS, data, CAN_MAX_DLC);
}

static void print_can_rx_line(const CAN_RxHeaderTypeDef *rx_header, const uint8_t data[8])
{
  uint8_t dlc = 0U;

  if ((rx_header == 0) || (data == 0) || (rx_header->RTR != CAN_RTR_DATA))
  {
    return;
  }

  dlc = (rx_header->DLC <= CAN_MAX_DLC) ? (uint8_t)rx_header->DLC : CAN_MAX_DLC;

  if (rx_header->IDE == CAN_ID_STD)
  {
    print_can_data_line("RX", rx_header->StdId, CAN_STD_ID_HEX_DIGITS, data, dlc);
  }
  else if (rx_header->IDE == CAN_ID_EXT)
  {
    print_can_data_line("RX", rx_header->ExtId, CAN_EXT_ID_HEX_DIGITS, data, dlc);
  }
}

static HAL_StatusTypeDef configure_filter(uint32_t filter_bank, uint16_t std_id, uint16_t std_id_mask, uint32_t fifo, uint32_t activation)
{
  CAN_FilterTypeDef filter = {0};

  filter.FilterBank = filter_bank;
  filter.FilterMode = CAN_FILTERMODE_IDMASK;
  filter.FilterScale = CAN_FILTERSCALE_32BIT;
  filter.FilterIdHigh = (uint32_t)std_id << CAN_FILTER_STDID_SHIFT;
  filter.FilterIdLow = 0U;
  filter.FilterMaskIdHigh = (uint32_t)std_id_mask << CAN_FILTER_STDID_SHIFT;
  filter.FilterMaskIdLow = 0U;
  filter.FilterFIFOAssignment = fifo;
  filter.FilterActivation = activation;
  filter.SlaveStartFilterBank = 14U;

  return HAL_CAN_ConfigFilter(s_hcan, &filter);
}

static HAL_StatusTypeDef configure_normal_filters(void)
{
  HAL_StatusTypeDef status = HAL_OK;

  status = configure_filter(CAN_RX_FILTER_BANK_200,
                            CAN_RX_FILTER_BASE_ID_200,
                            CAN_RX_FILTER_ARM_MASK,
                            CAN_FILTER_FIFO0,
                            CAN_FILTER_ENABLE);
  if (status != HAL_OK)
  {
    return status;
  }

  return configure_filter(CAN_RX_FILTER_BANK_300,
                          CAN_RX_FILTER_BASE_ID_300,
                          CAN_RX_FILTER_ARM_MASK,
                          CAN_FILTER_FIFO1,
                          CAN_FILTER_ENABLE);
}

static HAL_StatusTypeDef configure_all_id_filter(void)
{
  HAL_StatusTypeDef status = HAL_OK;

  status = configure_filter(CAN_RX_FILTER_BANK_200, 0U, 0U, CAN_FILTER_FIFO0, CAN_FILTER_ENABLE);
  if (status != HAL_OK)
  {
    return status;
  }

  return configure_filter(CAN_RX_FILTER_BANK_300, 0U, 0U, CAN_FILTER_FIFO1, CAN_FILTER_DISABLE);
}

void can_bus_init(CAN_HandleTypeDef *hcan)
{
  s_hcan = hcan;
  s_tx_enabled = true;

  if (s_hcan == 0)
  {
    Error_Handler();
  }

  if (configure_normal_filters() != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_CAN_Start(s_hcan) != HAL_OK)
  {
    Error_Handler();
  }
}

HAL_StatusTypeDef can_bus_set_rx_all_ids(bool enabled)
{
  if (s_hcan == 0)
  {
    return HAL_ERROR;
  }

  if (enabled)
  {
    return configure_all_id_filter();
  }

  return configure_normal_filters();
}

void can_bus_set_tx_enabled(bool enabled)
{
  s_tx_enabled = enabled;

  if (!enabled && (s_hcan != 0))
  {
    (void)HAL_CAN_AbortTxRequest(s_hcan, CAN_ALL_TX_MAILBOXES);
  }
}

HAL_StatusTypeDef can_bus_send(uint16_t std_id, const uint8_t data[8])
{
  CAN_TxHeaderTypeDef tx_header = {0};
  uint32_t tx_mailbox = 0U;
  uint32_t start_tick = HAL_GetTick();
  HAL_StatusTypeDef status = HAL_OK;

  if ((s_hcan == 0) || (data == 0))
  {
    return HAL_ERROR;
  }

  if (!s_tx_enabled)
  {
    return HAL_OK;
  }

  tx_header.StdId = std_id;
  tx_header.IDE = CAN_ID_STD;
  tx_header.RTR = CAN_RTR_DATA;
  tx_header.DLC = 8U;
  tx_header.TransmitGlobalTime = DISABLE;

  while (HAL_CAN_GetTxMailboxesFreeLevel(s_hcan) == 0U)
  {
    if ((HAL_GetTick() - start_tick) > CAN_TX_TIMEOUT_MS)
    {
      return HAL_TIMEOUT;
    }
  }

  status = HAL_CAN_AddTxMessage(s_hcan, &tx_header, (uint8_t *)data, &tx_mailbox);
  if (status == HAL_OK)
  {
    print_can_tx_line(std_id, data);
  }

  return status;
}

static void poll_rx(CanBusRxCallback callback, void *context, bool print_rx)
{
  CAN_RxHeaderTypeDef rx_header = {0};
  uint8_t rx_data[8] = {0};
  uint32_t fifos[2] = {CAN_RX_FIFO0, CAN_RX_FIFO1};

  if (s_hcan == 0)
  {
    return;
  }

  for (uint32_t fifo_index = 0U; fifo_index < 2U; ++fifo_index)
  {
    uint32_t fifo = fifos[fifo_index];
    uint32_t pending = HAL_CAN_GetRxFifoFillLevel(s_hcan, fifo);

    while (pending > 0U)
    {
      if (HAL_CAN_GetRxMessage(s_hcan, fifo, &rx_header, rx_data) != HAL_OK)
      {
        Error_Handler();
      }

      if (print_rx)
      {
        print_can_rx_line(&rx_header, rx_data);
      }

      if ((callback != 0) && (rx_header.IDE == CAN_ID_STD) && (rx_header.RTR == CAN_RTR_DATA))
      {
        callback((uint16_t)rx_header.StdId, rx_data, context);
      }

      --pending;
    }
  }
}

void can_bus_poll(CanBusRxCallback callback, void *context)
{
  poll_rx(callback, context, false);
}

void can_bus_log_rx(void)
{
  poll_rx(0, 0, true);
}

void can_bus_discard_rx(void)
{
  poll_rx(0, 0, false);
}
