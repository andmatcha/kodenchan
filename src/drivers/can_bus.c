#include "drivers/can_bus.h"

#include "main.h"

#define CAN_TX_TIMEOUT_MS 10U
#define CAN_FILTER_STDID_SHIFT 5U

static CAN_HandleTypeDef *s_hcan = 0;

static HAL_StatusTypeDef configure_filter(uint32_t filter_bank, uint16_t std_id, uint16_t std_id_mask, uint32_t fifo)
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
  filter.FilterActivation = ENABLE;
  filter.SlaveStartFilterBank = 14U;

  return HAL_CAN_ConfigFilter(s_hcan, &filter);
}

void can_bus_init(CAN_HandleTypeDef *hcan)
{
  s_hcan = hcan;

  if (s_hcan == 0)
  {
    Error_Handler();
  }

  if (configure_filter(0U, 0x200U, 0x7F8U, CAN_FILTER_FIFO0) != HAL_OK)
  {
    Error_Handler();
  }

  if (configure_filter(1U, 0x300U, 0x7F8U, CAN_FILTER_FIFO1) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_CAN_Start(s_hcan) != HAL_OK)
  {
    Error_Handler();
  }
}

HAL_StatusTypeDef can_bus_send(uint16_t std_id, const uint8_t data[8])
{
  CAN_TxHeaderTypeDef tx_header = {0};
  uint32_t tx_mailbox = 0U;
  uint32_t start_tick = HAL_GetTick();

  if ((s_hcan == 0) || (data == 0))
  {
    return HAL_ERROR;
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

  return HAL_CAN_AddTxMessage(s_hcan, &tx_header, (uint8_t *)data, &tx_mailbox);
}

void can_bus_poll(CanBusRxCallback callback, void *context)
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

      if ((callback != 0) && (rx_header.IDE == CAN_ID_STD) && (rx_header.RTR == CAN_RTR_DATA))
      {
        callback((uint16_t)rx_header.StdId, rx_data, context);
      }

      --pending;
    }
  }
}
