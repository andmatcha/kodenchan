#include "drivers/can_bus.h"

#include "main.h"

#include <string.h>

#define CAN_TX_TIMEOUT_MS 10U
#define CAN_STD_ID_SHIFT 5U
#define CAN_MAX_STD_ID 0x7FFU
#define CAN_MAX_DLC 8U

static CAN_HandleTypeDef *s_hcan = NULL;

static HAL_StatusTypeDef configure_id_list_filter(uint32_t bank,
                                                  uint16_t id0,
                                                  uint16_t id1,
                                                  uint16_t id2,
                                                  uint16_t id3)
{
  CAN_FilterTypeDef filter = {0};

  filter.FilterBank = bank;
  filter.FilterMode = CAN_FILTERMODE_IDLIST;
  filter.FilterScale = CAN_FILTERSCALE_16BIT;
  filter.FilterIdHigh = (uint32_t)id0 << CAN_STD_ID_SHIFT;
  filter.FilterIdLow = (uint32_t)id1 << CAN_STD_ID_SHIFT;
  filter.FilterMaskIdHigh = (uint32_t)id2 << CAN_STD_ID_SHIFT;
  filter.FilterMaskIdLow = (uint32_t)id3 << CAN_STD_ID_SHIFT;
  filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  filter.FilterActivation = CAN_FILTER_ENABLE;
  filter.SlaveStartFilterBank = 14U;

  return HAL_CAN_ConfigFilter(s_hcan, &filter);
}

static HAL_StatusTypeDef configure_id_mask_filter(uint32_t bank,
                                                  uint16_t id,
                                                  uint16_t mask)
{
  CAN_FilterTypeDef filter = {0};

  filter.FilterBank = bank;
  filter.FilterMode = CAN_FILTERMODE_IDMASK;
  filter.FilterScale = CAN_FILTERSCALE_32BIT;
  filter.FilterIdHigh = (uint32_t)id << CAN_STD_ID_SHIFT;
  filter.FilterIdLow = 0U;
  filter.FilterMaskIdHigh = (uint32_t)mask << CAN_STD_ID_SHIFT;
  filter.FilterMaskIdLow = 0U;
  filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  filter.FilterActivation = CAN_FILTER_ENABLE;
  filter.SlaveStartFilterBank = 14U;

  return HAL_CAN_ConfigFilter(s_hcan, &filter);
}

static HAL_StatusTypeDef configure_bridge_filters(void)
{
  HAL_StatusTypeDef status;

  status = configure_id_list_filter(0U, 0x203U, 0x204U, 0x205U, 0x206U);
  if (status != HAL_OK)
  {
    return status;
  }

  status = configure_id_list_filter(1U, 0x207U, 0x208U, 0x207U, 0x208U);
  if (status != HAL_OK)
  {
    return status;
  }

  status = configure_id_mask_filter(2U, 0x300U, 0x700U);
  if (status != HAL_OK)
  {
    return status;
  }

  status = configure_id_mask_filter(3U, 0x400U, 0x700U);
  if (status != HAL_OK)
  {
    return status;
  }

  return configure_id_mask_filter(4U, 0x500U, 0x700U);
}

void can_bus_init(CAN_HandleTypeDef *hcan)
{
  s_hcan = hcan;

  if ((s_hcan == NULL) ||
      (configure_bridge_filters() != HAL_OK) ||
      (HAL_CAN_Start(s_hcan) != HAL_OK))
  {
    Error_Handler();
  }
}

HAL_StatusTypeDef can_bus_send(uint16_t std_id, const uint8_t *data, uint8_t dlc)
{
  CAN_TxHeaderTypeDef header = {0};
  uint32_t mailbox = 0U;
  uint32_t started_at = HAL_GetTick();

  if ((s_hcan == NULL) || (data == NULL) || (std_id > CAN_MAX_STD_ID) ||
      (dlc > CAN_MAX_DLC))
  {
    return HAL_ERROR;
  }

  /* The reference implementation deliberately accepts ID 0 but emits no frame. */
  if (std_id == 0U)
  {
    return HAL_OK;
  }

  header.StdId = std_id;
  header.IDE = CAN_ID_STD;
  header.RTR = CAN_RTR_DATA;
  header.DLC = dlc;
  header.TransmitGlobalTime = DISABLE;

  while (HAL_CAN_GetTxMailboxesFreeLevel(s_hcan) == 0U)
  {
    if ((HAL_GetTick() - started_at) >= CAN_TX_TIMEOUT_MS)
    {
      return HAL_TIMEOUT;
    }
  }

  return HAL_CAN_AddTxMessage(s_hcan, &header, (uint8_t *)data, &mailbox);
}

uint32_t can_bus_poll(CanBusRxCallback callback, void *context)
{
  const uint32_t fifos[] = {CAN_RX_FIFO0, CAN_RX_FIFO1};
  CAN_RxHeaderTypeDef header;
  uint8_t data[CAN_MAX_DLC];
  uint32_t received = 0U;

  if (s_hcan == NULL)
  {
    return 0U;
  }

  for (uint32_t i = 0U; i < (sizeof(fifos) / sizeof(fifos[0])); ++i)
  {
    while (HAL_CAN_GetRxFifoFillLevel(s_hcan, fifos[i]) > 0U)
    {
      memset(&header, 0, sizeof(header));
      memset(data, 0, sizeof(data));

      if (HAL_CAN_GetRxMessage(s_hcan, fifos[i], &header, data) != HAL_OK)
      {
        Error_Handler();
      }

      ++received;
      if ((callback != NULL) && (header.IDE == CAN_ID_STD) &&
          (header.RTR == CAN_RTR_DATA))
      {
        uint8_t dlc = (header.DLC <= CAN_MAX_DLC) ? (uint8_t)header.DLC : CAN_MAX_DLC;
        callback((uint16_t)header.StdId, data, dlc, context);
      }
    }
  }

  return received;
}
