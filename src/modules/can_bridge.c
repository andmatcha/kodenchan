#include "modules/can_bridge.h"

#include "main.h"
#include "modules/uart_async.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define AC_PACKET_V3_LEN 31U
#define AC_PACKET_V6_LEN 39U
#define AC_STREAM_BUFFER_LEN 256U
#define SHORT_PACKET_IDLE_MS 3U
#define MODE_MANUAL 1U
#define MANUAL_UPLINK_CANID_1 0x200U
#define MANUAL_UPLINK_CANID_2 0x201U

typedef struct
{
  uint8_t seq;
  uint8_t flags;
  uint16_t current[7];
  uint8_t extra_flags;
} PacketAC_v3;

static CAN_HandleTypeDef *s_hcan = NULL;
static UART_HandleTypeDef *s_huart = NULL;
static uint8_t s_uart_stream_buffer[AC_STREAM_BUFFER_LEN];
static uint32_t s_uart_stream_length = 0U;
static uint32_t s_last_uart_rx_tick = 0U;

static uint16_t read_u16_le(const uint8_t *data)
{
  return (uint16_t)data[0] | ((uint16_t)data[1] << 8);
}

static void write_u16_le(uint8_t *data, uint16_t value)
{
  data[0] = (uint8_t)(value & 0xFFU);
  data[1] = (uint8_t)((value >> 8) & 0xFFU);
}

static uint16_t crc16_ccitt_false(const uint8_t *data, uint32_t length)
{
  uint16_t crc = 0xFFFFU;

  for (uint32_t i = 0; i < length; ++i)
  {
    crc ^= (uint16_t)data[i] << 8;

    for (uint32_t bit = 0; bit < 8; ++bit)
    {
      if ((crc & 0x8000U) != 0U)
      {
        crc = (uint16_t)((crc << 1) ^ 0x1021U);
      }
      else
      {
        crc <<= 1;
      }
    }
  }

  return crc;
}

static void shift_stream_buffer(uint32_t count)
{
  if (count >= s_uart_stream_length)
  {
    s_uart_stream_length = 0U;
    return;
  }

  memmove(s_uart_stream_buffer, s_uart_stream_buffer + count, s_uart_stream_length - count);
  s_uart_stream_length -= count;
}

static bool stream_has_header(void)
{
  for (uint32_t i = 0; i + 1U < s_uart_stream_length; ++i)
  {
    if ((s_uart_stream_buffer[i] == 'A') && (s_uart_stream_buffer[i + 1U] == 'C'))
    {
      if (i != 0U)
      {
        shift_stream_buffer(i);
      }
      return true;
    }
  }

  if (s_uart_stream_length > 1U)
  {
    uint8_t tail = s_uart_stream_buffer[s_uart_stream_length - 1U];
    s_uart_stream_buffer[0] = tail;
    s_uart_stream_length = (tail == 'A') ? 1U : 0U;
  }

  return false;
}

static bool packet_has_valid_v6_crc(const uint8_t *packet)
{
  uint16_t expected_crc = read_u16_le(packet + (AC_PACKET_V6_LEN - 2U));
  uint16_t actual_crc = crc16_ccitt_false(packet, AC_PACKET_V6_LEN - 2U);
  return expected_crc == actual_crc;
}

static void pump_uart_stream(void)
{
  uint8_t rx_bytes[32];
  uint16_t received = 0U;

  do
  {
    received = uart_async_read(rx_bytes, sizeof(rx_bytes));
    if (received == 0U)
    {
      break;
    }

    for (uint16_t i = 0U; i < received; ++i)
    {
      if (s_uart_stream_length >= AC_STREAM_BUFFER_LEN)
      {
        shift_stream_buffer(1U);
      }

      s_uart_stream_buffer[s_uart_stream_length++] = rx_bytes[i];
    }

    s_last_uart_rx_tick = HAL_GetTick();
  } while (received == sizeof(rx_bytes));
}

static bool extract_ac_packet(uint8_t *packet, bool allow_short_packet)
{
  while (s_uart_stream_length >= 2U)
  {
    if (!stream_has_header())
    {
      return false;
    }

    if (s_uart_stream_length < AC_PACKET_V3_LEN)
    {
      return false;
    }

    if ((s_uart_stream_length >= AC_PACKET_V6_LEN) && packet_has_valid_v6_crc(s_uart_stream_buffer))
    {
      memcpy(packet, s_uart_stream_buffer, AC_PACKET_V6_LEN);
      shift_stream_buffer(AC_PACKET_V6_LEN);
      return true;
    }

    if ((s_uart_stream_length >= (AC_PACKET_V3_LEN + 2U)) &&
        (s_uart_stream_buffer[AC_PACKET_V3_LEN] == 'A') &&
        (s_uart_stream_buffer[AC_PACKET_V3_LEN + 1U] == 'C'))
    {
      memcpy(packet, s_uart_stream_buffer, AC_PACKET_V3_LEN);
      shift_stream_buffer(AC_PACKET_V3_LEN);
      return true;
    }

    if (allow_short_packet)
    {
      memcpy(packet, s_uart_stream_buffer, AC_PACKET_V3_LEN);
      shift_stream_buffer(AC_PACKET_V3_LEN);
      return true;
    }

    return false;
  }

  return false;
}

static bool poll_ac_packet(uint8_t *packet)
{
  pump_uart_stream();

  if (extract_ac_packet(packet, false))
  {
    return true;
  }

  if ((s_uart_stream_length >= AC_PACKET_V3_LEN) &&
      ((HAL_GetTick() - s_last_uart_rx_tick) >= SHORT_PACKET_IDLE_MS))
  {
    return extract_ac_packet(packet, true);
  }

  return false;
}

static void load_packet_ac(PacketAC_v3 *packet, const uint8_t *raw_packet)
{
  packet->seq = raw_packet[2];
  packet->flags = raw_packet[3];

  for (uint32_t i = 0; i < 7U; ++i)
  {
    packet->current[i] = read_u16_le(raw_packet + 4U + (i * 2U));
  }

  packet->extra_flags = raw_packet[30];
}

static void print_can_tx_line(uint16_t std_id, const uint8_t *data)
{
  printf("CAN TX 0x%03X:", std_id);

  for (uint32_t i = 0; i < 8U; ++i)
  {
    printf(" %02X", data[i]);
  }

  printf("\r\n");
}

static HAL_StatusTypeDef send_can_frame(uint16_t std_id, const uint8_t *data)
{
  CAN_TxHeaderTypeDef tx_header = {0};
  uint32_t tx_mailbox = 0U;
  uint32_t start_tick = HAL_GetTick();
  HAL_StatusTypeDef status = HAL_OK;

  if (s_hcan == NULL)
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
    if ((HAL_GetTick() - start_tick) > 10U)
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

static void transmit_manual_packet(const PacketAC_v3 *packet)
{
  uint8_t frame[8] = {0};

  write_u16_le(&frame[0], packet->current[0]);
  write_u16_le(&frame[2], packet->current[1]);
  write_u16_le(&frame[4], packet->current[2]);
  write_u16_le(&frame[6], packet->current[3]);
  if (send_can_frame(MANUAL_UPLINK_CANID_1, frame) != HAL_OK)
  {
    printf("Failed CAN TX: 0x%03X\r\n", MANUAL_UPLINK_CANID_1);
  }

  memset(frame, 0, sizeof(frame));
  write_u16_le(&frame[0], packet->current[4]);
  write_u16_le(&frame[2], packet->current[5]);
  write_u16_le(&frame[4], packet->current[6]);
  frame[6] = packet->extra_flags;
  if (send_can_frame(MANUAL_UPLINK_CANID_2, frame) != HAL_OK)
  {
    printf("Failed CAN TX: 0x%03X\r\n", MANUAL_UPLINK_CANID_2);
  }
}

static void transmit_packet_as_can(const PacketAC_v3 *packet)
{
  uint8_t mode = (packet->flags >> 4) & 0x03U;

  if (mode == MODE_MANUAL)
  {
    transmit_manual_packet(packet);
    return;
  }

  printf("Ignored non-manual mode: %u\r\n", mode);
}

void can_bridge_init(CAN_HandleTypeDef *hcan, UART_HandleTypeDef *huart)
{
  s_hcan = hcan;
  s_huart = huart;
  s_uart_stream_length = 0U;
  s_last_uart_rx_tick = 0U;

  if ((s_hcan == NULL) || (s_huart == NULL))
  {
    Error_Handler();
  }

  if (HAL_CAN_Start(s_hcan) != HAL_OK)
  {
    printf("CAN Start failed\r\n");
    Error_Handler();
  }

  printf("Bridge ready: USART2 %lu DMA -> CAN 1Mbps\r\n", (unsigned long)s_huart->Init.BaudRate);
}

void can_bridge_poll(void)
{
  uint8_t raw_packet[AC_PACKET_V6_LEN] = {0};
  PacketAC_v3 packet = {0};

  if (!poll_ac_packet(raw_packet))
  {
    return;
  }

  if ((raw_packet[0] != 'A') || (raw_packet[1] != 'C'))
  {
    return;
  }

  load_packet_ac(&packet, raw_packet);
  transmit_packet_as_can(&packet);
}
