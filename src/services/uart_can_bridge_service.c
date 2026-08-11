#include "services/uart_can_bridge_service.h"

#include "drivers/can_bus.h"
#include "drivers/uart_async.h"
#include "protocol/crc16_ccitt.h"

#include <errno.h>
#include <limits.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define UART_READ_CHUNK_LEN 64U
#define AC_PACKET_LEN 39U
#define AC_MODE_IK 0U
#define AC_MODE_MANUAL 1U
#define AC_MODE_KEYBOARD_AUTO 2U
#define UART_BINARY_MAX_LEN AC_PACKET_LEN
#define UART_ASCII_MAX_LEN 47U

#define JF_PACKET_LEN 16U
#define UF_PACKET_LEN 40U
#define UF_PAYLOAD_LEN 32U
#define UF_CAN_FIRST_ID 0x205U
#define UF_CAN_LAST_ID 0x208U
#define UF_CAN_FRAME_LEN 8U
#define UF_INTER_FRAME_TIMEOUT_MS 250U
#define UF_NORMAL_OUTPUT_GUARD_MS 250U
#define JF_PERIOD_MS 20U
#define ASCII_PERIOD_MS 100U
#define LATEST_VALUE_CAPACITY 16U

#define UF_FLAGS 0x13U

typedef enum
{
  UART_PARSE_IDLE = 0,
  UART_PARSE_BINARY,
  UART_PARSE_ASCII,
  UART_PARSE_ASCII_DISCARD
} UartParseState;

typedef struct
{
  uint16_t std_id;
  int32_t value;
  bool used;
} LatestValue;

static UartParseState s_uart_parse_state;
static uint8_t s_binary_packet[UART_BINARY_MAX_LEN];
static uint8_t s_binary_length;
static uint8_t s_binary_expected_length;
static char s_ascii_line[UART_ASCII_MAX_LEN + 1U];
static uint8_t s_ascii_length;

static uint16_t s_jf_encoders[5];
static uint8_t s_jf_flags;
static uint8_t s_jf_seen_mask;
static uint32_t s_jf_last_sent_at;

static uint8_t s_uf_payload[UF_PAYLOAD_LEN];
static uint8_t s_uf_next_frame;
static bool s_uf_assembling;
static uint32_t s_uf_last_frame_at;
static bool s_uf_guard_active;
static uint32_t s_uf_last_activity_at;

static LatestValue s_latest_values[LATEST_VALUE_CAPACITY];
static uint8_t s_latest_value_count;
static uint8_t s_latest_replacement_index;
static uint32_t s_ascii_last_sent_at;

static uint8_t s_downlink_sequence;

static uint16_t read_le16(const uint8_t *data)
{
  return (uint16_t)data[0] | ((uint16_t)data[1] << 8);
}

static void write_le16(uint8_t *data, uint16_t value)
{
  data[0] = (uint8_t)(value & 0xFFU);
  data[1] = (uint8_t)(value >> 8);
}

static void write_be32(uint8_t *data, uint32_t value)
{
  data[0] = (uint8_t)(value >> 24);
  data[1] = (uint8_t)(value >> 16);
  data[2] = (uint8_t)(value >> 8);
  data[3] = (uint8_t)value;
}

static bool binary_crc_is_valid(const uint8_t *packet, uint8_t length)
{
  uint16_t expected_crc;

  if ((packet == NULL) || (length < 2U))
  {
    return false;
  }

  expected_crc = read_le16(&packet[length - 2U]);
  return crc16_ccitt_false(packet, (uint32_t)length - 2U) == expected_crc;
}

static void send_manual_ac_to_can(const uint8_t packet[AC_PACKET_LEN])
{
  uint8_t frame[8] = {0};

  memcpy(frame, &packet[4], 8U);
  (void)can_bus_send(0x200U, frame, 8U);

  memset(frame, 0, sizeof(frame));
  memcpy(frame, &packet[12], 6U);
  frame[6] = packet[30];
  (void)can_bus_send(0x201U, frame, 8U);
}

static void copy_ac_outer_currents(uint8_t frame[8],
                                   const uint8_t packet[AC_PACKET_LEN])
{
  memcpy(&frame[0], &packet[4], 4U);
  memcpy(&frame[4], &packet[14], 4U);
}

static void send_ik_ac_to_can(const uint8_t packet[AC_PACKET_LEN])
{
  uint8_t frame[8] = {0};

  memcpy(frame, &packet[18], 6U);
  (void)can_bus_send(0x210U, frame, 8U);

  memset(frame, 0, sizeof(frame));
  memcpy(frame, &packet[24], 6U);
  frame[6] = packet[30];
  (void)can_bus_send(0x211U, frame, 8U);

  memset(frame, 0, sizeof(frame));
  copy_ac_outer_currents(frame, packet);
  (void)can_bus_send(0x212U, frame, 8U);
}

static void send_keyboard_auto_ac_to_can(const uint8_t packet[AC_PACKET_LEN])
{
  uint8_t frame[8] = {0};

  memcpy(frame, &packet[18], 6U);
  memcpy(&frame[6], &packet[31], 2U);
  (void)can_bus_send(0x501U, frame, 8U);

  memset(frame, 0, sizeof(frame));
  frame[0] = packet[30];
  memcpy(&frame[1], &packet[33], 2U);
  memcpy(&frame[3], &packet[35], 2U);
  (void)can_bus_send(0x502U, frame, 8U);

  memset(frame, 0, sizeof(frame));
  copy_ac_outer_currents(frame, packet);
  (void)can_bus_send(0x503U, frame, 8U);
}

static bool handle_ac_packet(void)
{
  uint8_t mode;

  if ((s_binary_packet[0] != 'A') || (s_binary_packet[1] != 'C') ||
      !binary_crc_is_valid(s_binary_packet, AC_PACKET_LEN))
  {
    return false;
  }

  mode = (uint8_t)((s_binary_packet[3] >> 4) & 0x03U);
  switch (mode)
  {
    case AC_MODE_IK:
      send_ik_ac_to_can(s_binary_packet);
      break;

    case AC_MODE_MANUAL:
      send_manual_ac_to_can(s_binary_packet);
      break;

    case AC_MODE_KEYBOARD_AUTO:
      send_keyboard_auto_ac_to_can(s_binary_packet);
      break;

    default:
      break;
  }
  return true;
}

static bool ascii_line_to_can(const char *line)
{
  char *end = NULL;
  char *value_start;
  unsigned long parsed_id;
  long parsed_value;
  uint8_t frame[4];

  if (line == NULL)
  {
    return false;
  }

  errno = 0;
  parsed_id = strtoul(line, &end, 0);
  if ((end == line) || (errno == ERANGE) || (parsed_id > 0x7FFUL) || (*end != ','))
  {
    return false;
  }

  errno = 0;
  value_start = end + 1;
  parsed_value = strtol(value_start, &end, 0);
  if ((end == value_start) || (errno == ERANGE) ||
      (parsed_value < INT32_MIN) || (parsed_value > INT32_MAX))
  {
    return false;
  }

  while ((*end == ' ') || (*end == '\t'))
  {
    ++end;
  }
  if (*end != '\0')
  {
    return false;
  }

  write_be32(frame, (uint32_t)(int32_t)parsed_value);
  return can_bus_send((uint16_t)parsed_id, frame, 4U) == HAL_OK;
}

static void begin_ac_packet(void)
{
  s_uart_parse_state = UART_PARSE_BINARY;
  s_binary_packet[0] = 'A';
  s_binary_length = 1U;
  s_binary_expected_length = AC_PACKET_LEN;
}

static void reset_uart_parser(void)
{
  s_uart_parse_state = UART_PARSE_IDLE;
  s_binary_length = 0U;
  s_binary_expected_length = 0U;
  s_ascii_length = 0U;
}

static void resynchronize_ac_packet(void)
{
  for (uint8_t i = 1U; (uint8_t)(i + 1U) < s_binary_length; ++i)
  {
    if ((s_binary_packet[i] == 'A') && (s_binary_packet[i + 1U] == 'C'))
    {
      uint8_t remaining = (uint8_t)(s_binary_length - i);
      memmove(s_binary_packet, &s_binary_packet[i], remaining);
      s_binary_length = remaining;
      s_binary_expected_length = AC_PACKET_LEN;
      s_uart_parse_state = UART_PARSE_BINARY;
      return;
    }
  }

  if (s_binary_packet[s_binary_length - 1U] == 'A')
  {
    begin_ac_packet();
    return;
  }

  reset_uart_parser();
}

static void consume_uart_byte(uint8_t byte)
{
  if (s_uart_parse_state == UART_PARSE_BINARY)
  {
    s_binary_packet[s_binary_length++] = byte;

    if ((s_binary_length == 2U) && (s_binary_packet[1] != 'C'))
    {
      reset_uart_parser();
      if (byte == 'A')
      {
        begin_ac_packet();
      }
      return;
    }

    if (s_binary_length >= s_binary_expected_length)
    {
      if (handle_ac_packet())
      {
        reset_uart_parser();
      }
      else
      {
        resynchronize_ac_packet();
      }
    }
    return;
  }

  if ((byte == '\r') || (byte == '\n'))
  {
    if (s_uart_parse_state == UART_PARSE_ASCII)
    {
      s_ascii_line[s_ascii_length] = '\0';
      (void)ascii_line_to_can(s_ascii_line);
    }
    reset_uart_parser();
    return;
  }

  if (s_uart_parse_state == UART_PARSE_ASCII_DISCARD)
  {
    return;
  }

  if (s_uart_parse_state == UART_PARSE_IDLE)
  {
    if (byte == 'A')
    {
      begin_ac_packet();
      return;
    }

    s_uart_parse_state = UART_PARSE_ASCII;
  }

  if (s_ascii_length >= UART_ASCII_MAX_LEN)
  {
    s_uart_parse_state = UART_PARSE_ASCII_DISCARD;
    return;
  }

  s_ascii_line[s_ascii_length++] = (char)byte;
}

static void poll_uart_input(void)
{
  uint8_t chunk[UART_READ_CHUNK_LEN];
  uint16_t length;

  do
  {
    length = uart_async_read(chunk, sizeof(chunk));
    for (uint16_t i = 0U; i < length; ++i)
    {
      consume_uart_byte(chunk[i]);
    }
  } while (length == sizeof(chunk));
}

static void send_jf_packet(void)
{
  uint8_t packet[JF_PACKET_LEN] = {0};
  uint16_t crc;

  packet[0] = 'J';
  packet[1] = 'F';
  packet[2] = s_downlink_sequence++;
  packet[3] = s_jf_flags;
  for (uint8_t i = 0U; i < 5U; ++i)
  {
    write_le16(&packet[4U + (i * 2U)], s_jf_encoders[i]);
  }
  crc = crc16_ccitt_false(packet, JF_PACKET_LEN - 2U);
  write_le16(&packet[JF_PACKET_LEN - 2U], crc);
  (void)uart_async_write(packet, sizeof(packet));
}

static uint8_t uf_payload_length(void)
{
  for (uint8_t i = 0U; i < UF_PAYLOAD_LEN; ++i)
  {
    if (s_uf_payload[i] == 0U)
    {
      return i;
    }
  }
  return UF_PAYLOAD_LEN;
}

static void send_uf_packet(void)
{
  uint8_t packet[UF_PACKET_LEN] = {0};
  uint16_t crc;

  packet[0] = 'U';
  packet[1] = 'F';
  packet[2] = s_downlink_sequence++;
  packet[3] = UF_FLAGS;
  packet[4] = 0U;
  packet[5] = uf_payload_length();
  memcpy(&packet[6], s_uf_payload, UF_PAYLOAD_LEN);
  crc = crc16_ccitt_false(packet, UF_PACKET_LEN - 2U);
  write_le16(&packet[UF_PACKET_LEN - 2U], crc);
  (void)uart_async_write(packet, sizeof(packet));
}

static void reset_uf_assembly(void)
{
  s_uf_assembling = false;
  s_uf_next_frame = 0U;
  memset(s_uf_payload, 0, sizeof(s_uf_payload));
}

static void mark_uf_activity(uint32_t now_ms)
{
  if ((!s_uf_guard_active) ||
      ((now_ms - s_uf_last_activity_at) >= UF_NORMAL_OUTPUT_GUARD_MS))
  {
    uart_async_discard_pending_tx();
  }
  s_uf_guard_active = true;
  s_uf_last_activity_at = now_ms;
}

static void handle_uf_can_frame(uint16_t std_id,
                                const uint8_t data[8],
                                uint8_t dlc,
                                uint32_t now_ms)
{
  uint8_t frame_index = (uint8_t)(std_id - UF_CAN_FIRST_ID);

  mark_uf_activity(now_ms);

  if (s_uf_assembling &&
      ((now_ms - s_uf_last_frame_at) > UF_INTER_FRAME_TIMEOUT_MS))
  {
    reset_uf_assembly();
  }

  if (dlc < UF_CAN_FRAME_LEN)
  {
    reset_uf_assembly();
    return;
  }

  if (frame_index == 0U)
  {
    reset_uf_assembly();
    s_uf_assembling = true;
  }

  if ((!s_uf_assembling) || (frame_index != s_uf_next_frame))
  {
    reset_uf_assembly();
    return;
  }

  memcpy(&s_uf_payload[(uint32_t)frame_index * UF_CAN_FRAME_LEN],
         data,
         UF_CAN_FRAME_LEN);
  s_uf_last_frame_at = now_ms;
  ++s_uf_next_frame;

  if (std_id == UF_CAN_LAST_ID)
  {
    send_uf_packet();
    reset_uf_assembly();
  }
}

static void store_latest_value(uint16_t std_id, int32_t value)
{
  uint8_t index;

  for (index = 0U; index < s_latest_value_count; ++index)
  {
    if (s_latest_values[index].used && (s_latest_values[index].std_id == std_id))
    {
      s_latest_values[index].value = value;
      return;
    }
  }

  if (s_latest_value_count < LATEST_VALUE_CAPACITY)
  {
    index = s_latest_value_count++;
  }
  else
  {
    index = s_latest_replacement_index;
    s_latest_replacement_index =
      (uint8_t)((s_latest_replacement_index + 1U) % LATEST_VALUE_CAPACITY);
  }

  s_latest_values[index].std_id = std_id;
  s_latest_values[index].value = value;
  s_latest_values[index].used = true;
}

static int32_t generic_can_value(uint16_t std_id, const uint8_t data[8], uint8_t dlc)
{
  if ((std_id >= 0x410U) && (std_id <= 0x413U) && (dlc >= 4U))
  {
    uint32_t raw = ((uint32_t)data[0] << 24) |
                   ((uint32_t)data[1] << 16) |
                   ((uint32_t)data[2] << 8) |
                   (uint32_t)data[3];
    return (int32_t)raw;
  }

  uint32_t sum = 0U;
  for (uint8_t i = 0U; i < 8U; ++i)
  {
    sum += data[i];
  }
  return (int32_t)sum;
}

static void handle_can_frame(uint16_t std_id,
                             const uint8_t data[8],
                             uint8_t dlc,
                             void *context)
{
  uint32_t now_ms = HAL_GetTick();
  (void)context;

  if (std_id == 0x203U)
  {
    s_jf_encoders[0] = read_le16(&data[0]);
    s_jf_encoders[1] = read_le16(&data[2]);
    s_jf_flags = data[4];
    s_jf_seen_mask |= 0x01U;
    return;
  }

  if (std_id == 0x204U)
  {
    s_jf_encoders[2] = read_le16(&data[0]);
    s_jf_encoders[3] = read_le16(&data[2]);
    s_jf_encoders[4] = read_le16(&data[4]);
    s_jf_seen_mask |= 0x02U;
    return;
  }

  if ((std_id >= UF_CAN_FIRST_ID) && (std_id <= UF_CAN_LAST_ID))
  {
    handle_uf_can_frame(std_id, data, dlc, now_ms);
    return;
  }

  store_latest_value(std_id, generic_can_value(std_id, data, dlc));
}

static bool normal_uart_output_is_blocked(uint32_t now_ms)
{
  if (!s_uf_guard_active)
  {
    return false;
  }

  if ((now_ms - s_uf_last_activity_at) < UF_NORMAL_OUTPUT_GUARD_MS)
  {
    return true;
  }

  s_uf_guard_active = false;
  return false;
}

static void send_latest_ascii_values(void)
{
  char line[24];

  for (uint8_t i = 0U; i < s_latest_value_count; ++i)
  {
    int length;

    if (!s_latest_values[i].used)
    {
      continue;
    }

    length = snprintf(line,
                      sizeof(line),
                      "0x%03X,%ld\r\n",
                      (unsigned int)s_latest_values[i].std_id,
                      (long)s_latest_values[i].value);
    if ((length > 0) && ((size_t)length < sizeof(line)))
    {
      (void)uart_async_write((const uint8_t *)line, (uint16_t)length);
    }
  }
}

static void poll_downlink_output(uint32_t now_ms)
{
  if (s_uf_assembling &&
      ((now_ms - s_uf_last_frame_at) > UF_INTER_FRAME_TIMEOUT_MS))
  {
    reset_uf_assembly();
  }

  if (normal_uart_output_is_blocked(now_ms))
  {
    /* Pending normal transmissions are discarded while UF has priority. */
    s_jf_last_sent_at = now_ms;
    s_ascii_last_sent_at = now_ms;
    return;
  }

  if ((s_jf_seen_mask == 0x03U) && ((now_ms - s_jf_last_sent_at) >= JF_PERIOD_MS))
  {
    s_jf_last_sent_at = now_ms;
    send_jf_packet();
  }

  if ((now_ms - s_ascii_last_sent_at) >= ASCII_PERIOD_MS)
  {
    s_ascii_last_sent_at = now_ms;
    send_latest_ascii_values();
  }
}

void uart_can_bridge_init(CAN_HandleTypeDef *hcan, UART_HandleTypeDef *huart)
{
  uint32_t now_ms = HAL_GetTick();

  reset_uart_parser();
  memset(s_jf_encoders, 0, sizeof(s_jf_encoders));
  s_jf_flags = 0U;
  s_jf_seen_mask = 0U;
  s_jf_last_sent_at = now_ms;
  reset_uf_assembly();
  s_uf_last_frame_at = now_ms;
  s_uf_guard_active = false;
  s_uf_last_activity_at = now_ms;
  memset(s_latest_values, 0, sizeof(s_latest_values));
  s_latest_value_count = 0U;
  s_latest_replacement_index = 0U;
  s_ascii_last_sent_at = now_ms;
  s_downlink_sequence = 0U;

  uart_async_init(huart);
  can_bus_init(hcan);
}

void uart_can_bridge_poll(void)
{
  uint32_t now_ms;

  poll_uart_input();
  (void)can_bus_poll(handle_can_frame, NULL);
  now_ms = HAL_GetTick();
  poll_downlink_output(now_ms);
}
