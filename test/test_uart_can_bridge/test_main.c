#include <unity.h>

#include "drivers/can_bus.h"
#include "drivers/uart_async.h"
#include "protocol/crc16_ccitt.h"
#include "services/uart_can_bridge_service.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define MOCK_UART_RX_CAPACITY 2048U
#define MOCK_UART_TX_CAPACITY 4096U
#define MOCK_CAN_CAPACITY 64U
#define MOCK_CAN_DATA_LEN 8U

typedef struct
{
  uint16_t std_id;
  uint8_t dlc;
  uint8_t data[MOCK_CAN_DATA_LEN];
} MockCanFrame;

static uint32_t s_mock_tick;
static uint8_t s_uart_rx[MOCK_UART_RX_CAPACITY];
static size_t s_uart_rx_length;
static size_t s_uart_rx_offset;
static uint8_t s_uart_tx[MOCK_UART_TX_CAPACITY];
static size_t s_uart_tx_length;
static MockCanFrame s_can_tx[MOCK_CAN_CAPACITY];
static size_t s_can_tx_count;
static MockCanFrame s_can_rx[MOCK_CAN_CAPACITY];
static size_t s_can_rx_count;
static size_t s_can_rx_offset;
static uint32_t s_uart_discard_count;

uint32_t HAL_GetTick(void)
{
  return s_mock_tick;
}

void uart_async_init(UART_HandleTypeDef *huart)
{
  (void)huart;
}

uint16_t uart_async_read(uint8_t *data, uint16_t max_length)
{
  size_t remaining = s_uart_rx_length - s_uart_rx_offset;
  size_t length = (remaining < max_length) ? remaining : max_length;

  memcpy(data, &s_uart_rx[s_uart_rx_offset], length);
  s_uart_rx_offset += length;
  return (uint16_t)length;
}

int uart_async_write(const uint8_t *data, uint16_t length)
{
  TEST_ASSERT_LESS_OR_EQUAL_size_t(MOCK_UART_TX_CAPACITY,
                                   s_uart_tx_length + length);
  memcpy(&s_uart_tx[s_uart_tx_length], data, length);
  s_uart_tx_length += length;
  return length;
}

void uart_async_discard_pending_tx(void)
{
  ++s_uart_discard_count;
}

void uart_async_on_tx_complete(UART_HandleTypeDef *huart)
{
  (void)huart;
}

void uart_async_on_error(UART_HandleTypeDef *huart)
{
  (void)huart;
}

void can_bus_init(CAN_HandleTypeDef *hcan)
{
  (void)hcan;
}

HAL_StatusTypeDef can_bus_send(uint16_t std_id, const uint8_t *data, uint8_t dlc)
{
  TEST_ASSERT_LESS_OR_EQUAL_UINT8(MOCK_CAN_DATA_LEN, dlc);

  if (std_id == 0U)
  {
    return HAL_OK;
  }

  TEST_ASSERT_LESS_THAN_size_t(MOCK_CAN_CAPACITY, s_can_tx_count);
  s_can_tx[s_can_tx_count].std_id = std_id;
  s_can_tx[s_can_tx_count].dlc = dlc;
  memset(s_can_tx[s_can_tx_count].data, 0, MOCK_CAN_DATA_LEN);
  memcpy(s_can_tx[s_can_tx_count].data, data, dlc);
  ++s_can_tx_count;
  return HAL_OK;
}

uint32_t can_bus_poll(CanBusRxCallback callback, void *context)
{
  uint32_t count = 0U;

  while (s_can_rx_offset < s_can_rx_count)
  {
    MockCanFrame *frame = &s_can_rx[s_can_rx_offset++];
    callback(frame->std_id, frame->data, frame->dlc, context);
    ++count;
  }
  return count;
}

static void reset_mocks(void)
{
  s_mock_tick = 0U;
  memset(s_uart_rx, 0, sizeof(s_uart_rx));
  s_uart_rx_length = 0U;
  s_uart_rx_offset = 0U;
  memset(s_uart_tx, 0, sizeof(s_uart_tx));
  s_uart_tx_length = 0U;
  memset(s_can_tx, 0, sizeof(s_can_tx));
  s_can_tx_count = 0U;
  memset(s_can_rx, 0, sizeof(s_can_rx));
  s_can_rx_count = 0U;
  s_can_rx_offset = 0U;
  s_uart_discard_count = 0U;
}

static void append_uart_rx(const uint8_t *data, size_t length)
{
  TEST_ASSERT_LESS_OR_EQUAL_size_t(MOCK_UART_RX_CAPACITY,
                                   s_uart_rx_length + length);
  memcpy(&s_uart_rx[s_uart_rx_length], data, length);
  s_uart_rx_length += length;
}

static void append_uart_text(const char *text)
{
  append_uart_rx((const uint8_t *)text, strlen(text));
}

static void queue_can_rx(uint16_t std_id, const uint8_t *data, uint8_t dlc)
{
  TEST_ASSERT_LESS_THAN_size_t(MOCK_CAN_CAPACITY, s_can_rx_count);
  TEST_ASSERT_LESS_OR_EQUAL_UINT8(MOCK_CAN_DATA_LEN, dlc);
  s_can_rx[s_can_rx_count].std_id = std_id;
  s_can_rx[s_can_rx_count].dlc = dlc;
  memset(s_can_rx[s_can_rx_count].data, 0, MOCK_CAN_DATA_LEN);
  memcpy(s_can_rx[s_can_rx_count].data, data, dlc);
  ++s_can_rx_count;
}

static void write_le16_for_test(uint8_t *data, uint16_t value)
{
  data[0] = (uint8_t)value;
  data[1] = (uint8_t)(value >> 8);
}

static uint16_t read_le16_for_test(const uint8_t *data)
{
  return (uint16_t)data[0] | ((uint16_t)data[1] << 8);
}

static void finish_packet_crc(uint8_t *packet, size_t length)
{
  uint16_t crc = crc16_ccitt_false(packet, (uint32_t)length - 2U);
  write_le16_for_test(&packet[length - 2U], crc);
}

static void assert_can_frame(size_t index,
                             uint16_t expected_id,
                             uint8_t expected_dlc,
                             const uint8_t *expected_data)
{
  TEST_ASSERT_LESS_THAN_size_t(s_can_tx_count, index);
  TEST_ASSERT_EQUAL_HEX16(expected_id, s_can_tx[index].std_id);
  TEST_ASSERT_EQUAL_UINT8(expected_dlc, s_can_tx[index].dlc);
  TEST_ASSERT_EQUAL_HEX8_ARRAY(expected_data, s_can_tx[index].data, expected_dlc);
}

static void assert_uart_crc(size_t packet_offset, size_t packet_length)
{
  uint16_t expected = crc16_ccitt_false(&s_uart_tx[packet_offset],
                                        (uint32_t)packet_length - 2U);
  uint16_t actual = read_le16_for_test(
    &s_uart_tx[packet_offset + packet_length - 2U]);
  TEST_ASSERT_EQUAL_HEX16(expected, actual);
}

static bool uart_tx_contains_text(const char *text)
{
  size_t text_length = strlen(text);

  if (text_length > s_uart_tx_length)
  {
    return false;
  }

  for (size_t i = 0U; i <= (s_uart_tx_length - text_length); ++i)
  {
    if (memcmp(&s_uart_tx[i], text, text_length) == 0)
    {
      return true;
    }
  }
  return false;
}

static void queue_jf_source_frames(void)
{
  const uint8_t frame_203[8] = {0x34, 0x12, 0x78, 0x56, 0xA5, 0, 0, 0};
  const uint8_t frame_204[8] = {0xBC, 0x9A, 0xF0, 0xDE, 0x57, 0x13, 0, 0};
  queue_can_rx(0x203U, frame_203, 8U);
  queue_can_rx(0x204U, frame_204, 8U);
}

static void queue_complete_uf(uint8_t first_payload_byte)
{
  uint8_t frame[8];

  for (uint16_t id = 0x205U; id <= 0x208U; ++id)
  {
    for (uint8_t i = 0U; i < sizeof(frame); ++i)
    {
      frame[i] = (uint8_t)(first_payload_byte +
                           ((id - 0x205U) * sizeof(frame)) + i);
    }
    queue_can_rx(id, frame, 8U);
  }
}

void setUp(void)
{
  reset_mocks();
  uart_can_bridge_init((CAN_HandleTypeDef *)(uintptr_t)1U,
                       (UART_HandleTypeDef *)(uintptr_t)1U);
}

void tearDown(void)
{
}

static void test_crc16_ccitt_false_known_vector(void)
{
  const uint8_t data[] = "123456789";
  TEST_ASSERT_EQUAL_HEX16(0x29B1U,
                          crc16_ccitt_false(data, sizeof(data) - 1U));
}

static void test_manual_packet_converts_to_two_can_frames(void)
{
  uint8_t packet[19] = {'M', 0x42};
  const uint8_t expected_200[8] = {1, 2, 3, 4, 5, 6, 7, 8};
  const uint8_t expected_201[8] = {9, 10, 11, 12, 13, 14, 0xA5, 0};

  memcpy(&packet[2], expected_200, sizeof(expected_200));
  memcpy(&packet[10], expected_201, 6U);
  packet[16] = 0xA5U;
  finish_packet_crc(packet, sizeof(packet));
  append_uart_rx(packet, sizeof(packet));

  uart_can_bridge_poll();

  TEST_ASSERT_EQUAL_size_t(2U, s_can_tx_count);
  assert_can_frame(0U, 0x200U, 8U, expected_200);
  assert_can_frame(1U, 0x201U, 8U, expected_201);
}

static void test_ik_packet_converts_to_three_can_frames(void)
{
  uint8_t packet[19] = {'I', 9};
  const uint8_t currents[8] = {0x01, 0x10, 0x02, 0x20,
                               0x03, 0x30, 0x04, 0x40};
  const uint8_t angles[6] = {0x11, 0x21, 0x12, 0x22, 0x13, 0x23};
  const uint8_t expected_210[8] = {0x11, 0x21, 0x12, 0x22,
                                   0x13, 0x23, 0, 0};
  const uint8_t expected_211[8] = {0, 0, 0, 0, 0, 0, 0x5A, 0};

  memcpy(&packet[2], currents, sizeof(currents));
  memcpy(&packet[10], angles, sizeof(angles));
  packet[16] = 0x5AU;
  finish_packet_crc(packet, sizeof(packet));
  append_uart_rx(packet, sizeof(packet));

  uart_can_bridge_poll();

  TEST_ASSERT_EQUAL_size_t(3U, s_can_tx_count);
  assert_can_frame(0U, 0x210U, 8U, expected_210);
  assert_can_frame(1U, 0x211U, 8U, expected_211);
  assert_can_frame(2U, 0x212U, 8U, currents);
}

static void test_keyboard_auto_packet_converts_to_three_can_frames(void)
{
  uint8_t packet[15] = {'B', 3};
  const uint8_t angles[6] = {1, 2, 3, 4, 5, 6};
  const uint8_t expected_501[8] = {1, 2, 3, 4, 5, 6, 0xFE, 0xFF};
  const uint8_t expected_502[8] = {0xC3, 0x34, 0x12, 0, 0, 0, 0, 0};
  const uint8_t expected_503[8] = {0xFF, 0, 0xFF, 0, 0xFF, 0, 0xFF, 0};

  memcpy(&packet[2], angles, sizeof(angles));
  packet[8] = 0xC3U;
  packet[9] = 0xFEU;
  packet[10] = 0xFFU;
  packet[11] = 0x34U;
  packet[12] = 0x12U;
  finish_packet_crc(packet, sizeof(packet));
  append_uart_rx(packet, sizeof(packet));

  uart_can_bridge_poll();

  TEST_ASSERT_EQUAL_size_t(3U, s_can_tx_count);
  assert_can_frame(0U, 0x501U, 8U, expected_501);
  assert_can_frame(1U, 0x502U, 8U, expected_502);
  assert_can_frame(2U, 0x503U, 8U, expected_503);
}

static void test_binary_packet_with_bad_crc_is_rejected_and_parser_recovers(void)
{
  uint8_t invalid[19] = {'M', 1};
  uint8_t valid[15] = {'B', 2};

  finish_packet_crc(invalid, sizeof(invalid));
  invalid[5] ^= 0x80U;
  valid[8] = 0x11U;
  finish_packet_crc(valid, sizeof(valid));
  append_uart_rx(invalid, sizeof(invalid));
  append_uart_rx(valid, sizeof(valid));

  uart_can_bridge_poll();

  TEST_ASSERT_EQUAL_size_t(3U, s_can_tx_count);
  TEST_ASSERT_EQUAL_HEX16(0x501U, s_can_tx[0].std_id);
}

static void test_uplink_ascii_accepts_hex_decimal_negative_and_crlf(void)
{
  const uint8_t expected_123[4] = {0, 0, 0, 100};
  const uint8_t expected_124[4] = {0xFF, 0xFF, 0xFF, 0xFE};

  append_uart_text("0x123,100\r\n292,-2\n");
  uart_can_bridge_poll();

  TEST_ASSERT_EQUAL_size_t(2U, s_can_tx_count);
  assert_can_frame(0U, 0x123U, 4U, expected_123);
  assert_can_frame(1U, 0x124U, 4U, expected_124);
}

static void test_uplink_ascii_id_zero_is_accepted_without_can_output(void)
{
  append_uart_text("0,123\n");
  uart_can_bridge_poll();
  TEST_ASSERT_EQUAL_size_t(0U, s_can_tx_count);
}

static void test_malformed_and_overlong_ascii_lines_are_rejected_then_recover(void)
{
  const char *invalid_lines =
    "0x800,1\n1,2147483648\n1,-2147483649\n1,\n1,2x\n";
  const char *overlong =
    "123,0000000000000000000000000000000000000000000000000000001\n";
  const uint8_t expected[4] = {0, 0, 0, 3};

  append_uart_text(invalid_lines);
  append_uart_text(overlong);
  append_uart_text("2,3\n");
  uart_can_bridge_poll();

  TEST_ASSERT_EQUAL_size_t(1U, s_can_tx_count);
  assert_can_frame(0U, 2U, 4U, expected);
}

static void test_mixed_binary_and_ascii_stream_is_parsed_in_order(void)
{
  uint8_t manual[19] = {'M', 1};

  finish_packet_crc(manual, sizeof(manual));
  append_uart_rx(manual, sizeof(manual));
  append_uart_text("0x321,7\r");
  uart_can_bridge_poll();

  TEST_ASSERT_EQUAL_size_t(3U, s_can_tx_count);
  TEST_ASSERT_EQUAL_HEX16(0x200U, s_can_tx[0].std_id);
  TEST_ASSERT_EQUAL_HEX16(0x201U, s_can_tx[1].std_id);
  TEST_ASSERT_EQUAL_HEX16(0x321U, s_can_tx[2].std_id);
}

static void test_jf_packet_requires_both_frames_and_repeats_at_50_hz(void)
{
  const uint8_t frame_203[8] = {0x34, 0x12, 0x78, 0x56, 0xA5, 0, 0, 0};
  const uint8_t expected_payload[12] = {
    0, 0xA5, 0x34, 0x12, 0x78, 0x56,
    0xBC, 0x9A, 0xF0, 0xDE, 0x57, 0x13
  };

  queue_can_rx(0x203U, frame_203, 8U);
  s_mock_tick = 20U;
  uart_can_bridge_poll();
  TEST_ASSERT_EQUAL_size_t(0U, s_uart_tx_length);

  queue_jf_source_frames();
  uart_can_bridge_poll();
  TEST_ASSERT_EQUAL_size_t(16U, s_uart_tx_length);
  TEST_ASSERT_EQUAL_HEX8('J', s_uart_tx[0]);
  TEST_ASSERT_EQUAL_HEX8('F', s_uart_tx[1]);
  TEST_ASSERT_EQUAL_HEX8_ARRAY(expected_payload, &s_uart_tx[2], sizeof(expected_payload));
  assert_uart_crc(0U, 16U);

  s_mock_tick = 39U;
  uart_can_bridge_poll();
  TEST_ASSERT_EQUAL_size_t(16U, s_uart_tx_length);
  s_mock_tick = 40U;
  uart_can_bridge_poll();
  TEST_ASSERT_EQUAL_size_t(32U, s_uart_tx_length);
  TEST_ASSERT_EQUAL_UINT8(1U, s_uart_tx[18]);
  assert_uart_crc(16U, 16U);
}

static void test_jf_short_dlc_uses_zero_padded_missing_bytes(void)
{
  const uint8_t frame_203[4] = {1, 2, 3, 4};
  const uint8_t frame_204[2] = {5, 6};

  queue_can_rx(0x203U, frame_203, sizeof(frame_203));
  queue_can_rx(0x204U, frame_204, sizeof(frame_204));
  s_mock_tick = 20U;
  uart_can_bridge_poll();

  TEST_ASSERT_EQUAL_size_t(16U, s_uart_tx_length);
  TEST_ASSERT_EQUAL_UINT8(0U, s_uart_tx[3]);
  TEST_ASSERT_EQUAL_HEX8_ARRAY(frame_203, &s_uart_tx[4], sizeof(frame_203));
  TEST_ASSERT_EQUAL_HEX8_ARRAY(frame_204, &s_uart_tx[8], sizeof(frame_204));
  TEST_ASSERT_EACH_EQUAL_HEX8(0U, &s_uart_tx[10], 4U);
  assert_uart_crc(0U, 16U);
}

static void test_uf_packet_joins_four_frames_and_uses_first_nul_for_length(void)
{
  uint8_t frame[8];

  for (uint16_t id = 0x205U; id <= 0x208U; ++id)
  {
    for (uint8_t i = 0U; i < sizeof(frame); ++i)
    {
      frame[i] = (uint8_t)(1U + ((id - 0x205U) * 8U) + i);
    }
    if (id == 0x206U)
    {
      frame[3] = 0U;
    }
    queue_can_rx(id, frame, 8U);
  }
  uart_can_bridge_poll();

  TEST_ASSERT_EQUAL_size_t(40U, s_uart_tx_length);
  TEST_ASSERT_EQUAL_HEX8('U', s_uart_tx[0]);
  TEST_ASSERT_EQUAL_HEX8('F', s_uart_tx[1]);
  TEST_ASSERT_EQUAL_UINT8(0U, s_uart_tx[2]);
  TEST_ASSERT_EQUAL_HEX8(0x13U, s_uart_tx[3]);
  TEST_ASSERT_EQUAL_UINT8(0U, s_uart_tx[4]);
  TEST_ASSERT_EQUAL_UINT8(11U, s_uart_tx[5]);
  TEST_ASSERT_EQUAL_UINT8(0U, s_uart_tx[6U + 11U]);
  assert_uart_crc(0U, 40U);
}

static void test_uf_full_payload_has_length_32(void)
{
  queue_complete_uf(1U);
  uart_can_bridge_poll();
  TEST_ASSERT_EQUAL_size_t(40U, s_uart_tx_length);
  TEST_ASSERT_EQUAL_UINT8(32U, s_uart_tx[5]);
  assert_uart_crc(0U, 40U);
}

static void test_uf_out_of_order_sequence_is_discarded(void)
{
  const uint8_t data[8] = {1, 2, 3, 4, 5, 6, 7, 8};

  queue_can_rx(0x205U, data, 8U);
  queue_can_rx(0x207U, data, 8U);
  queue_can_rx(0x208U, data, 8U);
  uart_can_bridge_poll();
  TEST_ASSERT_EQUAL_size_t(0U, s_uart_tx_length);
}

static void test_uf_dlc_less_than_eight_is_discarded(void)
{
  uint8_t data[8] = {1, 2, 3, 4, 5, 6, 7, 8};

  queue_can_rx(0x205U, data, 8U);
  queue_can_rx(0x206U, data, 7U);
  queue_can_rx(0x207U, data, 8U);
  queue_can_rx(0x208U, data, 8U);
  uart_can_bridge_poll();
  TEST_ASSERT_EQUAL_size_t(0U, s_uart_tx_length);
}

static void test_uf_inter_frame_timeout_discards_assembly(void)
{
  const uint8_t data[8] = {1, 2, 3, 4, 5, 6, 7, 8};

  queue_can_rx(0x205U, data, 8U);
  uart_can_bridge_poll();
  s_mock_tick = 251U;
  queue_can_rx(0x206U, data, 8U);
  queue_can_rx(0x207U, data, 8U);
  queue_can_rx(0x208U, data, 8U);
  uart_can_bridge_poll();
  TEST_ASSERT_EQUAL_size_t(0U, s_uart_tx_length);
}

static void test_jf_and_uf_share_sequence_counter(void)
{
  queue_jf_source_frames();
  s_mock_tick = 20U;
  uart_can_bridge_poll();
  queue_complete_uf(1U);
  uart_can_bridge_poll();

  TEST_ASSERT_EQUAL_size_t(56U, s_uart_tx_length);
  TEST_ASSERT_EQUAL_UINT8(0U, s_uart_tx[2]);
  TEST_ASSERT_EQUAL_UINT8(1U, s_uart_tx[16U + 2U]);
}

static void test_generic_can_values_emit_sum_and_signed_big_endian_ascii(void)
{
  const uint8_t sum_data[8] = {1, 2, 3, 4, 5, 6, 7, 8};
  const uint8_t signed_data[4] = {0xFF, 0xFF, 0xFF, 0x9C};
  const char expected[] = "0x300,36\r\n0x410,-100\r\n";

  queue_can_rx(0x300U, sum_data, 8U);
  queue_can_rx(0x410U, signed_data, 4U);
  uart_can_bridge_poll();
  s_mock_tick = 100U;
  uart_can_bridge_poll();

  TEST_ASSERT_EQUAL_size_t(sizeof(expected) - 1U, s_uart_tx_length);
  TEST_ASSERT_EQUAL_CHAR_ARRAY(expected, s_uart_tx, sizeof(expected) - 1U);
}

static void test_410_with_short_dlc_falls_back_to_eight_byte_sum(void)
{
  const uint8_t data[3] = {1, 2, 3};
  const char expected[] = "0x410,6\r\n";

  queue_can_rx(0x410U, data, sizeof(data));
  uart_can_bridge_poll();
  s_mock_tick = 100U;
  uart_can_bridge_poll();
  TEST_ASSERT_EQUAL_CHAR_ARRAY(expected, s_uart_tx, sizeof(expected) - 1U);
}

static void test_latest_value_table_updates_existing_id(void)
{
  const uint8_t first[1] = {1};
  const uint8_t second[1] = {9};
  const char expected[] = "0x300,9\r\n";

  queue_can_rx(0x300U, first, sizeof(first));
  queue_can_rx(0x300U, second, sizeof(second));
  uart_can_bridge_poll();
  s_mock_tick = 100U;
  uart_can_bridge_poll();
  TEST_ASSERT_EQUAL_CHAR_ARRAY(expected, s_uart_tx, sizeof(expected) - 1U);
}

static void test_latest_value_table_replaces_oldest_slot_after_16_ids(void)
{
  const uint8_t data[1] = {1};

  for (uint16_t id = 0x300U; id <= 0x310U; ++id)
  {
    queue_can_rx(id, data, sizeof(data));
  }
  uart_can_bridge_poll();
  s_mock_tick = 100U;
  uart_can_bridge_poll();

  TEST_ASSERT_NULL(strstr((const char *)s_uart_tx, "0x300,1\r\n"));
  TEST_ASSERT_NOT_NULL(strstr((const char *)s_uart_tx, "0x301,1\r\n"));
  TEST_ASSERT_NOT_NULL(strstr((const char *)s_uart_tx, "0x310,1\r\n"));
}

static void test_uf_priority_suppresses_and_discards_normal_output_for_250_ms(void)
{
  const uint8_t generic[1] = {7};
  const uint8_t uf_frame[8] = {1, 2, 3, 4, 5, 6, 7, 8};

  queue_jf_source_frames();
  queue_can_rx(0x300U, generic, sizeof(generic));
  uart_can_bridge_poll();

  queue_can_rx(0x205U, uf_frame, 8U);
  s_mock_tick = 20U;
  uart_can_bridge_poll();
  TEST_ASSERT_EQUAL_UINT32(1U, s_uart_discard_count);
  TEST_ASSERT_EQUAL_size_t(0U, s_uart_tx_length);

  s_mock_tick = 269U;
  uart_can_bridge_poll();
  TEST_ASSERT_EQUAL_size_t(0U, s_uart_tx_length);
  s_mock_tick = 270U;
  uart_can_bridge_poll();
  TEST_ASSERT_EQUAL_size_t(0U, s_uart_tx_length);
  s_mock_tick = 290U;
  uart_can_bridge_poll();
  TEST_ASSERT_EQUAL_size_t(16U, s_uart_tx_length);
  s_mock_tick = 370U;
  uart_can_bridge_poll();
  TEST_ASSERT_TRUE(uart_tx_contains_text("0x300,7\r\n"));
}

static void test_periodic_timers_work_across_tick_wraparound(void)
{
  reset_mocks();
  s_mock_tick = UINT32_MAX - 9U;
  uart_can_bridge_init((CAN_HandleTypeDef *)(uintptr_t)1U,
                       (UART_HandleTypeDef *)(uintptr_t)1U);
  queue_jf_source_frames();
  uart_can_bridge_poll();

  s_mock_tick = 10U;
  uart_can_bridge_poll();
  TEST_ASSERT_EQUAL_size_t(16U, s_uart_tx_length);
}

int main(int argc, char **argv)
{
  (void)argc;
  (void)argv;

  UNITY_BEGIN();
  RUN_TEST(test_crc16_ccitt_false_known_vector);
  RUN_TEST(test_manual_packet_converts_to_two_can_frames);
  RUN_TEST(test_ik_packet_converts_to_three_can_frames);
  RUN_TEST(test_keyboard_auto_packet_converts_to_three_can_frames);
  RUN_TEST(test_binary_packet_with_bad_crc_is_rejected_and_parser_recovers);
  RUN_TEST(test_uplink_ascii_accepts_hex_decimal_negative_and_crlf);
  RUN_TEST(test_uplink_ascii_id_zero_is_accepted_without_can_output);
  RUN_TEST(test_malformed_and_overlong_ascii_lines_are_rejected_then_recover);
  RUN_TEST(test_mixed_binary_and_ascii_stream_is_parsed_in_order);
  RUN_TEST(test_jf_packet_requires_both_frames_and_repeats_at_50_hz);
  RUN_TEST(test_jf_short_dlc_uses_zero_padded_missing_bytes);
  RUN_TEST(test_uf_packet_joins_four_frames_and_uses_first_nul_for_length);
  RUN_TEST(test_uf_full_payload_has_length_32);
  RUN_TEST(test_uf_out_of_order_sequence_is_discarded);
  RUN_TEST(test_uf_dlc_less_than_eight_is_discarded);
  RUN_TEST(test_uf_inter_frame_timeout_discards_assembly);
  RUN_TEST(test_jf_and_uf_share_sequence_counter);
  RUN_TEST(test_generic_can_values_emit_sum_and_signed_big_endian_ascii);
  RUN_TEST(test_410_with_short_dlc_falls_back_to_eight_byte_sum);
  RUN_TEST(test_latest_value_table_updates_existing_id);
  RUN_TEST(test_latest_value_table_replaces_oldest_slot_after_16_ids);
  RUN_TEST(test_uf_priority_suppresses_and_discards_normal_output_for_250_ms);
  RUN_TEST(test_periodic_timers_work_across_tick_wraparound);
  return UNITY_END();
}
