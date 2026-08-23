#include "protocol/ac_packet_v6.h"
#include "protocol/crc16_ccitt.h"
#include "protocol/rover_up_general.h"
#include "protocol/uplink_stream_parser.h"

#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define ARRAY_LEN(array) (sizeof(array) / sizeof((array)[0]))

static void assert_rover_decode(const char *line, uint16_t expected_id, uint32_t expected_value)
{
  RoverUpGeneralFrame frame;

  assert(rover_up_general_decode((const uint8_t *)line, (uint16_t)strlen(line), &frame));
  assert(frame.std_id == expected_id);
  assert(frame.value == expected_value);
}

static void assert_rover_rejected(const char *line)
{
  RoverUpGeneralFrame frame;

  assert(!rover_up_general_decode((const uint8_t *)line, (uint16_t)strlen(line), &frame));
}

static void build_ac_packet(uint8_t sequence, uint8_t raw_packet[AC_PACKET_V6_LEN])
{
  uint16_t crc = 0U;

  memset(raw_packet, 0, AC_PACKET_V6_LEN);
  raw_packet[0] = 'A';
  raw_packet[1] = 'C';
  raw_packet[2] = sequence;
  raw_packet[3] = 0x10U;
  crc = crc16_ccitt_false(raw_packet, AC_PACKET_V6_CRC_OFFSET);
  raw_packet[AC_PACKET_V6_CRC_OFFSET] = (uint8_t)(crc & 0xFFU);
  raw_packet[AC_PACKET_V6_CRC_OFFSET + 1U] = (uint8_t)(crc >> 8U);
}

static void test_rover_decoder(void)
{
  uint8_t payload[ROVER_UP_GENERAL_CAN_PAYLOAD_LEN] = {0};

  assert_rover_decode("0x123,123456", 0x123U, 123456U);
  assert_rover_decode("0x300,1234", 0x300U, 1234U);
  assert_rover_decode("0X7fF,+2147483647", 0x7FFU, 0x7FFFFFFFU);
  assert_rover_decode("0x312,-140", 0x312U, 0xFFFFFF74U);
  assert_rover_decode("0x001,-2147483648", 0x001U, 0x80000000U);

  rover_up_general_pack_payload(123456U, payload);
  assert(payload[0] == 0x00U);
  assert(payload[1] == 0x01U);
  assert(payload[2] == 0xE2U);
  assert(payload[3] == 0x40U);

  assert_rover_rejected("0x000,1");
  assert_rover_rejected("0x800,1");
  assert_rover_rejected("0x12,1");
  assert_rover_rejected("0x123,");
  assert_rover_rejected("0x123,1,2");
  assert_rover_rejected("0x123, 1");
  assert_rover_rejected("0x123,2147483648");
  assert_rover_rejected("0x123,-2147483649");
}

static void test_split_rover_line(void)
{
  UplinkStreamParser parser;
  UplinkFrame frame;
  const uint8_t first[] = "noise0x123,123";
  const uint8_t second[] = "456\n";

  uplink_stream_parser_init(&parser);
  uplink_stream_parser_push(&parser, first, (uint16_t)(ARRAY_LEN(first) - 1U));
  assert(!uplink_stream_parser_next(&parser, &frame));

  uplink_stream_parser_push(&parser, second, (uint16_t)(ARRAY_LEN(second) - 1U));
  assert(uplink_stream_parser_next(&parser, &frame));
  assert(frame.type == UPLINK_FRAME_ROVER_UP_GENERAL);
  assert(frame.data.rover_up_general.std_id == 0x123U);
  assert(frame.data.rover_up_general.value == 123456U);
  assert(!uplink_stream_parser_next(&parser, &frame));
}

static void test_split_ac_packet(void)
{
  UplinkStreamParser parser;
  UplinkFrame frame;
  uint8_t raw_packet[AC_PACKET_V6_LEN];

  build_ac_packet(23U, raw_packet);
  uplink_stream_parser_init(&parser);
  uplink_stream_parser_push(&parser, raw_packet, 11U);
  assert(!uplink_stream_parser_next(&parser, &frame));

  uplink_stream_parser_push(&parser, raw_packet + 11U, AC_PACKET_V6_LEN - 11U);
  assert(uplink_stream_parser_next(&parser, &frame));
  assert(frame.type == UPLINK_FRAME_AC_PACKET_V6);
  assert(frame.data.ac_packet_v6.seq == 23U);
  assert(!uplink_stream_parser_next(&parser, &frame));
}

static void test_valid_ac_packet_hides_rover_like_payload(void)
{
  UplinkStreamParser parser;
  UplinkFrame frame;
  uint8_t raw_packet[AC_PACKET_V6_LEN];
  const uint8_t rover_like_bytes[] = "0x123,1\r\n";
  uint16_t crc = 0U;

  build_ac_packet(24U, raw_packet);
  memcpy(raw_packet + 4U, rover_like_bytes, sizeof(rover_like_bytes) - 1U);
  crc = crc16_ccitt_false(raw_packet, AC_PACKET_V6_CRC_OFFSET);
  raw_packet[AC_PACKET_V6_CRC_OFFSET] = (uint8_t)(crc & 0xFFU);
  raw_packet[AC_PACKET_V6_CRC_OFFSET + 1U] = (uint8_t)(crc >> 8U);

  uplink_stream_parser_init(&parser);
  uplink_stream_parser_push(&parser, raw_packet, AC_PACKET_V6_LEN);
  assert(uplink_stream_parser_next(&parser, &frame));
  assert(frame.type == UPLINK_FRAME_AC_PACKET_V6);
  assert(frame.data.ac_packet_v6.seq == 24U);
  assert(!uplink_stream_parser_next(&parser, &frame));
}

static void test_bad_ac_packet_resynchronizes_to_rover(void)
{
  UplinkStreamParser parser;
  UplinkFrame frame;
  uint8_t raw_packet[AC_PACKET_V6_LEN];
  const uint8_t rover_line[] = "0x120,1\n";
  uint8_t stream[AC_PACKET_V6_LEN + sizeof(rover_line) - 1U];

  build_ac_packet(25U, raw_packet);
  raw_packet[10] ^= 0x01U;
  memcpy(stream, raw_packet, AC_PACKET_V6_LEN);
  memcpy(stream + AC_PACKET_V6_LEN, rover_line, sizeof(rover_line) - 1U);

  uplink_stream_parser_init(&parser);
  uplink_stream_parser_push(&parser, stream, (uint16_t)sizeof(stream));
  assert(uplink_stream_parser_next(&parser, &frame));
  assert(frame.type == UPLINK_FRAME_ROVER_UP_GENERAL);
  assert(frame.data.rover_up_general.std_id == 0x120U);
  assert(frame.data.rover_up_general.value == 1U);
  assert(!uplink_stream_parser_next(&parser, &frame));
}

static void test_mixed_stream_preserves_frame_order(void)
{
  UplinkStreamParser parser;
  UplinkFrame frame;
  uint8_t first_ac[AC_PACKET_V6_LEN];
  uint8_t second_ac[AC_PACKET_V6_LEN];
  const uint8_t invalid_line[] = "0x123,12x\r\n";
  const uint8_t rover_line[] = "0x300,1234\r\n";
  uint8_t stream[sizeof(invalid_line) - 1U + AC_PACKET_V6_LEN +
                 sizeof(rover_line) - 1U + AC_PACKET_V6_LEN];
  uint16_t offset = 0U;

  build_ac_packet(17U, first_ac);
  build_ac_packet(18U, second_ac);
  memcpy(stream + offset, invalid_line, sizeof(invalid_line) - 1U);
  offset = (uint16_t)(offset + sizeof(invalid_line) - 1U);
  memcpy(stream + offset, first_ac, AC_PACKET_V6_LEN);
  offset = (uint16_t)(offset + AC_PACKET_V6_LEN);
  memcpy(stream + offset, rover_line, sizeof(rover_line) - 1U);
  offset = (uint16_t)(offset + sizeof(rover_line) - 1U);
  memcpy(stream + offset, second_ac, AC_PACKET_V6_LEN);
  offset = (uint16_t)(offset + AC_PACKET_V6_LEN);

  uplink_stream_parser_init(&parser);
  uplink_stream_parser_push(&parser, stream, offset);

  assert(uplink_stream_parser_next(&parser, &frame));
  assert(frame.type == UPLINK_FRAME_AC_PACKET_V6);
  assert(frame.data.ac_packet_v6.seq == 17U);

  assert(uplink_stream_parser_next(&parser, &frame));
  assert(frame.type == UPLINK_FRAME_ROVER_UP_GENERAL);
  assert(frame.data.rover_up_general.std_id == 0x300U);
  assert(frame.data.rover_up_general.value == 1234U);

  assert(uplink_stream_parser_next(&parser, &frame));
  assert(frame.type == UPLINK_FRAME_AC_PACKET_V6);
  assert(frame.data.ac_packet_v6.seq == 18U);
  assert(!uplink_stream_parser_next(&parser, &frame));
}

static void test_oversized_line_resynchronizes(void)
{
  UplinkStreamParser parser;
  UplinkFrame frame;
  const uint8_t stream[] = "0x123,12345678901234567890\nnoise0x121,-1\n";

  uplink_stream_parser_init(&parser);
  uplink_stream_parser_push(&parser, stream, (uint16_t)(ARRAY_LEN(stream) - 1U));

  assert(uplink_stream_parser_next(&parser, &frame));
  assert(frame.type == UPLINK_FRAME_ROVER_UP_GENERAL);
  assert(frame.data.rover_up_general.std_id == 0x121U);
  assert(frame.data.rover_up_general.value == 0xFFFFFFFFU);
  assert(!uplink_stream_parser_next(&parser, &frame));
}

int main(void)
{
  test_rover_decoder();
  test_split_rover_line();
  test_split_ac_packet();
  test_valid_ac_packet_hides_rover_like_payload();
  test_bad_ac_packet_resynchronizes_to_rover();
  test_mixed_stream_preserves_frame_order();
  test_oversized_line_resynchronizes();
  puts("uplink protocol tests passed");
  return 0;
}
