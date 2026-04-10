/*
 * 責務: PacketACv6 の header / CRC 検証と little endian field decode を行う。
 * 依存関係: protocol/ac_stream_parser から raw packet を受け、services/uart_packet_to_can_service へ mode 判定可能な AcPacketV6 を返す。
 */

#include "protocol/ac_packet_v6.h"

#include "protocol/crc16_ccitt.h"

static uint16_t read_u16_le(const uint8_t *data)
{
  return (uint16_t)data[0] | ((uint16_t)data[1] << 8);
}

static int16_t read_i16_le(const uint8_t *data)
{
  return (int16_t)read_u16_le(data);
}

bool ac_packet_v6_has_header(const uint8_t *raw_packet)
{
  return (raw_packet != 0) && (raw_packet[0] == 'A') && (raw_packet[1] == 'C');
}

bool ac_packet_v6_has_valid_crc(const uint8_t *raw_packet)
{
  uint16_t expected_crc = 0U;
  uint16_t actual_crc = 0U;

  if (raw_packet == 0)
  {
    return false;
  }

  expected_crc = read_u16_le(raw_packet + AC_PACKET_V6_CRC_OFFSET);
  actual_crc = crc16_ccitt_false(raw_packet, AC_PACKET_V6_CRC_OFFSET);

  return expected_crc == actual_crc;
}

bool ac_packet_v6_decode(const uint8_t *raw_packet, AcPacketV6 *packet)
{
  if ((raw_packet == 0) || (packet == 0))
  {
    return false;
  }

  if (!ac_packet_v6_has_header(raw_packet) || !ac_packet_v6_has_valid_crc(raw_packet))
  {
    return false;
  }

  packet->seq = raw_packet[2];
  packet->flags = raw_packet[3];

  for (uint32_t i = 0U; i < 7U; ++i)
  {
    packet->current[i] = read_u16_le(raw_packet + 4U + (i * 2U));
  }

  for (uint32_t i = 0U; i < 3U; ++i)
  {
    packet->angle[i] = read_u16_le(raw_packet + 18U + (i * 2U));
    packet->vel[i] = read_i16_le(raw_packet + 24U + (i * 2U));
  }

  packet->control_byte = raw_packet[30];
  packet->base_rel_mm_j0 = read_i16_le(raw_packet + 31U);
  packet->auto_flags = read_u16_le(raw_packet + 33U);
  packet->fault_code = read_u16_le(raw_packet + 35U);
  packet->crc16 = read_u16_le(raw_packet + 37U);

  return true;
}

uint8_t ac_packet_v6_mode(const AcPacketV6 *packet)
{
  if (packet == 0)
  {
    return 0xFFU;
  }

  return (packet->flags >> 4) & 0x03U;
}
