/*
 * 責務: UF packet を wire format へ little endian で encode する。
 * 依存関係: services から USB feedback 座標を受け、UART 送信用の binary packet を返す。
 */

#include "protocol/uf_packet.h"

#include "protocol/crc16_ccitt.h"

static void write_u16_le(uint8_t *data, uint16_t value)
{
  data[0] = (uint8_t)(value & 0xFFU);
  data[1] = (uint8_t)((value >> 8) & 0xFFU);
}

static void write_i32_le(uint8_t *data, int32_t value)
{
  uint32_t raw_value = (uint32_t)value;

  data[0] = (uint8_t)(raw_value & 0xFFU);
  data[1] = (uint8_t)((raw_value >> 8) & 0xFFU);
  data[2] = (uint8_t)((raw_value >> 16) & 0xFFU);
  data[3] = (uint8_t)((raw_value >> 24) & 0xFFU);
}

void uf_packet_encode(uint8_t seq, uint8_t flags, int32_t lat_e7, int32_t lon_e7, uint8_t raw_packet[UF_PACKET_LEN])
{
  uint16_t crc = 0U;

  if (raw_packet == 0)
  {
    return;
  }

  raw_packet[0] = 'U';
  raw_packet[1] = 'F';
  raw_packet[2] = seq;
  raw_packet[3] = flags & 0x0FU;
  write_i32_le(raw_packet + 4U, lat_e7);
  write_i32_le(raw_packet + 8U, lon_e7);

  crc = crc16_ccitt_false(raw_packet, UF_PACKET_CRC_OFFSET);
  write_u16_le(raw_packet + UF_PACKET_CRC_OFFSET, crc);
}
