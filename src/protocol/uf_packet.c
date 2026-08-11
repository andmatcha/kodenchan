/*
 * 責務: UF text packet を wire format へ little endian で encode する。
 * 依存関係: services から USB feedback text chunk を受け、UART 送信用の binary packet を返す。
 */

#include "protocol/uf_packet.h"

#include "protocol/crc16_ccitt.h"

#include <string.h>

static void write_u16_le(uint8_t *data, uint16_t value)
{
  data[0] = (uint8_t)(value & 0xFFU);
  data[1] = (uint8_t)((value >> 8) & 0xFFU);
}

void uf_packet_encode(uint8_t seq,
                      uint8_t flags,
                      uint8_t chunk_index,
                      uint8_t payload_len,
                      const uint8_t payload[UF_PACKET_PAYLOAD_LEN],
                      uint8_t raw_packet[UF_PACKET_LEN])
{
  uint16_t crc = 0U;

  if (raw_packet == 0)
  {
    return;
  }

  if (payload_len > UF_PACKET_PAYLOAD_LEN)
  {
    payload_len = UF_PACKET_PAYLOAD_LEN;
  }

  raw_packet[0] = 'U';
  raw_packet[1] = 'F';
  raw_packet[2] = seq;
  raw_packet[3] = flags & 0x1FU;
  raw_packet[4] = chunk_index;
  raw_packet[5] = payload_len;
  memset(raw_packet + 6U, 0, UF_PACKET_PAYLOAD_LEN);

  if ((payload != 0) && (payload_len > 0U))
  {
    memcpy(raw_packet + 6U, payload, payload_len);
  }

  crc = crc16_ccitt_false(raw_packet, UF_PACKET_CRC_OFFSET);
  write_u16_le(raw_packet + UF_PACKET_CRC_OFFSET, crc);
}
