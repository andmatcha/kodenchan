#ifndef KODENCHAN_AC_PACKET_V6_H
#define KODENCHAN_AC_PACKET_V6_H

#include <stdbool.h>
#include <stdint.h>

#define AC_PACKET_V6_LEN 39U
#define AC_PACKET_V6_CRC_OFFSET 37U
#define AC_PACKET_MODE_MANUAL 1U

typedef struct
{
  uint8_t seq;
  uint8_t flags;
  uint16_t current[7];
  uint16_t angle[3];
  int16_t vel[3];
  uint8_t control_byte;
  int16_t base_rel_mm_j0;
  uint16_t auto_flags;
  uint16_t fault_code;
  uint16_t crc16;
} AcPacketV6;

bool ac_packet_v6_has_header(const uint8_t *raw_packet);
bool ac_packet_v6_has_valid_crc(const uint8_t *raw_packet);
bool ac_packet_v6_decode(const uint8_t *raw_packet, AcPacketV6 *packet);
uint8_t ac_packet_v6_mode(const AcPacketV6 *packet);

#endif /* KODENCHAN_AC_PACKET_V6_H */
