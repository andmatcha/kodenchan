#ifndef KODENCHAN_UF_PACKET_H
#define KODENCHAN_UF_PACKET_H

#include <stdint.h>

#define UF_PACKET_LEN 40U
#define UF_PACKET_CRC_OFFSET 38U
#define UF_PACKET_PAYLOAD_LEN 32U
#define UF_PACKET_FLAG_VALID 0x01U
#define UF_PACKET_FLAG_USB_PRESENT 0x02U
#define UF_PACKET_FLAG_READ_BUSY 0x04U
#define UF_PACKET_FLAG_READ_ERROR 0x08U
#define UF_PACKET_FLAG_END 0x10U

void uf_packet_encode(uint8_t seq,
                      uint8_t flags,
                      uint8_t chunk_index,
                      uint8_t payload_len,
                      const uint8_t payload[UF_PACKET_PAYLOAD_LEN],
                      uint8_t raw_packet[UF_PACKET_LEN]);

#endif /* KODENCHAN_UF_PACKET_H */
