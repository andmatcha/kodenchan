#ifndef KODENCHAN_UF_PACKET_H
#define KODENCHAN_UF_PACKET_H

#include <stdint.h>

#define UF_PACKET_LEN 14U
#define UF_PACKET_CRC_OFFSET 12U
#define UF_PACKET_FLAG_VALID 0x01U
#define UF_PACKET_FLAG_USB_PRESENT 0x02U
#define UF_PACKET_FLAG_READ_BUSY 0x04U
#define UF_PACKET_FLAG_READ_ERROR 0x08U

void uf_packet_encode(uint8_t seq, uint8_t flags, int32_t lat_e7, int32_t lon_e7, uint8_t raw_packet[UF_PACKET_LEN]);

#endif /* KODENCHAN_UF_PACKET_H */
