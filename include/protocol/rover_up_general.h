#ifndef KODENCHAN_ROVER_UP_GENERAL_H
#define KODENCHAN_ROVER_UP_GENERAL_H

#include <stdbool.h>
#include <stdint.h>

#define ROVER_UP_GENERAL_MAX_CONTENT_LEN 17U
#define ROVER_UP_GENERAL_CAN_PAYLOAD_LEN 4U

typedef struct
{
  uint16_t std_id;
  uint32_t value;
} RoverUpGeneralFrame;

bool rover_up_general_decode(const uint8_t *line, uint16_t length, RoverUpGeneralFrame *frame);
void rover_up_general_pack_payload(uint32_t value, uint8_t payload[ROVER_UP_GENERAL_CAN_PAYLOAD_LEN]);

#endif /* KODENCHAN_ROVER_UP_GENERAL_H */
