#ifndef KODENCHAN_UPLINK_STREAM_PARSER_H
#define KODENCHAN_UPLINK_STREAM_PARSER_H

#include "protocol/ac_packet_v6.h"
#include "protocol/rover_up_general.h"

#include <stdbool.h>
#include <stdint.h>

#define UPLINK_STREAM_PARSER_BUFFER_LEN 256U

typedef enum
{
  UPLINK_FRAME_AC_PACKET_V6,
  UPLINK_FRAME_ROVER_UP_GENERAL
} UplinkFrameType;

typedef struct
{
  UplinkFrameType type;
  union
  {
    AcPacketV6 ac_packet_v6;
    RoverUpGeneralFrame rover_up_general;
  } data;
} UplinkFrame;

typedef struct
{
  uint8_t buffer[UPLINK_STREAM_PARSER_BUFFER_LEN];
  uint16_t length;
} UplinkStreamParser;

void uplink_stream_parser_init(UplinkStreamParser *parser);
void uplink_stream_parser_push(UplinkStreamParser *parser, const uint8_t *data, uint16_t length);
bool uplink_stream_parser_next(UplinkStreamParser *parser, UplinkFrame *frame);

#endif /* KODENCHAN_UPLINK_STREAM_PARSER_H */
