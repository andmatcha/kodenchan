#ifndef KODENCHAN_AC_STREAM_PARSER_H
#define KODENCHAN_AC_STREAM_PARSER_H

#include "protocol/ac_packet_v6.h"

#include <stdbool.h>
#include <stdint.h>

#define AC_STREAM_PARSER_BUFFER_LEN 256U

typedef struct
{
  uint8_t buffer[AC_STREAM_PARSER_BUFFER_LEN];
  uint32_t length;
} AcStreamParser;

void ac_stream_parser_init(AcStreamParser *parser);
void ac_stream_parser_push(AcStreamParser *parser, const uint8_t *data, uint16_t length);
bool ac_stream_parser_next(AcStreamParser *parser, AcPacketV6 *packet);

#endif /* KODENCHAN_AC_STREAM_PARSER_H */
