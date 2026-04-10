#include "protocol/ac_stream_parser.h"

#include <string.h>

static void shift_buffer(AcStreamParser *parser, uint32_t count)
{
  if ((parser == 0) || (count == 0U))
  {
    return;
  }

  if (count >= parser->length)
  {
    parser->length = 0U;
    return;
  }

  memmove(parser->buffer, parser->buffer + count, parser->length - count);
  parser->length -= count;
}

static bool align_to_header(AcStreamParser *parser)
{
  if (parser == 0)
  {
    return false;
  }

  for (uint32_t i = 0U; i + 1U < parser->length; ++i)
  {
    if ((parser->buffer[i] == 'A') && (parser->buffer[i + 1U] == 'C'))
    {
      shift_buffer(parser, i);
      return true;
    }
  }

  if (parser->length > 0U)
  {
    uint8_t tail = parser->buffer[parser->length - 1U];
    parser->buffer[0] = tail;
    parser->length = (tail == 'A') ? 1U : 0U;
  }

  return false;
}

void ac_stream_parser_init(AcStreamParser *parser)
{
  if (parser == 0)
  {
    return;
  }

  parser->length = 0U;
}

void ac_stream_parser_push(AcStreamParser *parser, const uint8_t *data, uint16_t length)
{
  if ((parser == 0) || (data == 0) || (length == 0U))
  {
    return;
  }

  for (uint16_t i = 0U; i < length; ++i)
  {
    if (parser->length >= AC_STREAM_PARSER_BUFFER_LEN)
    {
      shift_buffer(parser, 1U);
    }

    parser->buffer[parser->length++] = data[i];
  }
}

bool ac_stream_parser_next(AcStreamParser *parser, AcPacketV6 *packet)
{
  if ((parser == 0) || (packet == 0))
  {
    return false;
  }

  while (parser->length >= 2U)
  {
    if (!align_to_header(parser))
    {
      return false;
    }

    if (parser->length < AC_PACKET_V6_LEN)
    {
      return false;
    }

    if (ac_packet_v6_decode(parser->buffer, packet))
    {
      shift_buffer(parser, AC_PACKET_V6_LEN);
      return true;
    }

    shift_buffer(parser, 1U);
  }

  return false;
}
