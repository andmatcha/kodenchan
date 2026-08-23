/*
 * 責務: USART byte stream から PacketACv6 と RoverUpGeneral を順序どおり抽出する。
 * 依存関係: protocol/ac_packet_v6 と protocol/rover_up_general で候補frameを検証する。
 */

#include "protocol/uplink_stream_parser.h"

#include <stddef.h>
#include <string.h>

typedef enum
{
  UPLINK_PREFIX_NONE,
  UPLINK_PREFIX_PARTIAL,
  UPLINK_PREFIX_AC_PACKET_V6,
  UPLINK_PREFIX_ROVER_UP_GENERAL
} UplinkPrefixType;

typedef struct
{
  UplinkPrefixType type;
  uint16_t offset;
} UplinkPrefix;

static void shift_buffer(UplinkStreamParser *parser, uint16_t count)
{
  if ((parser == NULL) || (count == 0U))
  {
    return;
  }

  if (count >= parser->length)
  {
    parser->length = 0U;
    return;
  }

  memmove(parser->buffer, parser->buffer + count, parser->length - count);
  parser->length = (uint16_t)(parser->length - count);
}

static UplinkPrefix find_next_prefix(const UplinkStreamParser *parser)
{
  UplinkPrefix result = {UPLINK_PREFIX_NONE, 0U};

  for (uint16_t i = 0U; i < parser->length; ++i)
  {
    uint8_t first = parser->buffer[i];

    if ((first != 'A') && (first != '0'))
    {
      continue;
    }

    if ((i + 1U) >= parser->length)
    {
      result.type = UPLINK_PREFIX_PARTIAL;
      result.offset = i;
      return result;
    }

    if ((first == 'A') && (parser->buffer[i + 1U] == 'C'))
    {
      result.type = UPLINK_PREFIX_AC_PACKET_V6;
      result.offset = i;
      return result;
    }

    if ((first == '0') &&
        ((parser->buffer[i + 1U] == 'x') || (parser->buffer[i + 1U] == 'X')))
    {
      result.type = UPLINK_PREFIX_ROVER_UP_GENERAL;
      result.offset = i;
      return result;
    }
  }

  return result;
}

static bool find_line_terminator(const UplinkStreamParser *parser, uint16_t *terminator_index)
{
  for (uint16_t i = 0U; i < parser->length; ++i)
  {
    if ((parser->buffer[i] == '\r') || (parser->buffer[i] == '\n'))
    {
      *terminator_index = i;
      return true;
    }
  }

  return false;
}

static bool decode_ac_packet(UplinkStreamParser *parser, UplinkFrame *frame)
{
  AcPacketV6 packet;

  if (parser->length < AC_PACKET_V6_LEN)
  {
    return false;
  }

  if (!ac_packet_v6_decode(parser->buffer, &packet))
  {
    shift_buffer(parser, 1U);
    return false;
  }

  frame->type = UPLINK_FRAME_AC_PACKET_V6;
  frame->data.ac_packet_v6 = packet;
  shift_buffer(parser, AC_PACKET_V6_LEN);
  return true;
}

static bool decode_rover_line(UplinkStreamParser *parser, UplinkFrame *frame, bool *needs_more_data)
{
  RoverUpGeneralFrame rover_frame;
  uint16_t terminator_index = 0U;

  *needs_more_data = false;

  if (!find_line_terminator(parser, &terminator_index))
  {
    if (parser->length <= ROVER_UP_GENERAL_MAX_CONTENT_LEN)
    {
      *needs_more_data = true;
      return false;
    }

    shift_buffer(parser, 1U);
    return false;
  }

  if (!rover_up_general_decode(parser->buffer, terminator_index, &rover_frame))
  {
    shift_buffer(parser, (uint16_t)(terminator_index + 1U));
    return false;
  }

  frame->type = UPLINK_FRAME_ROVER_UP_GENERAL;
  frame->data.rover_up_general = rover_frame;
  shift_buffer(parser, (uint16_t)(terminator_index + 1U));
  return true;
}

void uplink_stream_parser_init(UplinkStreamParser *parser)
{
  if (parser == NULL)
  {
    return;
  }

  parser->length = 0U;
}

void uplink_stream_parser_push(UplinkStreamParser *parser, const uint8_t *data, uint16_t length)
{
  if ((parser == NULL) || (data == NULL) || (length == 0U))
  {
    return;
  }

  for (uint16_t i = 0U; i < length; ++i)
  {
    if (parser->length >= UPLINK_STREAM_PARSER_BUFFER_LEN)
    {
      shift_buffer(parser, 1U);
    }

    parser->buffer[parser->length++] = data[i];
  }
}

bool uplink_stream_parser_next(UplinkStreamParser *parser, UplinkFrame *frame)
{
  if ((parser == NULL) || (frame == NULL))
  {
    return false;
  }

  while (parser->length > 0U)
  {
    UplinkPrefix prefix = find_next_prefix(parser);

    if (prefix.type == UPLINK_PREFIX_NONE)
    {
      parser->length = 0U;
      return false;
    }

    if (prefix.offset > 0U)
    {
      shift_buffer(parser, prefix.offset);
    }

    if (prefix.type == UPLINK_PREFIX_PARTIAL)
    {
      return false;
    }

    if (prefix.type == UPLINK_PREFIX_AC_PACKET_V6)
    {
      if (parser->length < AC_PACKET_V6_LEN)
      {
        return false;
      }

      if (decode_ac_packet(parser, frame))
      {
        return true;
      }

      continue;
    }

    if (prefix.type == UPLINK_PREFIX_ROVER_UP_GENERAL)
    {
      bool needs_more_data = false;

      if (decode_rover_line(parser, frame, &needs_more_data))
      {
        return true;
      }

      if (needs_more_data)
      {
        return false;
      }
    }
  }

  return false;
}
