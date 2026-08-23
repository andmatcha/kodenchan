/*
 * 責務: RoverUpGeneral の ASCII 1行を標準 CAN ID と 32 bit 値へ変換する。
 * 依存関係: protocol/uplink_stream_parser から改行を除いた行を受け、service が送信できる値を返す。
 */

#include "protocol/rover_up_general.h"

#include <stddef.h>

#define ROVER_UP_GENERAL_ID_DIGITS 3U
#define ROVER_UP_GENERAL_COMMA_INDEX 5U
#define ROVER_UP_GENERAL_VALUE_INDEX 6U
#define ROVER_UP_GENERAL_MIN_CONTENT_LEN 7U
#define ROVER_UP_GENERAL_MAX_STD_ID 0x7FFU
#define ROVER_UP_GENERAL_MAX_POSITIVE_VALUE 0x7FFFFFFFUL
#define ROVER_UP_GENERAL_MAX_NEGATIVE_MAGNITUDE 0x80000000UL

static bool decode_hex_digit(uint8_t byte, uint8_t *value)
{
  if ((byte >= '0') && (byte <= '9'))
  {
    *value = (uint8_t)(byte - '0');
    return true;
  }

  if ((byte >= 'A') && (byte <= 'F'))
  {
    *value = (uint8_t)(byte - 'A' + 10U);
    return true;
  }

  if ((byte >= 'a') && (byte <= 'f'))
  {
    *value = (uint8_t)(byte - 'a' + 10U);
    return true;
  }

  return false;
}

static bool decode_std_id(const uint8_t *line, uint16_t *std_id)
{
  uint16_t value = 0U;

  for (uint32_t i = 0U; i < ROVER_UP_GENERAL_ID_DIGITS; ++i)
  {
    uint8_t digit = 0U;

    if (!decode_hex_digit(line[2U + i], &digit))
    {
      return false;
    }

    value = (uint16_t)((value << 4U) | digit);
  }

  if ((value == 0U) || (value > ROVER_UP_GENERAL_MAX_STD_ID))
  {
    return false;
  }

  *std_id = value;
  return true;
}

static bool decode_value(const uint8_t *text, uint16_t length, uint32_t *value)
{
  uint16_t index = 0U;
  bool negative = false;
  uint32_t magnitude = 0U;
  uint32_t limit = ROVER_UP_GENERAL_MAX_POSITIVE_VALUE;

  if (length == 0U)
  {
    return false;
  }

  if ((text[index] == '-') || (text[index] == '+'))
  {
    negative = text[index] == '-';
    ++index;

    if (index >= length)
    {
      return false;
    }
  }

  if (negative)
  {
    limit = ROVER_UP_GENERAL_MAX_NEGATIVE_MAGNITUDE;
  }

  for (; index < length; ++index)
  {
    uint8_t byte = text[index];
    uint32_t digit = 0U;

    if ((byte < '0') || (byte > '9'))
    {
      return false;
    }

    digit = (uint32_t)(byte - '0');
    if (magnitude > ((limit - digit) / 10U))
    {
      return false;
    }

    magnitude = (magnitude * 10U) + digit;
  }

  *value = negative ? (0U - magnitude) : magnitude;
  return true;
}

bool rover_up_general_decode(const uint8_t *line, uint16_t length, RoverUpGeneralFrame *frame)
{
  RoverUpGeneralFrame decoded = {0};

  if ((line == NULL) || (frame == NULL) ||
      (length < ROVER_UP_GENERAL_MIN_CONTENT_LEN) ||
      (length > ROVER_UP_GENERAL_MAX_CONTENT_LEN))
  {
    return false;
  }

  if ((line[0] != '0') || ((line[1] != 'x') && (line[1] != 'X')) ||
      (line[ROVER_UP_GENERAL_COMMA_INDEX] != ','))
  {
    return false;
  }

  if (!decode_std_id(line, &decoded.std_id) ||
      !decode_value(line + ROVER_UP_GENERAL_VALUE_INDEX,
                    (uint16_t)(length - ROVER_UP_GENERAL_VALUE_INDEX),
                    &decoded.value))
  {
    return false;
  }

  *frame = decoded;
  return true;
}

void rover_up_general_pack_payload(uint32_t value, uint8_t payload[ROVER_UP_GENERAL_CAN_PAYLOAD_LEN])
{
  if (payload == NULL)
  {
    return;
  }

  payload[0] = (uint8_t)((value >> 24U) & 0xFFU);
  payload[1] = (uint8_t)((value >> 16U) & 0xFFU);
  payload[2] = (uint8_t)((value >> 8U) & 0xFFU);
  payload[3] = (uint8_t)(value & 0xFFU);
}
