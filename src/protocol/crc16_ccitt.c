#include "protocol/crc16_ccitt.h"

uint16_t crc16_ccitt_false(const uint8_t *data, uint32_t length)
{
  uint16_t crc = 0xFFFFU;

  if (data == 0)
  {
    return crc;
  }

  for (uint32_t i = 0; i < length; ++i)
  {
    crc ^= (uint16_t)data[i] << 8;

    for (uint32_t bit = 0; bit < 8U; ++bit)
    {
      if ((crc & 0x8000U) != 0U)
      {
        crc = (uint16_t)((crc << 1) ^ 0x1021U);
      }
      else
      {
        crc = (uint16_t)(crc << 1);
      }
    }
  }

  return crc;
}
