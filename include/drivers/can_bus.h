#ifndef KODENCHAN_CAN_BUS_H
#define KODENCHAN_CAN_BUS_H

#include "stm32f3xx_hal.h"

#include <stdint.h>

typedef void (*CanBusRxCallback)(uint16_t std_id, const uint8_t data[8], void *context);

void can_bus_init(CAN_HandleTypeDef *hcan);
HAL_StatusTypeDef can_bus_send(uint16_t std_id, const uint8_t data[8]);
void can_bus_poll(CanBusRxCallback callback, void *context);

#endif /* KODENCHAN_CAN_BUS_H */
