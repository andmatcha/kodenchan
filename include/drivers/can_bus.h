#ifndef KODENCHAN_CAN_BUS_H
#define KODENCHAN_CAN_BUS_H

#include "stm32f3xx_hal.h"

#include <stdbool.h>
#include <stdint.h>

typedef void (*CanBusRxCallback)(uint16_t std_id, const uint8_t data[8], void *context);

void can_bus_init(CAN_HandleTypeDef *hcan);
HAL_StatusTypeDef can_bus_set_rx_all_ids(bool enabled);
void can_bus_set_tx_enabled(bool enabled);
HAL_StatusTypeDef can_bus_send(uint16_t std_id, const uint8_t data[8]);
uint32_t can_bus_poll(CanBusRxCallback callback, void *context);
uint32_t can_bus_log_rx(void);
uint32_t can_bus_discard_rx(void);

#endif /* KODENCHAN_CAN_BUS_H */
