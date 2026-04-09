#ifndef KODENCHAN_CAN_BRIDGE_H
#define KODENCHAN_CAN_BRIDGE_H

#include "stm32f3xx_hal.h"

void can_bridge_init(CAN_HandleTypeDef *hcan, UART_HandleTypeDef *huart);
void can_bridge_poll(void);

#endif /* KODENCHAN_CAN_BRIDGE_H */
