#ifndef KODENCHAN_UART_CAN_BRIDGE_SERVICE_H
#define KODENCHAN_UART_CAN_BRIDGE_SERVICE_H

#include "stm32f3xx_hal.h"

void uart_can_bridge_init(CAN_HandleTypeDef *hcan, UART_HandleTypeDef *huart);
void uart_can_bridge_poll(void);

#endif /* KODENCHAN_UART_CAN_BRIDGE_SERVICE_H */
