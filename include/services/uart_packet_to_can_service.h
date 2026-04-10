#ifndef KODENCHAN_UART_PACKET_TO_CAN_SERVICE_H
#define KODENCHAN_UART_PACKET_TO_CAN_SERVICE_H

#include "stm32f3xx_hal.h"

void uart_packet_to_can_service_init(CAN_HandleTypeDef *hcan, UART_HandleTypeDef *huart);
void uart_packet_to_can_service_reset_input(void);
void uart_packet_to_can_service_poll(void);

#endif /* KODENCHAN_UART_PACKET_TO_CAN_SERVICE_H */
