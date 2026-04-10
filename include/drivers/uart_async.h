#ifndef KODENCHAN_UART_ASYNC_H
#define KODENCHAN_UART_ASYNC_H

#include "stm32f3xx_hal.h"

#include <stdint.h>

void uart_async_init(UART_HandleTypeDef *huart);
uint16_t uart_async_read(uint8_t *data, uint16_t max_length);
int uart_async_write(const uint8_t *data, uint16_t length);
void uart_async_on_tx_complete(UART_HandleTypeDef *huart);
void uart_async_on_error(UART_HandleTypeDef *huart);

#endif /* KODENCHAN_UART_ASYNC_H */
