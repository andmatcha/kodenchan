#ifndef KODENCHAN_AC_DIRECT_ARM_SERVICE_H
#define KODENCHAN_AC_DIRECT_ARM_SERVICE_H

#include "stm32f3xx_hal.h"

void ac_direct_arm_service_init(CAN_HandleTypeDef *hcan, UART_HandleTypeDef *huart);
void ac_direct_arm_service_poll(void);

#endif /* KODENCHAN_AC_DIRECT_ARM_SERVICE_H */
