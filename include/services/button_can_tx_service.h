#ifndef KODENCHAN_BUTTON_CAN_TX_SERVICE_H
#define KODENCHAN_BUTTON_CAN_TX_SERVICE_H

#include <stdbool.h>

void button_can_tx_service_init(void);
bool button_can_tx_service_poll(bool can_tx_enabled);

#endif /* KODENCHAN_BUTTON_CAN_TX_SERVICE_H */
