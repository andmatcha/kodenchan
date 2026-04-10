#include "drivers/limit_switch.h"

#include "main.h"

#ifndef BASE_HORIZON_RIGHT_LIMIT_Pin
#define BASE_HORIZON_RIGHT_LIMIT_Pin GPIO_PIN_0
#define BASE_HORIZON_RIGHT_LIMIT_GPIO_Port GPIOA
#endif

#ifndef BASE_HORIZON_LEFT_LIMIT_Pin
#define BASE_HORIZON_LEFT_LIMIT_Pin GPIO_PIN_1
#define BASE_HORIZON_LEFT_LIMIT_GPIO_Port GPIOA
#endif

void limit_switch_read(LimitSwitchState *state)
{
  if (state == 0)
  {
    return;
  }

  state->base_horizon_right =
      HAL_GPIO_ReadPin(BASE_HORIZON_RIGHT_LIMIT_GPIO_Port, BASE_HORIZON_RIGHT_LIMIT_Pin) != GPIO_PIN_RESET;
  state->base_horizon_left =
      HAL_GPIO_ReadPin(BASE_HORIZON_LEFT_LIMIT_GPIO_Port, BASE_HORIZON_LEFT_LIMIT_Pin) != GPIO_PIN_RESET;
}
