#ifndef KODENCHAN_LIMIT_SWITCH_H
#define KODENCHAN_LIMIT_SWITCH_H

#include <stdbool.h>

typedef struct
{
  bool base_horizon_right;
  bool base_horizon_left;
} LimitSwitchState;

void limit_switch_read(LimitSwitchState *state);

#endif /* KODENCHAN_LIMIT_SWITCH_H */
