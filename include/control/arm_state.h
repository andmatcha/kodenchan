#ifndef KODENCHAN_ARM_STATE_H
#define KODENCHAN_ARM_STATE_H

#include "control/arm_types.h"
#include "drivers/limit_switch.h"

#include <stdint.h>

void arm_state_init(ArmState *state);
void arm_state_handle_can_feedback(ArmState *state, uint16_t std_id, const uint8_t data[8]);
void arm_state_set_limit_switches(ArmState *state, const LimitSwitchState *limit_switches);

#endif /* KODENCHAN_ARM_STATE_H */
