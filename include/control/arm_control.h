#ifndef KODENCHAN_ARM_CONTROL_H
#define KODENCHAN_ARM_CONTROL_H

#include "control/arm_types.h"

#include <stdbool.h>

void arm_control_init(ArmPidState *pid);
bool arm_control_make_command(const ManualInput *input, const ArmState *state, ArmPidState *pid, ArmMotorCommand *command);

#endif /* KODENCHAN_ARM_CONTROL_H */
