/*
 * 責務: CAN feedback とリミットスイッチ情報をアーム状態へ反映する。
 * 依存関係: drivers/can_bus と drivers/limit_switch から入力を受け、control/arm_control が参照する ArmState を更新する。
 */

#include "control/arm_state.h"

#include <string.h>

static int16_t read_i16_be(const uint8_t *data)
{
  return (int16_t)(((uint16_t)data[0] << 8) | (uint16_t)data[1]);
}

static uint16_t read_u16_le(const uint8_t *data)
{
  return (uint16_t)data[0] | ((uint16_t)data[1] << 8);
}

void arm_state_init(ArmState *state)
{
  if (state == 0)
  {
    return;
  }

  memset(state, 0, sizeof(*state));
}

void arm_state_handle_can_feedback(ArmState *state, uint16_t std_id, const uint8_t data[8])
{
  if ((state == 0) || (data == 0))
  {
    return;
  }

  if ((std_id >= 0x202U) && (std_id <= 0x206U))
  {
    uint32_t axis = std_id - 0x201U;
    if (axis < ARM_AXIS_COUNT)
    {
      state->rpm[axis] = read_i16_be(&data[2]);
    }
    return;
  }

  if ((std_id >= 0x300U) && (std_id <= 0x304U))
  {
    uint32_t axis = std_id - 0x300U;
    if (axis < ARM_AXIS_COUNT)
    {
      state->angle[axis] = read_u16_le(&data[0]);
    }
  }
}

void arm_state_set_limit_switches(ArmState *state, const LimitSwitchState *limit_switches)
{
  if ((state == 0) || (limit_switches == 0))
  {
    return;
  }

  state->limit_right = limit_switches->base_horizon_right;
  state->limit_left = limit_switches->base_horizon_left;
}
