/*
 * 責務: ACv6 manual 入力を snapshot として保持し、timeout と正規化を行う。
 * 依存関係: protocol/ac_packet_v6 から AcPacketV6 を受け、control/arm_control へ ManualInput を渡す。
 */

#include "control/manual_input.h"

static void set_raw_neutral(ManualInputSnapshot *snapshot)
{
  for (uint32_t i = 0U; i < ARM_AXIS_COUNT; ++i)
  {
    snapshot->raw_current[i] = ARM_RAW_NEUTRAL;
  }

  snapshot->control_byte = 0U;
}

void manual_input_init(ManualInputSnapshot *snapshot)
{
  if (snapshot == 0)
  {
    return;
  }

  set_raw_neutral(snapshot);
  snapshot->updated_at_ms = 0U;
  snapshot->valid = false;
}

void manual_input_force_neutral(ManualInputSnapshot *snapshot, uint32_t now_ms)
{
  if (snapshot == 0)
  {
    return;
  }

  set_raw_neutral(snapshot);
  snapshot->updated_at_ms = now_ms;
  snapshot->valid = false;
}

void manual_input_update_from_packet(ManualInputSnapshot *snapshot, const AcPacketV6 *packet, uint32_t now_ms)
{
  if ((snapshot == 0) || (packet == 0))
  {
    return;
  }

  for (uint32_t i = 0U; i < ARM_AXIS_COUNT; ++i)
  {
    snapshot->raw_current[i] = packet->current[i];
  }

  snapshot->control_byte = packet->control_byte;
  snapshot->updated_at_ms = now_ms;
  snapshot->valid = true;
}

void manual_input_apply_timeout(ManualInputSnapshot *snapshot, uint32_t now_ms)
{
  if ((snapshot == 0) || !snapshot->valid)
  {
    return;
  }

  if ((now_ms - snapshot->updated_at_ms) > MANUAL_INPUT_TIMEOUT_MS)
  {
    manual_input_force_neutral(snapshot, now_ms);
  }
}

bool manual_input_to_normalized(const ManualInputSnapshot *snapshot, ManualInput *input)
{
  if ((snapshot == 0) || (input == 0))
  {
    return false;
  }

  for (uint32_t i = 0U; i < ARM_AXIS_COUNT; ++i)
  {
    int32_t normalized = ((int32_t)snapshot->raw_current[i] - (int32_t)ARM_RAW_NEUTRAL) * ARM_RAW_SCALE;
    if ((normalized < ARM_COMMAND_MIN) || (normalized > ARM_COMMAND_MAX))
    {
      return false;
    }

    input->normalized[i] = (int16_t)normalized;
  }

  input->kbd_pp = snapshot->control_byte & 0x01U;
  input->kbd_en = (snapshot->control_byte >> 1) & 0x01U;
  input->kbd_yaman = (snapshot->control_byte >> 2) & 0x01U;
  input->nyokki_push = (snapshot->control_byte >> 3) & 0x01U;
  input->nyokki_pull = (snapshot->control_byte >> 4) & 0x01U;
  input->initialize = (snapshot->control_byte >> 5) & 0x01U;
  input->home = (snapshot->control_byte >> 6) & 0x01U;
  input->kbd_start = (snapshot->control_byte >> 7) & 0x01U;

  return true;
}
