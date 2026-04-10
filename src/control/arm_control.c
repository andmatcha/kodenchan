/*
 * 責務: マニュアル入力、PID、リミット保護を合成して最終モーター指令を生成する。
 * 依存関係: 入力は ManualInput と ArmState、出力は protocol/arm_can_protocol に渡す ArmMotorCommand。
 */

#include "control/arm_control.h"

#include <string.h>

#define LIMIT_ESCAPE_COMMAND 2000

static int16_t clamp_command(float value)
{
  if (value > (float)ARM_COMMAND_MAX)
  {
    return ARM_COMMAND_MAX;
  }

  if (value < (float)ARM_COMMAND_MIN)
  {
    return ARM_COMMAND_MIN;
  }

  return (int16_t)value;
}

static void reset_axis_pid(ArmPidState *pid, uint32_t axis)
{
  if ((pid == 0) || (axis >= ARM_AXIS_COUNT))
  {
    return;
  }

  pid->rpm_integral[axis] = 0.0f;
  pid->rpm_prev_error[axis] = 0.0f;
  pid->rpm_output[axis] = 0;
}

static void update_manual_rpm_pid(const ManualInput *input, const ArmState *state, ArmPidState *pid)
{
  for (uint32_t i = 0U; i < ARM_AXIS_COUNT; ++i)
  {
    if (input->normalized[i] == 0)
    {
      float error = 0.0f - (float)state->rpm[i];
      float output = 0.0f;

      pid->rpm_integral[i] += error;
      output = (pid->manual_kp[i] * error) +
               (pid->manual_ki[i] * pid->rpm_integral[i]) +
               (pid->manual_kd[i] * (error - pid->rpm_prev_error[i]));

      pid->rpm_output[i] = clamp_command(output);
      pid->rpm_prev_error[i] = error;
    }
    else
    {
      reset_axis_pid(pid, i);
    }
  }
}

void arm_control_init(ArmPidState *pid)
{
  static const float manual_kp[ARM_AXIS_COUNT] = {1.0f, 1.0f, 2.2f, 2.0f, 0.2f, 0.13f, 1.0f};
  static const float manual_ki[ARM_AXIS_COUNT] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
  static const float manual_kd[ARM_AXIS_COUNT] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.03f, 0.0f};

  if (pid == 0)
  {
    return;
  }

  memset(pid, 0, sizeof(*pid));

  for (uint32_t i = 0U; i < ARM_AXIS_COUNT; ++i)
  {
    pid->manual_kp[i] = manual_kp[i];
    pid->manual_ki[i] = manual_ki[i];
    pid->manual_kd[i] = manual_kd[i];
  }
}

bool arm_control_make_command(const ManualInput *input, const ArmState *state, ArmPidState *pid, ArmMotorCommand *command)
{
  if ((input == 0) || (state == 0) || (pid == 0) || (command == 0))
  {
    return false;
  }

  update_manual_rpm_pid(input, state, pid);

  for (uint32_t i = 0U; i < ARM_AXIS_COUNT; ++i)
  {
    command->motor[i] = (input->normalized[i] != 0) ? input->normalized[i] : pid->rpm_output[i];
  }

  if (state->limit_right && state->limit_left)
  {
    command->motor[ARM_AXIS_BASE_HORIZON] = 0;
    reset_axis_pid(pid, ARM_AXIS_BASE_HORIZON);
  }
  else if (state->limit_right && (input->normalized[ARM_AXIS_BASE_HORIZON] > 0))
  {
    command->motor[ARM_AXIS_BASE_HORIZON] = -LIMIT_ESCAPE_COMMAND;
    reset_axis_pid(pid, ARM_AXIS_BASE_HORIZON);
  }
  else if (state->limit_left && (input->normalized[ARM_AXIS_BASE_HORIZON] < 0))
  {
    command->motor[ARM_AXIS_BASE_HORIZON] = LIMIT_ESCAPE_COMMAND;
    reset_axis_pid(pid, ARM_AXIS_BASE_HORIZON);
  }

  command->keyboard_nyokki_enabled = input->keyboard_nyokki_enabled;
  command->usb_nyokki_push = input->usb_nyokki_push;
  command->usb_nyokki_pull = input->usb_nyokki_pull;

  return true;
}
