/*
 * 責務: アーム制御指令を motor CAN ID 0x200 / 0x1FF / 0x208 の data へ詰める。
 * 依存関係: control/arm_control の ArmMotorCommand を入力にし、drivers/can_bus が送る ArmCanFrame を出力する。
 */

#include "protocol/arm_can_protocol.h"

#include <string.h>

static void write_i16_be(uint8_t *data, int16_t value)
{
  uint16_t raw_value = (uint16_t)value;
  data[0] = (uint8_t)((raw_value >> 8) & 0xFFU);
  data[1] = (uint8_t)(raw_value & 0xFFU);
}

void arm_can_protocol_pack_manual_command(const ArmMotorCommand *command, ArmCanFrame frames[ARM_CAN_FRAME_COUNT])
{
  if ((command == 0) || (frames == 0))
  {
    return;
  }

  memset(frames, 0, sizeof(ArmCanFrame) * ARM_CAN_FRAME_COUNT);

  frames[0].std_id = ARM_CAN_STD_ID_MOTOR_0_3;
  write_i16_be(&frames[0].data[0], command->motor[0]);
  write_i16_be(&frames[0].data[2], command->motor[1]);
  write_i16_be(&frames[0].data[4], command->motor[2]);
  write_i16_be(&frames[0].data[6], command->motor[3]);

  frames[1].std_id = ARM_CAN_STD_ID_MOTOR_4_6;
  write_i16_be(&frames[1].data[0], command->motor[4]);
  write_i16_be(&frames[1].data[2], command->motor[5]);
  write_i16_be(&frames[1].data[4], command->motor[6]);
  frames[1].data[6] = 0U;
  frames[1].data[7] = 0U;

  frames[2].std_id = ARM_CAN_STD_ID_AUX;
  frames[2].data[0] = command->kbd_pp;
  frames[2].data[1] = command->kbd_en;
  frames[2].data[2] = command->kbd_yaman;
  frames[2].data[3] = command->nyokki_push;
  frames[2].data[4] = command->nyokki_pull;
  frames[2].data[5] = command->initialize;
  frames[2].data[6] = command->home;
  frames[2].data[7] = command->kbd_start;
}
