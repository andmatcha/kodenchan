#ifndef KODENCHAN_ARM_CAN_PROTOCOL_H
#define KODENCHAN_ARM_CAN_PROTOCOL_H

#include "control/arm_types.h"

#include <stdint.h>

#define ARM_CAN_STD_ID_MOTOR_0_3 0x200U
#define ARM_CAN_STD_ID_MOTOR_4_6 0x1FFU
#define ARM_CAN_STD_ID_AUX 0x208U
#define ARM_CAN_FRAME_COUNT 3U

typedef struct
{
  uint16_t std_id;
  uint8_t data[8];
} ArmCanFrame;

void arm_can_protocol_pack_manual_command(const ArmMotorCommand *command, ArmCanFrame frames[ARM_CAN_FRAME_COUNT]);

#endif /* KODENCHAN_ARM_CAN_PROTOCOL_H */
