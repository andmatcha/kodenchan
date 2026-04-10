#ifndef KODENCHAN_ARM_TYPES_H
#define KODENCHAN_ARM_TYPES_H

#include <stdbool.h>
#include <stdint.h>

#define ARM_AXIS_COUNT 7U
#define ARM_COMMAND_MIN (-16384)
#define ARM_COMMAND_MAX 16384
#define ARM_RAW_NEUTRAL 255U
#define ARM_RAW_SCALE 64

typedef enum
{
  ARM_AXIS_BASE_HORIZON = 0,
  ARM_AXIS_BASE_ROLL = 1,
  ARM_AXIS_JOINT1 = 2,
  ARM_AXIS_JOINT2 = 3,
  ARM_AXIS_JOINT3 = 4,
  ARM_AXIS_JOINT4 = 5,
  ARM_AXIS_GRIPPER = 6
} ArmAxis;

typedef struct
{
  int16_t rpm[ARM_AXIS_COUNT];
  uint16_t angle[ARM_AXIS_COUNT];
} ArmState;

typedef struct
{
  uint16_t raw_current[ARM_AXIS_COUNT];
  uint8_t control_byte;
  uint32_t updated_at_ms;
  bool valid;
} ManualInputSnapshot;

typedef struct
{
  int16_t normalized[ARM_AXIS_COUNT];
  uint8_t kbd_pp;
  uint8_t kbd_en;
  uint8_t kbd_yaman;
  uint8_t nyokki_push;
  uint8_t nyokki_pull;
  uint8_t initialize;
  uint8_t home;
  uint8_t kbd_start;
} ManualInput;

typedef struct
{
  int16_t motor[ARM_AXIS_COUNT];
  uint8_t kbd_pp;
  uint8_t kbd_en;
  uint8_t kbd_yaman;
  uint8_t nyokki_push;
  uint8_t nyokki_pull;
  uint8_t initialize;
  uint8_t home;
  uint8_t kbd_start;
} ArmMotorCommand;

typedef struct
{
  float manual_kp[ARM_AXIS_COUNT];
  float manual_ki[ARM_AXIS_COUNT];
  float manual_kd[ARM_AXIS_COUNT];
  float rpm_integral[ARM_AXIS_COUNT];
  float rpm_prev_error[ARM_AXIS_COUNT];
  int16_t rpm_output[ARM_AXIS_COUNT];
} ArmPidState;

#endif /* KODENCHAN_ARM_TYPES_H */
