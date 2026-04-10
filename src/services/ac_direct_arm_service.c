/*
 * 責務: UARTで受けた PacketACv6 manual 入力を直接アーム制御CANへ変換して送信する。
 * 依存関係: app の init/poll から呼ばれ、drivers / protocol / control の隣接レイヤーを接続してCAN送信まで進める。
 */

#include "services/ac_direct_arm_service.h"

#include "control/arm_control.h"
#include "control/arm_state.h"
#include "control/manual_input.h"
#include "drivers/can_bus.h"
#include "drivers/uart_async.h"
#include "main.h"
#include "protocol/ac_stream_parser.h"
#include "protocol/arm_can_protocol.h"

#include <stdbool.h>
#include <stdint.h>

#define SERVICE_UART_READ_CHUNK_LEN 32U
#define SERVICE_CONTROL_PERIOD_MS 10U

static AcStreamParser s_parser;
static ManualInputSnapshot s_manual_snapshot;
static ManualInput s_manual_input;
static ArmState s_arm_state;
static ArmPidState s_pid;
static ArmMotorCommand s_command;
static uint32_t s_last_control_tick = 0U;

static void handle_can_feedback(uint16_t std_id, const uint8_t data[8], void *context)
{
  ArmState *state = (ArmState *)context;
  arm_state_handle_can_feedback(state, std_id, data);
}

static void drain_uart_stream(bool feed_parser)
{
  uint8_t rx_data[SERVICE_UART_READ_CHUNK_LEN];
  uint16_t received = 0U;

  do
  {
    received = uart_async_read(rx_data, sizeof(rx_data));
    if (feed_parser && (received > 0U))
    {
      ac_stream_parser_push(&s_parser, rx_data, received);
    }
  } while (received == sizeof(rx_data));
}

static void pump_uart_stream(void)
{
  drain_uart_stream(true);
}

static void consume_ac_packets(uint32_t now_ms)
{
  AcPacketV6 packet;

  while (ac_stream_parser_next(&s_parser, &packet))
  {
    if (ac_packet_v6_mode(&packet) == AC_PACKET_MODE_MANUAL)
    {
      manual_input_update_from_packet(&s_manual_snapshot, &packet, now_ms);
    }
  }
}

static bool control_period_elapsed(uint32_t now_ms)
{
  if ((now_ms - s_last_control_tick) < SERVICE_CONTROL_PERIOD_MS)
  {
    return false;
  }

  s_last_control_tick = now_ms;
  return true;
}

static void send_command(const ArmMotorCommand *command)
{
  ArmCanFrame frames[ARM_CAN_FRAME_COUNT];

  arm_can_protocol_pack_manual_command(command, frames);

  for (uint32_t i = 0U; i < ARM_CAN_FRAME_COUNT; ++i)
  {
    if (can_bus_send(frames[i].std_id, frames[i].data) != HAL_OK)
    {
      Error_Handler();
    }
  }
}

static void run_control_period(uint32_t now_ms)
{
  manual_input_apply_timeout(&s_manual_snapshot, now_ms);

  if (!manual_input_to_normalized(&s_manual_snapshot, &s_manual_input))
  {
    manual_input_force_neutral(&s_manual_snapshot, now_ms);
    (void)manual_input_to_normalized(&s_manual_snapshot, &s_manual_input);
  }

  if (arm_control_make_command(&s_manual_input, &s_arm_state, &s_pid, &s_command))
  {
    send_command(&s_command);
  }
}

void ac_direct_arm_service_init(CAN_HandleTypeDef *hcan, UART_HandleTypeDef *huart)
{
  uart_async_init(huart);
  can_bus_init(hcan);
  ac_stream_parser_init(&s_parser);
  manual_input_init(&s_manual_snapshot);
  arm_state_init(&s_arm_state);
  arm_control_init(&s_pid);
  s_last_control_tick = HAL_GetTick();
}

void ac_direct_arm_service_discard_input(void)
{
  uint32_t now_ms = HAL_GetTick();

  drain_uart_stream(false);
  ac_stream_parser_init(&s_parser);
  manual_input_force_neutral(&s_manual_snapshot, now_ms);
  arm_control_init(&s_pid);
  s_last_control_tick = now_ms;
}

void ac_direct_arm_service_poll(void)
{
  uint32_t now_ms = HAL_GetTick();

  pump_uart_stream();
  consume_ac_packets(now_ms);
  can_bus_poll(handle_can_feedback, &s_arm_state);

  if (control_period_elapsed(now_ms))
  {
    run_control_period(now_ms);
  }
}
