/*
 * 責務: UARTで受けたmanual packetをアーム制御CANへ変換して送信する。
 * 依存関係: app の init/poll から呼ばれ、drivers / protocol / control の隣接レイヤーを接続してCAN送信まで進める。
 */

#include "services/uart_packet_to_can_service.h"

#include "control/arm_control.h"
#include "control/arm_state.h"
#include "control/manual_input.h"
#include "drivers/can_bus.h"
#include "drivers/uart_async.h"
#include "main.h"
#include "protocol/ac_stream_parser.h"
#include "protocol/arm_can_protocol.h"
#include "protocol/uf_packet.h"

#include <stdbool.h>
#include <stdint.h>

#define SERVICE_UART_READ_CHUNK_LEN 32U
#define SERVICE_CONTROL_PERIOD_MS 10U
#define SERVICE_USB_FEEDBACK_CAN_STD_ID 0x209U
#define SERVICE_UF_CAN_FLAGS (UF_PACKET_FLAG_VALID | UF_PACKET_FLAG_USB_PRESENT)
#define SERVICE_ARM_AUX_USB_READ_DATA_INDEX 2U

static AcStreamParser s_parser;
static ManualInputSnapshot s_manual_snapshot;
static ManualInput s_manual_input;
static ArmState s_arm_state;
static ArmPidState s_pid;
static ArmMotorCommand s_command;
static uint8_t s_uf_seq = 0U;
static bool s_usb_read_requested = false;
static bool s_usb_read_fresh = false;
static uint32_t s_usb_read_updated_at_ms = 0U;
static uint32_t s_last_control_tick = 0U;

static int32_t read_i32_le(const uint8_t *data)
{
  uint32_t raw_value = (uint32_t)data[0] |
                       ((uint32_t)data[1] << 8) |
                       ((uint32_t)data[2] << 16) |
                       ((uint32_t)data[3] << 24);

  return (int32_t)raw_value;
}

static void send_uf_packet_from_can(const uint8_t data[8])
{
  uint8_t raw_packet[UF_PACKET_LEN];
  int32_t lat_e7 = 0;
  int32_t lon_e7 = 0;

  if (data == 0)
  {
    return;
  }

  lat_e7 = read_i32_le(data);
  lon_e7 = read_i32_le(data + 4U);

  uf_packet_encode(s_uf_seq++, SERVICE_UF_CAN_FLAGS, lat_e7, lon_e7, raw_packet);
  (void)uart_async_write(raw_packet, UF_PACKET_LEN);
}

static void handle_can_feedback(uint16_t std_id, const uint8_t data[8], void *context)
{
  ArmState *state = (ArmState *)context;

  if (std_id == SERVICE_USB_FEEDBACK_CAN_STD_ID)
  {
    send_uf_packet_from_can(data);
    return;
  }

  arm_state_handle_can_feedback(state, std_id, data);
}

static void read_uart_stream(bool feed_parser)
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
  read_uart_stream(true);
}

static void consume_uart_packets(uint32_t now_ms)
{
  AcPacketV6 packet;

  while (ac_stream_parser_next(&s_parser, &packet))
  {
    s_usb_read_requested = ((packet.flags & AC_PACKET_V6_FLAG_USB_READ) != 0U);
    s_usb_read_fresh = true;
    s_usb_read_updated_at_ms = now_ms;

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

static bool usb_read_is_active(uint32_t now_ms)
{
  if (!s_usb_read_fresh)
  {
    return false;
  }

  if ((now_ms - s_usb_read_updated_at_ms) > MANUAL_INPUT_TIMEOUT_MS)
  {
    s_usb_read_fresh = false;
    s_usb_read_requested = false;
    return false;
  }

  return s_usb_read_requested;
}

static void send_command(const ArmMotorCommand *command, bool usb_read_requested)
{
  ArmCanFrame frames[ARM_CAN_FRAME_COUNT];

  arm_can_protocol_pack_manual_command(command, frames);

  if (usb_read_requested)
  {
    frames[2].data[SERVICE_ARM_AUX_USB_READ_DATA_INDEX] = 1U;
  }

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
    send_command(&s_command, usb_read_is_active(now_ms));
  }
}

void uart_packet_to_can_service_init(CAN_HandleTypeDef *hcan, UART_HandleTypeDef *huart)
{
  uart_async_init(huart);
  can_bus_init(hcan);
  ac_stream_parser_init(&s_parser);
  manual_input_init(&s_manual_snapshot);
  arm_state_init(&s_arm_state);
  arm_control_init(&s_pid);
  s_uf_seq = 0U;
  s_usb_read_requested = false;
  s_usb_read_fresh = false;
  s_usb_read_updated_at_ms = 0U;
  s_last_control_tick = HAL_GetTick();
}

void uart_packet_to_can_service_reset_input(void)
{
  uint32_t now_ms = HAL_GetTick();

  read_uart_stream(false);
  ac_stream_parser_init(&s_parser);
  manual_input_force_neutral(&s_manual_snapshot, now_ms);
  arm_control_init(&s_pid);
  s_usb_read_requested = false;
  s_usb_read_fresh = false;
  s_usb_read_updated_at_ms = now_ms;
  s_last_control_tick = now_ms;
}

void uart_packet_to_can_service_poll(void)
{
  uint32_t now_ms = HAL_GetTick();

  pump_uart_stream();
  consume_uart_packets(now_ms);
  can_bus_poll(handle_can_feedback, &s_arm_state);

  if (control_period_elapsed(now_ms))
  {
    run_control_period(now_ms);
  }
}
