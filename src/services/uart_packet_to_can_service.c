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
#include <string.h>

#define SERVICE_UART_READ_CHUNK_LEN 32U
#define SERVICE_CONTROL_PERIOD_MS 10U
#define SERVICE_UF_TEXT_CAN_BASE_STD_ID 0x220U
#define SERVICE_UF_TEXT_CAN_END_STD_ID 0x225U
#define SERVICE_UF_TEXT_CAN_FRAME_COUNT 4U
#define SERVICE_UF_TEXT_CAN_FRAME_DATA_LEN 8U
#define SERVICE_UF_CAN_FLAGS (UF_PACKET_FLAG_VALID | UF_PACKET_FLAG_USB_PRESENT)
#define SERVICE_UF_READ_ERROR_FLAGS UF_PACKET_FLAG_READ_ERROR
#define SERVICE_USB_READ_RESPONSE_TIMEOUT_MS 5000U
#define SERVICE_USB_READ_PENDING_RESPONSE_COUNT 128U
#define SERVICE_ARM_AUX_USB_READ_DATA_INDEX 2U

static AcStreamParser s_parser;
static ManualInputSnapshot s_manual_snapshot;
static ManualInput s_manual_input;
static ArmState s_arm_state;
static ArmPidState s_pid;
static ArmMotorCommand s_command;
static uint8_t s_uf_seq = 0U;
static uint8_t s_uf_text_chunk_index = 0U;
static uint8_t s_uf_text_payload[UF_PACKET_PAYLOAD_LEN];
static uint8_t s_uf_text_received_mask = 0U;
static uint16_t s_usb_read_requests_queued_for_send = 0U;
static uint32_t s_usb_read_pending_response_started_at_ms[SERVICE_USB_READ_PENDING_RESPONSE_COUNT];
static uint16_t s_usb_read_pending_response_head = 0U;
static uint16_t s_usb_read_pending_response_tail = 0U;
static uint16_t s_usb_read_pending_response_count = 0U;
static uint32_t s_last_control_tick = 0U;

static void send_uf_packet(uint8_t flags, uint8_t chunk_index, uint8_t payload_len, const uint8_t *payload)
{
  uint8_t raw_packet[UF_PACKET_LEN];

  uf_packet_encode(s_uf_seq++, flags, chunk_index, payload_len, payload, raw_packet);
  (void)uart_async_write(raw_packet, UF_PACKET_LEN);
}

static void send_uf_empty_packet(uint8_t flags)
{
  send_uf_packet(flags, 0U, 0U, 0);
}

static void uf_text_reset_buffer(void)
{
  memset(s_uf_text_payload, 0, sizeof(s_uf_text_payload));
  s_uf_text_received_mask = 0U;
}

static void uf_text_reset_transfer(void)
{
  uf_text_reset_buffer();
  s_uf_text_chunk_index = 0U;
}

static bool uf_text_has_buffered_payload(void)
{
  return s_uf_text_received_mask != 0U;
}

static uint8_t uf_text_payload_len(bool trim_padding)
{
  uint8_t payload_len = 0U;

  for (uint8_t i = 0U; i < SERVICE_UF_TEXT_CAN_FRAME_COUNT; ++i)
  {
    if ((s_uf_text_received_mask & (uint8_t)(1U << i)) == 0U)
    {
      break;
    }

    payload_len = (uint8_t)(payload_len + SERVICE_UF_TEXT_CAN_FRAME_DATA_LEN);
  }

  if (trim_padding)
  {
    while ((payload_len > 0U) && (s_uf_text_payload[payload_len - 1U] == 0U))
    {
      --payload_len;
    }
  }

  return payload_len;
}

static void send_uf_text_buffered_packet(uint8_t extra_flags)
{
  uint8_t payload_len = uf_text_payload_len((extra_flags & UF_PACKET_FLAG_END) != 0U);

  send_uf_packet((uint8_t)(SERVICE_UF_CAN_FLAGS | extra_flags),
                 s_uf_text_chunk_index,
                 payload_len,
                 s_uf_text_payload);
  ++s_uf_text_chunk_index;
  uf_text_reset_buffer();

  if ((extra_flags & UF_PACKET_FLAG_END) != 0U)
  {
    s_uf_text_chunk_index = 0U;
  }
}

static void send_uf_text_end_packet(void)
{
  if (uf_text_has_buffered_payload())
  {
    send_uf_text_buffered_packet(UF_PACKET_FLAG_END);
  }
  else
  {
    send_uf_packet((uint8_t)(SERVICE_UF_CAN_FLAGS | UF_PACKET_FLAG_END),
                   s_uf_text_chunk_index,
                   0U,
                   0);
    s_uf_text_chunk_index = 0U;
  }
}

static void usb_read_dequeue_pending_response(void)
{
  if (s_usb_read_pending_response_count == 0U)
  {
    return;
  }

  s_usb_read_pending_response_head =
    (uint16_t)((s_usb_read_pending_response_head + 1U) % SERVICE_USB_READ_PENDING_RESPONSE_COUNT);
  --s_usb_read_pending_response_count;
}

static void usb_read_reset_requests(void)
{
  s_usb_read_requests_queued_for_send = 0U;
  s_usb_read_pending_response_head = 0U;
  s_usb_read_pending_response_tail = 0U;
  s_usb_read_pending_response_count = 0U;
  uf_text_reset_transfer();
}

static void usb_read_queue_send_request(void)
{
  if (s_usb_read_requests_queued_for_send >= SERVICE_USB_READ_PENDING_RESPONSE_COUNT)
  {
    send_uf_empty_packet(SERVICE_UF_READ_ERROR_FLAGS);
    uf_text_reset_transfer();
    return;
  }

  ++s_usb_read_requests_queued_for_send;
}

static bool usb_read_has_request_to_send(void)
{
  return s_usb_read_requests_queued_for_send > 0U;
}

static void usb_read_enqueue_pending_response(uint32_t now_ms)
{
  if (s_usb_read_pending_response_count >= SERVICE_USB_READ_PENDING_RESPONSE_COUNT)
  {
    send_uf_empty_packet(SERVICE_UF_READ_ERROR_FLAGS);
    uf_text_reset_transfer();
    usb_read_dequeue_pending_response();
  }

  s_usb_read_pending_response_started_at_ms[s_usb_read_pending_response_tail] = now_ms;
  s_usb_read_pending_response_tail =
    (uint16_t)((s_usb_read_pending_response_tail + 1U) % SERVICE_USB_READ_PENDING_RESPONSE_COUNT);
  ++s_usb_read_pending_response_count;
}

static void usb_read_note_request_sent(uint32_t now_ms)
{
  bool first_pending_response = (s_usb_read_pending_response_count == 0U);

  if (s_usb_read_requests_queued_for_send > 0U)
  {
    --s_usb_read_requests_queued_for_send;
  }

  if (first_pending_response)
  {
    uf_text_reset_transfer();
  }

  usb_read_enqueue_pending_response(now_ms);
}

static bool usb_read_note_response_received(void)
{
  if (s_usb_read_pending_response_count == 0U)
  {
    return false;
  }

  usb_read_dequeue_pending_response();
  return true;
}

static void usb_read_check_response_timeout(uint32_t now_ms)
{
  while (s_usb_read_pending_response_count > 0U)
  {
    uint32_t started_at_ms = s_usb_read_pending_response_started_at_ms[s_usb_read_pending_response_head];

    if ((now_ms - started_at_ms) <= SERVICE_USB_READ_RESPONSE_TIMEOUT_MS)
    {
      return;
    }

    send_uf_empty_packet(SERVICE_UF_READ_ERROR_FLAGS);
    uf_text_reset_transfer();
    usb_read_dequeue_pending_response();
  }
}

static bool handle_uf_text_can_frame(uint16_t std_id, const uint8_t data[8])
{
  uint8_t frame_index = 0U;
  uint32_t payload_offset = 0U;

  if (data == 0)
  {
    return false;
  }

  if ((std_id >= SERVICE_UF_TEXT_CAN_BASE_STD_ID) &&
      (std_id < (SERVICE_UF_TEXT_CAN_BASE_STD_ID + SERVICE_UF_TEXT_CAN_FRAME_COUNT)))
  {
    frame_index = (uint8_t)(std_id - SERVICE_UF_TEXT_CAN_BASE_STD_ID);
    payload_offset = (uint32_t)frame_index * SERVICE_UF_TEXT_CAN_FRAME_DATA_LEN;

    if ((frame_index == 0U) && uf_text_has_buffered_payload())
    {
      send_uf_text_buffered_packet(0U);
    }

    memcpy(&s_uf_text_payload[payload_offset], data, SERVICE_UF_TEXT_CAN_FRAME_DATA_LEN);
    s_uf_text_received_mask |= (uint8_t)(1U << frame_index);
    return true;
  }

  if (std_id == SERVICE_UF_TEXT_CAN_END_STD_ID)
  {
    send_uf_text_end_packet();
    (void)usb_read_note_response_received();
    return true;
  }

  return false;
}

static void handle_can_feedback(uint16_t std_id, const uint8_t data[8], void *context)
{
  ArmState *state = (ArmState *)context;

  if (handle_uf_text_can_frame(std_id, data))
  {
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
    if ((packet.flags & AC_PACKET_V6_FLAG_USB_READ) != 0U)
    {
      usb_read_queue_send_request();
    }

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

static void send_command(const ArmMotorCommand *command, bool usb_read_requested, uint32_t now_ms)
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

  if (usb_read_requested)
  {
    usb_read_note_request_sent(now_ms);
  }
}

static void run_control_period(uint32_t now_ms)
{
  bool usb_read_requested = false;

  manual_input_apply_timeout(&s_manual_snapshot, now_ms);

  if (!manual_input_to_normalized(&s_manual_snapshot, &s_manual_input))
  {
    manual_input_force_neutral(&s_manual_snapshot, now_ms);
    (void)manual_input_to_normalized(&s_manual_snapshot, &s_manual_input);
  }

  if (arm_control_make_command(&s_manual_input, &s_arm_state, &s_pid, &s_command))
  {
    usb_read_requested = usb_read_has_request_to_send();
    send_command(&s_command, usb_read_requested, now_ms);
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
  usb_read_reset_requests();
  s_last_control_tick = HAL_GetTick();
}

void uart_packet_to_can_service_reset_input(void)
{
  uint32_t now_ms = HAL_GetTick();

  read_uart_stream(false);
  ac_stream_parser_init(&s_parser);
  manual_input_force_neutral(&s_manual_snapshot, now_ms);
  arm_control_init(&s_pid);
  usb_read_reset_requests();
  s_last_control_tick = now_ms;
}

void uart_packet_to_can_service_poll(void)
{
  uint32_t now_ms = HAL_GetTick();

  pump_uart_stream();
  consume_uart_packets(now_ms);
  can_bus_poll(handle_can_feedback, &s_arm_state);
  usb_read_check_response_timeout(now_ms);

  if (control_period_elapsed(now_ms))
  {
    run_control_period(now_ms);
  }
}
