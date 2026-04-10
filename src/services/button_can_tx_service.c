/*
 * 責務: ボタン押下を検出して、設定テーブルに従ったCANフレームを送信する。
 * 依存関係: app の init/poll から呼ばれ、drivers/can_bus の送信APIだけを利用する。
 */

#include "services/button_can_tx_service.h"

#include "drivers/can_bus.h"
#include "services/button_can_tx_config.h"

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
  GPIO_TypeDef *gpio_port;
  uint16_t gpio_pin;
  uint32_t gpio_pull;
  GPIO_PinState active_state;
  uint16_t std_id;
  uint8_t data[8];
} ButtonCanTxConfig;

typedef struct
{
  GPIO_PinState raw_state;
  GPIO_PinState stable_state;
  uint32_t raw_changed_at_ms;
  uint32_t last_tx_at_ms;
  bool initialized;
} ButtonCanTxState;

static const ButtonCanTxConfig s_configs[BUTTON_CAN_TX_CONFIG_COUNT] = BUTTON_CAN_TX_CONFIGS;
static ButtonCanTxState s_states[BUTTON_CAN_TX_CONFIG_COUNT];
static uint32_t s_last_poll_at_ms = 0U;
static bool s_combo_active = false;
static bool s_combo_toggle_reported = false;
static bool s_combo_release_blocked = false;
static uint32_t s_combo_started_at_ms = 0U;

static void enable_gpio_clock(GPIO_TypeDef *gpio_port)
{
#ifdef GPIOA
  if (gpio_port == GPIOA)
  {
    __HAL_RCC_GPIOA_CLK_ENABLE();
    return;
  }
#endif

#ifdef GPIOB
  if (gpio_port == GPIOB)
  {
    __HAL_RCC_GPIOB_CLK_ENABLE();
    return;
  }
#endif

#ifdef GPIOC
  if (gpio_port == GPIOC)
  {
    __HAL_RCC_GPIOC_CLK_ENABLE();
    return;
  }
#endif

#ifdef GPIOD
  if (gpio_port == GPIOD)
  {
    __HAL_RCC_GPIOD_CLK_ENABLE();
    return;
  }
#endif

#ifdef GPIOE
  if (gpio_port == GPIOE)
  {
    __HAL_RCC_GPIOE_CLK_ENABLE();
    return;
  }
#endif

#ifdef GPIOF
  if (gpio_port == GPIOF)
  {
    __HAL_RCC_GPIOF_CLK_ENABLE();
    return;
  }
#endif

  (void)gpio_port;
}

static void init_button_gpio(const ButtonCanTxConfig *config)
{
  GPIO_InitTypeDef gpio = {0};

  enable_gpio_clock(config->gpio_port);

  gpio.Pin = config->gpio_pin;
  gpio.Mode = GPIO_MODE_INPUT;
  gpio.Pull = config->gpio_pull;
  HAL_GPIO_Init(config->gpio_port, &gpio);
}

static void init_button_state(const ButtonCanTxConfig *config, ButtonCanTxState *state, uint32_t now_ms)
{
  GPIO_PinState pin_state = HAL_GPIO_ReadPin(config->gpio_port, config->gpio_pin);

  state->raw_state = pin_state;
  state->stable_state = pin_state;
  state->raw_changed_at_ms = now_ms;
  state->last_tx_at_ms = 0U;
  state->initialized = true;
}

static bool update_button_state(const ButtonCanTxConfig *config, ButtonCanTxState *state, uint32_t now_ms)
{
  GPIO_PinState pin_state = HAL_GPIO_ReadPin(config->gpio_port, config->gpio_pin);

  if (!state->initialized)
  {
    init_button_state(config, state, now_ms);
    return false;
  }

  if (pin_state != state->raw_state)
  {
    state->raw_state = pin_state;
    state->raw_changed_at_ms = now_ms;
  }

  if ((pin_state != state->stable_state) && ((now_ms - state->raw_changed_at_ms) >= BUTTON_CAN_TX_DEBOUNCE_MS))
  {
    state->stable_state = pin_state;
    return true;
  }

  return false;
}

static bool button_is_active(const ButtonCanTxConfig *config, const ButtonCanTxState *state)
{
  return state->initialized && (state->stable_state == config->active_state);
}

static bool combo_buttons_are_active(void)
{
#if BUTTON_CAN_TX_CONFIG_COUNT >= 2U
  return button_is_active(&s_configs[0], &s_states[0]) &&
         button_is_active(&s_configs[1], &s_states[1]);
#else
  return false;
#endif
}

static bool combo_button_is_active(void)
{
#if BUTTON_CAN_TX_CONFIG_COUNT >= 2U
  return button_is_active(&s_configs[0], &s_states[0]) ||
         button_is_active(&s_configs[1], &s_states[1]);
#else
  return false;
#endif
}

static bool poll_combo_toggle(bool combo_active, uint32_t now_ms)
{
  if (!combo_active)
  {
    s_combo_active = false;
    s_combo_toggle_reported = false;
    s_combo_started_at_ms = now_ms;
    return false;
  }

  if (!s_combo_active)
  {
    s_combo_active = true;
    s_combo_toggle_reported = false;
    s_combo_started_at_ms = now_ms;
  }

  if (!s_combo_toggle_reported && ((now_ms - s_combo_started_at_ms) >= BUTTON_CAN_TX_MODE_TOGGLE_HOLD_MS))
  {
    s_combo_toggle_reported = true;
    return true;
  }

  return false;
}

static void send_configured_frame(const ButtonCanTxConfig *config, ButtonCanTxState *state, uint32_t now_ms)
{
  if (can_bus_send(config->std_id, config->data) == HAL_OK)
  {
    state->last_tx_at_ms = now_ms;
  }
}

static void poll_button_tx(const ButtonCanTxConfig *config, ButtonCanTxState *state, bool stable_changed, uint32_t now_ms)
{
  if (stable_changed && (state->stable_state == config->active_state))
  {
    send_configured_frame(config, state, now_ms);
  }

#if BUTTON_CAN_TX_REPEAT_WHILE_PRESSED
  if ((state->stable_state == config->active_state) &&
      ((now_ms - state->last_tx_at_ms) >= BUTTON_CAN_TX_REPEAT_PERIOD_MS))
  {
    send_configured_frame(config, state, now_ms);
  }
#endif
}

void button_can_tx_service_init(void)
{
  uint32_t now_ms = HAL_GetTick();

  for (uint32_t i = 0U; i < BUTTON_CAN_TX_CONFIG_COUNT; ++i)
  {
    init_button_gpio(&s_configs[i]);
    init_button_state(&s_configs[i], &s_states[i], now_ms);
  }

  s_last_poll_at_ms = now_ms;
  s_combo_active = false;
  s_combo_toggle_reported = false;
  s_combo_release_blocked = false;
  s_combo_started_at_ms = now_ms;
}

bool button_can_tx_service_poll(bool can_tx_enabled)
{
  uint32_t now_ms = HAL_GetTick();
  bool stable_changed[BUTTON_CAN_TX_CONFIG_COUNT] = {false};
  bool combo_active = false;
  bool combo_toggle_requested = false;
  bool tx_blocked = false;

  if ((now_ms - s_last_poll_at_ms) < BUTTON_CAN_TX_POLL_PERIOD_MS)
  {
    return false;
  }

  s_last_poll_at_ms = now_ms;

  for (uint32_t i = 0U; i < BUTTON_CAN_TX_CONFIG_COUNT; ++i)
  {
    stable_changed[i] = update_button_state(&s_configs[i], &s_states[i], now_ms);
  }

  combo_active = combo_buttons_are_active();
  combo_toggle_requested = poll_combo_toggle(combo_active, now_ms);
  if (combo_active)
  {
    s_combo_release_blocked = true;
  }
  else if (!combo_button_is_active())
  {
    s_combo_release_blocked = false;
  }

  tx_blocked = combo_active || s_combo_release_blocked;

  if (!can_tx_enabled || tx_blocked)
  {
    return combo_toggle_requested;
  }

  for (uint32_t i = 0U; i < BUTTON_CAN_TX_CONFIG_COUNT; ++i)
  {
    poll_button_tx(&s_configs[i], &s_states[i], stable_changed[i], now_ms);
  }

  return combo_toggle_requested;
}
