/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
{
  uint32_t id;
  uint8_t dlc;
  uint8_t is_extended;
  uint8_t is_remote;
  uint8_t data[8];
} CanMonitorFrame;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CAN_MONITOR_STATS_INTERVAL_MS 1000U
#define CAN_MONITOR_QUEUE_CAPACITY 256U
#define CAN_MONITOR_QUEUE_MASK (CAN_MONITOR_QUEUE_CAPACITY - 1U)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
static CanMonitorFrame can_monitor_queue[CAN_MONITOR_QUEUE_CAPACITY];
static volatile uint16_t can_monitor_queue_head = 0U;
static volatile uint16_t can_monitor_queue_tail = 0U;
static volatile uint16_t can_monitor_queue_peak = 0U;
static volatile uint32_t can_monitor_rx_total = 0U;
static volatile uint32_t can_monitor_queue_drop_total = 0U;
static volatile uint32_t can_monitor_fifo_full_total = 0U;
static volatile uint32_t can_monitor_fifo_overrun_total = 0U;
static volatile uint32_t can_monitor_read_error_total = 0U;
static volatile uint32_t can_monitor_can_error_total = 0U;
static uint32_t can_monitor_output_total = 0U;
static uint32_t can_monitor_uart_drop_total = 0U;
static uint32_t can_monitor_stats_last_tick_ms = 0U;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_CAN_Init(void);
/* USER CODE BEGIN PFP */
static void CanMonitorEnqueue(const CAN_RxHeaderTypeDef *header, const uint8_t data[8]);
static uint8_t CanMonitorPop(CanMonitorFrame *frame);
static void CanMonitorWriteFrame(const CanMonitorFrame *frame);
static void CanMonitorWriteStats(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static uint16_t CanMonitorQueueDepthUnsafe(void)
{
  uint16_t head = can_monitor_queue_head;
  uint16_t tail = can_monitor_queue_tail;

  if (head >= tail)
  {
    return (uint16_t)(head - tail);
  }

  return (uint16_t)(CAN_MONITOR_QUEUE_CAPACITY - tail + head);
}

static void CanMonitorEnqueue(const CAN_RxHeaderTypeDef *header, const uint8_t data[8])
{
  uint16_t head = can_monitor_queue_head;
  uint16_t next = (uint16_t)((head + 1U) & CAN_MONITOR_QUEUE_MASK);
  uint8_t dlc = (header->DLC <= 8U) ? (uint8_t)header->DLC : 8U;

  can_monitor_rx_total++;
  if (next == can_monitor_queue_tail)
  {
    can_monitor_queue_drop_total++;
    return;
  }

  CanMonitorFrame *frame = &can_monitor_queue[head];
  frame->id = (header->IDE == CAN_ID_STD) ? header->StdId : header->ExtId;
  frame->dlc = dlc;
  frame->is_extended = (header->IDE == CAN_ID_EXT) ? 1U : 0U;
  frame->is_remote = (header->RTR == CAN_RTR_REMOTE) ? 1U : 0U;
  for (uint8_t i = 0U; i < dlc; i++)
  {
    frame->data[i] = data[i];
  }

  __DMB();
  can_monitor_queue_head = next;

  uint16_t depth = CanMonitorQueueDepthUnsafe();
  if (depth > can_monitor_queue_peak)
  {
    can_monitor_queue_peak = depth;
  }
}

static uint8_t CanMonitorPop(CanMonitorFrame *frame)
{
  uint32_t primask = __get_PRIMASK();
  __disable_irq();

  uint16_t tail = can_monitor_queue_tail;
  if (tail == can_monitor_queue_head)
  {
    if (primask == 0U)
    {
      __enable_irq();
    }
    return 0U;
  }

  *frame = can_monitor_queue[tail];
  can_monitor_queue_tail = (uint16_t)((tail + 1U) & CAN_MONITOR_QUEUE_MASK);

  if (primask == 0U)
  {
    __enable_irq();
  }
  return 1U;
}

static uint16_t CanMonitorAppendHex(char *line, uint16_t position, uint32_t value, uint8_t width)
{
  static const char hex[] = "0123456789ABCDEF";

  for (uint8_t i = 0U; i < width; i++)
  {
    uint8_t shift = (uint8_t)((width - i - 1U) * 4U);
    line[position++] = hex[(value >> shift) & 0x0FU];
  }

  return position;
}

static void CanMonitorWriteFrame(const CanMonitorFrame *frame)
{
  char line[32];
  uint16_t length = 0U;

  if (frame->is_extended != 0U)
  {
    length = CanMonitorAppendHex(line, length, frame->id & 0x1FFFFFFFU, 8U);
  }
  else
  {
    length = CanMonitorAppendHex(line, length, frame->id & 0x7FFU, 3U);
  }
  line[length++] = '#';

  if (frame->is_remote != 0U)
  {
    line[length++] = 'R';
    length = CanMonitorAppendHex(line, length, frame->dlc, 1U);
  }
  else
  {
    for (uint8_t i = 0U; i < frame->dlc; i++)
    {
      length = CanMonitorAppendHex(line, length, frame->data[i], 2U);
    }
  }

  line[length++] = '\r';
  line[length++] = '\n';
  if (HAL_UART_Transmit(&huart2, (uint8_t *)line, length, HAL_MAX_DELAY) == HAL_OK)
  {
    can_monitor_output_total++;
  }
  else
  {
    can_monitor_uart_drop_total++;
  }
}

static void CanMonitorWriteStats(void)
{
  uint32_t now = HAL_GetTick();
  if ((now - can_monitor_stats_last_tick_ms) < CAN_MONITOR_STATS_INTERVAL_MS)
  {
    return;
  }
  can_monitor_stats_last_tick_ms = now;

  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  uint32_t rx_total = can_monitor_rx_total;
  uint32_t queue_drop_total = can_monitor_queue_drop_total;
  uint32_t fifo_full_total = can_monitor_fifo_full_total;
  uint32_t fifo_overrun_total = can_monitor_fifo_overrun_total;
  uint32_t read_error_total = can_monitor_read_error_total;
  uint32_t can_error_total = can_monitor_can_error_total;
  uint16_t queued = CanMonitorQueueDepthUnsafe();
  uint16_t queue_peak = can_monitor_queue_peak;
  if (primask == 0U)
  {
    __enable_irq();
  }

  char line[224];
  int length = snprintf(line, sizeof(line),
                        "STAT rx=%lu output=%lu queued=%u queue_drop=%lu fifo_overrun=%lu uart_drop=%lu fifo_full=%lu read_error=%lu can_error=%lu queue_peak=%u\r\n",
                        (unsigned long)rx_total,
                        (unsigned long)can_monitor_output_total,
                        (unsigned int)queued,
                        (unsigned long)queue_drop_total,
                        (unsigned long)fifo_overrun_total,
                        (unsigned long)can_monitor_uart_drop_total,
                        (unsigned long)fifo_full_total,
                        (unsigned long)read_error_total,
                        (unsigned long)can_error_total,
                        (unsigned int)queue_peak);
  if ((length > 0) && ((size_t)length < sizeof(line)))
  {
    (void)HAL_UART_Transmit(&huart2, (uint8_t *)line, (uint16_t)length, HAL_MAX_DELAY);
  }
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_CAN_Init();
  /* USER CODE BEGIN 2 */
  if (HAL_CAN_Start(&hcan) != HAL_OK)
  {
    static const char message[] = "CAN start failed\r\n";
    (void)HAL_UART_Transmit(&huart2, (uint8_t *)message, sizeof(message) - 1U, HAL_MAX_DELAY);
    Error_Handler();
  }
  if (HAL_CAN_ActivateNotification(&hcan,
                                   CAN_IT_RX_FIFO0_MSG_PENDING |
                                       CAN_IT_RX_FIFO0_FULL |
                                       CAN_IT_RX_FIFO0_OVERRUN) != HAL_OK)
  {
    static const char message[] = "CAN notification setup failed\r\n";
    (void)HAL_UART_Transmit(&huart2, (uint8_t *)message, sizeof(message) - 1U, HAL_MAX_DELAY);
    Error_Handler();
  }
  can_monitor_stats_last_tick_ms = HAL_GetTick();
  static const char ready_message[] = "CAN monitor ready (CAN 1000000 bit/s, UART 115200 baud)\r\n";
  (void)HAL_UART_Transmit(&huart2, (uint8_t *)ready_message, sizeof(ready_message) - 1U, HAL_MAX_DELAY);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    CanMonitorFrame frame;
    if (CanMonitorPop(&frame) != 0U)
    {
      CanMonitorWriteFrame(&frame);
    }
    CanMonitorWriteStats();
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
 * @brief CAN Initialization Function
 * @param None
 * @retval None
 */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN;
  hcan.Init.Prescaler = 3;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  // hcan.Init.Mode = CAN_MODE_LOOPBACK; // ループバック
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_9TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */
  // 受信フィルタ: 全受信（FIFO0へ）
  CAN_FilterTypeDef sFilterConfig = {0};
  sFilterConfig.FilterBank = 0;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterIdHigh = 0x0000;
  sFilterConfig.FilterIdLow = 0x0000;
  sFilterConfig.FilterMaskIdHigh = 0x0000;
  sFilterConfig.FilterMaskIdLow = 0x0000;
  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.SlaveStartFilterBank = 14; // 単CANでも無害
  if (HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE END CAN_Init 2 */
}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA0 PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *can_handle)
{
  while (HAL_CAN_GetRxFifoFillLevel(can_handle, CAN_RX_FIFO0) > 0U)
  {
    CAN_RxHeaderTypeDef header;
    uint8_t data[8] = {0U};
    if (HAL_CAN_GetRxMessage(can_handle, CAN_RX_FIFO0, &header, data) != HAL_OK)
    {
      can_monitor_read_error_total++;
      break;
    }
    CanMonitorEnqueue(&header, data);
  }
}

void HAL_CAN_RxFifo0FullCallback(CAN_HandleTypeDef *can_handle)
{
  (void)can_handle;
  can_monitor_fifo_full_total++;
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *can_handle)
{
  uint32_t error = HAL_CAN_GetError(can_handle);
  if ((error & HAL_CAN_ERROR_RX_FOV0) != 0U)
  {
    can_monitor_fifo_overrun_total++;
  }
  if ((error & ~HAL_CAN_ERROR_RX_FOV0) != HAL_CAN_ERROR_NONE)
  {
    can_monitor_can_error_total++;
  }
  (void)HAL_CAN_ResetError(can_handle);
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
