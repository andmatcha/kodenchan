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
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CAN_CHUNK_SIZE 8U
#define ROVER_CAN_BASE_ID 0x300U
#define ARM_CAN_BASE_ID 0x203U
#define ROVER_PERIOD_MS 100U
#define ARM_PERIOD_MS 50U
#define CAN_TX_TIMEOUT_MS 10U
#define ROVER_DUMMY_BUFFER_SIZE 64U
#define ARM_DUMMY_FRAME_SIZE 16U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
static uint32_t rover_sequence = 0U;
static uint32_t arm_sequence = 0U;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_CAN_Init(void);
/* USER CODE BEGIN PFP */
static HAL_StatusTypeDef CAN_SendBytes(uint16_t std_id, const uint8_t *data, uint8_t len);
static HAL_StatusTypeDef CAN_SendChunked(uint16_t base_id, const uint8_t *data, uint8_t len);
static uint8_t BuildRoverDummyFrame(uint8_t *buffer, size_t buffer_size, uint32_t sequence);
static void BuildArmDummyFrame(uint8_t *buffer, uint32_t sequence);
static void LogRoverPayload(const uint8_t *data, uint8_t len);
static void LogArmPayload(const uint8_t *data, uint8_t len);
static void ServiceDummyCanTraffic(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
    printf("CAN Start failed\r\n");
    Error_Handler();
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    ServiceDummyCanTraffic();
    HAL_Delay(1);
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
  huart2.Init.BaudRate = 38400;
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
static HAL_StatusTypeDef CAN_SendBytes(uint16_t std_id, const uint8_t *data, uint8_t len)
{
  CAN_TxHeaderTypeDef tx_header = {0};
  uint32_t tx_mailbox = 0U;
  uint32_t start_tick = HAL_GetTick();

  while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0U)
  {
    if ((HAL_GetTick() - start_tick) >= CAN_TX_TIMEOUT_MS)
    {
      return HAL_TIMEOUT;
    }
  }

  tx_header.StdId = std_id;
  tx_header.IDE = CAN_ID_STD;
  tx_header.RTR = CAN_RTR_DATA;
  tx_header.DLC = len;
  tx_header.TransmitGlobalTime = DISABLE;

  return HAL_CAN_AddTxMessage(&hcan, &tx_header, (uint8_t *)data, &tx_mailbox);
}

static HAL_StatusTypeDef CAN_SendChunked(uint16_t base_id, const uint8_t *data, uint8_t len)
{
  uint8_t offset = 0U;
  uint16_t frame_id = base_id;

  while (offset < len)
  {
    HAL_StatusTypeDef status = HAL_OK;
    uint8_t chunk_len = len - offset;

    if (chunk_len > CAN_CHUNK_SIZE)
    {
      chunk_len = CAN_CHUNK_SIZE;
    }

    status = CAN_SendBytes(frame_id, &data[offset], chunk_len);
    if (status != HAL_OK)
    {
      return status;
    }

    offset += chunk_len;
    frame_id++;
  }

  return HAL_OK;
}

static uint8_t BuildRoverDummyFrame(uint8_t *buffer, size_t buffer_size, uint32_t sequence)
{
  int written = 0;
  size_t payload_len = 0U;
  uint32_t rover_id = ROVER_CAN_BASE_ID + (sequence % 4U);
  uint32_t rover_value = (sequence * 123456U) % 1000000U;

  written = snprintf((char *)buffer,
                     buffer_size,
                     "0x%03lX, %06lu",
                     (unsigned long)rover_id,
                     (unsigned long)rover_value);

  if (written < 0)
  {
    return 0U;
  }

  payload_len = (size_t)written;
  if (payload_len > (buffer_size - 2U))
  {
    payload_len = buffer_size - 2U;
  }

  buffer[payload_len++] = '\n';
  buffer[payload_len++] = '\r';

  return (uint8_t)payload_len;
}

static void BuildArmDummyFrame(uint8_t *buffer, uint32_t sequence)
{
  uint8_t index = 0U;

  buffer[0] = 'J';
  buffer[1] = 'F';

  for (index = 2U; index < ARM_DUMMY_FRAME_SIZE; index++)
  {
    buffer[index] = (uint8_t)(sequence + (uint32_t)(index * 13U));
  }
}

static void LogRoverPayload(const uint8_t *data, uint8_t len)
{
  uint8_t index = 0U;

  printf("Rover payload: \"");
  for (index = 0U; index < len; index++)
  {
    if (data[index] == '\n')
    {
      printf("\\n");
    }
    else if (data[index] == '\r')
    {
      printf("\\r");
    }
    else
    {
      printf("%c", data[index]);
    }
  }
  printf("\"\r\n");
}

static void LogArmPayload(const uint8_t *data, uint8_t len)
{
  uint8_t index = 0U;

  printf("Arm payload:");
  for (index = 0U; index < len; index++)
  {
    printf(" %02X", data[index]);
  }
  printf("\r\n");
}

static void ServiceDummyCanTraffic(void)
{
  static uint32_t last_rover_tick = 0U;
  static uint32_t last_arm_tick = 0U;
  uint32_t now = HAL_GetTick();

  if ((last_rover_tick == 0U) || ((now - last_rover_tick) >= ROVER_PERIOD_MS))
  {
    uint8_t rover_payload[ROVER_DUMMY_BUFFER_SIZE];
    uint8_t rover_length = BuildRoverDummyFrame(rover_payload, sizeof(rover_payload), rover_sequence);

    if ((rover_length > 0U) &&
        (CAN_SendChunked(ROVER_CAN_BASE_ID, rover_payload, rover_length) == HAL_OK))
    {
      LogRoverPayload(rover_payload, rover_length);
      rover_sequence++;
    }
    else
    {
      printf("Rover dummy send failed\r\n");
    }

    last_rover_tick = now;
  }

  if ((last_arm_tick == 0U) || ((now - last_arm_tick) >= ARM_PERIOD_MS))
  {
    uint8_t arm_payload[ARM_DUMMY_FRAME_SIZE];

    BuildArmDummyFrame(arm_payload, arm_sequence);
    if (CAN_SendChunked(ARM_CAN_BASE_ID, arm_payload, sizeof(arm_payload)) == HAL_OK)
    {
      LogArmPayload(arm_payload, sizeof(arm_payload));
      arm_sequence++;
    }
    else
    {
      printf("Arm dummy send failed\r\n");
    }

    last_arm_tick = now;
  }
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
    printf("Error occurred!\r\n");
    HAL_Delay(1000);
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
