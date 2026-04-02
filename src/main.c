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

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_CAN_Init(void);
/* USER CODE BEGIN PFP */
static void CAN_SendMockArmData(uint16_t e0, uint16_t e1, uint8_t flags,
                                uint16_t e2, uint16_t e3, uint16_t e4);
static void CAN_SendMockRoverData(uint16_t id300, uint16_t id400, uint8_t base);
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
  uint16_t arm_base = 0;
  uint8_t rover_base = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      uint16_t e0 = 1000 + arm_base;
      uint16_t e1 = 2000 + arm_base;
      uint16_t e2 = 3000 + arm_base;
      uint16_t e3 = 4000 + arm_base;
      uint16_t e4 = 5000 + arm_base;
      uint8_t flags = 0x01; // READY

      CAN_SendMockArmData(e0, e1, flags, e2, e3, e4);

      CAN_SendMockRoverData(0x301, 0x401, rover_base);

      arm_base += 10;
      if (arm_base > 1000)
      {
        arm_base = 0;
      }

      rover_base += 1;

      HAL_Delay(100);

  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    
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
static void CAN_WaitTxComplete(CAN_HandleTypeDef *hcan, uint32_t mailbox)
{
  while (HAL_CAN_IsTxMessagePending(hcan, mailbox))
  {
  }
}

static void CAN_SendFrame(CAN_HandleTypeDef *hcan, uint16_t std_id, const uint8_t data[8])
{
  CAN_TxHeaderTypeDef TxHeader;
  uint32_t TxMailbox;

  TxHeader.StdId = std_id;
  TxHeader.IDE = CAN_ID_STD;
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.DLC = 8;
  TxHeader.TransmitGlobalTime = DISABLE;

  if (HAL_CAN_AddTxMessage(hcan, &TxHeader, (uint8_t *)data, &TxMailbox) != HAL_OK)
  {
    printf("TX failed: ID=0x%03X\r\n", std_id);
    Error_Handler();
  }

  CAN_WaitTxComplete(hcan, TxMailbox);
}

static void CAN_SendMockArmData(uint16_t e0, uint16_t e1, uint8_t flags,
                                uint16_t e2, uint16_t e3, uint16_t e4)
{
  uint8_t data203[8] = {0};
  uint8_t data204[8] = {0};

  // ID 0x203
  data203[0] = (uint8_t)(e0 & 0xFF);
  data203[1] = (uint8_t)((e0 >> 8) & 0xFF);
  data203[2] = (uint8_t)(e1 & 0xFF);
  data203[3] = (uint8_t)((e1 >> 8) & 0xFF);
  data203[4] = (uint8_t)(flags & 0x3F);
  data203[5] = 0x00;
  data203[6] = 0x00;
  data203[7] = 0x00;

  // ID 0x204
  data204[0] = (uint8_t)(e2 & 0xFF);
  data204[1] = (uint8_t)((e2 >> 8) & 0xFF);
  data204[2] = (uint8_t)(e3 & 0xFF);
  data204[3] = (uint8_t)((e3 >> 8) & 0xFF);
  data204[4] = (uint8_t)(e4 & 0xFF);
  data204[5] = (uint8_t)((e4 >> 8) & 0xFF);
  data204[6] = 0x00;
  data204[7] = 0x00;

  CAN_SendFrame(&hcan, 0x203, data203);
  CAN_SendFrame(&hcan, 0x204, data204);

  printf("ARM mock: 0x203/0x204 sent\r\n");
}

static void CAN_SendMockRoverData(uint16_t id300, uint16_t id400, uint8_t base)
{
  uint8_t data300[8];
  uint8_t data400[8];

  // 0x300番台の適当データ
  data300[0] = base + 0;
  data300[1] = base + 1;
  data300[2] = base + 2;
  data300[3] = base + 3;
  data300[4] = base + 4;
  data300[5] = base + 5;
  data300[6] = base + 6;
  data300[7] = base + 7;

  // 0x400番台の適当データ
  data400[0] = base + 10;
  data400[1] = base + 11;
  data400[2] = base + 12;
  data400[3] = base + 13;
  data400[4] = base + 14;
  data400[5] = base + 15;
  data400[6] = base + 16;
  data400[7] = base + 17;

  CAN_SendFrame(&hcan, id300, data300);
  CAN_SendFrame(&hcan, id400, data400);

  printf("ROVER mock: ID=0x%03X sum=%u, ID=0x%03X sum=%u\r\n",
         id300,
         data300[0] + data300[1] + data300[2] + data300[3] +
         data300[4] + data300[5] + data300[6] + data300[7],
         id400,
         data400[0] + data400[1] + data400[2] + data400[3] +
         data400[4] + data400[5] + data400[6] + data400[7]);
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
