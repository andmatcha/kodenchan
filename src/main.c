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
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
{
  uint8_t seq;
  uint8_t flags;
  uint16_t current[7];
  uint8_t extra_flags;
} PacketAC_v3;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define AC_PACKET_V3_LEN 31U
#define AC_PACKET_V6_LEN 39U
#define AC_STREAM_BUFFER_LEN 96U
#define UART_RX_TIMEOUT_MS 1U
#define SHORT_PACKET_IDLE_MS 3U

#define MODE_MANUAL 1U

#define MANUAL_UPLINK_CANID_1 0x200U
#define MANUAL_UPLINK_CANID_2 0x201U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
static uint8_t uart_stream_buffer[AC_STREAM_BUFFER_LEN];
static uint32_t uart_stream_length = 0U;
static uint32_t last_uart_rx_tick = 0U;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_CAN_Init(void);
/* USER CODE BEGIN PFP */
static bool PollACPacket(uint8_t *packet, uint32_t *packet_length);
static bool ExtractACPacket(uint8_t *packet, uint32_t *packet_length, bool allow_short_packet);
static void LoadPacketAC(PacketAC_v3 *packet, const uint8_t *raw_packet);
static void TransmitPacketAsCan(const PacketAC_v3 *packet);
static HAL_StatusTypeDef SendCanFrame(uint16_t std_id, const uint8_t *data);
static void PrintCanTxLine(uint16_t std_id, const uint8_t *data);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static uint16_t ReadU16LE(const uint8_t *data)
{
  return (uint16_t)data[0] | ((uint16_t)data[1] << 8);
}

static void WriteU16LE(uint8_t *data, uint16_t value)
{
  data[0] = (uint8_t)(value & 0xFFU);
  data[1] = (uint8_t)((value >> 8) & 0xFFU);
}

static void PrintCanTxLine(uint16_t std_id, const uint8_t *data)
{
  printf("CAN TX 0x%03X:", std_id);

  for (uint32_t i = 0; i < 8U; ++i)
  {
    printf(" %02X", data[i]);
  }

  printf("\r\n");
}

static uint16_t CRC16CcittFalse(const uint8_t *data, uint32_t length)
{
  uint16_t crc = 0xFFFFU;

  for (uint32_t i = 0; i < length; ++i)
  {
    crc ^= (uint16_t)data[i] << 8;

    for (uint32_t bit = 0; bit < 8; ++bit)
    {
      if ((crc & 0x8000U) != 0U)
      {
        crc = (uint16_t)((crc << 1) ^ 0x1021U);
      }
      else
      {
        crc <<= 1;
      }
    }
  }

  return crc;
}

static void ShiftStreamBuffer(uint32_t count)
{
  if (count >= uart_stream_length)
  {
    uart_stream_length = 0U;
    return;
  }

  memmove(uart_stream_buffer, uart_stream_buffer + count, uart_stream_length - count);
  uart_stream_length -= count;
}

static bool StreamHasHeader(void)
{
  for (uint32_t i = 0; i + 1U < uart_stream_length; ++i)
  {
    if ((uart_stream_buffer[i] == 'A') && (uart_stream_buffer[i + 1U] == 'C'))
    {
      if (i != 0U)
      {
        ShiftStreamBuffer(i);
      }
      return true;
    }
  }

  if (uart_stream_length > 1U)
  {
    uint8_t tail = uart_stream_buffer[uart_stream_length - 1U];
    uart_stream_buffer[0] = tail;
    uart_stream_length = (tail == 'A') ? 1U : 0U;
  }

  return false;
}

static bool PacketHasValidV6Crc(const uint8_t *packet)
{
  uint16_t expected_crc = ReadU16LE(packet + (AC_PACKET_V6_LEN - 2U));
  uint16_t actual_crc = CRC16CcittFalse(packet, AC_PACKET_V6_LEN - 2U);
  return expected_crc == actual_crc;
}

static bool ExtractACPacket(uint8_t *packet, uint32_t *packet_length, bool allow_short_packet)
{
  while (uart_stream_length >= 2U)
  {
    if (!StreamHasHeader())
    {
      return false;
    }

    if (uart_stream_length < AC_PACKET_V3_LEN)
    {
      return false;
    }

    if ((uart_stream_length >= AC_PACKET_V6_LEN) && PacketHasValidV6Crc(uart_stream_buffer))
    {
      memcpy(packet, uart_stream_buffer, AC_PACKET_V6_LEN);
      *packet_length = AC_PACKET_V6_LEN;
      ShiftStreamBuffer(AC_PACKET_V6_LEN);
      return true;
    }

    if ((uart_stream_length >= (AC_PACKET_V3_LEN + 2U)) &&
        (uart_stream_buffer[AC_PACKET_V3_LEN] == 'A') &&
        (uart_stream_buffer[AC_PACKET_V3_LEN + 1U] == 'C'))
    {
      memcpy(packet, uart_stream_buffer, AC_PACKET_V3_LEN);
      *packet_length = AC_PACKET_V3_LEN;
      ShiftStreamBuffer(AC_PACKET_V3_LEN);
      return true;
    }

    if (allow_short_packet)
    {
      memcpy(packet, uart_stream_buffer, AC_PACKET_V3_LEN);
      *packet_length = AC_PACKET_V3_LEN;
      ShiftStreamBuffer(AC_PACKET_V3_LEN);
      return true;
    }

    return false;
  }

  return false;
}

static bool PollACPacket(uint8_t *packet, uint32_t *packet_length)
{
  uint8_t byte = 0U;
  HAL_StatusTypeDef status = HAL_UART_Receive(&huart2, &byte, 1U, UART_RX_TIMEOUT_MS);

  if (status == HAL_OK)
  {
    if (uart_stream_length >= AC_STREAM_BUFFER_LEN)
    {
      ShiftStreamBuffer(1U);
    }

    uart_stream_buffer[uart_stream_length++] = byte;
    last_uart_rx_tick = HAL_GetTick();

    return ExtractACPacket(packet, packet_length, false);
  }

  if ((uart_stream_length >= AC_PACKET_V3_LEN) &&
      ((HAL_GetTick() - last_uart_rx_tick) >= SHORT_PACKET_IDLE_MS))
  {
    return ExtractACPacket(packet, packet_length, true);
  }

  return false;
}

static void LoadPacketAC(PacketAC_v3 *packet, const uint8_t *raw_packet)
{
  packet->seq = raw_packet[2];
  packet->flags = raw_packet[3];

  for (uint32_t i = 0; i < 7U; ++i)
  {
    packet->current[i] = ReadU16LE(raw_packet + 4U + (i * 2U));
  }

  packet->extra_flags = raw_packet[30];
}

static HAL_StatusTypeDef SendCanFrame(uint16_t std_id, const uint8_t *data)
{
  CAN_TxHeaderTypeDef tx_header = {0};
  uint32_t tx_mailbox = 0U;
  uint32_t start_tick = HAL_GetTick();
  HAL_StatusTypeDef status = HAL_OK;

  tx_header.StdId = std_id;
  tx_header.IDE = CAN_ID_STD;
  tx_header.RTR = CAN_RTR_DATA;
  tx_header.DLC = 8U;
  tx_header.TransmitGlobalTime = DISABLE;

  while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0U)
  {
    if ((HAL_GetTick() - start_tick) > 10U)
    {
      return HAL_TIMEOUT;
    }
  }

  status = HAL_CAN_AddTxMessage(&hcan, &tx_header, (uint8_t *)data, &tx_mailbox);
  if (status == HAL_OK)
  {
    PrintCanTxLine(std_id, data);
  }

  return status;
}

static void TransmitManualPacket(const PacketAC_v3 *packet)
{
  uint8_t frame[8] = {0};

  WriteU16LE(&frame[0], packet->current[0]);
  WriteU16LE(&frame[2], packet->current[1]);
  WriteU16LE(&frame[4], packet->current[2]);
  WriteU16LE(&frame[6], packet->current[3]);
  if (SendCanFrame(MANUAL_UPLINK_CANID_1, frame) != HAL_OK)
  {
    printf("Failed CAN TX: 0x%03X\r\n", MANUAL_UPLINK_CANID_1);
  }

  memset(frame, 0, sizeof(frame));
  WriteU16LE(&frame[0], packet->current[4]);
  WriteU16LE(&frame[2], packet->current[5]);
  WriteU16LE(&frame[4], packet->current[6]);
  frame[6] = packet->extra_flags;
  if (SendCanFrame(MANUAL_UPLINK_CANID_2, frame) != HAL_OK)
  {
    printf("Failed CAN TX: 0x%03X\r\n", MANUAL_UPLINK_CANID_2);
  }
}

static void TransmitPacketAsCan(const PacketAC_v3 *packet)
{
  uint8_t mode = (packet->flags >> 4) & 0x03U;

  if (mode == MODE_MANUAL)
  {
    TransmitManualPacket(packet);
    return;
  }

  printf("Ignored non-manual mode: %u\r\n", mode);
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
    printf("CAN Start failed\r\n");
    Error_Handler();
  }
  printf("Bridge ready: USART2 57600 -> CAN 1Mbps\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    uint8_t raw_packet[AC_PACKET_V6_LEN] = {0};
    uint32_t raw_packet_length = 0U;
    PacketAC_v3 packet = {0};

    if (!PollACPacket(raw_packet, &raw_packet_length))
    {
      continue;
    }

    if ((raw_packet[0] != 'A') || (raw_packet[1] != 'C'))
    {
      continue;
    }

    LoadPacketAC(&packet, raw_packet);
    TransmitPacketAsCan(&packet);
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
  huart2.Init.BaudRate = 57600;
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
