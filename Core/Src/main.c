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
#include <stdint.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct __attribute__((packed))
{
    uint8_t  header;        // 0xAC
    uint8_t  sequence;      // Rolling counter
    uint32_t timestamp;     // HAL_GetTick()
    uint8_t  ad7193_ch0[3]; // 24-bit channel 0 (MSB first)
    uint8_t  ad7193_ch1[3]; // 24-bit channel 1 (MSB first)
    uint16_t stm32_adc;     // 16-bit internal ADC
    uint16_t crc;           // CRC-16
} DataPacket_t;

typedef enum
{
  IGNITER_IDLE,
  IGNITER_FIRING,
  IGNITER_PILOT,
  IGNITER_COMPLETE
} igniterState_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CMD_HEADER 0xAA
#define ACK_HEADER 0xAB

#define CMD_IGNITER 'I'
#define CMD_SPARE 'S'
#define CMD_TANKS 'T'
#define CMD_PILOT 'P'

#define CMD_ON 0x01
#define CMD_OFF 0x02

#define UART_BUFFER_LEN 4

#define UART_TIMEOUT 10

#define DATA_HEADER 0xAC

#define PILOT_VALVE_DELAY 5000
#define IGNITER_TURN_OFF 10000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint8_t ack_buffer[4];
volatile uint8_t ack_busy = 0;

volatile uint8_t cmd_rdy = 0;
uint8_t cmd_buf[UART_BUFFER_LEN];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t uart_rx_buf[UART_BUFFER_LEN];

uint8_t computeCRC(uint8_t *data, uint8_t len)
{
  uint8_t crc = 0;
  for (int i = 0; i < len; i++)
  {
    crc ^= data[i];
  }
  return crc;
}

void sendAck(uint8_t cmd, GPIO_PinState state)
{
    while (ack_busy);
    ack_busy = 1;
    ack_buffer[0] = ACK_HEADER;
    ack_buffer[1] = cmd;
    ack_buffer[2] = (state == GPIO_PIN_SET) ? 1 : 0;
    ack_buffer[3] = computeCRC(ack_buffer, 3);
    HAL_UART_Transmit_IT(&huart1, ack_buffer, 4);
}

volatile igniterState_t state = IGNITER_IDLE;
volatile uint32_t igniter_start = 0;

void handleCommand(uint8_t cmd, uint8_t val)
{
  GPIO_TypeDef *port = NULL;
  uint16_t pin;

  switch (cmd)
  {
    case CMD_IGNITER:
      port = IGINITER_GPIO_Port;
      pin = IGINITER_Pin;

      if (val == CMD_ON)
      {
        HAL_GPIO_WritePin(IGINITER_GPIO_Port, IGINITER_Pin, GPIO_PIN_SET);
        state = IGNITER_FIRING;
        igniter_start = HAL_GetTick();

        GPIO_PinState curr = HAL_GPIO_ReadPin(port, pin);
        sendAck(cmd, curr);
        return;
      }
      else if (val == CMD_OFF)
      {
        // abort!!!!
        HAL_GPIO_WritePin(IGINITER_GPIO_Port, IGINITER_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(PILOT_VALVE_GPIO_Port, PILOT_VALVE_Pin, GPIO_PIN_RESET);
        state = IGNITER_IDLE;

        GPIO_PinState curr = HAL_GPIO_ReadPin(port, pin);
        sendAck(cmd, curr);
        return;
      }
  }
  switch (cmd)
  {
    case CMD_PILOT:
      port = PILOT_VALVE_GPIO_Port;
      pin = PILOT_VALVE_Pin;
      break;

    case CMD_TANKS:
      port = TANKS_GPIO_Port;
      pin = TANKS_Pin;
      break;

    case CMD_SPARE:
      port = SPARE_GPIO_Port;
      pin = SPARE_Pin;
      break;

    default:
      return;
  }
  switch (val)
  {
    case CMD_ON:
      HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
      break;

    case CMD_OFF:
      HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
      break;

    default:
      return;
  }

  GPIO_PinState curr = HAL_GPIO_ReadPin(port, pin);
  sendAck(cmd, curr);
}

void handleIgniterSequence(void)
{
  uint32_t elapsed;

  switch (state)
  {
    case IGNITER_IDLE:
      break;
    case IGNITER_FIRING:
      elapsed = HAL_GetTick() - igniter_start;
      if (elapsed >= PILOT_VALVE_DELAY)
      {
        HAL_GPIO_WritePin(PILOT_VALVE_GPIO_Port, PILOT_VALVE_Pin, GPIO_PIN_SET);
        sendAck(CMD_PILOT, GPIO_PIN_SET);
        state = IGNITER_PILOT;
      }
      break;
    case IGNITER_PILOT:
      elapsed = HAL_GetTick() - igniter_start;
      if (elapsed > IGNITER_TURN_OFF)
      {
        HAL_GPIO_WritePin(IGINITER_GPIO_Port, IGINITER_Pin, GPIO_PIN_RESET);
        sendAck(CMD_IGNITER, GPIO_PIN_RESET);
        state = IGNITER_COMPLETE;
      }
      break;
    case IGNITER_COMPLETE:
      state = IGNITER_IDLE;
      break;
  }
}

void ad7193_init()
{
  // TODO
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
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  ad7193_init();
  HAL_UART_Receive_IT(&huart1, uart_rx_buf, UART_BUFFER_LEN);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    handleIgniterSequence();
    if (cmd_rdy)
    {
      cmd_rdy = 0;
      uint8_t cmd = cmd_buf[1];
      uint8_t val = cmd_buf[2];
      handleCommand(cmd, val);
    }
    /* USER CODE BEGIN 3 */
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, PILOT_VALVE_Pin|TANKS_Pin|IGINITER_Pin|SPARE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : ADC_CS_Pin */
  GPIO_InitStruct.Pin = ADC_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ADC_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PILOT_VALVE_Pin TANKS_Pin IGINITER_Pin SPARE_Pin */
  GPIO_InitStruct.Pin = PILOT_VALVE_Pin|TANKS_Pin|IGINITER_Pin|SPARE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    if (cmd_rdy == 0)
    {
      uint8_t header = uart_rx_buf[0];
      uint8_t crc = uart_rx_buf[3];
      if (header == CMD_HEADER && computeCRC(uart_rx_buf, (UART_BUFFER_LEN-1)) == crc)
      {
        memcpy(cmd_buf, uart_rx_buf, UART_BUFFER_LEN);
        cmd_rdy = 1;
      }
    }
    HAL_UART_Receive_IT(&huart1, uart_rx_buf, UART_BUFFER_LEN);
  }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    ack_busy = 0;
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  // interrupt handler for AD7193
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
