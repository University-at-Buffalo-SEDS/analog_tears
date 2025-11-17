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
#include "ad7193.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define CHANNELS 2
#define READY_MASK 0x03
#define VREF 3.3

typedef struct __attribute__((packed))
{
    uint8_t  header;        // 0xAC
    uint8_t  sequence;      // Rolling counter
    uint32_t timestamp;     // HAL_GetTick()
    uint8_t  ad7193_data[CHANNELS][3]; // 24-bit channel (MSB first)
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
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;

PCD_HandleTypeDef hpcd_USB_FS;

AD7193_HandleTypeDef hadc7193;

/* USER CODE BEGIN PV */
uint8_t ack_buffer[4];
volatile uint8_t uart_busy = 0;

volatile uint8_t cmd_rdy = 0;
uint8_t cmd_buf[UART_BUFFER_LEN];

volatile uint8_t ad7193_ready = 0;
volatile uint16_t internal_adc_buffer[1];
uint8_t fresh_mask = 0;
uint8_t sequence = 0;
uint8_t ad7193_buffer[CHANNELS][3];
DataPacket_t data_packet;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USB_PCD_Init(void);
static void MX_ADC1_Init(void);
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

uint16_t computeCRC_16(const uint8_t *data, uint16_t len)
{
  // TODO
}

void sendAck(uint8_t cmd, GPIO_PinState state)
{
    while (uart_busy);
    uart_busy = 1;
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
  // Initialize AD7193 driver with configurable pins
  if (AD7193_Init(&hadc7193, &hspi1, 
                   ADC_CS_GPIO_Port, ADC_CS_Pin,
                   DATA_READY_GPIO_Port, DATA_READY_Pin,
                   VREF) != HAL_OK) {
    Error_Handler();
  }

  // Reset AD7193
  if (AD7193_Reset(&hadc7193) != HAL_OK) {
    Error_Handler();
  }

  // Configure AD7193 registers
  uint32_t config = 
    AD7193_CONFIG_CHOP(0) |        // CHOP = 0 (chop disabled for faster conversion)
    AD7193_CONFIG_REFSEL(0) |      // REFSEL = 0 (use REFIN1)
    AD7193_CONFIG_PSEUDO(0) |      // Pseudo
    AD7193_CONFIG_SHORT(0) |       // Short 
    AD7193_CONFIG_TEMP(0) |        // TEMP 
    AD7193_CONFIG_CH7(0) |         // CH7 
    AD7193_CONFIG_CH6(0) |         // CH6 
    AD7193_CONFIG_CH5(0) |         // CH5 
    AD7193_CONFIG_CH4(0) |         // CH4 
    AD7193_CONFIG_CH3(1) |         // CH3 
    AD7193_CONFIG_CH2(1) |         // CH2 
    AD7193_CONFIG_CH1(0) |         // CH1 
    AD7193_CONFIG_CH0(0) |         // CH0 (AIN1-AIN2 pair enabled)
    AD7193_CONFIG_BURN(0) |        // BURN = 0
    AD7193_CONFIG_REFDET(0) |      // REFDET = 0
    AD7193_CONFIG_BUF(1) |         // BUF (buffer enabled) // HAS TO BE ENABLED FOR ADC TO WORK
    AD7193_CONFIG_UB(0) |          // U/B  (When this bit is set, unipolar operation is selected.) // HAS TO BE bipolar FOR ADC TO WORK
    AD7193_CONFIG_G2(1) |          // G2
    AD7193_CONFIG_G1(1) |          // G1  
    AD7193_CONFIG_G0(1);           // G0

  // Configure mode register for continuous conversion
  uint32_t mode = 
    AD7193_MODE_MD2(0) |           // MD2 = 0
    AD7193_MODE_MD1(0) |           // MD1 = 0
    AD7193_MODE_MD0(0) |           // MD0 = 0 (Continuous conversion mode)
    AD7193_MODE_DAT_STA(1) |       // DAT_STA = 1
    AD7193_MODE_CLK1(1) |          // CLK1 = 1
    AD7193_MODE_CLK0(0) |          // CLK0 = 0 (Internal clock)
    AD7193_MODE_AVG1(0) |          // AVG1 = 0
    AD7193_MODE_AVG0(0) |          // AVG0 = 0 (no averaging)
    AD7193_MODE_SINC3(0) |         // SINC3 = 0 (use SINC4 filter)
    AD7193_MODE_ENPAR(0) |         // ENPAR = 0
    AD7193_MODE_CLK_DIV(0) |       // CLK_DIV = 0
    AD7193_MODE_SINGLE(0) |        // Single = 0
    AD7193_MODE_REJ60(0) |         // REJ60 = 0
    AD7193_MODE_FS(48);            // FS[9:0] = 96 (50Hz output rate)

  AD7193_Configure(&hadc7193, config, mode);
  AD7193_EnableContinuousRead(&hadc7193);
  AD7193_SetCSlowForTheInterruptToWork(&hadc7193);
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
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_USB_PCD_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  ad7193_init();
  HAL_UART_Receive_IT(&huart1, uart_rx_buf, UART_BUFFER_LEN);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)internal_adc_buffer, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    handleIgniterSequence();
    if (cmd_rdy)
    {
      // atomic clear
      __disable_irq();
      cmd_rdy = 0;
      uint8_t cmd = cmd_buf[1];
      uint8_t val = cmd_buf[2];
      __enable_irq();
      handleCommand(cmd, val);
    }
    if (ad7193_ready)
    {
      ad7193_ready = 0;
      /**
      1. Read over SPI when interrupt is triggered
      2. Identify channel -> curr_channel
      3. Store three bytes in ad7193_buffer[curr_channel]
      4. Update fresh_mask for the current channel (fresh_mask |= 1 << curr_channel)
      5. Check if fresh_mask == READY_MASK
         a. Collect internal ADC data from DMA buffer
         b. Send packet over non-blocking call
         c. Reset fresh_mask flag
      */
    }
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_USB;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  * @brief USB Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_PCD_Init(void)
{

  /* USER CODE BEGIN USB_Init 0 */

  /* USER CODE END USB_Init 0 */

  /* USER CODE BEGIN USB_Init 1 */

  /* USER CODE END USB_Init 1 */
  hpcd_USB_FS.Instance = USB;
  hpcd_USB_FS.Init.dev_endpoints = 8;
  hpcd_USB_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_FS.Init.battery_charging_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_Init 2 */

  /* USER CODE END USB_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  HAL_GPIO_WritePin(ADC_CS_GPIO_Port, ADC_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, PILOT_VALVE_Pin|TANKS_Pin|IGINITER_Pin|SPARE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : ADC_CS_Pin */
  GPIO_InitStruct.Pin = ADC_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(ADC_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PILOT_VALVE_Pin TANKS_Pin IGINITER_Pin SPARE_Pin */
  GPIO_InitStruct.Pin = PILOT_VALVE_Pin|TANKS_Pin|IGINITER_Pin|SPARE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : DATA_READY_Pin */
  GPIO_InitStruct.Pin = DATA_READY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(DATA_READY_GPIO_Port, &GPIO_InitStruct);

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
    uart_busy = 0;
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == DATA_READY_Pin)
  {
    ad7193_ready = 1;
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
