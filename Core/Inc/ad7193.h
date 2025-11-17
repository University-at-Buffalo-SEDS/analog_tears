/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    ad7193.h
 * @brief   AD7193 ADC driver header file
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 ******************************************************************************
 */
/* USER CODE END Header */

#ifndef __AD7193_H__
#define __AD7193_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"

/* Exported types ------------------------------------------------------------*/
typedef struct {
  SPI_HandleTypeDef *hspi;
  GPIO_TypeDef *cs_port;
  uint16_t cs_pin;
  GPIO_TypeDef *interrupt_port;
  uint16_t interrupt_pin;
  float vref;
} AD7193_HandleTypeDef;

/* Exported constants --------------------------------------------------------*/
// Register addresses
#define AD7193_COMM_REG_ADDR 0x00
#define AD7193_STATUS_REG_ADDR 0x00 // Same as comm reg but for read
#define AD7193_MODE_REG_ADDR 0x01
#define AD7193_CONFIG_REG_ADDR 0x02
#define AD7193_DATA_REG_ADDR 0x03

// Status register bits
#define AD7193_STATUS_RDY (1 << 7) // RDY bit (inverted: 0 = ready)

// Configuration register bit masks
#define AD7193_CONFIG_CHOP_SHIFT 23
#define AD7193_CONFIG_REFSEL_SHIFT 20
#define AD7193_CONFIG_PSEUDO_SHIFT 18
#define AD7193_CONFIG_SHORT_SHIFT 17
#define AD7193_CONFIG_TEMP_SHIFT 16
#define AD7193_CONFIG_CH7_SHIFT 15
#define AD7193_CONFIG_CH6_SHIFT 14
#define AD7193_CONFIG_CH5_SHIFT 13
#define AD7193_CONFIG_CH4_SHIFT 12
#define AD7193_CONFIG_CH3_SHIFT 11
#define AD7193_CONFIG_CH2_SHIFT 10
#define AD7193_CONFIG_CH1_SHIFT 9
#define AD7193_CONFIG_CH0_SHIFT 8
#define AD7193_CONFIG_BURN_SHIFT 7
#define AD7193_CONFIG_REFDET_SHIFT 6
#define AD7193_CONFIG_BUF_SHIFT 4
#define AD7193_CONFIG_UB_SHIFT 3
#define AD7193_CONFIG_G2_SHIFT 2
#define AD7193_CONFIG_G1_SHIFT 1
#define AD7193_CONFIG_G0_SHIFT 0

// Configuration register bit values
#define AD7193_CONFIG_CHOP(x) ((x) << AD7193_CONFIG_CHOP_SHIFT)
#define AD7193_CONFIG_REFSEL(x) ((x) << AD7193_CONFIG_REFSEL_SHIFT)
#define AD7193_CONFIG_PSEUDO(x) ((x) << AD7193_CONFIG_PSEUDO_SHIFT)
#define AD7193_CONFIG_SHORT(x) ((x) << AD7193_CONFIG_SHORT_SHIFT)
#define AD7193_CONFIG_TEMP(x) ((x) << AD7193_CONFIG_TEMP_SHIFT)
#define AD7193_CONFIG_CH7(x) ((x) << AD7193_CONFIG_CH7_SHIFT)
#define AD7193_CONFIG_CH6(x) ((x) << AD7193_CONFIG_CH6_SHIFT)
#define AD7193_CONFIG_CH5(x) ((x) << AD7193_CONFIG_CH5_SHIFT)
#define AD7193_CONFIG_CH4(x) ((x) << AD7193_CONFIG_CH4_SHIFT)
#define AD7193_CONFIG_CH3(x) ((x) << AD7193_CONFIG_CH3_SHIFT)
#define AD7193_CONFIG_CH2(x) ((x) << AD7193_CONFIG_CH2_SHIFT)
#define AD7193_CONFIG_CH1(x) ((x) << AD7193_CONFIG_CH1_SHIFT)
#define AD7193_CONFIG_CH0(x) ((x) << AD7193_CONFIG_CH0_SHIFT)
#define AD7193_CONFIG_BURN(x) ((x) << AD7193_CONFIG_BURN_SHIFT)
#define AD7193_CONFIG_REFDET(x) ((x) << AD7193_CONFIG_REFDET_SHIFT)
#define AD7193_CONFIG_BUF(x) ((x) << AD7193_CONFIG_BUF_SHIFT)
#define AD7193_CONFIG_UB(x) ((x) << AD7193_CONFIG_UB_SHIFT)
#define AD7193_CONFIG_G2(x) ((x) << AD7193_CONFIG_G2_SHIFT)
#define AD7193_CONFIG_G1(x) ((x) << AD7193_CONFIG_G1_SHIFT)
#define AD7193_CONFIG_G0(x) ((x) << AD7193_CONFIG_G0_SHIFT)

// Mode register bit masks
#define AD7193_MODE_MD2_SHIFT 23
#define AD7193_MODE_MD1_SHIFT 22
#define AD7193_MODE_MD0_SHIFT 21
#define AD7193_MODE_DAT_STA_SHIFT 20
#define AD7193_MODE_CLK1_SHIFT 19
#define AD7193_MODE_CLK0_SHIFT 18
#define AD7193_MODE_AVG1_SHIFT 17
#define AD7193_MODE_AVG0_SHIFT 16
#define AD7193_MODE_SINC3_SHIFT 15
#define AD7193_MODE_ENPAR_SHIFT 13
#define AD7193_MODE_CLK_DIV_SHIFT 12
#define AD7193_MODE_SINGLE_SHIFT 11
#define AD7193_MODE_REJ60_SHIFT 10
#define AD7193_MODE_FS_SHIFT 0
#define AD7193_MODE_FS_MASK 0x3FF

// Mode register bit values
#define AD7193_MODE_MD2(x) ((x) << AD7193_MODE_MD2_SHIFT)
#define AD7193_MODE_MD1(x) ((x) << AD7193_MODE_MD1_SHIFT)
#define AD7193_MODE_MD0(x) ((x) << AD7193_MODE_MD0_SHIFT)
#define AD7193_MODE_DAT_STA(x) ((x) << AD7193_MODE_DAT_STA_SHIFT)
#define AD7193_MODE_CLK1(x) ((x) << AD7193_MODE_CLK1_SHIFT)
#define AD7193_MODE_CLK0(x) ((x) << AD7193_MODE_CLK0_SHIFT)
#define AD7193_MODE_AVG1(x) ((x) << AD7193_MODE_AVG1_SHIFT)
#define AD7193_MODE_AVG0(x) ((x) << AD7193_MODE_AVG0_SHIFT)
#define AD7193_MODE_SINC3(x) ((x) << AD7193_MODE_SINC3_SHIFT)
#define AD7193_MODE_ENPAR(x) ((x) << AD7193_MODE_ENPAR_SHIFT)
#define AD7193_MODE_CLK_DIV(x) ((x) << AD7193_MODE_CLK_DIV_SHIFT)
#define AD7193_MODE_SINGLE(x) ((x) << AD7193_MODE_SINGLE_SHIFT)
#define AD7193_MODE_REJ60(x) ((x) << AD7193_MODE_REJ60_SHIFT)
#define AD7193_MODE_FS(x) (((x) & AD7193_MODE_FS_MASK) << AD7193_MODE_FS_SHIFT)

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/

/**
 * @brief  Initialize AD7193 driver
 * @param  hadc7193: Pointer to AD7193_HandleTypeDef structure
 * @param  hspi: SPI handle
 * @param  cs_port: CS GPIO port
 * @param  cs_pin: CS GPIO pin
 * @param  interrupt_port: Interrupt GPIO port
 * @param  interrupt_pin: Interrupt GPIO pin
 * @param  vref: Reference voltage in volts
 * @retval HAL status
 */
HAL_StatusTypeDef AD7193_Init(AD7193_HandleTypeDef *hadc7193,
                              SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port,
                              uint16_t cs_pin, GPIO_TypeDef *interrupt_port,
                              uint16_t interrupt_pin, float vref);

/**
 * @brief  Reset AD7193
 * @param  hadc7193: Pointer to AD7193_HandleTypeDef structure
 * @retval HAL status
 */
HAL_StatusTypeDef AD7193_Reset(AD7193_HandleTypeDef *hadc7193);

/**
 * @brief  Write 24-bit register
 * @param  hadc7193: Pointer to AD7193_HandleTypeDef structure
 * @param  regAddr: Register address
 * @param  data: 24-bit data to write
 * @retval HAL status
 */
HAL_StatusTypeDef AD7193_WriteReg(AD7193_HandleTypeDef *hadc7193,
                                  uint8_t regAddr, uint32_t data);

/**
 * @brief  Read 24-bit register
 * @param  hadc7193: Pointer to AD7193_HandleTypeDef structure
 * @param  regAddr: Register address
 * @param  data: Pointer to store read data
 * @retval HAL status
 */
HAL_StatusTypeDef AD7193_ReadReg32(AD7193_HandleTypeDef *hadc7193,
                                   uint8_t regAddr, uint32_t *data);

/**
 * @brief  Read 8-bit register
 * @param  hadc7193: Pointer to AD7193_HandleTypeDef structure
 * @param  regAddr: Register address
 * @param  data: Pointer to store read data
 * @retval HAL status
 */
HAL_StatusTypeDef AD7193_ReadReg8(AD7193_HandleTypeDef *hadc7193,
                                  uint8_t regAddr, uint8_t *data);

/**
 * @brief  Read status register
 * @param  hadc7193: Pointer to AD7193_HandleTypeDef structure
 * @param  status: Pointer to store status byte
 * @retval HAL status
 */
HAL_StatusTypeDef AD7193_ReadStatus(AD7193_HandleTypeDef *hadc7193,
                                    uint8_t *status);

/**
 * @brief  Read data register
 * @param  hadc7193: Pointer to AD7193_HandleTypeDef structure
 * @param  data: Pointer to store 24-bit data
 * @retval HAL status
 */
HAL_StatusTypeDef AD7193_ReadData(AD7193_HandleTypeDef *hadc7193,
                                  uint32_t *data);

/**
 * @brief  Convert raw ADC value to voltage (bipolar mode)
 * @param  hadc7193: Pointer to AD7193_HandleTypeDef structure
 * @param  rawData: Raw 24-bit ADC value
 * @param  gain: Current gain setting
 * @param  voltage: Pointer to store converted voltage
 * @retval HAL status
 */
HAL_StatusTypeDef
AD7193_BipolarModeConvertToVoltage(AD7193_HandleTypeDef *hadc7193,
                                   uint32_t rawData, uint32_t gain,
                                   float *voltage);

/**
 * @brief  Configure AD7193 for continuous conversion mode
 * @param  hadc7193: Pointer to AD7193_HandleTypeDef structure
 * @param  config: Configuration register value
 * @param  mode: Mode register value
 * @retval HAL status
 */
HAL_StatusTypeDef AD7193_Configure(AD7193_HandleTypeDef *hadc7193,
                                   uint32_t config, uint32_t mode);

/**
 * @brief  Enable continuous read mode (DAT_STA bit set)
 * @param  hadc7193: Pointer to AD7193_HandleTypeDef structure
 * @retval HAL status
 */
HAL_StatusTypeDef AD7193_EnableContinuousRead(AD7193_HandleTypeDef *hadc7193);

/**
 * @brief  Set the AD7193 cs_pin LOW. This is required for the interrupt to
 * work.
 */
void AD7193_SetCSlowForTheInterruptToWork(AD7193_HandleTypeDef *hadc7193);

#ifdef __cplusplus
}
#endif

#endif /* __AD7193_H__ */
