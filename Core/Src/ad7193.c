#include "ad7193.h"
#define AD7193_SPI_TIMEOUT 1000

static void AD7193_SPI_Start(AD7193_HandleTypeDef *hadc7193);
static void AD7193_SPI_End(AD7193_HandleTypeDef *hadc7193);

/**
 * @brief  Start SPI transaction (CS LOW)
 */
static void AD7193_SPI_Start(AD7193_HandleTypeDef *hadc7193) {
  HAL_GPIO_WritePin(hadc7193->cs_port, hadc7193->cs_pin, GPIO_PIN_RESET);
}

/**
 * @brief  End SPI transaction (CS HIGH)
 */
static void AD7193_SPI_End(AD7193_HandleTypeDef *hadc7193) {
  HAL_GPIO_WritePin(hadc7193->cs_port, hadc7193->cs_pin, GPIO_PIN_SET);
}

/**
 * @brief  Initialize AD7193 driver
 */
HAL_StatusTypeDef AD7193_Init(AD7193_HandleTypeDef *hadc7193, SPI_HandleTypeDef *hspi,
                              GPIO_TypeDef *cs_port, uint16_t cs_pin, GPIO_TypeDef *interrupt_port,
                              uint16_t interrupt_pin, float vref, uint32_t gain) {
  if (hadc7193 == NULL || hspi == NULL) {
    return HAL_ERROR;
  }

  // Store configuration
  hadc7193->hspi = hspi;
  hadc7193->cs_port = cs_port;
  hadc7193->cs_pin = cs_pin;
  hadc7193->interrupt_port = interrupt_port;
  hadc7193->interrupt_pin = interrupt_pin;
  hadc7193->vref = vref;
  hadc7193->gain = gain;

  // Initialize CS pin (ensure it's high/inactive)
  HAL_GPIO_WritePin(hadc7193->cs_port, hadc7193->cs_pin, GPIO_PIN_SET);

  return HAL_OK;
}

/**
 * @brief  Reset AD7193
 */
HAL_StatusTypeDef AD7193_Reset(AD7193_HandleTypeDef *hadc7193) {
  uint8_t reset_bytes[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

  AD7193_SPI_Start(hadc7193);
  HAL_StatusTypeDef status =
      HAL_SPI_Transmit(hadc7193->hspi, reset_bytes, sizeof(reset_bytes), AD7193_SPI_TIMEOUT);
  AD7193_SPI_End(hadc7193);

  if (status == HAL_OK) {
    HAL_Delay(500); // Wait for reset to complete
  }

  return status;
}

/**
 * @brief  Write 24-bit register
 */
HAL_StatusTypeDef AD7193_WriteReg(AD7193_HandleTypeDef *hadc7193, uint8_t regAddr, uint32_t data) {
  // Communication byte: WEN=0, R/W=0 (write), Register address, CREAD=0
  uint8_t comm_byte = (0 << 7) |                // WEN = 0
                      (0 << 6) |                // R/W = 0 (write)
                      ((regAddr & 0x07) << 3) | // Register address
                      (0 << 2) |                // CREAD = 0
                      (0 << 1) |                // 0
                      (0 << 0);                 // 0

  uint8_t tx_buffer[4];
  tx_buffer[0] = comm_byte;
  tx_buffer[1] = (data >> 16) & 0xFF;
  tx_buffer[2] = (data >> 8) & 0xFF;
  tx_buffer[3] = data & 0xFF;

  AD7193_SPI_Start(hadc7193);
  HAL_StatusTypeDef status =
      HAL_SPI_Transmit(hadc7193->hspi, tx_buffer, sizeof(tx_buffer), AD7193_SPI_TIMEOUT);
  AD7193_SPI_End(hadc7193);

  return status;
}

/**
 * @brief  Read 24-bit register
 */
HAL_StatusTypeDef AD7193_ReadReg32(AD7193_HandleTypeDef *hadc7193, uint8_t regAddr,
                                   uint32_t *data) {
  if (data == NULL) {
    return HAL_ERROR;
  }

  // Communication byte: WEN=0, R/W=1 (read), Register address, CREAD=0
  uint8_t comm_byte = (0 << 7) |                // WEN = 0
                      (1 << 6) |                // R/W = 1 (read)
                      ((regAddr & 0x07) << 3) | // Register address
                      (0 << 2) |                // CREAD = 0
                      (0 << 1) |                // 0
                      (0 << 0);                 // 0

  uint8_t rx_buffer[3] = {0x00, 0x00, 0x00};

  AD7193_SPI_Start(hadc7193);

  // Send command byte
  if (HAL_SPI_Transmit(hadc7193->hspi, &comm_byte, 1, AD7193_SPI_TIMEOUT) != HAL_OK) {
    AD7193_SPI_End(hadc7193);
    return HAL_ERROR;
  }

  // Read 24-bit data
  if (HAL_SPI_Receive(hadc7193->hspi, rx_buffer, sizeof(rx_buffer), AD7193_SPI_TIMEOUT) != HAL_OK) {
    AD7193_SPI_End(hadc7193);
    return HAL_ERROR;
  }

  AD7193_SPI_End(hadc7193);

  // Combine bytes into 24-bit value
  *data = ((uint32_t)rx_buffer[0] << 16) | ((uint32_t)rx_buffer[1] << 8) | rx_buffer[2];

  return HAL_OK;
}

/**
 * @brief  Read 8-bit register
 */
HAL_StatusTypeDef AD7193_ReadReg8(AD7193_HandleTypeDef *hadc7193, uint8_t regAddr, uint8_t *data) {
  if (data == NULL) {
    return HAL_ERROR;
  }

  // Communication byte: WEN=0, R/W=1 (read), Register address, CREAD=0
  uint8_t comm_byte = (0 << 7) |                // WEN = 0
                      (1 << 6) |                // R/W = 1 (read)
                      ((regAddr & 0x07) << 3) | // Register address
                      (0 << 2) |                // CREAD = 0
                      (0 << 1) |                // 0
                      (0 << 0);                 // 0

  uint8_t rx_byte = 0x00;

  AD7193_SPI_Start(hadc7193);

  // Send command byte
  if (HAL_SPI_Transmit(hadc7193->hspi, &comm_byte, 1, AD7193_SPI_TIMEOUT) != HAL_OK) {
    AD7193_SPI_End(hadc7193);
    return HAL_ERROR;
  }

  // Read 8-bit data
  if (HAL_SPI_Receive(hadc7193->hspi, &rx_byte, 1, AD7193_SPI_TIMEOUT) != HAL_OK) {
    AD7193_SPI_End(hadc7193);
    return HAL_ERROR;
  }

  AD7193_SPI_End(hadc7193);

  *data = rx_byte;
  return HAL_OK;
}

/**
 * @brief  Read status register
 */
HAL_StatusTypeDef AD7193_ReadStatus(AD7193_HandleTypeDef *hadc7193, uint8_t *status) {
  return AD7193_ReadReg8(hadc7193, AD7193_STATUS_REG_ADDR, status);
}

/**
 * @brief  Read data register
 */
HAL_StatusTypeDef AD7193_ReadData(AD7193_HandleTypeDef *hadc7193, uint32_t *data) {
  return AD7193_ReadReg32(hadc7193, AD7193_DATA_REG_ADDR, data);
}

/**
 * @brief  Read data register
 */
HAL_StatusTypeDef
AD7193_ReadData_WithContinuousReadMode_WithCSPinALwaysLowToMakeTheInterruptWork_WithDAT_STAModeSet(
    AD7193_HandleTypeDef *hadc7193, uint32_t *data, uint8_t *statusRegister) {

  if (data == NULL || statusRegister == NULL) {
    return HAL_ERROR;
  }

  // The DAT_STA bit in the mode register is SET.
  uint8_t rx_buffer[4] = {0x00, 0x00, 0x00, 0x00};
  if (HAL_SPI_Receive(hadc7193->hspi, rx_buffer, sizeof(rx_buffer), AD7193_SPI_TIMEOUT) != HAL_OK) {
    return HAL_ERROR;
  }

  // Combine bytes into 24-bit value
  *data = ((uint32_t)rx_buffer[0] << 16) | ((uint32_t)rx_buffer[1] << 8) | rx_buffer[2];
  *statusRegister = rx_buffer[3];

  return HAL_OK;
}

/**
 * @brief  Convert raw ADC value to voltage (bipolar mode)
 */
HAL_StatusTypeDef AD7193_BipolarModeConvertToVoltage(AD7193_HandleTypeDef *hadc7193,
                                                     uint32_t rawData, float *voltage) {
  if (voltage == NULL || hadc7193 == NULL) {
    return HAL_ERROR;
  }

  // Bipolar mode conversion formula:
  // voltage = ((rawData / 2^23) - 1) * (VREF / gain)
  *voltage = (((float)rawData / 8388608.0f) - 1.0f) * (hadc7193->vref / (float)hadc7193->gain);

  return HAL_OK;
}

/**
 * @brief  Configure AD7193 using config and mode
 */
HAL_StatusTypeDef AD7193_Configure(AD7193_HandleTypeDef *hadc7193, uint32_t config, uint32_t mode) {
  HAL_StatusTypeDef status;

  // Write configuration register
  status = AD7193_WriteReg(hadc7193, AD7193_CONFIG_REG_ADDR, config);
  if (status != HAL_OK) {
    return status;
  }
  HAL_Delay(100);

  // Write mode register
  status = AD7193_WriteReg(hadc7193, AD7193_MODE_REG_ADDR, mode);
  if (status != HAL_OK) {
    return status;
  }
  HAL_Delay(100);

  return HAL_OK;
}

/**
 * @brief  Enable continuous read mode (DAT_STA bit set)
 */
HAL_StatusTypeDef AD7193_EnableContinuousRead(AD7193_HandleTypeDef *hadc7193) {
  // Communication byte for continuous read mode:
  // WEN=0, R/W=1 (read), Register=011 (DATA_REG), CREAD=1
  uint8_t comm_byte = (0 << 7) | // WEN = 0
                      (1 << 6) | // R/W = 1 (read)
                      (0 << 5) | // register addr bit 2
                      (1 << 4) | // register addr bit 1
                      (1 << 3) | // register addr bit 0 (DATA_REG = 011)
                      (1 << 2) | // CREAD = 1
                      (0 << 1) | // 0
                      (0 << 0);  // 0

  AD7193_SPI_Start(hadc7193);
  HAL_StatusTypeDef status = HAL_SPI_Transmit(hadc7193->hspi, &comm_byte, 1, AD7193_SPI_TIMEOUT);
  AD7193_SPI_End(hadc7193);

  return status;
}

/**
 * @brief  Set the AD7193 cs_pin LOW. This is required for the interrupt to
 * work.
 */
void AD7193_SetCSlowForTheInterruptToWork(AD7193_HandleTypeDef *hadc7193) {
  HAL_GPIO_WritePin(hadc7193->cs_port, hadc7193->cs_pin, GPIO_PIN_RESET);
}

HAL_StatusTypeDef AD7193_RawStatusRegisterToStruct(uint8_t *rawData,
                                                   AD7193_StatusRegisterTypeDef *statusStruct) {
  if (!rawData) {
    return HAL_ERROR;
  }

  // Parse status bits
  statusStruct->ready = (*rawData >> 7) & 0x01;
  statusStruct->error = (*rawData >> 6) & 0x01;
  statusStruct->noReference = (*rawData >> 5) & 0x01;
  statusStruct->parity = (*rawData >> 4) & 0x01;
  statusStruct->channelNum = *rawData & 0x0F;
  return HAL_OK;
}