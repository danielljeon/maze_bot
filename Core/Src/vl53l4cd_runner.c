/*******************************************************************************
 * @file vl53l4cd_runner.c
 * @brief VL53L4CD runner: init and data reading.
 *******************************************************************************
 */

/** Includes. *****************************************************************/

#include "vl53l4cd_runner.h"

/** Private variables. ********************************************************/

static Dev_t vl53l4cd_dev = VL53L4CD_DEVICE_ADDRESS;

// DMA RX array for VL53L4CD I2C read (16-bit).
static uint8_t vl53l4cd_dma_rx_buffer[2] = {0};

// DMA TX array for VL53L4CD I2C read (16-bit + 8-bit).
static uint8_t vl53l4cd_dma_tx_buffer[3] = {0};

/** STM32 port and pin configs. ***********************************************/

extern I2C_HandleTypeDef hi2c1;

// I2C.
#define VL53L4CD_HI2C hi2c1

/** Public variables. *********************************************************/

uint8_t vl53l4cd_range_status = 0;
uint16_t vl53l4cd_distance_mm = 0;
uint32_t vl53l4cd_ambient_rate_kcps = 0;
uint32_t vl53l4cd_ambient_per_spad_kcps = 0;
uint32_t vl53l4cd_signal_rate_kcps = 0;
uint32_t vl53l4cd_signal_per_spad_kcps = 0;
uint16_t vl53l4cd_number_of_spad = 0;
uint16_t vl53l4cd_sigma_mm = 0;

volatile bool int_ready = false;

/** Private functions. ********************************************************/

/*
 * Note: This section of private functions are actually lower level logic. The
 * functions are made as a simplified DMA based operation. The functions follow
 * the similar structure and formatting to those in the `VL53L4CD_ULD_Driver`.
 */

/**
 * @brief This function clears the interrupt using DMA (DMA version of
 * VL53L4CD_ClearInterrupt from the ULD).
 * @param dev : Device instance.
 * @return (VL53L4CD_ERROR) status : 0 if OK.
 */
VL53L4CD_Error vl53l4cd_clear_interrupt_dma(Dev_t dev) {
  VL53L4CD_Error status = VL53L4CD_ERROR_NONE;
  vl53l4cd_dma_tx_buffer[0] = (VL53L4CD_SYSTEM__INTERRUPT_CLEAR >> 8) & 0xFF;
  vl53l4cd_dma_tx_buffer[1] = (VL53L4CD_SYSTEM__INTERRUPT_CLEAR >> 0) & 0xFF;
  vl53l4cd_dma_tx_buffer[2] = 0x01 & 0xFF;
  status |= HAL_I2C_Master_Transmit_DMA(&hi2c1, dev, vl53l4cd_dma_tx_buffer, 3);

  return status;
}

/**
 * @brief This function triggers the DMA read distance results from the sensor.
 * @param dev instance of selected VL53L4CD sensor.
 * @return (VL53L4CD_ERROR) status : 0 if OK.
 */
VL53L4CD_Error vl53l4cd_start_distance_dma(Dev_t dev) {
  VL53L4CD_Error status = VL53L4CD_ERROR_NONE;
  status |=
      HAL_I2C_Mem_Read_DMA(&hi2c1, dev, VL53L4CD_RESULT__DISTANCE,
                           I2C_MEMADD_SIZE_16BIT, vl53l4cd_dma_rx_buffer, 2);
  return status;
}

/**
 * @brief This function gets the distance reported by the sensor stored by DMA.
 * @param p_result Pointer of structure, filled with the distance result.
 * @return (VL53L4CD_ERROR) status : 0 if OK.
 */
VL53L4CD_Error vl53l4cd_get_distance_dma(VL53L4CD_ResultsData_t *p_result) {
  const uint16_t temp_16 = ((uint16_t)vl53l4cd_dma_rx_buffer[0] << 8) |
                           vl53l4cd_dma_rx_buffer[1]; // MSB first.
  p_result->distance_mm = temp_16;
  return 0;
}

/** User implementations into STM32 HAL (overwrite weak HAL functions). *******/

void HAL_GPIO_EXTI_Callback_vl53l4cd(uint16_t n) {
  if (n == VL53L4CD_INT_PIN) {
    int_ready = true;
  }
}

/** Public functions. *********************************************************/

int8_t vl53l4cd_init(void) {
  int8_t status = 0;

  // Reset interrupt flag.
  int_ready = false;

  // Ensure XSHUT is high.
  HAL_GPIO_WritePin(VL53L4CD_XSHUT_PORT, VL53L4CD_XSHUT_PIN, GPIO_PIN_SET);

  HAL_Delay(5);

  // Get sensor ID.
  uint16_t sensor_id = 0;
  status = (int8_t)VL53L4CD_GetSensorId(vl53l4cd_dev, &sensor_id);

  if (sensor_id != VL53L4CD_SENSOR_ID) {
    return status;
  }

  // Initialize the sensor.
  status = (int8_t)VL53L4CD_SensorInit(vl53l4cd_dev);

  return status;
}

int8_t vl53l4cd_start(void) {
  const int8_t status = (int8_t)VL53L4CD_StartRanging(vl53l4cd_dev);

  if (status == 0) {
    // Ready for data ready interrupt.
  }
  return status;
}

void vl53l4cd_process(void) {
  if (int_ready == true) {
    uint8_t data_write[2];
    uint8_t data_read[2];

    // Get result.
    VL53L4CD_Error status = VL53L4CD_ERROR_NONE;

    data_write[0] = (VL53L4CD_RESULT__DISTANCE >> 8) & 0xFF;
    data_write[1] = VL53L4CD_RESULT__DISTANCE & 0xFF;
    status = HAL_I2C_Mem_Read(&VL53L4CD_HI2C, vl53l4cd_dev,
                              VL53L4CD_RESULT__DISTANCE, I2C_MEMADD_SIZE_16BIT,
                              data_read, 2, 100);

    if (status == VL53L4CD_ERROR_NONE) {
      vl53l4cd_distance_mm = (data_read[0] << 8) | (data_read[1]);

      // End, clear interrupt.
      int_ready = false;
      VL53L4CD_ClearInterrupt(vl53l4cd_dev);
    } else {
      // Clear interrupt, but no advance internally.
      VL53L4CD_ClearInterrupt(vl53l4cd_dev);
    }

    // vl53l4cd_clear_interrupt_dma(vl53l4cd_dev);
  }
}

int8_t vl53l4cd_stop(void) {
  return (int8_t)VL53L4CD_StopRanging(vl53l4cd_dev);
}
