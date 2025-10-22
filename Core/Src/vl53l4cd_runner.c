/*******************************************************************************
 * @file vl53l4cd_runner.c
 * @brief VL53L4CD runner: init and data reading.
 *******************************************************************************
 */

/** Includes. *****************************************************************/

#include "vl53l4cd_runner.h"

/** Private types. ************************************************************/

typedef enum i2c_dma_state_e {
  IDLE,                       // VL53L4CD not yet configured.
  I2C_WAITING_DATA_READY_INT, // Awaiting next data ready call.
  I2C_DATA_RX_PENDING,        // Pending DMA RX arrival.
  I2C_DATA_RX_LOADED,         // Data arrived and loaded in DMA buffer.
} i2c_dma_state_t;

/** Private variables. ********************************************************/

static Dev_t vl53l4cd_dev = VL53L4CD_DEVICE_ADDRESS;

static i2c_dma_state_t i2c_dma_state = IDLE;

// DMA RX array for VL53L4CD I2C read (16-bit).
static uint8_t vl53l4cd_dma_rx_buffer[2] = {0};

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

/** Private functions. ********************************************************/

/*
 * Note: This section of private functions are actually lower level logic. The
 * functions are made as a simplified DMA based operation. The functions follow
 * the similar structure and formatting to those in the `VL53L4CD_ULD_Driver`.
 */

/**
 * @brief This function triggers the DMA read distance results from the sensor.
 * @param dev instance of selected VL53L4CD sensor.
 * @return (uint8_t) status : 0 if OK.
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
 * @return (uint8_t) status : 0 if OK.
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

#ifndef VL53L4CD_USE_ONLY_DMA_DISTANCE
    VL53L4CD_ResultsData_t data = {0};

    VL53L4CD_GetResult(vl53l4cd_dev, &data);

    vl53l4cd_range_status = data.range_status;
    vl53l4cd_distance_mm = data.distance_mm;
    vl53l4cd_ambient_rate_kcps = data.ambient_rate_kcps;
    vl53l4cd_ambient_per_spad_kcps = data.ambient_per_spad_kcps;
    vl53l4cd_signal_rate_kcps = data.signal_rate_kcps;
    vl53l4cd_signal_per_spad_kcps = data.signal_per_spad_kcps;
    vl53l4cd_number_of_spad = data.number_of_spad;
    vl53l4cd_sigma_mm = data.sigma_mm;

    VL53L4CD_ClearInterrupt(vl53l4cd_dev);
#else
    if (i2c_dma_state == I2C_WAITING_DATA_READY_INT) {
      i2c_dma_state = I2C_DATA_RX_PENDING;
      vl53l4cd_start_distance_dma(vl53l4cd_dev);
    }
#endif
  }
}

void HAL_I2C_MemRxCpltCallback_vl53l4cd(I2C_HandleTypeDef *hi2c) {
  if (hi2c == &VL53L4CD_HI2C && i2c_dma_state == I2C_DATA_RX_PENDING) {
    i2c_dma_state = I2C_DATA_RX_LOADED;
  }
}

/** Public functions. *********************************************************/

int8_t vl53l4cd_init(void) {
  i2c_dma_state = IDLE; // Device not yet configured.

  int8_t status = 0;

  // Pulse XSHUT.
  HAL_GPIO_WritePin(VL53L4CD_XSHUT_PORT, VL53L4CD_XSHUT_PIN, GPIO_PIN_RESET);
  HAL_Delay(100);

  // Ensure XSHUT is high.
  HAL_GPIO_WritePin(VL53L4CD_XSHUT_PORT, VL53L4CD_XSHUT_PIN, GPIO_PIN_SET);

  HAL_Delay(25);

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
    i2c_dma_state = I2C_WAITING_DATA_READY_INT;
  }
  return status;
}

void vl53l4cd_process_dma(void) {
  if (i2c_dma_state == I2C_DATA_RX_LOADED) {

    VL53L4CD_ResultsData_t data = {0};

    vl53l4cd_get_distance_dma(&data); // DMA based distance only update.

    vl53l4cd_range_status = data.range_status;
    vl53l4cd_distance_mm = data.distance_mm;
    vl53l4cd_ambient_rate_kcps = data.ambient_rate_kcps;
    vl53l4cd_ambient_per_spad_kcps = data.ambient_per_spad_kcps;
    vl53l4cd_signal_rate_kcps = data.signal_rate_kcps;
    vl53l4cd_signal_per_spad_kcps = data.signal_per_spad_kcps;
    vl53l4cd_number_of_spad = data.number_of_spad;
    vl53l4cd_sigma_mm = data.sigma_mm;

    i2c_dma_state = I2C_WAITING_DATA_READY_INT;

    // Clear interrupt.
    VL53L4CD_ClearInterrupt(vl53l4cd_dev);
  }
}

int8_t vl53l4cd_stop(void) {
  return (int8_t)VL53L4CD_StopRanging(vl53l4cd_dev);
}
