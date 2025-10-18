/*******************************************************************************
 * @file vl53l4cd_runner.c
 * @brief VL53L4CD runner: init and data reading.
 *******************************************************************************
 */

/** Includes. *****************************************************************/

#include "vl53l4cd_runner.h"

/** Private variables. ********************************************************/

static Dev_t vl53l4cd_dev = VL53L4CD_DEVICE_ADDRESS;

/** Public variables. *********************************************************/

uint8_t range_status = 0;
uint16_t distance_mm = 0;
uint32_t ambient_rate_kcps = 0;
uint32_t ambient_per_spad_kcps = 0;
uint32_t signal_rate_kcps = 0;
uint32_t signal_per_spad_kcps = 0;
uint16_t number_of_spad = 0;
uint16_t sigma_mm = 0;

/** User implementations into STM32 GPIO NVIC HAL. ****************************/

void HAL_GPIO_EXTI_Callback_vl53l4cd(uint16_t n) {
  if (n == VL53L4CD_INT_PIN) {
    VL53L4CD_ResultsData_t data = {0};

    VL53L4CD_GetResult(vl53l4cd_dev, &data);

    range_status = data.range_status;
    distance_mm = data.distance_mm;
    ambient_rate_kcps = data.ambient_rate_kcps;
    ambient_per_spad_kcps = data.ambient_per_spad_kcps;
    signal_rate_kcps = data.signal_rate_kcps;
    signal_per_spad_kcps = data.signal_per_spad_kcps;
    number_of_spad = data.number_of_spad;
    sigma_mm = data.sigma_mm;

    VL53L4CD_ClearInterrupt(vl53l4cd_dev);
  }
}

/** Public functions. *********************************************************/

int8_t vl53l4cd_init(void) {
  int8_t status = 0;

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
  return (int8_t)VL53L4CD_StartRanging(vl53l4cd_dev);
}

int8_t vl53l4cd_stop(void) {
  return (int8_t)VL53L4CD_StopRanging(vl53l4cd_dev);
}
