/*******************************************************************************
 * @file vl53l4cd_runner.c
 * @brief VL53L4CD runner: init and data reading.
 *******************************************************************************
 */

/** Includes. *****************************************************************/

#include "vl53l4cd_runner.h"

/** Public variables. *********************************************************/

uint16_t vl53l4cd_distance_mm[3] = {0, 0, 0};

/** Private variables. ********************************************************/

static Dev_t vl53l4cd_dev = VL53L4CD_DEVICE_ADDRESS + 2;
// If using "n" VL53L4CD TOFs, device addresses are set to:
// 1. VL53L4CD_DEVICE_ADDRESS + 2
// 2. VL53L4CD_DEVICE_ADDRESS + 4
// 3. VL53L4CD_DEVICE_ADDRESS + 6
//    ...
// n. VL53L4CD_DEVICE_ADDRESS + (2 * n)
// 7-bit I2C addresses (left shift by 1) so must increment by 2.

// 1 of 3 VL53L4CDs do not have the XSHUT, it will be assigned new address of
// (VL53L4CD_DEVICE_ADDRESS + 2) first. Then, the remaining XSHUT pins below
// will be enabled from index 0 onwards. See the edge case initialization note.
static GPIO_TypeDef *vl53l4cd_xshut_ports[2] = {VL53L4CD_XSHUT_PORT_2,
                                                VL53L4CD_XSHUT_PORT};
static uint16_t vl53l4cd_xshut_pins[2] = {VL53L4CD_XSHUT_PIN_2,
                                          VL53L4CD_XSHUT_PIN};

#ifdef VL53L4CD_USE_3
static const uint8_t device_count = 3;
#else
static const uint8_t device_count = 1;
#endif

static volatile bool int_ready = false;

/** STM32 port and pin configs. ***********************************************/

extern I2C_HandleTypeDef hi2c1;

// I2C.
#define VL53L4CD_HI2C hi2c1

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

  // Set all TOFs to shutdown.
  for (uint8_t i = 0; i < device_count - 1; i++) {
    HAL_GPIO_WritePin(vl53l4cd_xshut_ports[i], vl53l4cd_xshut_pins[i],
                      GPIO_PIN_RESET);
  }
  HAL_Delay(5);

  for (uint8_t i = 0; i < device_count; i++) {
    // Entry.
    if (i >= 1) {
      HAL_GPIO_WritePin(vl53l4cd_xshut_ports[i - 1], vl53l4cd_xshut_pins[i - 1],
                        GPIO_PIN_SET);
    }
    HAL_Delay(10);

    // Get sensor ID.
    uint16_t sensor_id = 0;
    status = (int8_t)VL53L4CD_GetSensorId(VL53L4CD_DEVICE_ADDRESS, &sensor_id);

    if (sensor_id == VL53L4CD_SENSOR_ID) {
      // Initialize the sensor.
      status = (int8_t)VL53L4CD_SensorInit(VL53L4CD_DEVICE_ADDRESS);

      // Update the I2C address.
      status =
          VL53L4CD_SetI2CAddress(VL53L4CD_DEVICE_ADDRESS, vl53l4cd_dev + i * 2);

    } else {
      // Handle edge case: 1 of 3 VL53L4CDs do not have XSHUT. If the program
      // has already been run, a new address (VL53L4CD_DEVICE_ADDRESS + 2) would
      // have already been assigned. Try VL53L4CD_DEVICE_ADDRESS + 2.
      if (sensor_id) {
        status = (int8_t)VL53L4CD_GetSensorId(VL53L4CD_DEVICE_ADDRESS + 2,
                                              &sensor_id);

        // If both the default (VL53L4CD_DEVICE_ADDRESS) and edge case
        // (VL53L4CD_DEVICE_ADDRESS + 2) address failed, return error status.
        if (sensor_id != VL53L4CD_SENSOR_ID) {
          return status;
        }

        // Initialize the sensor.
        status = (int8_t)VL53L4CD_SensorInit(VL53L4CD_DEVICE_ADDRESS + 2);
      }
    }
    HAL_Delay(1);
  }

  return status;
}

int8_t vl53l4cd_start(void) {
  int8_t status = 0;
  for (uint8_t i = 0; i < device_count; i++) {
    status = (int8_t)VL53L4CD_StartRanging(vl53l4cd_dev + i * 2);
    HAL_Delay(5);
  }
  return status;
}

void vl53l4cd_process(void) {
  if (int_ready == true) {
    for (uint8_t i = 0; i < device_count; i++) {

      // Get result.
      uint8_t data_read[2];
      VL53L4CD_Error status = VL53L4CD_ERROR_NONE;

      status = HAL_I2C_Mem_Read(&VL53L4CD_HI2C, vl53l4cd_dev + i * 2,
                                VL53L4CD_RESULT__DISTANCE,
                                I2C_MEMADD_SIZE_16BIT, data_read, 2, 100);

      if (status == VL53L4CD_ERROR_NONE) {
        vl53l4cd_distance_mm[i] = ((uint16_t)data_read[0] << 8) | data_read[1];

        VL53L4CD_ClearInterrupt(vl53l4cd_dev + i * 2);
      } else {
        // Clear interrupt, but no advance internally.
        VL53L4CD_ClearInterrupt(vl53l4cd_dev + i * 2);
      }
    }

    // End, clear interrupt flag.
    int_ready = false;
  }
}

int8_t vl53l4cd_stop(void) {
  int8_t status = 0;
  for (uint8_t i = 0; i < device_count; i++) {
    status = (int8_t)VL53L4CD_StopRanging(vl53l4cd_dev + i * 2);
  }
  return status;
}
