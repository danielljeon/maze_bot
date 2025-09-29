/*******************************************************************************
* @file vl53l4cd_runner.h
 * @brief VL53L4CD runner: init and data reading.
 *******************************************************************************
 */

#ifndef MAZE_BOT__VL53L4CD_RUNNER_H
#define MAZE_BOT__VL53L4CD_RUNNER_H

/** Includes. *****************************************************************/

#include "VL53L4CD_api.h"
#include "VL53L4CD_calibration.h"
#include "platform.h"

/** Definitions. **************************************************************/

// GPIO for XSHUT.
#define VL53L4CD_XSHUT_PORT GPIOB
#define VL53L4CD_XSHUT_PIN GPIO_PIN_5

// GPIO for GPIO1 (interrupt).
#define VL53L4CD_INT_PORT GPIOB
#define VL53L4CD_INT_PIN GPIO_PIN_4

#define VL53L4CD_SENSOR_ID 0xEBAA
#define VL53L4CD_DEVICE_ADDRESS 0x52 // 8-bit, 0x29 for 7-bit address.

/** Public variables. *********************************************************/

extern uint8_t range_status;
extern uint16_t distance_mm;
extern uint32_t ambient_rate_kcps;
extern uint32_t ambient_per_spad_kcps;
extern uint32_t signal_rate_kcps;
extern uint32_t signal_per_spad_kcps;
extern uint16_t number_of_spad;
extern uint16_t sigma_mm;

/** User implementations into STM32 GPIO NVIC HAL. ****************************/

void HAL_GPIO_EXTI_Callback_vl53l4cd(uint16_t n);

/** Public functions. *********************************************************/

/**
 * @brief Initialize VL53L4CD.
 *
 * @return Result of BMP3 API execution status.
 * @retval == 0 -> Success.
 * @retval > 0  -> Warning.
 * @retval < 0  -> Error.
 */
int8_t vl53l4cd_init(void);

/**
 * @brief Start interrupt based ranging.
 *
 * @return Result of BMP3 API execution status.
 * @retval == 0 -> Success.
 * @retval > 0  -> Warning.
 * @retval < 0  -> Error.
 */
int8_t vl53l4cd_start(void);

/**
 * @brief Stop interrupt based ranging.
 *
 * @return Result of BMP3 API execution status.
 * @retval == 0 -> Success.
 * @retval > 0  -> Warning.
 * @retval < 0  -> Error.
 */
int8_t vl53l4cd_stop(void);

#endif
