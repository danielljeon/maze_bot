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
#include <stdbool.h>

/** STM32 port and pin configs. ***********************************************/

#define VL53L4CD_USE_3 // Assumes 1 if not defined.

// GPIO for XSHUT.
#define VL53L4CD_XSHUT_PORT GPIOB
#define VL53L4CD_XSHUT_PIN GPIO_PIN_5

// XSHUT defined for 2 out of 3, 1 out of 3 uses the default address.
// GPIO for XSHUT.
#define VL53L4CD_XSHUT_PORT_2 GPIOA
#define VL53L4CD_XSHUT_PIN_2 GPIO_PIN_3

// GPIO for GPIO1 (interrupt).
#define VL53L4CD_INT_PORT GPIOB
#define VL53L4CD_INT_PIN GPIO_PIN_4

/** Definitions. **************************************************************/

#define VL53L4CD_SENSOR_ID 0xEBAA
#define VL53L4CD_DEVICE_ADDRESS 0x52 // 8-bit, 0x29 for 7-bit address.

/** Public variables. *********************************************************/

extern uint16_t vl53l4cd_distance_mm[3];

extern volatile bool int_ready;

/** User implementations into STM32 HAL (overwrite weak HAL functions). *******/

void HAL_GPIO_EXTI_Callback_vl53l4cd(uint16_t n);

/** Public functions. *********************************************************/

/**
 * @brief Initialize VL53L4CD.
 *
 * @return Result of VL53L4CD ULD status.
 * @retval == 0 -> Success.
 * @retval > 0  -> Warning.
 * @retval < 0  -> Error.
 */
int8_t vl53l4cd_init(void);

/**
 * @brief Start interrupt based ranging.
 *
 * @return Result of VL53L4CD ULD status.
 * @retval == 0 -> Success.
 * @retval > 0  -> Warning.
 * @retval < 0  -> Error.
 */
int8_t vl53l4cd_start(void);

void vl53l4cd_process(void);

/**
 * @brief Stop interrupt based ranging.
 *
 * @return Result of VL53L4CD ULD status.
 * @retval == 0 -> Success.
 * @retval > 0  -> Warning.
 * @retval < 0  -> Error.
 */
int8_t vl53l4cd_stop(void);

#endif
