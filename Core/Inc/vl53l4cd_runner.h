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

// GPIO for XSHUT.
#define VL53L4CD_XSHUT_PORT GPIOB
#define VL53L4CD_XSHUT_PIN GPIO_PIN_5

// GPIO for GPIO1 (interrupt).
#define VL53L4CD_INT_PORT GPIOB
#define VL53L4CD_INT_PIN GPIO_PIN_4

/** Definitions. **************************************************************/

#define VL53L4CD_SENSOR_ID 0xEBAA
#define VL53L4CD_DEVICE_ADDRESS 0x52 // 8-bit, 0x29 for 7-bit address.

/** Public variables. *********************************************************/

extern uint8_t vl53l4cd_range_status;
extern uint16_t vl53l4cd_distance_mm;
extern uint32_t vl53l4cd_ambient_rate_kcps;
extern uint32_t vl53l4cd_ambient_per_spad_kcps;
extern uint32_t vl53l4cd_signal_rate_kcps;
extern uint32_t vl53l4cd_signal_per_spad_kcps;
extern uint16_t vl53l4cd_number_of_spad;
extern uint16_t vl53l4cd_sigma_mm;

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
