/*******************************************************************************
 * @file callbacks.c
 * @brief STM32 HAL callback implementations overriding weak declarations.
 *******************************************************************************
 */

/** Includes. *****************************************************************/

#include "bno085_runner.h"
#include "stm32l4xx_hal.h"
#include "vl53l4cd_runner.h"

/** Collection of user implementations of STM32 HAL (overwriting HAL). ********/

/** GPIO. */

void HAL_GPIO_EXTI_Callback(uint16_t n) {
  HAL_GPIO_EXTI_Callback_sh2(n);
  HAL_GPIO_EXTI_Callback_vl53l4cd(n);
}

/** SPI. */

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
  HAL_SPI_TxRxCpltCallback_sh2(hspi);
}
