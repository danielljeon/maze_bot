/*******************************************************************************
 * @file h_bridge_control.c
 * @brief H-bridge control.
 *******************************************************************************
 */

/** Includes. *****************************************************************/

#include "h_bridge_control.h"
#include "stm32l4xx_hal_gpio.h"
#include "stm32l4xx_hal_tim.h"

/** Public variables. *********************************************************/

volatile uint16_t tim_h_bridge_pwm[2] = {0, 0};

/** Public functions. *********************************************************/

void h_bridge_command_init(void) {
  HAL_GPIO_WritePin(H_BRIDGE_1_A_PORT, H_BRIDGE_1_A_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(H_BRIDGE_1_B_PORT, H_BRIDGE_1_B_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(H_BRIDGE_2_A_PORT, H_BRIDGE_2_A_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(H_BRIDGE_2_B_PORT, H_BRIDGE_2_B_PIN, GPIO_PIN_RESET);

  HAL_TIM_PWM_Init(&H_BRIDGE_TIM);

  HAL_TIM_PWM_Start(&H_BRIDGE_TIM, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&H_BRIDGE_TIM, TIM_CHANNEL_2);

  HAL_TIM_DMABurst_WriteStart(&H_BRIDGE_TIM, TIM_DMABASE_CCR1, TIM_DMA_UPDATE,
                              (uint32_t *)tim_h_bridge_pwm,
                              TIM_DMABURSTLENGTH_2TRANSFERS);
}

void h_bridge_1_command(const GPIO_PinState pin_1_a_state,
                        const GPIO_PinState pin_1_b_state,
                        const uint16_t pwm_1) {
  // Set direction pins.
  HAL_GPIO_WritePin(H_BRIDGE_1_A_PORT, H_BRIDGE_1_A_PIN, pin_1_a_state);
  HAL_GPIO_WritePin(H_BRIDGE_1_B_PORT, H_BRIDGE_1_B_PIN, pin_1_b_state);

  // Set PWM.
  tim_h_bridge_pwm[0] = pwm_1;
}

void h_bridge_2_command(const GPIO_PinState pin_2_a_state,
                        const GPIO_PinState pin_2_b_state,
                        const uint16_t pwm_2) {
  // Set direction pins.
  HAL_GPIO_WritePin(H_BRIDGE_2_A_PORT, H_BRIDGE_2_A_PIN, pin_2_a_state);
  HAL_GPIO_WritePin(H_BRIDGE_2_B_PORT, H_BRIDGE_2_B_PIN, pin_2_b_state);

  // Set PWM.
  tim_h_bridge_pwm[1] = pwm_2;
}
