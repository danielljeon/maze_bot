/*******************************************************************************
 * @file servo_control.c
 * @brief Servo control.
 *******************************************************************************
 */

/** Includes. *****************************************************************/

#include "servo_control.h"
#include "stm32l4xx_hal_tim.h"

/** Public variables. *********************************************************/

volatile uint16_t tim_servo_pwm[1] = {0};

/** Public functions. *********************************************************/

void servo_command_init(void) {
  HAL_TIM_PWM_Init(&SERVOS_TIM);

  HAL_TIM_PWM_Start(&SERVOS_TIM, TIM_CHANNEL_1);

  HAL_TIM_DMABurst_WriteStart(&SERVOS_TIM, TIM_DMABASE_CCR1, TIM_DMA_UPDATE,
                              (uint32_t *)tim_servo_pwm,
                              TIM_DMABURSTLENGTH_1TRANSFER);
}

void servo_command(const uint16_t pwm_1) { tim_servo_pwm[0] = pwm_1; }
