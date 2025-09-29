/*******************************************************************************
 * @file h_bridge_control.h
 * @brief H-bridge control.
 *******************************************************************************
 */

#ifndef MAZE_BOT__H_BRIDGE_CONTROL_H
#define MAZE_BOT__H_BRIDGE_CONTROL_H

/** Includes. *****************************************************************/

#include "stm32l4xx_hal.h"

/** STM32 port and pin configs. ***********************************************/

extern TIM_HandleTypeDef htim1;

// PWM timer channel.
#define H_BRIDGE_TIM htim1

#define H_BRIDGE_1_A_PORT GPIOA
#define H_BRIDGE_1_A_PIN GPIO_PIN_11
#define H_BRIDGE_1_B_PORT GPIOA
#define H_BRIDGE_1_B_PIN GPIO_PIN_12

#define H_BRIDGE_2_A_PORT GPIOA
#define H_BRIDGE_2_A_PIN GPIO_PIN_0
#define H_BRIDGE_2_B_PORT GPIOA
#define H_BRIDGE_2_B_PIN GPIO_PIN_10

/** Public variables. *********************************************************/

extern volatile uint16_t tim_h_bridge_pwm[2];

/** Public functions. *********************************************************/

void h_bridge_command_init(void);

void h_bridge_1_command(GPIO_PinState pin_1_a_state,
                        GPIO_PinState pin_1_b_state, uint16_t pwm_1);

void h_bridge_2_command(GPIO_PinState pin_2_a_state,
                        GPIO_PinState pin_2_b_state, uint16_t pwm_2);

#endif
