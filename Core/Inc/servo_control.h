/*******************************************************************************
 * @file servo_control.h
 * @brief Servo control callbacks.
 *******************************************************************************
 */

#ifndef MAZE_BOT__SERVO_CONTROL_H
#define MAZE_BOT__SERVO_CONTROL_H

/** Includes. *****************************************************************/

#include "stm32l4xx_hal.h"

/** STM32 port and pin configs. ***********************************************/

extern TIM_HandleTypeDef htim15;

// PWM timer channel.
#define SERVOS_TIM htim15

/** Public variables. *********************************************************/

extern volatile uint16_t tim_servo_pwm[1];

/** Public functions. *********************************************************/

void servo_command_init(void);

void servo_command(uint16_t pwm_1);

#endif
