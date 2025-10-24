/*******************************************************************************
 * @file drive.c
 * @brief High level Maze Bot drive code.
 *******************************************************************************
 */

#include "drive.h"
#include "h_bridge_control.h"
#include "stm32l4xx_hal_gpio.h"

/** Definitions. **************************************************************/

// H-bridge motor control.
#define MAX_PWM_DUTY 100.0f
#define MIN_PWM_DUTY 20.0f
#define PWM_DUTY_DELTA (MAX_PWM_DUTY - MIN_PWM_DUTY)

/** Public variables. *********************************************************/

float h_bridge_1_yaw_control = 0;
float h_bridge_2_yaw_control = 0;
float h_bridge_linear = 0;

/** Private functions. ********************************************************/

static uint16_t scale_pwm_duty(float drive_value) {
  // Clamp drive_value to [-1, 1].
  if (drive_value > 1.0f)
    drive_value = 1.0f;
  if (drive_value < -1.0f)
    drive_value = -1.0f;

  // Scale [-1,1] -> [MIN, MAX].
  const float scaled =
      ((drive_value + 1.0f) * 0.5f) * PWM_DUTY_DELTA + MIN_PWM_DUTY;

  return (uint16_t)scaled;
}

/** Public functions. *********************************************************/

// Unified driving code.
void drive(void) {

  const float h_bridge_1_drive = h_bridge_linear + h_bridge_1_yaw_control;
  const float h_bridge_2_drive = h_bridge_linear + h_bridge_2_yaw_control;

  if (h_bridge_1_drive == 0) {
    h_bridge_1_command(GPIO_PIN_RESET, 0);
  } else {
    const GPIO_PinState pin_1_a_state =
        h_bridge_1_drive > 0 ? GPIO_PIN_SET : GPIO_PIN_RESET;
    h_bridge_1_command(pin_1_a_state, scale_pwm_duty(h_bridge_1_drive));
  }

  if (h_bridge_2_drive == 0) {
    h_bridge_2_command(GPIO_PIN_RESET, 0);
  } else {
    const GPIO_PinState pin_2_a_state =
        h_bridge_2_drive > 0 ? GPIO_PIN_SET : GPIO_PIN_RESET;
    h_bridge_2_command(pin_2_a_state, scale_pwm_duty(h_bridge_2_drive));
  }
}
