/*******************************************************************************
 * @file controls.h
 * @brief High level Maze Bot control systems.
 *******************************************************************************
 */

/** Includes. *****************************************************************/

#include "controls.h"
#include "bno085_runner.h"
#include "h_bridge_control.h"
#include "pid.h"
#include "stm32l4xx_hal_gpio.h"
#include <math.h>

/** Definitions. **************************************************************/

#define MAX_PWM_DELTA 70
#define TRUE_MIN_PWM 30
#define TURN_COMMAND_DEADBAND 0.1f

/** Private variables. ********************************************************/

// Reference variables.
volatile float yaw_zero_world = 0.0f; // Set local "zero" heading.

// Outer loop.
static pid_controller_t heading_pid_controller; // Outer loop controller.
volatile float heading_setpoint_rad = 0.0f;     // Input command.
// Heading setpoint (absolute yaw, radians, ENU: +Z up, +yaw counterclockwise).
volatile float yaw_meas = 0.0f;    // Input measurement.
volatile float yaw_rate_sp = 0.0f; // Output, (rad/s).

// Inner loop.
static pid_controller_t yaw_rate_pid_controller; // Inner loop controller.
//yaw_rate_sp                   // Input command.
//bno085_gyro_z                 // Input measurement.
volatile float turn_cmd = 0.0f; // Output.

/** Private functions. ********************************************************/

// Convert quaternion (x=i, y=j, z=k, w=real) to yaw (Z-YX Euler, radians).
static float quat_to_yaw(void) {
  const float x = bno085_quaternion_i;
  const float y = bno085_quaternion_j;
  const float z = bno085_quaternion_k;
  const float w = bno085_quaternion_real;

  // yaw = atan2(2(wz + xy), 1 - 2(y^2 + z^2)).
  float s = 2.0f * (w * z + x * y);
  float c = 1.0f - 2.0f * (y * y + z * z);
  return atan2f(s, c);
}

// Wrap to [-pi, pi].
static float wrap_pi(float a) {
  while (a > M_PI)
    a -= 2.0f * (float)M_PI;
  while (a < -M_PI)
    a += 2.0f * (float)M_PI;
  return a;
}

/** Private functions. ********************************************************/

// Inverse kinematics and motor control.
void actuate(void) {
  const float tc = turn_cmd;

  // Add deadband to command.
  if (fabsf(tc) < TURN_COMMAND_DEADBAND) {
    h_bridge_1_command(GPIO_PIN_RESET, GPIO_PIN_RESET, 0);
    h_bridge_2_command(GPIO_PIN_RESET, GPIO_PIN_RESET, 0);
    return;
  }

  // Direction from sign, magnitude from |turn_cmd|.
  const int forward = tc > 0.0f;

  // Scale |turn_cmd| [0,1] -> delta [0, MAX_PWM_DELTA].
  const int pwm_delta =
      (int)lroundf(fminf(fabsf(tc), 1.0f) * (float)MAX_PWM_DELTA);

  // Duty is now [TRUE_MIN_PWM, TRUE_MIN_PWM + MAX_PWM_DELTA].
  const uint16_t duty = (uint16_t)(TRUE_MIN_PWM + pwm_delta);

  // Direction pins.
  if (duty == 0) {
    h_bridge_1_command(GPIO_PIN_RESET, GPIO_PIN_RESET, 0);
    h_bridge_2_command(GPIO_PIN_RESET, GPIO_PIN_RESET, 0);
  } else {
    const GPIO_PinState pin_1_a_state = forward ? GPIO_PIN_RESET : GPIO_PIN_SET;
    const GPIO_PinState pin_1_b_state = forward ? GPIO_PIN_SET : GPIO_PIN_RESET;
    const GPIO_PinState pin_2_a_state = forward ? GPIO_PIN_SET : GPIO_PIN_RESET;
    const GPIO_PinState pin_2_b_state = forward ? GPIO_PIN_RESET : GPIO_PIN_SET;
    h_bridge_1_command(pin_1_a_state, pin_1_b_state, duty);
    h_bridge_2_command(pin_2_a_state, pin_2_b_state, duty);
  }
}

/** Public functions. *********************************************************/

// Set a heading from the current heading.
void set_relative_heading(const float delta_rad) {
  const float yaw_now = quat_to_yaw();
  heading_setpoint_rad = yaw_now + wrap_pi(delta_rad);
}

// Initialization.
void control_loops_init(void) {
  // Outer loop.
  pid_init(&heading_pid_controller);
  heading_pid_controller.k_p = 2.0f;
  heading_pid_controller.k_i = 0.0f;
  heading_pid_controller.k_d = 0.0f;
  heading_pid_controller.tau = 0.1f;
  heading_pid_controller.T = 0.010f;
  heading_pid_controller.output_min = -2.5f;
  heading_pid_controller.output_max = 2.5f;
  heading_pid_controller.integral_min = -0.6f;
  heading_pid_controller.integral_max = 0.6f;

  // Inner loop.
  pid_init(&yaw_rate_pid_controller);
  yaw_rate_pid_controller.k_p = 0.1f;
  yaw_rate_pid_controller.k_i = 0.0f;
  yaw_rate_pid_controller.k_d = 0.0f;
  yaw_rate_pid_controller.tau = 0.02f;
  yaw_rate_pid_controller.T = 0.005f;
  yaw_rate_pid_controller.output_min = -1.0f;
  yaw_rate_pid_controller.output_max = 1.0f;
  yaw_rate_pid_controller.integral_min = -0.3f;
  yaw_rate_pid_controller.integral_max = 0.3f;
}

// Outer loop.
void heading_loop(void) {
  // Measure current yaw from quaternion (world frame).
  yaw_meas = quat_to_yaw();

  // Choose nearest equivalent of the stored world-frame setpoint.
  const float err = wrap_pi(heading_setpoint_rad - yaw_meas);
  const float sp_closest = yaw_meas + err;

  yaw_rate_sp = pid_update(&heading_pid_controller, sp_closest, yaw_meas);
}

// Inner loop.
void yaw_rate_loop(void) {
  const float rate_meas = bno085_gyro_z; // rad/s, +CCW.

  turn_cmd = pid_update(&yaw_rate_pid_controller, yaw_rate_sp, rate_meas);

  actuate();
}
