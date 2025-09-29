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
#include <math.h>

/** Public variables. *********************************************************/

// Outer loop.
volatile float heading_setpoint_rad = 0.0f; // Input command.
// Heading setpoint (absolute yaw, radians, ENU: +Z up, +yaw counterclockwise).
volatile float yaw_meas = 0.0f; // Input measurement.
volatile float yaw_rate_sp = 0.0f; // Output, (rad/s).

// Inner loop.
//yaw_rate_sp                   // Input command.
//bno085_gyro_z                 // Input measurement.
volatile float turn_cmd = 0.0f; // Output.

/** Private variables. ********************************************************/

static pid_controller_t heading_pid_controller; // Outer loop.
static pid_controller_t yaw_rate_pid_controller; // Inner loop.

/** Public functions. *********************************************************/

// Initialization.
void control_loops_init(void) {
  // Outer loop.
  pid_init(&heading_pid_controller);
  heading_pid_controller.k_p = 2.0f;
  heading_pid_controller.k_i = 0.0f;
  heading_pid_controller.k_d = 0.02f;
  heading_pid_controller.tau = 0.1f;
  heading_pid_controller.T = 0.010f;
  heading_pid_controller.output_min = -2.5f;
  heading_pid_controller.output_max = 2.5f;
  heading_pid_controller.integral_min = -0.6f;
  heading_pid_controller.integral_max = 0.6f;

  // Inner loop.
  pid_init(&yaw_rate_pid_controller);
  yaw_rate_pid_controller.k_p = 0.08f;
  yaw_rate_pid_controller.k_i = 0.50f;
  yaw_rate_pid_controller.k_d = 0.001f;
  yaw_rate_pid_controller.tau = 0.02f;
  yaw_rate_pid_controller.T = 0.005f;
  yaw_rate_pid_controller.output_min = -1.0f;
  yaw_rate_pid_controller.output_max = 1.0f;
  yaw_rate_pid_controller.integral_min = -0.3f;
  yaw_rate_pid_controller.integral_max = 0.3f;
}

/* Helpers */
// Convert quaternion (x=i, y=j, z=k, w=real) to yaw (Z-YX Euler, radians).
static float quat_to_yaw(void) {
  const float x = bno085_quaternion_i;
  const float y = bno085_quaternion_j;
  const float z = bno085_quaternion_k;
  const float w = bno085_quaternion_real;

  // yaw = atan2(2(wz + xy), 1 - 2(y^2 + z^2))
  float s = 2.0f * (w * z + x * y);
  float c = 1.0f - 2.0f * (y * y + z * z);
  return atan2f(s, c);
}

// // Wrap an angle to [-pi, +pi]
// static inline float wrap_pi(float a) {
//   while (a > M_PI)
//     a -= 2.0f * M_PI;
//   while (a < -M_PI)
//     a += 2.0f * M_PI;
//   return a;
// }

/* Outer loop */
void heading_loop(void) {
  // Measure current yaw from quaternion
  yaw_meas = quat_to_yaw();

  // Feed directly into PID (pid_update does setpoint - measurement internally)
  // Output is desired yaw rate (rad/s), limited by heading_pid_controller.output_*.
  yaw_rate_sp =
      pid_update(&heading_pid_controller, heading_setpoint_rad, yaw_meas);
}

/* Inner loop */
void yaw_rate_loop(void) {
  const float rate_meas = bno085_gyro_z; // rad/s, +CCW

  // PID output is a normalized turn command (or torque request) in [-1, 1]
  turn_cmd = pid_update(&yaw_rate_pid_controller, yaw_rate_sp, rate_meas);
}

void actuate_loop(void) {
  // TODO FORWARD DIRECTION COMMAND KINEMATICS
  // const float left  = forward_cmd - turn_cmd;
  // const float right = forward_cmd + turn_cmd;

  // TODO SCALE COMMANDS AND DIRECTION
  const uint16_t pwm = (uint16_t)fabsf(turn_cmd * 1000 + 1000);

  GPIO_PinState pin_1_a_state;
  GPIO_PinState pin_1_b_state;
  GPIO_PinState pin_2_a_state;
  GPIO_PinState pin_2_b_state;
  if (pwm > 0) {
    pin_1_a_state = GPIO_PIN_RESET;
    pin_1_b_state = GPIO_PIN_SET;
    pin_2_a_state = GPIO_PIN_SET;
    pin_2_b_state = GPIO_PIN_RESET;
  } else {
    pin_1_a_state = GPIO_PIN_SET;
    pin_1_b_state = GPIO_PIN_RESET;
    pin_2_a_state = GPIO_PIN_RESET;
    pin_2_b_state = GPIO_PIN_SET;
  }

  h_bridge_1_command(pin_1_a_state, pin_1_b_state, pwm);
  h_bridge_2_command(pin_2_a_state, pin_2_b_state, pwm);
}
