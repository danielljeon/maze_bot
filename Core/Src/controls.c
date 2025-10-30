/*******************************************************************************
 * @file controls.c
 * @brief High level Maze Bot control systems.
 *******************************************************************************
 */

/** Includes. *****************************************************************/

#include "controls.h"
#include "bno085_runner.h"
#include "drive.h"
#include "pid.h"
#include <math.h>

/** Definitions. **************************************************************/

// Command deadband (unitless; |turn_cmd| = [0,1]).
#define TURN_COMMAND_DEADBAND 0.08f

/** Private variables. ********************************************************/

// Reference variables.
static volatile float yaw_zero = 0.0f; // Set local "zero" heading.

// Outer loop.
static pid_controller_t heading_pid_controller;    // Outer loop controller.
static volatile float heading_setpoint_rad = 0.0f; // Input command.
// Heading setpoint (absolute yaw, radians, ENU: +Z up, +yaw counterclockwise).
static volatile float yaw_rate_sp = 0.0f; // Output, (rad/s).

// Inner loop.
static pid_controller_t yaw_rate_pid_controller; // Inner loop controller.
//yaw_rate_sp                   // Input command.
//bno085_gyro_z                 // Input measurement.
static volatile float turn_cmd = 0.0f; // Output.

/** Private functions. ********************************************************/

// Convert quaternion (x=i, y=j, z=k, w=real) to yaw (Z-YX Euler, radians).
static float quat_to_yaw(void) {
  const float x = bno085_game_quaternion_i;
  const float y = bno085_game_quaternion_j;
  const float z = bno085_game_quaternion_k;
  const float w = bno085_game_quaternion_real;

  // yaw = atan2(2(wz + xy), 1 - 2(y^2 + z^2)).
  const float s = 2.0f * (w * z + x * y);
  const float c = 1.0f - 2.0f * (y * y + z * z);
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

// Local yaw (relative to our chosen zero).
static float yaw_local(void) { return wrap_pi(quat_to_yaw() - yaw_zero); }

// Inverse kinematics and motor control.
static void actuate(void) {
  const float tc = turn_cmd;

  // Add deadband to command.
  if (fabsf(tc) < TURN_COMMAND_DEADBAND) {
    h_bridge_1_yaw_control = 0;
    h_bridge_2_yaw_control = 0;
    return;
  }

  // H-bridge 1 is left, 2 is right. Turn convention: CCW = +ive, CW = -ive.
  h_bridge_1_yaw_control = tc;
  h_bridge_2_yaw_control = -tc;
}

/** Public functions. *********************************************************/

void zero_heading(void) { yaw_zero = quat_to_yaw(); }

// Relative heading from current local yaw.
void set_relative_heading(const float delta_rad) {
  const float yaw_now_local = yaw_local();
  heading_setpoint_rad = wrap_pi(yaw_now_local + delta_rad);
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
  yaw_rate_pid_controller.k_p = 0.35f;
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
  const float yaw_meas_local = yaw_local();

  // Choose nearest equivalent of setpoint to avoid wrap jumps.
  const float err = wrap_pi(heading_setpoint_rad - yaw_meas_local);
  const float sp_closest_local = yaw_meas_local + err;

  yaw_rate_sp =
      pid_update(&heading_pid_controller, sp_closest_local, yaw_meas_local);
}

// Inner loop.
void yaw_rate_loop(void) {
  const float rate_meas = bno085_gyro_z; // rad/s, +CCW.

  turn_cmd = pid_update(&yaw_rate_pid_controller, yaw_rate_sp, rate_meas);

  actuate();
}
