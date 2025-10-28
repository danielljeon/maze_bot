/*******************************************************************************
 * @file maze_navigation.c
 * @brief Maze navigation software.
 *******************************************************************************
 */

/** Includes. *****************************************************************/

#include "maze_navigation.h"
#include "configuration.h"
#include "controls.h"
#include "drive.h"
#include "math.h"
#include "vl53l4cd_runner.h"

/** Public types. *************************************************************/

typedef enum { STRAIGHT, TURN } mode_t;

/** Public variables. *********************************************************/

volatile float heading_error_rad_calc = 0;
volatile float position_error_mm_calc = 0;

/** Private variables. ********************************************************/

static mode_t mode = STRAIGHT;

// Calibrations.
static const float V_FAST = 0.32f;      // Forward command in [-1,1].
static const float K_THETA = 1.10f;     // Corridor parallel gain (rad -> cmd).
static const float KX_OVER_L = 0.02f;   // Centering bias gain (mm^-1).
static const float ERR_PAR_OK = 0.2f;   //Consider "aligned".
static const float FRONT_STOP = 100.0f; // Stop and turn if front < this (mm).
static const float FRONT_GO = 130.0f;   // Resume straight if front > this (mm).
static const float FRONT_INVALID = 15.0f; // Treat tiny/invalid as no wall.
static const float DHEADING_STEP = 0.12f; // Max heading nudge per tick (rad).

/** Private functions. ********************************************************/

static float clamp(const float value, const float low, const float high) {
  return value < low ? low : (value > high ? high : value);
}

/** Public functions. *********************************************************/

float heading_error_rad(const float left_mm, const float right_mm,
                        const float angle_rad, const float width_mm) {
  const float s = sinf(angle_rad);
  const float c = cosf(angle_rad);

  // Numerator and denominator for atan2.
  // (dL - dR) cos alpha:
  const float num = (left_mm - right_mm) * c;
  // w + (dL + dR) sin alpha:
  const float den = width_mm + (left_mm + right_mm) * s;

  // Signed heading error, angle from +Y (forward) to wall normal (CCW+).
  return atan2f(num, den);
}

float position_error_mm(const float d_left_mm, const float d_right_mm,
                        const float alpha_rad) {
  return 0.5f * (d_left_mm - d_right_mm) * sinf(alpha_rad);
}

void maze_control_step(void) {
  const float dL = vl53l4cd_distance_mm[0];
  const float dF = vl53l4cd_distance_mm[1];
  const float dR = vl53l4cd_distance_mm[2];
  const float alpha = MAZE_BOT_TOF_ANGLE_RAD;
  const float robot_w = MAZE_BOT_TOF_WIDTH_SPACING_MM;

  // Corridor errors for straights.
  heading_error_rad_calc = heading_error_rad(dL, dR, alpha, robot_w);
  position_error_mm_calc = position_error_mm(dL, dR, alpha);

  float v = 0.0f;    // Forward command.
  float dpsi = 0.0f; // Small heading increment for this tick.

  switch (mode) {
  case STRAIGHT: {
    v = V_FAST;

    // Convert corridor errors to a small heading setpoint nudge (rad).
    float steer =
        K_THETA * heading_error_rad_calc + KX_OVER_L * position_error_mm_calc;

    // Rate-limit the nudge so the outer PID can track smoothly.
    if (steer > DHEADING_STEP)
      steer = DHEADING_STEP;
    if (steer < -DHEADING_STEP)
      steer = -DHEADING_STEP;

    dpsi = steer; // Nudge setpoint by this much.

    set_relative_heading(dpsi); // Call for heading controller to take control.

    // Corner detection.
    if (dF > FRONT_INVALID && dF < FRONT_STOP) {
      mode = TURN;
    }
  } break;

  case TURN: {
    v = 0.0f;

    // Spin-in-place by nudging heading setpoint steadily.
    const float turn_dir = (dR >= dL) ? +1.0f : -1.0f;
    dpsi = turn_dir * DHEADING_STEP; // Constant-rate spin.
    set_relative_heading(dpsi);

    // Exit when aligned & front is clear (or front invalid).
    if (fabsf(heading_error_rad_calc) < ERR_PAR_OK &&
        (dF > FRONT_GO || dF < FRONT_INVALID)) {
      mode = STRAIGHT;
    }
  } break;
  }

  // Control H-bridge forward channel (yaw/heading handled by controls).
  h_bridge_linear = clamp(v, -1.0f, 1.0f);
}
