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

typedef enum { IDLE, STRAIGHT, TURN } mode_t;

/** Public variables. *********************************************************/

volatile float heading_error_rad_calc = 0;
volatile float position_error_mm_calc = 0;

/** Private variables. ********************************************************/

static mode_t mode = IDLE;
static uint8_t state_standby_counter = 0;

// Calibrations.
static const float V_FAST = 0.2f;       // Forward command in [-1,1].
static const float K_THETA = 1.10f;     // Corridor parallel gain (rad -> cmd).
static const float KX_OVER_L = 0.02f;   // Centering bias gain (mm^-1).
static const float ERR_PAR_OK = 0.17f;  // Consider "aligned".
static const float FRONT_STOP = 100.0f; // Stop and turn if front < this (mm).
static const float FRONT_GO = 165.0f;   // Resume straight if front > this (mm).
static const uint16_t MIN_STRAIGHT_TICKS = 20; // Minimum ticks in STRAIGHT.

// Tank-turn step.
static const float DHEADING_STEP_MAX = 0.25f;        // Max heading nudge (rad).
static const float DHEADING_STEP_MULTIPLIER = 0.48f; // Step multipler.

// Minimum ticks in state counters.
static uint16_t straight_counter = 0;

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

static float corridor_parallel_error_rad(const float dL, const float dR,
                                         const float alpha,
                                         const float robot_w) {
  const float s = sinf(alpha), c = cosf(alpha);
  const float vx = robot_w + (dL + dR) * s; // Across corridor.
  const float vy = (dR - dL) * c;           // Along corridor.
  float e = atan2f(vx, vy) - (float)M_PI_2;
  while (e > M_PI)
    e -= 2.0f * (float)M_PI;
  while (e <= -M_PI)
    e += 2.0f * (float)M_PI;
  return e;
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
  heading_error_rad_calc = corridor_parallel_error_rad(dL, dR, alpha, robot_w);
  position_error_mm_calc = position_error_mm(dL, dR, alpha);

  float v = 0.0f;    // Forward command.
  float dpsi = 0.0f; // Small heading increment for this tick.

  switch (mode) {

  case IDLE:

    if (state_standby_counter > 10) {
      // Initialize startup controls.
      zero_heading();
      mode = STRAIGHT;

    } else if (vl53l4cd_distance_mm[1] > 200) { // Ensure middle TOF clear.
      state_standby_counter++;
    }
    break;

  case STRAIGHT: {
    v = V_FAST;

    // Convert corridor errors to a small heading setpoint nudge (rad).
    float steer =
        K_THETA * heading_error_rad_calc + KX_OVER_L * position_error_mm_calc;

    // Rate-limit the nudge so the outer PID can track smoothly.
    if (steer > DHEADING_STEP_MAX)
      steer = DHEADING_STEP_MAX;
    if (steer < -DHEADING_STEP_MAX)
      steer = -DHEADING_STEP_MAX;

    // Update straight counter to ensure minimum straight travel time.
    if (straight_counter <= MIN_STRAIGHT_TICKS) {
      straight_counter++; // Enforce constant heading for a fixed time.
      dpsi = 0.0f;
    } else {
      dpsi = steer; // Nudge setpoint by this much.
    }

    set_relative_heading(
        dpsi * DHEADING_STEP_MULTIPLIER); // Call for heading controller.

    // Corner detection.
    if (dF < FRONT_STOP) {
      mode = TURN;
    }
  } break;

  case TURN: {
    // Pure point/tank turn, no forward motion.
    v = 0.0f;

    // Spin-in-place by nudging heading setpoint steadily.
    const float turn_dir = (dL >= dR) ? +1.0f : -1.0f;

    // Steady heading nudge for pivot.
    dpsi = turn_dir * DHEADING_STEP_MULTIPLIER;
    set_relative_heading(dpsi);

    // Exit when aligned and the front is clear (or front invalid).
    if (fabsf(heading_error_rad_calc) < ERR_PAR_OK && (dF > FRONT_GO)) {
      straight_counter = 0;
      mode = STRAIGHT;
      zero_heading(); // TODO(Maze bot): Band-aid.
    }
  } break;
  }

  // Control H-bridge forward channel (yaw/heading handled by controls).
  h_bridge_linear = clamp(v, -1.0f, 1.0f);
}
