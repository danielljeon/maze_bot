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

typedef enum { IDLE, STANDBY, STRAIGHT, TURN, SETTLING } mode_t;

/** Public variables. *********************************************************/

volatile float heading_error_rad_calc = 0;
volatile float position_error_mm_calc = 0;

/** Private variables. ********************************************************/

static mode_t mode = STANDBY;

// Calibrations.
static const float V_FAST = 0.2f;      // Forward command in [-1,1].
static const float K_THETA = 1.10f;    // Corridor parallel gain (rad -> cmd).
static const float KX_OVER_L = 0.02f;  // Centering bias gain (mm^-1).
static const float ERR_PAR_OK = 0.17f; // Consider "aligned".

// Heading nudge calibrations.
static const float HEADING_STEP_MAX = 0.25f;        // Max heading nudge (rad).
static const float HEADING_STEP_MULTIPLIER = 0.48f; // Step multiplier.

// IDLE state calibrations.
static const float STANDBY_TICKS = 100.0f; // Ticks for standby duration.
static uint8_t state_standby_counter = 0;

// STRAIGHT state calibrations.
static const uint16_t MIN_STRAIGHT_TICKS = 30; // Minimum ticks in STRAIGHT.
static uint16_t straight_counter = 0; // Minimum ticks in state counters.

// TURN state calibrations.
static const float FRONT_STOP = 100.0f; // Stop and turn if front < this (mm).
static const float FRONT_GO = 165.0f;   // Resume straight if front > this (mm).

// SETTLING state calibrations.
static const uint16_t SETTLING_TICKS = 100; // Settle after turn.
static uint16_t settling_tick_counter = 0;

/** Private functions. ********************************************************/

static float clamp(const float value, const float low, const float high) {
  return value < low ? low : (value > high ? high : value);
}

/** Public functions. *********************************************************/

float heading_error_rad(const float left_mm, const float right_mm,
                        const float angle_rad, const float width_mm) {
  const float sin = sinf(angle_rad);
  const float cos = cosf(angle_rad);

  // Numerator and denominator for atan2.
  // (left_mm - right_mm) cos angle_rad:
  const float numerator = (left_mm - right_mm) * cos;
  // w + (left_mm + right_mm) sin angle_rad:
  const float denominator = width_mm + (left_mm + right_mm) * sin;

  // Signed heading error, angle from +Y (forward) to wall normal (CCW+).
  return atan2f(numerator, denominator);
}

static float corridor_parallel_error_rad(const float left_mm,
                                         const float right_mm,
                                         const float angle_rad,
                                         const float width_mm) {
  const float sin = sinf(angle_rad);
  const float cos = cosf(angle_rad);

  // Across corridor.
  const float vx = width_mm + (left_mm + right_mm) * sin;

  // Along corridor.
  const float vy = (right_mm - left_mm) * cos;

  float error = atan2f(vx, vy) - (float)M_PI_2;
  while (error > M_PI)
    error -= 2.0f * (float)M_PI;
  while (error <= -M_PI)
    error += 2.0f * (float)M_PI;

  return error;
}

float position_error_mm(const float d_left_mm, const float d_right_mm,
                        const float alpha_rad) {
  return 0.5f * (d_left_mm - d_right_mm) * sinf(alpha_rad);
}

void maze_control_step(void) {
  const float left_mm = vl53l4cd_distance_mm[0];
  const float front_mm = vl53l4cd_distance_mm[1];
  const float right_mm = vl53l4cd_distance_mm[2];
  const float angle_rad = MAZE_BOT_TOF_ANGLE_RAD;
  const float width_mm = MAZE_BOT_TOF_WIDTH_SPACING_MM;

  // Corridor errors.
  heading_error_rad_calc =
      corridor_parallel_error_rad(left_mm, right_mm, angle_rad, width_mm);
  position_error_mm_calc = position_error_mm(left_mm, right_mm, angle_rad);

  float v = 0.0f; // Forward command.

  switch (mode) {

  case IDLE:
    break;

  case STANDBY:
    if (state_standby_counter > 10) {
      // Initialize startup controls.
      zero_heading();
      mode = STRAIGHT;

    } else if (front_mm > STANDBY_TICKS) { // Ensure middle TOF clear.
      state_standby_counter++;
    }
    break;

  case STRAIGHT:
    v = V_FAST;
    float nudge; // Small heading increment/nudge.

    // Convert corridor errors to a small heading setpoint nudge (rad).
    float steer =
        K_THETA * heading_error_rad_calc + KX_OVER_L * position_error_mm_calc;

    // Rate-limit the nudge so the outer PID can track smoothly.
    if (steer > HEADING_STEP_MAX)
      steer = HEADING_STEP_MAX;
    if (steer < -HEADING_STEP_MAX)
      steer = -HEADING_STEP_MAX;

    // Update straight counter to ensure minimum straight travel time.
    if (straight_counter <= MIN_STRAIGHT_TICKS) {
      straight_counter++; // Enforce constant heading for a fixed time.
      nudge = 0.0f;
    } else {
      nudge = steer; // Nudge setpoint by this much.
    }

    // Set controls setpoint (heading controller).
    set_relative_heading(nudge * HEADING_STEP_MULTIPLIER);

    // Corner detection.
    if (front_mm < FRONT_STOP) {
      straight_counter = 0;
      mode = TURN;
    }
    break;

  case TURN:
    // Pure point/tank turn, no forward motion.
    v = 0.0f;

    // Spin-in-place by nudging heading setpoint steadily.
    const float turn_dir = left_mm >= right_mm ? +1.0f : -1.0f;

    // Set controls setpoint (heading controller).
    set_relative_heading(turn_dir * HEADING_STEP_MULTIPLIER);

    // Exit when aligned and the front is clear (or front invalid).
    if (fabsf(heading_error_rad_calc) < ERR_PAR_OK && (front_mm > FRONT_GO)) {
      mode = SETTLING;
    }
    break;

  case SETTLING:
    if (settling_tick_counter <= SETTLING_TICKS) {
      settling_tick_counter++;
    } else {
      settling_tick_counter = 0;
      mode = STRAIGHT;
    }
    break;
  }

  // Control H-bridge forward channel (yaw/heading handled by controls).
  h_bridge_linear = clamp(v, -1.0f, 1.0f);
}
