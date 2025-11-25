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

/** Public variables. *********************************************************/

volatile float heading_error_rad_calc = 0;
volatile float position_error_mm_calc = 0;
volatile float v1 = 0;
volatile float v2 = 0;
volatile float v3 = 0;
volatile float v4 = 0;

/** Public functions. *********************************************************/

void process_msg1(can_header_t *header, uint8_t *data) {
  memcpy(&v1, &data[0], 4);
  memcpy(&v2, &data[4], 4);
}

void process_msg2(can_header_t *header, uint8_t *data) {
  memcpy(&v3, &data[0], 4);
  memcpy(&v4, &data[4], 4);
}

/** Private variables. ********************************************************/

// Calibrations.
static const float V_FAST = 0.2f;     // Forward command in [-1,1].
static const float K_THETA = 0.8f;    // Corridor parallel gain (rad -> cmd).
static const float KX_OVER_L = 0.03f; // Centering bias gain (mm^-1).

// Heading nudge calibrations.
static const float HEADING_STEP_MAX = 0.25f; // Max heading nudge (rad).

/** Private functions. ********************************************************/

static float clamp(const float value, const float low, const float high) {
  return value < low ? low : (value > high ? high : value);
}

/**
 * @brief Computes the robot's heading error (in radians) relative to a wall
 *        using two time-of-flight (ToF) distance measurements.
 *
 * The function assumes two ToF sensors are mounted symmetrically on the
 * front-left and front-right corners of the robot, each angled outward by
 * `angle_rad` from the forward axis. Using the measured left and right
 * distances, it computes the angular deviation between the robot's forward
 * direction and the wall's normal vector.
 *
 * Geometry reference:
 *  - Robot width: w.
 *  - Sensor mounting angle: +- angle_rad from forward.
 *  - Measured distances: left_mm, right_mm.
 *
 * Formula:
 *    theta_err = atan2(
 *                  (left_mm - right_mm) * cos(angle_rad),
 *                  width_mm + (left_mm + right_mm) * sin(angle_rad)
 *                )
 *
 * Positive theta_err indicates the robot is rotated counterclockwise
 * relative to the wall normal (i.e., facing slightly left).
 *
 * @param left_mm Distance (mm) from the left ToF sensor to the wall.
 * @param right_mm Distance (mm) from the right ToF sensor to the wall.
 * @param angle_rad Angle (radians) of each sensor from the forward axis.
 * @param width_mm Distance (mm) between the two sensors (robot width).
 *
 * @return Heading error in radians (positive CCW from forward to wall normal).
 */
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

/**
 * @brief Computes the robot's lateral offset (centering error) in a corridor.
 *
 * The function computes how far the robot is displaced from the corridor
 * centerline, assuming two side-mounted distance sensors are angled outward by
 * `alpha_rad` relative to the robot's forward axis and are roughly parallel to
 * the corridor walls.
 *
* Geometry reference:
 *  - Sensor mounting angle: +- angle_rad from forward.
 *  - Measured distances: left_mm, right_mm.
 *  - The perpendicular wall distances are d*sin(alpha).
 *  - The offset from the corridor centerline is half the difference between.
 *
 * Formula:
 *    x_err = 0.5 * (d_left_mm - d_right_mm) * sin(alpha_rad)
 *
 * Sign convention:
 *  - Positive x_err indicates the robot is left of the corridor center (left
 *    wall closer).
*   - Negative x_err indicates the robot is right of the corridor center (right
 *    wall closer).
 *  - Zero x_err is the center.
 *
 * @param d_left_mm Distance (mm) from the left side sensor to its wall.
 * @param d_right_mm Distance (mm) from the right side sensor to its wall.
 * @param alpha_rad Angle (radians) of each sensor from the forward axis.
 *
 * @return Lateral offset (mm) of the robot from the corridor centerline.
 */
float position_error_mm(const float d_left_mm, const float d_right_mm,
                        const float alpha_rad) {
  return 0.5f * (d_left_mm - d_right_mm) * sinf(alpha_rad);
}

float corridor_parallel_error_rad(const float left_mm, const float right_mm,
                                  const float angle_rad, const float width_mm) {
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

/** Public functions. *********************************************************/

void corridor_straight(void) {
  const float left_mm = vl53l4cd_distance_mm[0];
  const float right_mm = vl53l4cd_distance_mm[2];
  const float angle_rad = MAZE_BOT_TOF_ANGLE_RAD;
  const float width_mm = MAZE_BOT_TOF_WIDTH_SPACING_MM;

  // Corridor errors.
  heading_error_rad_calc =
      corridor_parallel_error_rad(left_mm, right_mm, angle_rad, width_mm);
  position_error_mm_calc = position_error_mm(left_mm, right_mm, angle_rad);

  // Convert corridor errors to a small heading setpoint nudge (rad).
  float steer =
      K_THETA * heading_error_rad_calc + KX_OVER_L * position_error_mm_calc;

  // Rate-limit the nudge so the outer PID can track smoothly.
  if (steer > HEADING_STEP_MAX)
    steer = HEADING_STEP_MAX;
  if (steer < -HEADING_STEP_MAX)
    steer = -HEADING_STEP_MAX;

  // Set controls setpoint (heading controller).
  set_relative_heading(steer);

  // Control H-bridge forward channel (yaw/heading handled by controls).
  h_bridge_linear = clamp(V_FAST, -1.0f, 1.0f);
}

void corridor_stop(void) { h_bridge_linear = 0; }
