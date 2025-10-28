/*******************************************************************************
* @file maze_navigation.c
 * @brief Maze navigation software.
 *******************************************************************************
 */

/** Includes. *****************************************************************/

#include "maze_navigation.h"
#include "math.h"

/** Public variables. *********************************************************/

volatile float heading_error_rad_calc = 0;

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
