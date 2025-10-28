/*******************************************************************************
* @file maze_navigation.c
 * @brief Maze navigation software.
 *******************************************************************************
 */

/** Includes. *****************************************************************/

#include "maze_navigation.h"
#include "configuration.h"
#include "math.h"
#include "vl53l4cd_runner.h"

/** Private variables. ********************************************************/

/** Public variables. *********************************************************/

volatile float heading_error_rad_calc = 0;

/** Private functions. *********************************************************/

static float acos_clamped_rad(float x) {
  // Clamps input to [-1, 1] to avoid NaN from tiny numeric drift.
  if (x > 1.0f)
    x = 1.0f;
  if (x < -1.0f)
    x = -1.0f;
  return acosf(x);
}

static float asin_clamped_rad(float x) {
  // Clamps input to [-1, 1] to avoid NaN from tiny numeric drift.
  if (x > 1.0f)
    x = 1.0f;
  if (x < -1.0f)
    x = -1.0f;
  return asinf(x);
}

static float compute_line_from_points(const float left_mm, const float right_mm,
                                      const float angle_difference_rad) {
  return sqrtf(left_mm * left_mm + right_mm * right_mm -
               2 * left_mm * right_mm * cosf(angle_difference_rad));
}

/** Public functions. *********************************************************/

float heading_error_rad(const float left_mm, const float right_mm,
                        const float angle_rad, const float width_mm) {
  const float line =
      compute_line_from_points(left_mm, right_mm, M_PI - 2 * angle_rad);

  return M_PI / 2 - asin_clamped_rad(width_mm / line) -
         acos_clamped_rad(
             (line * line + left_mm * left_mm + right_mm * right_mm) /
             (2 * line * left_mm)) +
         angle_rad;
}
