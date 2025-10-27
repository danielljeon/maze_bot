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

/** Private functions. *********************************************************/

static float acos_clamped_rad(float x) {
  // Clamps input to [-1, 1] to avoid NaN from tiny numeric drift.
  if (x > 1.0f)
    x = 1.0f;
  if (x < -1.0f)
    x = -1.0f;
  return acosf(x);
}

static float compute_line_from_points(const float left_mm, const float right_mm,
                                      const float angle_rad) {
  return sqrtf(left_mm * left_mm + right_mm * right_mm -
               2 * left_mm * right_mm * cosf(M_PI - 2 * angle_rad));
}

/** Public functions. *********************************************************/

float heading_error_rad(const float left_mm, const float right_mm,
                        const float angle_rad) {
  const float line = compute_line_from_points(left_mm, right_mm, angle_rad);
  return acos_clamped_rad(MAZE_BOT_NOMINAL_MAZE_WIDTH_MM / line);
}
