/*******************************************************************************
 * @file maze_navigation.h
 * @brief Maze navigation software.
 *******************************************************************************
 */

#ifndef MAZE_BOT__MAZE_NAVIGATION_H
#define MAZE_BOT__MAZE_NAVIGATION_H

/** Public variables. *********************************************************/

extern volatile float heading_error_rad_calc;

/** Public functions. *********************************************************/

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
 * @param angle_rad Mounting angle (radians) of each sensor relative to the
 *                  robot's forward axis.
 * @param width_mm Distance (mm) between the two sensors (robot width).
 *
 * @return Heading error in radians (positive CCW from forward to wall normal).
 */
float heading_error_rad(float left_mm, float right_mm, float angle_rad,
                        float width_mm);

#endif
