/*******************************************************************************
 * @file configuration.h <- TODO: DEV CONFIGURATIONS!
 * @brief High level hardcoded configuration related declarations/definitions.
 *******************************************************************************
 */

#ifndef MAZE_BOT__CONFIGURATION_H
#define MAZE_BOT__CONFIGURATION_H

/** Includes. *****************************************************************/

#include "math.h"

/** Configuration definitions. ************************************************/

// Maze navigation related constraints.
#define MAZE_BOT_NOMINAL_MAZE_WIDTH_MM 155.0f
#define MAZE_BOT_TOF_ANGLE_RAD (0.25f * M_PI)

// Dev related features:

// Define to use direct public TOF (distance_mm) data.
// #define MAZE_BOT_VL53L4CD_PUBLIC_DIRECT_VAR

#endif
