/*******************************************************************************
 * @file maze_navigation.h
 * @brief Maze navigation software.
 *******************************************************************************
 */

#ifndef MAZE_BOT__MAZE_NAVIGATION_H
#define MAZE_BOT__MAZE_NAVIGATION_H

/** Includes. *****************************************************************/

#include "../maze_bot_driver/maze_bot_can_dbc.h"

/** Public variables. *********************************************************/

extern volatile float heading_error_rad_calc;
extern volatile float position_error_mm_calc;
extern volatile float v1;
extern volatile float v2;
extern volatile float v3;
extern volatile float v4;

/** Public functions. *********************************************************/

void process_msg1(can_header_t *header, uint8_t *data);
void process_msg2(can_header_t *header, uint8_t *data);

void corridor_straight(void);
void corridor_stop(void);

#endif
