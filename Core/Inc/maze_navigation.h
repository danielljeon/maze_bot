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
extern volatile uint16_t vision_x1;
extern volatile uint16_t vision_y1;
extern volatile uint16_t vision_x2;
extern volatile uint16_t vision_y2;

/** Public functions. *********************************************************/

void process_vision(can_header_t *header, uint8_t *data);

void corridor_straight(void);
void corridor_stop(void);

#endif
