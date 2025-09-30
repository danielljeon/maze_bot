/*******************************************************************************
 * @file controls.h
 * @brief High level Maze Bot control systems.
 *******************************************************************************
 */

#ifndef MAZE_BOT__CONTROLS_H
#define MAZE_BOT__CONTROLS_H

/** Public functions. *********************************************************/

void set_relative_heading(float delta_rad);

void control_loops_init(void);
void zero_heading(void);
void yaw_rate_loop(void);
void heading_loop(void);
void actuate_loop(void);

#endif
