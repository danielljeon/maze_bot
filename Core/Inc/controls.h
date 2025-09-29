/*******************************************************************************
 * @file controls.h
 * @brief High level Maze Bot control systems.
 *******************************************************************************
 */

#ifndef MAZE_BOT__CONTROLS_H
#define MAZE_BOT__CONTROLS_H

/** Public variables. *********************************************************/

extern volatile float heading_setpoint_rad;
extern volatile float yaw_meas;
extern volatile float yaw_rate_sp;
extern volatile float turn_cmd;

/** Public functions. *********************************************************/

void control_loops_init(void);
void yaw_rate_loop(void);
void heading_loop(void);
void actuate_loop(void);

#endif
