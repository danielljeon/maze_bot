/*******************************************************************************
* @file drive.h
 * @brief High level Maze Bot drive code.
 *******************************************************************************
 */

#ifndef MAZE_BOT__DRIVE_H
#define MAZE_BOT__DRIVE_H

/** Public variables. *********************************************************/

extern float h_bridge_1_yaw_control;
extern float h_bridge_2_yaw_control;
extern float h_bridge_linear;

/** Public functions. *********************************************************/

void drive(void);

#endif
