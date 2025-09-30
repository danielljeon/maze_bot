/*******************************************************************************
* @file state_machine.h
 * @brief High level Maze Bot state machine.
 *******************************************************************************
 */

#ifndef MAZE_BOT__STATE_MACHINE_H
#define MAZE_BOT__STATE_MACHINE_H

/** Includes. *****************************************************************/

#include <stdint.h>

/** Public types. *************************************************************/

typedef enum state_machine {
  STATE_IDLE = 0,
  STATE_INIT = 1,
  STATE_MOMENTARY_SENSE = 2,
  STATE_TURN = 3,
  STATE_ADVANCE = 4,
  STATE_PICKUP_PACKAGE = 5,
  STATE_DROP_PACKAGE = 6,
} state_t;

/** Public variables. *********************************************************/

extern state_t bot_state;
extern uint8_t angle_target_reach_count;
extern uint16_t advance_reach_count;

/** Public functions. *********************************************************/

void drive(void);
void run_state_machine(void);

#endif
