/*******************************************************************************
 * @file state_machine.h
 * @brief High level Maze Bot state machine.
 *******************************************************************************
 */

#ifndef MAZE_BOT__STATE_MACHINE_H
#define MAZE_BOT__STATE_MACHINE_H

/** Public types. *************************************************************/

typedef enum state_machine {
  STATE_IDLE,
  STATE_INIT,
  STATE_STANDBY,
  STATE_TURN,
  STATE_ADVANCE_FOR_TICKS,
  STATE_ADVANCE_UNTIL_MM,
  STATE_PICKUP_PACKAGE,
  STATE_DROP_PACKAGE,
  STATE_MAZE_NAV_FOR_N_TURNS,
  STATE_MAZE_NAV_UNTIL_MM,
} state_t;

/** Public variables. *********************************************************/

extern state_t bot_state;

/** Public functions. *********************************************************/

void run_state_machine(void);

#endif
